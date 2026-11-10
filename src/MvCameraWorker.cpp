#include <opencv2/imgproc.hpp>

#include "MvCameraWorker.h"

#include <QTimer>

#include <cstring>
#include <new>

namespace {

constexpr int kGrabTimeoutMs = 1000;

int boundedLength(const char *text, int capacity)
{
    int length = 0;
    if (text == nullptr) {
        return 0;
    }
    while (length < capacity && text[length] != '\0') {
        ++length;
    }
    return length;
}

template <typename T, size_t N>
QString sdkString(const T (&field)[N])
{
    const char *text = reinterpret_cast<const char *>(field);
    return QString::fromLocal8Bit(text, boundedLength(text, static_cast<int>(N)));
}

QString hexRet(int nRet)
{
    return QString("0x%1").arg(static_cast<uint>(nRet), 0, 16);
}

} // namespace

MvCameraWorker::MvCameraWorker(QObject *parent)
    : QObject(parent)
{
}

MvCameraWorker::~MvCameraWorker()
{
    releaseDevice(false);
}

void MvCameraWorker::searchDevices()
{
    std::memset(&m_deviceList, 0, sizeof(m_deviceList));
    const int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_deviceList);
    if (nRet != MV_OK) {
        emit errorOccurred(QString("Enum Devices fail! nRet [%1]").arg(hexRet(nRet)));
        emit devicesFound(QStringList());
        return;
    }

    QStringList devices;
    for (unsigned int i = 0; i < m_deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO *info = m_deviceList.pDeviceInfo[i];
        if (info == nullptr) {
            break;
        }
        devices << deviceText(info, i, true);
    }

    emit devicesFound(devices);
    if (devices.isEmpty()) {
        emit statusMessage(QStringLiteral("Find No Devices!"));
    } else {
        emit statusMessage(QString("Find %1 device(s).").arg(devices.size()));
    }
}

void MvCameraWorker::connectDevice(int index)
{
    if (index < 0
        || static_cast<unsigned int>(index) >= m_deviceList.nDeviceNum
        || m_deviceList.pDeviceInfo[index] == nullptr) {
        emit errorOccurred(QStringLiteral("Invalid camera index."));
        return;
    }

    if (m_handle != nullptr || m_opened) {
        releaseDevice(true);
    }

    MV_CC_DEVICE_INFO *info = m_deviceList.pDeviceInfo[index];
    if (!MV_CC_IsDeviceAccessible(info, MV_ACCESS_Exclusive)) {
        emit errorOccurred(QStringLiteral("Can't connect! Device is not accessible."));
        return;
    }

    int nRet = MV_CC_CreateHandle(&m_handle, info);
    if (nRet != MV_OK) {
        m_handle = nullptr;
        emit errorOccurred(QString("Create Handle fail! nRet [%1]").arg(hexRet(nRet)));
        return;
    }

    nRet = MV_CC_OpenDevice(m_handle);
    if (nRet != MV_OK) {
        emit errorOccurred(QString("Open Device fail! nRet [%1]").arg(hexRet(nRet)));
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
        return;
    }
    m_opened = true;

    if (info->nTLayerType == MV_GIGE_DEVICE) {
        const int packetSize = MV_CC_GetOptimalPacketSize(m_handle);
        if (packetSize > 0) {
            nRet = MV_CC_SetIntValue(m_handle, "GevSCPSPacketSize", static_cast<unsigned int>(packetSize));
            if (nRet != MV_OK) {
                emit statusMessage(QString("Warning: Set Packet Size fail! nRet [%1]").arg(hexRet(nRet)));
            }
        } else {
            emit statusMessage(QString("Warning: Get Packet Size fail! nRet [%1]").arg(hexRet(packetSize)));
        }
    }

    // ch:连续采集，关闭触发模式 | en:Free-run preview, trigger mode off.
    nRet = MV_CC_SetEnumValue(m_handle, "TriggerMode", 0);
    if (nRet != MV_OK) {
        emit errorOccurred(QString("Set Trigger Mode fail! nRet [%1]").arg(hexRet(nRet)));
        releaseDevice(false);
        return;
    }

    MVCC_INTVALUE stParam;
    std::memset(&stParam, 0, sizeof(stParam));
    nRet = MV_CC_GetIntValue(m_handle, "PayloadSize", &stParam);
    if (nRet != MV_OK || stParam.nCurValue == 0) {
        emit errorOccurred(QString("Get PayloadSize fail! nRet [%1]").arg(hexRet(nRet)));
        releaseDevice(false);
        return;
    }

    m_payloadSize = stParam.nCurValue;
    try {
        m_payload.assign(m_payloadSize, 0);
    } catch (const std::bad_alloc &) {
        emit errorOccurred(QStringLiteral("Allocate memory failed."));
        releaseDevice(false);
        return;
    }

    m_connectedIndex = index;
    emit deviceConnected(index);
    emit statusMessage(QString("Connected: %1").arg(deviceText(info, static_cast<unsigned int>(index), false)));
}

void MvCameraWorker::startGrabbing()
{
    if (!m_opened || m_handle == nullptr || m_payload.empty()) {
        emit errorOccurred(QStringLiteral("Start Grabbing fail! Device is not connected."));
        return;
    }
    if (m_grabbingStarted) {
        return;
    }

    const int nRet = MV_CC_StartGrabbing(m_handle);
    if (nRet != MV_OK) {
        emit errorOccurred(QString("Start Grabbing fail! nRet [%1]").arg(hexRet(nRet)));
        return;
    }

    m_grabbing = true;
    m_grabbingStarted = true;
    m_displayBusy = false;
    m_loggedFirstFrame = false;
    m_hasFailedPixel = false;
    m_grabErrorTimer.invalidate();
    emit grabbingStarted();
    emit statusMessage(QStringLiteral("Start grabbing."));
    QTimer::singleShot(0, this, [this]() { grabFrame(); });
}

void MvCameraWorker::stopGrabbing()
{
    endGrabbing(true);
}

void MvCameraWorker::disconnectDevice()
{
    releaseDevice(true);
}

void MvCameraWorker::notifyFrameDisplayed()
{
    m_displayBusy = false;
}

void MvCameraWorker::grabFrame()
{
    if (!m_grabbing || m_handle == nullptr || m_payload.empty()) {
        return;
    }

    MV_FRAME_OUT_INFO_EX info;
    std::memset(&info, 0, sizeof(info));
    const int nRet = MV_CC_GetOneFrameTimeout(
        m_handle, m_payload.data(), m_payloadSize, &info, kGrabTimeoutMs);

    if (!m_grabbing) {
        return;
    }

    if (nRet == MV_OK) {
        if (!m_loggedFirstFrame) {
            m_loggedFirstFrame = true;
            emit statusMessage(QString("Get One Frame: Width[%1], Height[%2], FrameNum[%3]")
                                   .arg(info.nWidth)
                                   .arg(info.nHeight)
                                   .arg(info.nFrameNum));
        }

        // 界面还没显示完上一帧时只把相机缓冲读走，避免事件队列堆积整幅图像。
        if (!m_displayBusy) {
            cv::Mat image;
            try {
                if (frameToMat(&info, m_payload.data(), image)) {
                    m_displayBusy = true;
                    emit imageReady(image);
                }
            } catch (const cv::Exception &ex) {
                if (!m_hasFailedPixel) {
                    m_hasFailedPixel = true;
                    emit errorOccurred(QString("Convert frame fail: %1").arg(QString::fromLocal8Bit(ex.what())));
                }
            }
        }
    } else if (!m_grabErrorTimer.isValid() || m_grabErrorTimer.elapsed() >= 1000) {
        emit errorOccurred(QString("Get Frame fail! nRet [%1]").arg(hexRet(nRet)));
        m_grabErrorTimer.start();
    }

    if (m_grabbing) {
        // 取到图后马上读下一帧。SDK 立刻返回失败时稍等，避免空转占满线程。
        const int delayMs = (nRet == MV_OK) ? 0 : 30;
        QTimer::singleShot(delayMs, this, [this]() { grabFrame(); });
    }
}

void MvCameraWorker::releaseDevice(bool notify)
{
    const bool announced = m_connectedIndex >= 0;
    endGrabbing(notify);

    if (m_handle != nullptr && m_opened) {
        const int nRet = MV_CC_CloseDevice(m_handle);
        m_opened = false;
        if (notify && nRet != MV_OK) {
            emit errorOccurred(QString("CloseDevice fail! nRet [%1]").arg(hexRet(nRet)));
        }
    }

    if (m_handle != nullptr) {
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
    }

    m_payload.clear();
    m_payloadSize = 0;
    m_connectedIndex = -1;
    m_opened = false;

    if (notify && announced) {
        emit deviceDisconnected();
        emit statusMessage(QStringLiteral("Device closed."));
    }
}

bool MvCameraWorker::endGrabbing(bool notify)
{
    if (!m_grabbing && !m_grabbingStarted) {
        return false;
    }

    m_grabbing = false;
    int nRet = MV_OK;
    if (m_handle != nullptr && m_grabbingStarted) {
        nRet = MV_CC_StopGrabbing(m_handle);
        m_grabbingStarted = false;
    } else {
        m_grabbingStarted = false;
    }

    m_loggedFirstFrame = false;
    m_displayBusy = false;
    m_hasFailedPixel = false;

    if (notify) {
        if (nRet != MV_OK) {
            emit errorOccurred(QString("Stop Grabbing fail! nRet [%1]").arg(hexRet(nRet)));
        }
        emit grabbingStopped();
    }
    return true;
}

bool MvCameraWorker::frameToMat(const MV_FRAME_OUT_INFO_EX *info, unsigned char *data, cv::Mat &out)
{
    if (info == nullptr || data == nullptr || info->nWidth == 0 || info->nHeight == 0) {
        return false;
    }

    if (info->enPixelType == PixelType_Gvsp_Mono8) {
        const cv::Mat mono(info->nHeight, info->nWidth, CV_8UC1, data);
        out = mono.clone();
        return !out.empty();
    }
    if (info->enPixelType == PixelType_Gvsp_BGR8_Packed) {
        const cv::Mat bgr(info->nHeight, info->nWidth, CV_8UC3, data);
        out = bgr.clone();
        return !out.empty();
    }
    if (info->enPixelType == PixelType_Gvsp_RGB8_Packed) {
        const cv::Mat rgb(info->nHeight, info->nWidth, CV_8UC3, data);
        cv::cvtColor(rgb, out, cv::COLOR_RGB2BGR);
        return !out.empty();
    }

    // Bayer、YUV 等格式在关句柄前转成 BGR8，再包装成 Mat。
    if (convertWithSdk(info, data, PixelType_Gvsp_BGR8_Packed, out)) {
        return true;
    }
    if (convertWithSdk(info, data, PixelType_Gvsp_Mono8, out)) {
        return true;
    }

    const unsigned int pixel = static_cast<unsigned int>(info->enPixelType);
    if (!m_hasFailedPixel || m_lastFailedPixel != pixel) {
        m_hasFailedPixel = true;
        m_lastFailedPixel = pixel;
        emit errorOccurred(QString("Unsupported pixel format: 0x%1").arg(pixel, 0, 16));
    }
    return false;
}

bool MvCameraWorker::convertWithSdk(const MV_FRAME_OUT_INFO_EX *info, unsigned char *data,
                                    MvGvspPixelType dstType, cv::Mat &out)
{
    if (m_handle == nullptr || info == nullptr || data == nullptr) {
        return false;
    }
    if (info->nWidth == 0 || info->nHeight == 0) {
        return false;
    }

    const int channels = (dstType == PixelType_Gvsp_Mono8) ? 1 : 3;
    const size_t dstSize = static_cast<size_t>(info->nWidth) * static_cast<size_t>(info->nHeight)
                           * static_cast<size_t>(channels);
    if (m_convertBuffer.size() < dstSize) {
        m_convertBuffer.resize(dstSize);
    }

    MV_CC_PIXEL_CONVERT_PARAM param;
    std::memset(&param, 0, sizeof(param));
    param.nWidth = static_cast<unsigned short>(info->nWidth);
    param.nHeight = static_cast<unsigned short>(info->nHeight);
    param.enSrcPixelType = info->enPixelType;
    param.pSrcData = data;
    param.nSrcDataLen = info->nFrameLen > 0 ? info->nFrameLen : m_payloadSize;
    param.enDstPixelType = dstType;
    param.pDstBuffer = m_convertBuffer.data();
    param.nDstBufferSize = static_cast<unsigned int>(m_convertBuffer.size());

    if (MV_CC_ConvertPixelType(m_handle, &param) != MV_OK) {
        return false;
    }

    const int cvType = (channels == 1) ? CV_8UC1 : CV_8UC3;
    const cv::Mat view(info->nHeight, info->nWidth, cvType, m_convertBuffer.data());
    out = view.clone();
    return !out.empty();
}

QString MvCameraWorker::deviceText(MV_CC_DEVICE_INFO *info, unsigned int index, bool markOccupied) const
{
    if (info == nullptr) {
        return QString("[%1] NULL info").arg(index);
    }

    QString text;
    if (info->nTLayerType == MV_GIGE_DEVICE) {
        const unsigned int ip = info->SpecialInfo.stGigEInfo.nCurrentIp;
        const int ip1 = static_cast<int>((ip & 0xff000000u) >> 24);
        const int ip2 = static_cast<int>((ip & 0x00ff0000u) >> 16);
        const int ip3 = static_cast<int>((ip & 0x0000ff00u) >> 8);
        const int ip4 = static_cast<int>(ip & 0x000000ffu);
        text = QString("[%1] GigE  IP: %2.%3.%4.%5  Model: %6  Name: %7  SN: %8")
                   .arg(QString::number(index),
                        QString::number(ip1),
                        QString::number(ip2),
                        QString::number(ip3),
                        QString::number(ip4),
                        sdkString(info->SpecialInfo.stGigEInfo.chModelName),
                        sdkString(info->SpecialInfo.stGigEInfo.chUserDefinedName),
                        sdkString(info->SpecialInfo.stGigEInfo.chSerialNumber));
    } else if (info->nTLayerType == MV_USB_DEVICE) {
        text = QString("[%1] USB3  Model: %2  Name: %3  SN: %4")
                   .arg(QString::number(index),
                        sdkString(info->SpecialInfo.stUsb3VInfo.chModelName),
                        sdkString(info->SpecialInfo.stUsb3VInfo.chUserDefinedName),
                        sdkString(info->SpecialInfo.stUsb3VInfo.chSerialNumber));
    } else {
        text = QString("[%1] Not support.").arg(index);
    }

    if (markOccupied
        && (info->nTLayerType == MV_GIGE_DEVICE || info->nTLayerType == MV_USB_DEVICE)
        && !MV_CC_IsDeviceAccessible(info, MV_ACCESS_Exclusive)) {
        text += QStringLiteral("  [occupied]");
    }
    return text;
}
