/**
 * @file meltCam.cpp
 * @brief MvCameraWorker 实现：海康 MVS SDK 枚举、连接、连续采集与格式转换。
 */
#include <opencv2/imgproc.hpp>

#include "meltCam.h"

#include <QTimer>

#include <cstring>
#include <new>

namespace {

    constexpr int kGrabTimeoutMs = 1000;

    /**
     * @brief 计算 C 字符串有效长度（不超过 capacity）。
     * @param text 输入字符串。
     * @param capacity 字段最大容量。
     * @return 不含结尾空字符的长度。
     */
    int boundedLength(const char* text, int capacity)
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

    /**
     * @brief 将 SDK 定长字符数组转为 QString。
     * @param field SDK 结构体内的 char 数组。
     * @return 本地化字符串。
     */
    template <typename T, size_t N>
    QString sdkString(const T(&field)[N])
    {
        const char* text = reinterpret_cast<const char*>(field);
        return QString::fromLocal8Bit(text, boundedLength(text, static_cast<int>(N)));
    }

    /**
     * @brief 将 SDK 返回码格式化为十六进制字符串。
     * @param nRet MVS 错误码。
     */
    QString hexRet(int nRet)
    {
        return QString("0x%1").arg(static_cast<uint>(nRet), 0, 16);
    }

} // namespace

/**
 * @brief 构造并绑定定时器到 grabFrame。
 * @param parent Qt 父对象。
 */
MvCameraWorker::MvCameraWorker(QObject* parent)
    : QObject(parent)
{
    m_timer = new QTimer();

    connect(m_timer, &QTimer::timeout, this, &MvCameraWorker::grabFrame);
}

/** @brief 析构时静默释放设备。 */
MvCameraWorker::~MvCameraWorker()
{
    releaseDevice(false);
}

/** @brief 枚举 GigE/USB 设备，上报型号列表或枚举失败。 */
void MvCameraWorker::searchDevices()
{
    std::memset(&m_deviceList, 0, sizeof(m_deviceList));
    const int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_deviceList);
    if (nRet != MV_OK) {
        emit errorOccurred(QString("Enum Devices fail! nRet [%1]").arg(hexRet(nRet)));
        return;
    }

    std::vector<QString> devices_model;
    for (unsigned int i = 0; i < m_deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* info = m_deviceList.pDeviceInfo[i];
        if (info == nullptr) {
            break;
        }
        devices_model.push_back(sdkString(info->SpecialInfo.stGigEInfo.chModelName));
    }
    emit devicesFound(devices_model);
    
    if (devices_model.size() == 0) {
        emit statusMessage(QStringLiteral("Find No Devices!"));
    }
    else {
        emit statusMessage(QString("Find device done!s."));
    }
}

/**
 * @brief 创建句柄、打开设备、配置 GigE 包长与连续采集模式，分配 Payload 缓冲。
 * @param index 枚举列表中的设备下标。
 */
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

    MV_CC_DEVICE_INFO* info = m_deviceList.pDeviceInfo[index];
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
        }
        else {
            emit statusMessage(QString("Warning: Get Packet Size fail! nRet [%1]").arg(hexRet(packetSize)));
        }
    }

    // 连续采集：关闭触发模式
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
    }
    catch (const std::bad_alloc&) {
        emit errorOccurred(QStringLiteral("Allocate memory failed."));
        releaseDevice(false);
        return;
    }

    m_connectedIndex = index;
    emit deviceConnected(index);
    emit statusMessage(QString("Connected melt cam done!"));
}

/** @brief 启动 SDK 取流并开启 100ms 定时 grabFrame。 */
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

    m_grabbingStarted = true;
    m_loggedFirstFrame = false;
    m_hasFailedPixel = false;
    m_grabErrorTimer.invalidate();
    emit grabbingStarted();
    emit statusMessage(QStringLiteral("Start grabbing."));
    m_timer->start(100);
}

/** @brief 停止取流（保留连接）。 */
void MvCameraWorker::stopGrabbing()
{
    endGrabbing(true);
}

/** @brief 停止取流并释放设备。 */
void MvCameraWorker::disconnectDevice()
{
    releaseDevice(true);
}

/** @brief 超时获取一帧，转换后发 imageReady；取帧失败按 1s 节流报错。 */
void MvCameraWorker::grabFrame()
{
    if (m_handle == nullptr || m_payload.empty()) {
        return;
    }

    MV_FRAME_OUT_INFO_EX info;
    std::memset(&info, 0, sizeof(info));
    const int nRet = MV_CC_GetOneFrameTimeout(
        m_handle, m_payload.data(), m_payloadSize, &info, kGrabTimeoutMs);

    if (nRet == MV_OK) {
        cv::Mat image;
        try {
            if (frameToMat(&info, m_payload.data(), image)) {
                emit imageReady(image);
            }
        }
        catch (const cv::Exception& ex) {
            if (!m_hasFailedPixel) {
                m_hasFailedPixel = true;
                emit errorOccurred(QString("Convert frame fail: %1").arg(QString::fromLocal8Bit(ex.what())));
            }
        }
    }
    else if (!m_grabErrorTimer.isValid() || m_grabErrorTimer.elapsed() >= 1000) {
        emit errorOccurred(QString("Get Frame fail! nRet [%1]").arg(hexRet(nRet)));
        m_grabErrorTimer.start();
    }
}

/**
 * @brief 关闭设备并清理状态。
 * @param notify 是否向 UI 发送断开相关信号。
 */
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

/**
 * @brief 调用 MV_CC_StopGrabbing 并停止定时器。
 * @param notify 为 true 时发送 grabbingStopped。
 * @return 若此前正在取流则返回 true。
 */
bool MvCameraWorker::endGrabbing(bool notify)
{
    if (!m_grabbingStarted) {
        return false;
    }

    int nRet = MV_OK;
    if (m_handle != nullptr && m_grabbingStarted) {
        nRet = MV_CC_StopGrabbing(m_handle);
        m_grabbingStarted = false;
    }
    else {
        m_grabbingStarted = false;
    }

    m_loggedFirstFrame = false;
    m_hasFailedPixel = false;
    m_timer->stop();

    if (notify) {
        if (nRet != MV_OK) {
            emit errorOccurred(QString("Stop Grabbing fail! nRet [%1]").arg(hexRet(nRet)));
        }
        emit grabbingStopped();
    }
    return true;
}

/**
 * @brief 按像素类型克隆 Mono8/BGR/RGB，否则经 SDK 转 BGR 或灰度。
 * @param info 帧元数据。
 * @param data 帧数据。
 * @param out 输出 Mat。
 */
bool MvCameraWorker::frameToMat(const MV_FRAME_OUT_INFO_EX* info, unsigned char* data, cv::Mat& out)
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

    // Bayer、YUV 等：在关句柄前经 SDK 转为 BGR8 或 Mono8
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

/**
 * @brief 填充 MV_CC_PIXEL_CONVERT_PARAM 并克隆转换结果为 Mat。
 * @param dstType 目标 Gvsp 像素类型。
 */
bool MvCameraWorker::convertWithSdk(const MV_FRAME_OUT_INFO_EX* info, unsigned char* data,
    MvGvspPixelType dstType, cv::Mat& out)
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