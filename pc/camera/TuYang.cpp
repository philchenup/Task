/**
 * @file TuYang.cpp
 * @brief 图漾相机采集。设备枚举、开流、配准和深度转点云来自 SimpleView_Point3D。
 */
#include "TuYang.h"

#include "TYImageProc.h"

#include <pcl/common/io.h>
#include <pcl/filters/filter.h>

#include <QDebug>
#include <QFileInfo>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

#define TUYANG_PRE_FLAG "tuyang-preview"
#define TUYANG_ADD_FLAG "tuyang-pointcloud"

namespace {

void tuyangEventCallback(TY_EVENT_INFO* eventInfo, void*)
{
    if (!eventInfo)
        return;
    if (eventInfo->eventId == TY_EVENT_DEVICE_OFFLINE)
        qWarning("TuYang: device offline");
    else if (eventInfo->eventId == TY_EVENT_LICENSE_ERROR)
        qWarning("TuYang: license error");
}

int decodeCsiRaw10(unsigned char* src, uint16_t* dst, int width, int height)
{
    if (width & 0x3)
        return -1;
    const int raw10LineSize = 5 * width / 4;
    for (int i = 0, j = 0; i < raw10LineSize * height; i += 5, j += 4) {
        dst[j + 0] = (static_cast<uint16_t>(src[i + 0]) << 2) | ((src[i + 4] & 0x3) >> 0);
        dst[j + 1] = (static_cast<uint16_t>(src[i + 1]) << 2) | ((src[i + 4] & 0xc) >> 2);
        dst[j + 2] = (static_cast<uint16_t>(src[i + 2]) << 2) | ((src[i + 4] & 0x30) >> 4);
        dst[j + 3] = (static_cast<uint16_t>(src[i + 3]) << 2) | ((src[i + 4] & 0xc0) >> 6);
    }
    return 0;
}

int decodeCsiRaw12(unsigned char* src, uint16_t* dst, int width, int height)
{
    if (width & 0x1)
        return -1;
    const int raw12LineSize = 3 * width / 2;
    for (int i = 0, j = 0; i < raw12LineSize * height; i += 3, j += 2) {
        dst[j + 0] = (static_cast<uint16_t>(src[i + 0]) << 4) | ((src[i + 2] & 0x0f) >> 0);
        dst[j + 1] = (static_cast<uint16_t>(src[i + 1]) << 4) | ((src[i + 2] & 0xf0) >> 4);
    }
    return 0;
}

int parseCsiRaw10(unsigned char* src, cv::Mat& dst, int width, int height)
{
    cv::Mat raw(height, width, CV_16U);
    if (decodeCsiRaw10(src, reinterpret_cast<uint16_t*>(raw.data), width, height) != 0)
        return -1;
    dst = raw * 64;
    return 0;
}

int parseCsiRaw12(unsigned char* src, cv::Mat& dst, int width, int height)
{
    cv::Mat raw(height, width, CV_16U);
    if (decodeCsiRaw12(src, reinterpret_cast<uint16_t*>(raw.data), width, height) != 0)
        return -1;
    dst = raw * 16;
    return 0;
}

TY_IMAGE_DATA makeImage(int width, int height, int size, int format, void* buffer)
{
    TY_IMAGE_DATA image;
    std::memset(&image, 0, sizeof(image));
    image.width = width;
    image.height = height;
    image.size = size;
    image.pixelFormat = format;
    image.buffer = buffer;
    return image;
}

int bayer8Code(int pixelFormat)
{
    switch (pixelFormat) {
    case TY_PIXEL_FORMAT_BAYER8GBRG: return cv::COLOR_BayerGR2BGR;
    case TY_PIXEL_FORMAT_BAYER8BGGR: return cv::COLOR_BayerRG2BGR;
    case TY_PIXEL_FORMAT_BAYER8GRBG: return cv::COLOR_BayerGB2BGR;
    case TY_PIXEL_FORMAT_BAYER8RGGB: return cv::COLOR_BayerBG2BGR;
    default: return -1;
    }
}

int bayer10Code(int pixelFormat)
{
    switch (pixelFormat) {
    case TY_PIXEL_FORMAT_CSI_BAYER10GBRG: return cv::COLOR_BayerGR2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER10BGGR: return cv::COLOR_BayerRG2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER10GRBG: return cv::COLOR_BayerGB2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER10RGGB: return cv::COLOR_BayerBG2BGR;
    default: return -1;
    }
}

int bayer12Code(int pixelFormat)
{
    switch (pixelFormat) {
    case TY_PIXEL_FORMAT_CSI_BAYER12GBRG: return cv::COLOR_BayerGR2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER12BGGR: return cv::COLOR_BayerRG2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER12GRBG: return cv::COLOR_BayerGB2BGR;
    case TY_PIXEL_FORMAT_CSI_BAYER12RGGB: return cv::COLOR_BayerBG2BGR;
    default: return -1;
    }
}

} // namespace

TuYang::TuYang(QWidget* parent)
    : ICamera(parent),
    m_captured_cloud(new ct::Cloud)
{
}

TuYang::~TuYang()
{
    closeDevice();
    if (m_libInited) {
        TYDeinitLib();
        m_libInited = false;
    }
}

bool TuYang::ensureLib()
{
    if (m_libInited)
        return true;
    if (TYInitLib() != TY_STATUS_OK) {
        printE("Init TuYang lib failed");
        return false;
    }
    TY_VERSION_INFO ver;
    if (TYLibVersion(&ver) == TY_STATUS_OK) {
        printI(tr("TuYang lib version: %1.%2.%3").arg(ver.major).arg(ver.minor).arg(ver.patch));
    }
    m_libInited = true;
    return true;
}

int TuYang::selectDevices(const std::string& id, const std::string& ip, uint32_t deviceNum,
    std::vector<TY_DEVICE_BASE_INFO>& out)
{
    out.clear();
    if (TYUpdateInterfaceList() != TY_STATUS_OK)
        return TY_STATUS_ERROR;

    uint32_t ifaceCount = 0;
    if (TYGetInterfaceNumber(&ifaceCount) != TY_STATUS_OK || ifaceCount == 0)
        return TY_STATUS_ERROR;

    std::vector<TY_INTERFACE_INFO> ifaces(ifaceCount);
    if (TYGetInterfaceList(ifaces.data(), ifaceCount, &ifaceCount) != TY_STATUS_OK)
        return TY_STATUS_ERROR;
    ifaces.resize(ifaceCount);

    const TY_INTERFACE_TYPE kTypes[] = {
        TY_INTERFACE_USB,
        TY_INTERFACE_ETHERNET,
        TY_INTERFACE_IEEE80211
    };
    for (TY_INTERFACE_TYPE type : kTypes) {
        for (const TY_INTERFACE_INFO& iface : ifaces) {
            if (iface.type != type || out.size() >= deviceNum)
                continue;

            TY_INTERFACE_HANDLE hIface = nullptr;
            if (TYOpenInterface(iface.id, &hIface) != TY_STATUS_OK)
                continue;

            if (TYUpdateDeviceList(hIface) == TY_STATUS_OK) {
                uint32_t devCount = 0;
                TYGetDeviceNumber(hIface, &devCount);
                if (devCount > 0) {
                    std::vector<TY_DEVICE_BASE_INFO> devs(devCount);
                    if (TYGetDeviceList(hIface, devs.data(), devCount, &devCount) == TY_STATUS_OK) {
                        for (uint32_t j = 0; j < devCount && out.size() < deviceNum; ++j) {
                            const bool idMatch = !id.empty() && id == devs[j].id;
                            const bool network = devs[j].iface.type == TY_INTERFACE_ETHERNET
                                || devs[j].iface.type == TY_INTERFACE_IEEE80211;
                            const bool ipMatch = !ip.empty() && network && ip == devs[j].netInfo.ip;
                            if ((id.empty() && ip.empty()) || idMatch || ipMatch)
                                out.push_back(devs[j]);
                        }
                    }
                }
            }
            TYCloseInterface(hIface);
        }
    }
    return TY_STATUS_OK;
}

void TuYang::searchDevice()
{
    if (!ensureLib())
        return;

    if (selectDevices("", "", 128, deviceInfoList) != TY_STATUS_OK || deviceInfoList.empty()) {
        printE("TuYang has found 0 devices");
        return;
    }

    QVector<QString> camModel;
    for (const auto& device : deviceInfoList) {
        const char* model = device.modelName[0] ? device.modelName : device.id;
        camModel.push_back(model);
    }
    emit sendCamModel(camModel);
    this->updateCamInfo(0);
}

void TuYang::connectDevice(std::string id, std::string ip)
{
    if (id.empty() && ip.empty()) {
        printW("Device id and ip are empty");
        return;
    }
    if (!ensureLib())
        return;

    if (connected) {
        closeDevice();
        emit sendCamConnectStatus(false);
    }

    std::vector<TY_DEVICE_BASE_INFO> selected;
    if (selectDevices(id, ip, 1, selected) != TY_STATUS_OK || selected.empty()) {
        printE(tr("Connect to the device failed. id:%1 ip:%2")
            .arg(QString::fromStdString(id), QString::fromStdString(ip)));
        return;
    }

    deviceInfoList = selected;
    QVector<QString> camModel;
    camModel.push_back(selected[0].modelName[0] ? selected[0].modelName : selected[0].id);
    emit sendCamModel(camModel);
    updateCamInfo(0);

    if (!openDevice(selected[0])) {
        printE(tr("Connect to the device: %1 failed.").arg(selected[0].modelName));
        connected = false;
        emit sendCamConnectStatus(connected);
        return;
    }

    printI(tr("Connect to the device: %1 successfully.").arg(selected[0].modelName));
    connected = true;
    emit sendCamConnectStatus(connected);
}

void TuYang::connectDevice(int index)
{
    if (deviceInfoList.empty()) {
        printW("Device list is empty");
        return;
    }
    if (index < 0 || index >= static_cast<int>(deviceInfoList.size())) {
        printW("Selected device is invalid.");
        return;
    }

    if (connected) {
        closeDevice();
        printI(tr("Disconnect the device: %1 successfully.").arg(deviceInfoList[index].modelName));
        emit sendCamConnectStatus(connected);
        return;
    }

    if (!openDevice(deviceInfoList[index])) {
        printE(tr("Connect to the device: %1 failed.").arg(deviceInfoList[index].modelName));
        connected = false;
        emit sendCamConnectStatus(connected);
        return;
    }

    printI(tr("Connect to the device: %1 successfully.").arg(deviceInfoList[index].modelName));
    connected = true;
    emit sendCamConnectStatus(connected);
}

bool TuYang::openDevice(const TY_DEVICE_BASE_INFO& info)
{
    if (!ensureLib())
        return false;

    if (TYOpenInterface(info.iface.id, &m_hIface) != TY_STATUS_OK) {
        printE("Open interface failed");
        m_hIface = nullptr;
        return false;
    }
    if (TYOpenDevice(m_hIface, info.id, &m_hDevice) != TY_STATUS_OK) {
        printE("Open device failed");
        closeDevice();
        return false;
    }
    if (!configureComponents() || !prepareBuffers()) {
        closeDevice();
        return false;
    }

    std::memset(&m_depthCalib, 0, sizeof(m_depthCalib));
    if (TYGetStruct(m_hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA,
            &m_depthCalib, sizeof(m_depthCalib)) != TY_STATUS_OK) {
        printE("Read depth intrinsic failed");
        closeDevice();
        return false;
    }

    m_isDepthDistortion = false;
    TYHasFeature(m_hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_DISTORTION, &m_isDepthDistortion);

    if (TYRegisterEventCallback(m_hDevice, tuyangEventCallback, nullptr) != TY_STATUS_OK)
        printW("Register event callback failed");

    bool hasTrigger = false;
    if (TYHasFeature(m_hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX, &hasTrigger) == TY_STATUS_OK
        && hasTrigger) {
        TY_TRIGGER_PARAM_EX trigger;
        std::memset(&trigger, 0, sizeof(trigger));
        trigger.mode = TY_TRIGGER_MODE_OFF;
        if (TYSetStruct(m_hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM_EX,
                &trigger, sizeof(trigger)) != TY_STATUS_OK) {
            printW("Disable trigger mode failed");
        }
    }

    if (TYStartCapture(m_hDevice) != TY_STATUS_OK) {
        printE("Start capture failed");
        closeDevice();
        return false;
    }
    return true;
}

void TuYang::closeDevice()
{
    if (m_hDevice) {
        TYStopCapture(m_hDevice);
        TYCloseDevice(m_hDevice);
        m_hDevice = nullptr;
    }
    if (m_hIface) {
        TYCloseInterface(m_hIface);
        m_hIface = nullptr;
    }
    for (char* buffer : m_frameBuffers)
        delete[] buffer;
    m_frameBuffers.clear();
    m_frameSize = 0;
    if (m_ispHandle) {
        TYISPRelease(&m_ispHandle);
        m_ispHandle = nullptr;
    }
    m_hasColorCalib = false;
    m_isDepthDistortion = false;
    connected = false;
}

bool TuYang::getDefaultImageMode(TY_COMPONENT_ID component, TY_IMAGE_MODE& imageMode)
{
    uint32_t count = 0;
    if (TYGetEnumEntryCount(m_hDevice, component, TY_ENUM_IMAGE_MODE, &count) != TY_STATUS_OK || count == 0)
        return false;

    std::vector<TY_ENUM_ENTRY> entries(count);
    uint32_t filled = 0;
    if (TYGetEnumEntryInfo(m_hDevice, component, TY_ENUM_IMAGE_MODE,
            entries.data(), count, &filled) != TY_STATUS_OK || filled == 0) {
        return false;
    }
    imageMode = entries[0].value;
    return true;
}

void TuYang::colorIspInitSetting()
{
    if (!m_ispHandle || !m_hDevice)
        return;

    TY_DEVICE_BASE_INFO info;
    std::memset(&info, 0, sizeof(info));
    const bool hasInfo = TYGetDeviceInfo(m_hDevice, &info) == TY_STATUS_OK;
    bool oldColorDevice = false;
    if (hasInfo) {
        oldColorDevice = info.iface.type == TY_INTERFACE_USB;
        const TY_VERSION_INFO& ver = info.firmwareVersion;
        const bool olderThan22 = ver.major < 2 || (ver.major == 2 && ver.minor < 2);
        if ((info.iface.type == TY_INTERFACE_ETHERNET || info.iface.type == TY_INTERFACE_RAW) && olderThan22)
            oldColorDevice = true;
    }

    if (oldColorDevice) {
        TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_BLACK_LEVEL, 11);
        TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 256.f / (256 - 11));
    } else {
        TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_BLACK_LEVEL, 0);
        TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_BLACK_LEVEL_GAIN, 1.f);
        bool hasAnalogGain = false;
        if (TYHasFeature(m_hDevice, TY_COMPONENT_RGB_CAM, TY_INT_ANALOG_GAIN, &hasAnalogGain) == TY_STATUS_OK
            && hasAnalogGain) {
            TYSetInt(m_hDevice, TY_COMPONENT_RGB_CAM, TY_INT_ANALOG_GAIN, 1);
        }
    }

    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_BAYER_PATTERN, TY_ISP_BAYER_AUTO);
    float shading[9] = {
        0.30890417098999026f, 10.63355541229248f, -6.433426856994629f,
        0.24413758516311646f, 11.739893913269043f, -8.148622512817383f,
        0.1255662441253662f, 11.88359546661377f, -7.865192413330078f
    };
    int shadingCenter[2] = { 640, 480 };
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_SHADING, reinterpret_cast<uint8_t*>(shading), sizeof(shading));
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_SHADING_CENTER,
        reinterpret_cast<uint8_t*>(shadingCenter), sizeof(shadingCenter));
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_CCM_ENABLE, 0);
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_CAM_DEV_HANDLE,
        reinterpret_cast<uint8_t*>(&m_hDevice), sizeof(m_hDevice));
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_CAM_DEV_COMPONENT, static_cast<int32_t>(TY_COMPONENT_RGB_CAM));
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_GAMMA, 1.f);
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_AUTOBRIGHT, 1);
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_ENABLE_AUTO_EXPOSURE_GAIN, 0);

    int defaultSize[2] = { 1280, 960 };
    int currentSize[2] = { 1280, 960 };
    TY_IMAGE_MODE imageMode;
    if (TYGetEnum(m_hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, &imageMode) == TY_STATUS_OK) {
        currentSize[0] = TYImageWidth(imageMode);
        currentSize[1] = TYImageHeight(imageMode);
    }
    TY_ENUM_ENTRY modeEntry[10];
    uint32_t modeCount = 0;
    if (TYGetEnumEntryInfo(m_hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE,
            modeEntry, 10, &modeCount) == TY_STATUS_OK && modeCount > 0) {
        defaultSize[0] = TYImageWidth(modeEntry[0].value);
        defaultSize[1] = TYImageHeight(modeEntry[0].value);
    }
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_IMAGE_SIZE,
        reinterpret_cast<uint8_t*>(defaultSize), sizeof(defaultSize));
    int resample = 1;
    if (currentSize[0] > 0)
        resample = defaultSize[0] / currentSize[0];
    if (resample <= 0)
        resample = 1;
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_INPUT_RESAMPLE_SCALE, resample);
    TYISPSetFeature(m_ispHandle, TY_ISP_FEATURE_ENABLE_AUTO_WHITEBALANCE, 1);

    TY_COMPONENT_ID components = 0;
    if (TYGetComponentIDs(m_hDevice, &components) != TY_STATUS_OK || !(components & TY_COMPONENT_STORAGE))
        return;
    bool hasIspBlock = false;
    if (TYHasFeature(m_hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_ISP_BLOCK, &hasIspBlock) != TY_STATUS_OK
        || !hasIspBlock) {
        return;
    }
    uint32_t blockSize = 0;
    if (TYGetByteArraySize(m_hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_ISP_BLOCK, &blockSize) != TY_STATUS_OK
        || blockSize == 0) {
        return;
    }
    std::vector<uint8_t> block(blockSize);
    if (TYGetByteArray(m_hDevice, TY_COMPONENT_STORAGE, TY_BYTEARRAY_ISP_BLOCK,
            block.data(), static_cast<uint32_t>(block.size())) != TY_STATUS_OK) {
        return;
    }
    if (TYISPLoadConfig(m_ispHandle, block.data(), static_cast<uint32_t>(block.size())) == TY_STATUS_OK)
        printI("Load RGB ISP Config From Device");
}

bool TuYang::configureComponents()
{
    if (TYEnableComponents(m_hDevice, TY_COMPONENT_DEPTH_CAM) != TY_STATUS_OK) {
        printE("Enable depth camera failed");
        return false;
    }

    TY_IMAGE_MODE imageMode;
    if (!getDefaultImageMode(TY_COMPONENT_DEPTH_CAM, imageMode)) {
        printE("Get depth image mode failed");
        return false;
    }
    printI(tr("Select Depth Image Mode: %1x%2")
        .arg(TYImageWidth(imageMode))
        .arg(TYImageHeight(imageMode)));
    if (TYSetEnum(m_hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, imageMode) != TY_STATUS_OK) {
        printE("Set depth image mode failed");
        return false;
    }
    if (TYEnableComponents(m_hDevice, TY_COMPONENT_DEPTH_CAM) != TY_STATUS_OK) {
        printE("Enable depth camera failed");
        return false;
    }

    m_depthScale = 1.f;
    bool hasScaleUnit = false;
    if (TYHasFeature(m_hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &hasScaleUnit) == TY_STATUS_OK
        && hasScaleUnit) {
        TYGetFloat(m_hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &m_depthScale);
    }

    TY_COMPONENT_ID allComps = 0;
    m_hasColorCalib = false;
    if (TYGetComponentIDs(m_hDevice, &allComps) != TY_STATUS_OK)
        return true;

    if ((allComps & TY_COMPONENT_RGB_CAM) && m_withColorCam) {
        if (TYEnableComponents(m_hDevice, TY_COMPONENT_RGB_CAM) != TY_STATUS_OK) {
            printW("Open RGB camera failed");
            return true;
        }
        bool hasColorCalib = false;
        if (TYHasFeature(m_hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &hasColorCalib) == TY_STATUS_OK
            && hasColorCalib) {
            std::memset(&m_colorCalib, 0, sizeof(m_colorCalib));
            if (TYGetStruct(m_hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA,
                    &m_colorCalib, sizeof(m_colorCalib)) == TY_STATUS_OK) {
                m_hasColorCalib = true;
            }
        }
        if (TYISPCreate(&m_ispHandle) == TY_STATUS_OK) {
            colorIspInitSetting();
        } else {
            m_ispHandle = nullptr;
            printW("Create color ISP failed");
        }
    }
    return true;
}

bool TuYang::prepareBuffers()
{
    uint32_t frameSize = 0;
    if (TYGetFrameBufferSize(m_hDevice, &frameSize) != TY_STATUS_OK || frameSize == 0) {
        printE("Get frame buffer size failed");
        return false;
    }
    m_frameSize = frameSize;
    for (int i = 0; i < 2; ++i) {
        char* buffer = new (std::nothrow) char[frameSize];
        if (!buffer) {
            printE("Allocate frame buffer failed");
            return false;
        }
        m_frameBuffers.push_back(buffer);
        if (TYEnqueueBuffer(m_hDevice, buffer, frameSize) != TY_STATUS_OK) {
            printE("Enqueue frame buffer failed");
            return false;
        }
    }
    return true;
}

void TuYang::reset()
{
    for (int i = 0; i < 24; ++i)
        m_cloudview->removeText3D("text" + QString::number(i));
    m_cloudview->removeAllCoordinateSystems();
    m_cloudview->removePointCloud(TUYANG_PRE_FLAG);
    m_captured_cloud->clear();
}

void TuYang::parseFrame(const TY_FRAME_DATA& frame, cv::Mat& depth, cv::Mat& color)
{
    depth.release();
    color.release();
    const int imageCount = std::min<int>(frame.validCount,
        static_cast<int>(sizeof(frame.image) / sizeof(frame.image[0])));
    for (int i = 0; i < imageCount; ++i) {
        const TY_IMAGE_DATA& image = frame.image[i];
        if (image.status != TY_STATUS_OK)
            continue;
        if (image.componentID == TY_COMPONENT_DEPTH_CAM && image.pixelFormat == TY_PIXEL_FORMAT_DEPTH16) {
            depth = cv::Mat(image.height, image.width, CV_16U, image.buffer).clone();
        } else if (image.componentID == TY_COMPONENT_RGB_CAM) {
            parseColor(image, color);
        }
    }
}

bool TuYang::parseColor(const TY_IMAGE_DATA& image, cv::Mat& color)
{
    if (!image.buffer || image.size <= 0)
        return false;

    const int format = image.pixelFormat;
    if (format == TY_PIXEL_FORMAT_JPEG) {
        std::vector<uchar> bytes(static_cast<uchar*>(image.buffer),
            static_cast<uchar*>(image.buffer) + image.size);
        color = cv::imdecode(bytes, cv::IMREAD_COLOR);
        return !color.empty();
    }
    if (format == TY_PIXEL_FORMAT_YVYU) {
        cv::Mat yuv(image.height, image.width, CV_8UC2, image.buffer);
        cv::cvtColor(yuv, color, cv::COLOR_YUV2BGR_YVYU);
        return true;
    }
    if (format == TY_PIXEL_FORMAT_YUYV) {
        cv::Mat yuv(image.height, image.width, CV_8UC2, image.buffer);
        cv::cvtColor(yuv, color, cv::COLOR_YUV2BGR_YUYV);
        return true;
    }
    if (format == TY_PIXEL_FORMAT_RGB) {
        cv::Mat rgb(image.height, image.width, CV_8UC3, image.buffer);
        cv::cvtColor(rgb, color, cv::COLOR_RGB2BGR);
        return true;
    }
    if (format == TY_PIXEL_FORMAT_BGR) {
        color = cv::Mat(image.height, image.width, CV_8UC3, image.buffer).clone();
        return true;
    }
    if (format == TY_PIXEL_FORMAT_MONO) {
        cv::Mat gray(image.height, image.width, CV_8U, image.buffer);
        cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
        return true;
    }
    if (format == TY_PIXEL_FORMAT_MONO16 || format == TY_PIXEL_FORMAT_TOF_IR_MONO16) {
        color = cv::Mat(image.height, image.width, CV_16U, image.buffer).clone();
        return true;
    }
    if (format == TY_PIXEL_FORMAT_CSI_MONO10) {
        return parseCsiRaw10(static_cast<uchar*>(image.buffer), color, image.width, image.height) == 0;
    }
    if (format == TY_PIXEL_FORMAT_CSI_MONO12) {
        return parseCsiRaw12(static_cast<uchar*>(image.buffer), color, image.width, image.height) == 0;
    }

    const int code8 = bayer8Code(format);
    if (code8 >= 0) {
        if (m_ispHandle) {
            color.create(image.height, image.width, CV_8UC3);
            const int bytes = image.height * image.width * 3;
            TY_IMAGE_DATA out = TYInitImageData(bytes, color.data, image.width, image.height);
            out.pixelFormat = TY_PIXEL_FORMAT_BGR;
            if (TYISPProcessImage(m_ispHandle, &image, &out) == TY_STATUS_OK)
                return true;
        }
        cv::Mat raw(image.height, image.width, CV_8U, image.buffer);
        cv::cvtColor(raw, color, code8);
        return true;
    }

    const int code10 = bayer10Code(format);
    if (code10 >= 0) {
        cv::Mat raw16;
        if (parseCsiRaw10(static_cast<uchar*>(image.buffer), raw16, image.width, image.height) != 0)
            return false;
        cv::cvtColor(raw16, color, code10);
        return true;
    }
    const int code12 = bayer12Code(format);
    if (code12 >= 0) {
        cv::Mat raw16;
        if (parseCsiRaw12(static_cast<uchar*>(image.buffer), raw16, image.width, image.height) != 0)
            return false;
        cv::cvtColor(raw16, color, code12);
        return true;
    }

    printW(tr("Unsupported color pixel format: %1").arg(format));
    return false;
}

void TuYang::normalizeColor(cv::Mat& color)
{
    cv::Mat tmp;
    switch (color.type()) {
    case CV_16U:
        color.convertTo(tmp, CV_8U, 1.f / 256);
        cv::cvtColor(tmp, color, cv::COLOR_GRAY2BGR);
        break;
    case CV_16UC3:
        color.convertTo(tmp, CV_8UC3, 1.f / 256);
        color = tmp;
        break;
    default:
        break;
    }
}

bool TuYang::undistortDepth(cv::Mat& depth)
{
    if (depth.empty() || depth.type() != CV_16U)
        return false;

    cv::Mat undistorted(depth.size(), CV_16U);
    TY_IMAGE_DATA src = makeImage(depth.cols, depth.rows,
        static_cast<int>(depth.total() * 2), TY_PIXEL_FORMAT_DEPTH16, depth.data);
    TY_IMAGE_DATA dst = makeImage(undistorted.cols, undistorted.rows,
        static_cast<int>(undistorted.total() * 2), TY_PIXEL_FORMAT_DEPTH16, undistorted.data);
    if (TYUndistortImage(&m_depthCalib, &src, nullptr, &dst) != TY_STATUS_OK)
        return false;
    depth = undistorted;
    return true;
}

bool TuYang::doRegister(const cv::Mat& depth, const cv::Mat& color, cv::Mat& undistortColor, cv::Mat& out)
{
    if (depth.empty() || color.empty() || color.type() != CV_8UC3 || color.cols == 0)
        return false;

    undistortColor = cv::Mat(color.size(), CV_8UC3);
    TY_IMAGE_DATA src = makeImage(color.cols, color.rows,
        static_cast<int>(color.total() * 3), TY_PIXEL_FORMAT_RGB, color.data);
    TY_IMAGE_DATA dst = makeImage(undistortColor.cols, undistortColor.rows,
        static_cast<int>(undistortColor.total() * 3), TY_PIXEL_FORMAT_RGB, undistortColor.data);
    if (TYUndistortImage(&m_colorCalib, &src, nullptr, &dst) != TY_STATUS_OK)
        return false;

    if (m_mapDepthToColor) {
        const int outW = depth.cols;
        const int outH = depth.cols * undistortColor.rows / undistortColor.cols;
        out = cv::Mat::zeros(cv::Size(outW, outH), CV_16U);
        if (TYMapDepthImageToColorCoordinate(
                &m_depthCalib,
                depth.cols, depth.rows, depth.ptr<uint16_t>(),
                &m_colorCalib,
                out.cols, out.rows, out.ptr<uint16_t>(), m_depthScale) != TY_STATUS_OK) {
            return false;
        }
        cv::Mat resized;
        cv::resize(out, resized, undistortColor.size(), 0, 0, cv::INTER_NEAREST);
        out = resized;
    } else {
        out = cv::Mat::zeros(depth.size(), CV_8UC3);
        if (TYMapRGBImageToDepthCoordinate(
                &m_depthCalib,
                depth.cols, depth.rows, depth.ptr<uint16_t>(),
                &m_colorCalib,
                undistortColor.cols, undistortColor.rows, undistortColor.ptr<uint8_t>(),
                out.ptr<uint8_t>(), m_depthScale) != TY_STATUS_OK) {
            return false;
        }
        undistortColor = out.clone();
    }
    return true;
}

bool TuYang::handleFrame(const TY_FRAME_DATA& frame, float time)
{
    cv::Mat depth;
    cv::Mat color;
    parseFrame(frame, depth, color);
    if (depth.empty()) {
        printE("Capture depth image failed.");
        return false;
    }

    if (m_isDepthDistortion && !undistortDepth(depth))
        printW("Undistort depth image failed");

    cv::Mat colorRgb;
    const TY_CAMERA_CALIB_INFO* calib = &m_depthCalib;
    if (!color.empty())
        normalizeColor(color);
    if (!color.empty() && m_hasColorCalib) {
        cv::Mat undistortColor;
        cv::Mat mapped;
        if (!doRegister(depth, color, undistortColor, mapped)) {
            printW("Register depth and color failed");
        } else {
            cv::cvtColor(undistortColor, colorRgb, cv::COLOR_BGR2RGB);
            if (m_mapDepthToColor && !mapped.empty()) {
                depth = mapped;
                calib = &m_colorCalib;
            }
        }
    }

    if (depth.empty() || depth.type() != CV_16U) {
        printE("Capture depth image failed.");
        return false;
    }

    std::vector<TY_VECT_3F> points(static_cast<size_t>(depth.total()));
    if (TYMapDepthImageToPoint3d(calib, depth.cols, depth.rows,
            depth.ptr<uint16_t>(), points.data(), m_depthScale) != TY_STATUS_OK) {
        printE("Map depth to point cloud failed");
        return false;
    }

    depth.convertTo(tuYangCamData.depth_Image, CV_32FC1, m_depthScale);
    if (!colorRgb.empty())
        cv::cvtColor(colorRgb, tuYangCamData.color_Image, cv::COLOR_RGB2BGR);
    else if (!color.empty())
        tuYangCamData.color_Image = color.clone();
    else
        tuYangCamData.color_Image.release();

    updatePointCloud(points, colorRgb, time);
    return true;
}

void TuYang::captureDevice()
{
    reset();
    if (!connected || !m_hDevice) {
        printW("Please connect a device first!");
        return;
    }

    ct::TicToc ticToc;
    ticToc.tic();

    TY_FRAME_DATA frame;
    std::memset(&frame, 0, sizeof(frame));
    if (TYFetchFrame(m_hDevice, &frame, -1) != TY_STATUS_OK) {
        printE("Capture frame failed.");
        return;
    }

    const float elapsed = ticToc.toc();
    handleFrame(frame, elapsed);
    TYEnqueueBuffer(m_hDevice, frame.userBuffer, frame.bufferSize);
    if (m_ispHandle)
        TYISPUpdateDevice(m_ispHandle);
}

void TuYang::add()
{
    if (m_captured_cloud->empty()) {
        printW("Please capture a cloud first!");
        return;
    }
    m_cloudview->removePointCloud(TUYANG_PRE_FLAG);
    ct::Cloud::Ptr newCloud = m_captured_cloud->makeShared();
    newCloud->setId(TUYANG_ADD_FLAG);
    newCloud->setInfo(QFileInfo(QString(DATA_PATH) + QString(TUYANG_ADD_FLAG)));
    newCloud->update(false);
    m_cloudtree->appendCloud(newCloud, true);
    printI(QString("Add captured cloud[id:%1] done.").arg(newCloud->id()));
    m_captured_cloud->clear();
}

void TuYang::updateCamInfo(int index)
{
    if (index < 0)
        return;
    if (deviceInfoList.empty()) {
        printW("Device List is empty!");
        return;
    }
    if (index >= static_cast<int>(deviceInfoList.size())) {
        printW("Device is not exist, please search device again!");
        return;
    }

    const TY_DEVICE_BASE_INFO& info = deviceInfoList[index];
    QString ip = "-";
    if (info.iface.type == TY_INTERFACE_ETHERNET || info.iface.type == TY_INTERFACE_IEEE80211)
        ip = info.netInfo.ip;

    const QString camInfo = tr("Model:\n%1\n").arg(info.modelName)
        + tr("serialNumber:\n%1\n").arg(info.id)
        + tr("vendor:\n%1\n").arg(info.vendorName)
        + tr("ipAddress:\n%1\n").arg(ip)
        + tr("interface:\n%1\n").arg(info.iface.id);
    emit sendCamInfo(camInfo);
}

bool TuYang::containsInvalidPoint(const std::vector<TY_VECT_3F>& cloud)
{
    return std::any_of(cloud.begin(), cloud.end(), [](const TY_VECT_3F& point) {
        return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)
            || std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
    });
}

void TuYang::convertToPCL(const std::vector<TY_VECT_3F>& points, const cv::Mat& color,
    pcl::PointCloud<pcl::PointXYZRGB>& pclCloud)
{
    pclCloud.clear();
    pclCloud.resize(points.size());
    const bool hasColor = !color.empty() && color.type() == CV_8UC3
        && static_cast<size_t>(color.total()) == points.size();
    const cv::Vec3b* colors = hasColor ? color.ptr<cv::Vec3b>() : nullptr;

    for (size_t i = 0; i < points.size(); ++i) {
        pcl::PointXYZRGB& point = pclCloud[i];
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = points[i].z;
        if (colors) {
            point.r = colors[i][0];
            point.g = colors[i][1];
            point.b = colors[i][2];
        } else {
            point.r = point.g = point.b = 255;
        }
    }
    pclCloud.is_dense = !containsInvalidPoint(points);
}

void TuYang::updatePointCloud(const std::vector<TY_VECT_3F>& points, const cv::Mat& color, float time)
{
    pcl::PointCloud<pcl::PointXYZRGB> textured;
    convertToPCL(points, color, textured);

    std::vector<int> mapping;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr withoutNan(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::removeNaNFromPointCloud(textured, *withoutNan, mapping);
    pcl::copyPointCloud(*withoutNan, *m_captured_cloud);

    // Camera frame, in millimeters. The sample flips Y/Z only for its own GL viewer.
    m_captured_cloud->setId(TUYANG_PRE_FLAG);
    printI(tr("Number of points in PCL Cloud : %1").arg(m_captured_cloud->size()));
    printI(tr("Capture cloud[id:%1] done, take time %2 ms.").arg(m_captured_cloud->id()).arg(time));
    if (!m_captured_cloud->empty()) {
        m_cloudview->addPointCloud(m_captured_cloud);
        m_cloudview->resetCamera();
    }

    tuYangCamData.cloud = m_captured_cloud->makeShared();
    emit sendCamCloud(tuYangCamData, QString::fromStdString(TUYANG_PRE_FLAG));
}
