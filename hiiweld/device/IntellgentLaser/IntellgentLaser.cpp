/**
 * @file IntellgentLaser.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "IntellgentLaser.h"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <QDateTime>

/**
 * @brief ?? PF400 ?????? SDK?
 * @param parent Qt ????
 */
ILaser_PF400::ILaser_PF400(QObject* parent)
    : ILaser(parent), 
    connected(false)
{
    Laser_Init();
}

/** @brief ???????????? SDK? */
ILaser_PF400::~ILaser_PF400()
{
    DisConnSensor();
    connected = false;

    UnInitSDK();
}

/**
 * @brief ?? InitSDK ?????????? YUV ???
 * @return ?????? true?
 */
bool ILaser_PF400::Laser_Init() {
    int32_t dwRes = InitSDK();
    if (dwRes != ITL_SUCCESS)
    {
        emit sendMsg(1, "Laser sdk init failed, please check the sdk version!");
        return false;
    }

    IntelligentRegisterCallBack_MillionOutlineInspect(sdkCallbackMMProfile, this);
    IntelligentRegisterCallBack_CaptureCameraYUVData(sdkCallbackScatterBitmap1, this);
    IntelligentRegisterCallBack_MonitorCameraYUVData(sdkCallbackScatterBitmap2, this);

    return true;
}

/**
 * @brief ??????????? IPv4 ? ConnectToSensor???????? sendConStatus?
 * @param camIp ??? IP?
 */
void ILaser_PF400::connectDevice(const std::string& camIp) {
    
    if (connected) {
        DisConnSensor();
        connected = false;
    }
    else {
        QStringList strList = QString::fromStdString(camIp).split(".");

        int dwSize = strList.size();
        if (dwSize != 4) {
            emit sendMsg(2, "Input laser cam ip format is Wrong!");
            return;
        }

        int32_t ret = ConnectToSensor(camIp.c_str());

        QString strText;
        if ((ret & EM_INTELLIGENT_CONNECT_E_MASK) == 0)
        {
            strText += "Port: 4861 Connect Success!   ";
        }
        if ((ret & EM_INTELLIGENT_CONNECT_E_CORE) == 0)
        {
            strText += "\nPort: 4276 Connect Success!   ";
        }
        if ((ret & EM_INTELLIGENT_CONNECT_E_LOGS) == 0)
        {
            strText += "\nPort: 6708 Connect Success!   ";
        }
        if ((ret & EM_INTELLIGENT_CONNECT_E_CAPTURE) == 0)
        {
            strText += "\nPort: 7868 Connect Success!   ";
        }
        if ((ret & EM_INTELLIGENT_CONNECT_E_MONITOR) == 0)
        {
            strText += "\nPort: 9658 Connect Success!   ";
        }
        if ((ret & EM_INTELLIGENT_CONNECT_E_FILES) == 0)
        {
            strText += "\nPort: 7126 Connect Success!   ";
        }

        if (strText.length() == 0)
        {
            emit sendMsg(2, "Connect ILaser Failed!");
            connected = false;
        }
        else {
            emit sendMsg(0, strText);
            connected = true;
        }
    }

    emit sendConStatus(connected);

    return;
}

/** @brief DetectAllSensor ??? FetchAllSensorInfo???????? IP? */
void ILaser_PF400::searchDevice()
{
    DetectAllSensor();

    for (int attempt = 0; attempt < 6; ++attempt) {
        QThread::msleep(1000);

        std::string strNetCard;
        std::list<IntelligentSensorMsg> lst;
        (void)FetchAllSensorInfo(strNetCard, lst);

        if (lst.empty()) {
            continue;
        }

        for (const auto& stDev : lst) {
            const IntelligentSensorMsg& msg = stDev;
            emit sendLaserIp(QString(reinterpret_cast<const char*>(msg.szIP)));
        }
        return;
    }

    emit sendMsg(1, "Not found valid laser device, please check the internet!");
}

/**
 * @brief ?? m_capture ??? UI ?????
 * @param bEnable true ?????????false ???
 */
void ILaser_PF400::captureDevice(const bool& bEnable)
{
    if (!connected) {
        emit sendMsg(1, "Laser Device not connected!");
        return;
    }
    m_capture = bEnable;

    if (m_capture) {
        emit sendMsg(0, "Start Capture data now!");
    }
    else {
        emit sendMsg(1, "Stop Capture data done!");
    }
    emit sendCaptureStatus(m_capture);
}

/**
 * @brief ?? SetLaserEnable ????????
 * @param bEnable true ???false ???
 */
void ILaser_PF400::openLaser(const bool& bEnable) {
    if (!connected) {
        emit sendMsg(1, "Laser not connecting now!");
        return;
    }
    int32_t ret = ::SetLaserEnable(bEnable);
    if (ret >= 0) {
        if(bEnable) emit sendMsg(0, "Open Laser done!");
        else emit sendMsg(0, "Stop Laser done!");
        emit sendLaserStatus(bEnable);
    }
    else {
        emit sendMsg(2, "Open Laser failed!");
    }
}

/**
 * @brief ??????????????????
 * @param bEnable true ?????false ?????
 */
void ILaser_PF400::openSensor(const bool& bEnable) {
    if (!connected) {
        emit sendMsg(1, "Camera not connected now!");
        return;
    }
    int32_t ret = ::SetSensorEnable(bEnable);
    if (ret >= 0) {
        if (bEnable) emit sendMsg(0, "Open Sensor done!");
        else emit sendMsg(0, "Stop Sensor done!");
    }
    else {
        emit sendMsg(2, "Open Sensor failed!");
        return;
    }
    ret = SetSensorCaptureCameraEnable(bEnable);
    if (ret >= 0) {
        if(bEnable) emit sendMsg(0, "Open Capture Camera done!");
        else emit sendMsg(0, "Stop Capture Camera done!");
    }
    else {
        emit sendMsg(2, "Open Capture Camera failed!");
        return;
    }
    ret = SetSensorMonitorCameraEnable(bEnable);
    if (ret >= 0) {
        if (bEnable) emit sendMsg(0, "Open Monitor Camera done!");
        else emit sendMsg(0, "Stop Monitor Camera done!");
    }
    else {
        emit sendMsg(2, "Open Monitor Camera failed!");
        return;
    }
    emit sendSensorStatus(bEnable);
}

/**
 * @brief ???? SDK ???50ms ?????????????????
 * @param pMMContourData ?????
 * @param pUser ILaser_PF400*?
 * @param dwInstanceId ?? ID??????
 */
void ILaser_PF400::sdkCallbackMMProfile(IntelligentMillionOutlineInspect* pMMContourData, void* pUser, uint32_t dwInstanceId)
{
    ILaser_PF400* pSdkObj = (ILaser_PF400*)pUser;
    if (!pSdkObj->m_capture) return;

    if (pMMContourData->contourData.contourPointCount == 0)
        return;

    uint64_t lldCurTime = QDateTime::currentMSecsSinceEpoch();

    static uint64_t lldOrgTime = 0;
    if (lldCurTime - lldOrgTime < 50)
        return;

    lldOrgTime = lldCurTime;

    // if (pMMContourData[0].InspectResult->parseRes != 1) return;

    ct::Cloud::Ptr cloud_contour(new ct::Cloud);
    cloud_contour->resize(INTELLIGENT_MAX_PIXEL_NUM);
    for (int i = 0; i < INTELLIGENT_MAX_PIXEL_NUM; ++i) {
        cloud_contour->points[i].x = 0;
        cloud_contour->points[i].y = pMMContourData->contourData.pointArray[i].y_pos;
        cloud_contour->points[i].z = pMMContourData->contourData.pointArray[i].z_pos;
        cloud_contour->points[i].r = 255; 
        cloud_contour->points[i].g = 255; 
        cloud_contour->points[i].b = 255;
    }
    pcl::PointXYZRGBNormal p;
    p.x = 0.0;
    p.y = pMMContourData[0].InspectResult->currentFeaturePoint.y_pos;
    p.z = pMMContourData[0].InspectResult->currentFeaturePoint.z_pos;
    p.r = 0; p.g = 255; p.b = 0;
    cloud_contour->push_back(p);

    emit pSdkObj->send3DContour(cloud_contour);
}

/**
 * @brief ???? YUV ??????? BGR ? sendCaptureImage?
 * @param pBitmapData YUV ??
 * @param pUser ILaser_PF400*?
 * @param dwInstanceId ?? ID?
 */
void ILaser_PF400::sdkCallbackScatterBitmap1(IntelligentYUV* pBitmapData, void* pUser, uint32_t dwInstanceId)
{
    ILaser_PF400* pSdkObj = (ILaser_PF400*)pUser;
    if (!pSdkObj->m_capture) return;

    uint64_t lldCurTime = QDateTime::currentMSecsSinceEpoch();

    static uint64_t lldOrgTime = 0;
    if (lldCurTime - lldOrgTime < 50)
        return;

    lldOrgTime = lldCurTime;

    cv::Mat bgr = pSdkObj->convertYUV420PtoMat(pBitmapData);
    if (bgr.empty()) return;

    emit pSdkObj->sendCaptureImage(bgr);
}

/**
 * @brief ???? YUV ??????? BGR ? sendMonitorImage?
 * @param pBitmapData YUV ??
 * @param pUser ILaser_PF400*?
 * @param dwInstanceId ?? ID?
 */
void ILaser_PF400::sdkCallbackScatterBitmap2(IntelligentYUV* pBitmapData, void* pUser, uint32_t dwInstanceId)
{
    ILaser_PF400* pSdkObj = (ILaser_PF400*)pUser;
    if (!pSdkObj->m_capture) return;

    uint64_t lldCurTime = QDateTime::currentMSecsSinceEpoch();

    static uint64_t lldOrgTime = 0;
    if (lldCurTime - lldOrgTime < 50)
        return;

    lldOrgTime = lldCurTime;

    cv::Mat bgr = pSdkObj->convertYUV420PtoMat(pBitmapData);
    if (bgr.empty()) return;

    emit pSdkObj->sendMonitorImage(bgr);
}


/**
 * @brief ?? I420 ??? cv::cvtColor ? BGR?
 * @param pBitmapData SDK YUV ?????????
 * @return BGR Mat??????????
 */
cv::Mat ILaser_PF400::convertYUV420PtoMat(IntelligentYUV* pBitmapData)
{
    const int w = pBitmapData->image_width;
    const int h = pBitmapData->image_height;
    if (w <= 0 || h <= 0 || !pBitmapData->data_plane[0]
        || !pBitmapData->data_plane[1] || !pBitmapData->data_plane[2])
        return cv::Mat();

    const int chromaW = w / 2;
    const int chromaH = h / 2;

    cv::Mat yuvI420(h + h / 2, w, CV_8UC1);
    uint8_t* pDst = yuvI420.data;

    memcpy(pDst, pBitmapData->data_plane[0], w * h);                       // Y
    pDst += w * h;
    memcpy(pDst, pBitmapData->data_plane[1], chromaW * chromaH);           // U
    pDst += chromaW * chromaH;
    memcpy(pDst, pBitmapData->data_plane[2], chromaW * chromaH);           // V

    cv::Mat bgr;
    cv::cvtColor(yuvI420, bgr, cv::COLOR_YUV2BGR_I420);
    return bgr;
}