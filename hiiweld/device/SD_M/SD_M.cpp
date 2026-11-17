/**
 * @file SD_M.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "SD_M.h"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <cmath>

#define RVS_PRE_FLAG "rvs-preview"
#define RVS_ADD_FLAG "rvs-pointcloud"

namespace {
    /** @brief 定线扫预览刷新是否已排队（避免重复 invokeMethod）。 */
    std::atomic<bool> g_refresh_pending{ false };
}

/**
 * @brief 构造相机对象并初始化 SDCamera 系统。
 * @param parent 父 QWidget。
 */
SD_M::SD_M(QWidget* parent)
    : ICamera(parent),
    m_captured_cloud(new ct::Cloud),
    m_merged_cloud(new ct::Cloud),
    connected(false)
{
    SDCamera::SystemInit();
}

/** @brief 关闭 X2 并关闭 SDCamera 系统。 */
SD_M::~SD_M()
{
    x2.Close();
    SDCamera::X2::Destroy(x2);
    SDCamera::SystemShutdown();
}

/** @brief 枚举设备，校验固件，上报型号并刷新设备信息。 */
void SD_M::searchDevice()
{
    SDCamera::Device devices[10];
    size_t actual_size = 0;
    SDCamera::SystemListDevices(devices, 5, &actual_size, SDCamera::SystemListDeviceType::All);
    if (actual_size == 0) {
        printE("No camera found.");
        SDCamera::SystemShutdown();
        return;
    }

    device = devices[0];
    if (!device.IsFirmwareMatch()) {
        printE("Firmware mismatch. Please upgrade in manager software.");
        SDCamera::SystemShutdown();
        return;
    }

    device.GetDeviceInfo(&info);

    emit sendCamModel(QString(info.name));

    this->updateCamInfo(0);
}

/**
 * @brief 切换连接状态：已连接则断开，否则创建 X2、打开设备并加载 JSON 参数。
 * @param json_path 相机参数配置文件路径。
 */
void SD_M::connectDevice(const std::string& json_path)
{
    if (!device.IsValid())
    {
        printW("Device is invalid!");
        return;
    }

    if (connected) {
        x2.Close();
        SDCamera::X2::Destroy(x2);
        SDCamera::SystemShutdown();
        printI(tr("Disconnect the device: %1 successfully.").arg(info.name));
        connected = false;
        emit sendCamConnectStatus(connected);
        return;
    }
    else {

        x2 = SDCamera::X2::Create(device);
        x2.Open();
        if (!x2.IsOpen()) {
            printE(tr("Connect to the device: %1 failed.").arg(info.name));
            SDCamera::X2::Destroy(x2);
            SDCamera::SystemShutdown();

            connected = false;
            emit sendCamConnectStatus(connected);
            return;
        }

        if (!x2.LoadSettingFromFile(json_path.c_str())) {
            printE("LoadSettingFromFile failed!");
            x2.Close();
            SDCamera::X2::Destroy(x2);
            SDCamera::SystemShutdown();
            connected = false;
            emit sendCamConnectStatus(connected);
            return;
        }

        printI(tr("Connect to the device: %1 successfully.").arg(info.name));
        connected = true;
        emit sendCamConnectStatus(connected);
        return;
    }
}

/** @brief 清除 CloudView 预览点云、坐标系及内部缓存点云。 */
void SD_M::reset() {
    m_cloudview->removeAllCoordinateSystems();
    m_cloudview->removePointCloud(RVS_PRE_FLAG);
    m_captured_cloud->clear();
}

/**
 * @brief 执行单次 Capture，按 IsFace 决定是否输出 2D 图并更新点云视图。
 * @param IsFace true=面阵（彩色+深度+点云），false=仅点云。
 */
void SD_M::captureDevice(const bool& IsFace)
{
    reset();
    if (!connected || !device.IsValid()) {
        printE("Please connect a device first!");
        return;
    }
    ct::TicToc ticToc;
    ticToc.tic();

    try
    {
        if (!x2.Capture()) {
            printE("Capture data failed!");
            x2.Close();
            SDCamera::X2::Destroy(x2);
            SDCamera::SystemShutdown();
            connected = false;
            emit sendCamConnectStatus(connected);
            return;
        }

        const SDCamera::CameraID colorCid = info.support_extra ? SDCamera::CameraID_Extra : SDCamera::CameraID_Left;
        SDCamera::Image color = x2.GetImage(colorCid);
        SDCamera::PointMap pm = x2.GetPointMap();
        SDCamera::DepthMap depth = x2.GetDepthMap();
        if (!color.IsValid() || !pm.IsValid()) {
            printE("Color or PointCloud data capture failed!");
            return;
        }

        ct::Cloud::Ptr cloud(new ct::Cloud);
        cv::Mat color_mat;
        cv::Mat depth_mat;

        if (IsFace) {
            convertToPCL(pm, color, color_mat, depth, depth_mat, true, cloud, true);
            rvsCamData.color_Image = std::move(color_mat);
            rvsCamData.depth_Image = std::move(depth_mat);
        }
        else {
            convertToPCL(pm, color, color_mat, depth, depth_mat, false, cloud, true);
        }

        if (cloud != nullptr)
            updatePointCloud(cloud, ticToc.toc());
        else
            printE("Failed to capture the frame!");
    }
    catch (const std::exception& e)
    {
        printE(QString("Failed to capture the frame, error %1").arg(e.what()));
    }
}

/**
 * @brief 配置硬触发定线扫、注册回调并 StartFixedLineScan。
 * @param jsonPath 可选采集参数 JSON 路径。
 */
void SD_M::captureFixedLineScan(const std::string& jsonPath)
{
    if (!connected || !device.IsValid()) { printE("Please connect a device first!"); return; }
    if (m_scanning) { printW("Fixed line scan already running."); return; }

    reset();
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        m_merged_cloud->clear();
        m_frame_index = 0;
    }

    if (!jsonPath.empty() && !x2.LoadSettingFromFile(jsonPath.c_str())) 
        printW("LoadSettingFromFile failed, using camera current settings.");

    SDCamera::X2::CaptureOptions cap_opt;
    cap_opt.capture_mode = SDCamera::CaptureMode_FixedLineScan;
    cap_opt.trigger_mode = SDCamera::TriggerMode_HardWare;
    if (!x2.LoadCaptureOptionParameters(cap_opt)) {
        printE("LoadCaptureOptionParameters failed!");
        x2.Close();
        SDCamera::X2::Destroy(x2);
        SDCamera::SystemShutdown();
        m_scanning = false;
        emit sendCamRunningStatus(m_scanning);
        return;
    }

    if (!x2.SetFixedLineScanCallback(&SD_M::FixedLineScanCB, this)) {
        printE("SetFixedLineScanCallback failed!");
        x2.Close();
        SDCamera::X2::Destroy(x2);
        SDCamera::SystemShutdown();
        m_scanning = false;
        emit sendCamRunningStatus(m_scanning);
        return;
    }

    x2.ResetTimestamp();

    if (!x2.StartFixedLineScan(cap_opt)) {
        printE(tr("Start fixed line scan failed: %1").arg(SDCamera::GetLastErrorMessage()));
        x2.Close();
        SDCamera::X2::Destroy(x2);
        SDCamera::SystemShutdown();
        m_scanning = false;
        emit sendCamRunningStatus(m_scanning);
        return;
    }

    m_scanning = true;
    emit sendCamRunningStatus(m_scanning);
    printI("Fixed line scan started. Waiting for encoder triggers...");
}

/** @brief 停止定线扫，将融合点云设为预览并 emit sendCamCloud。 */
void SD_M::stopFixedLineScan()
{
    if (!m_scanning) return;
    x2.StopFixedLineScan();
    m_scanning = false;
    emit sendCamRunningStatus(m_scanning);

    std::lock_guard<std::mutex> lock(m_scan_mutex);
    printI(tr("Fixed line scan stopped. Frames=%1, merged points=%2").arg(m_frame_index).arg(m_merged_cloud->size()));
    if (!m_merged_cloud->empty()) {
        m_captured_cloud.swap(m_merged_cloud);          // 指针交换，不拷贝点
        if (!m_merged_cloud)
            m_merged_cloud.reset(new ct::Cloud);
        else
            m_merged_cloud->clear();

        m_captured_cloud->setId(RVS_PRE_FLAG);
        rvsCamData.cloud = m_captured_cloud;
        emit sendCamCloud(rvsCamData, QString::fromStdString(RVS_PRE_FLAG));
    }
}

/**
 * @brief SDK 定线扫静态入口，转发至实例 onFixedScanFrame。
 * @param data 回调点图等数据。
 * @param pUser SD_M 实例指针。
 */
void SD_M::FixedLineScanCB(SDCamera::X2::FixedLineScanCallBackInfo data, SDCamera::UserPtr pUser)
{
    SD_M* self = static_cast<SD_M*>(pUser);
    if (self) self->onFixedScanFrame(data.pointmap);
}

/**
 * @brief 单帧转 PCL、按帧序号 Y 偏移后并入融合云并请求刷新视图。
 * @param pm 当前帧点图。
 */
void SD_M::onFixedScanFrame(SDCamera::PointMap& pm)
{
    if (!pm.IsValid()) { printW("Invalid fixed-line-scan frame, skipped."); return; }
    int idx;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        idx = m_frame_index;
    }
    ct::Cloud::Ptr frame(new ct::Cloud);
    convertToPCL(pm, frame, static_cast<float>(idx * m_y_step_mm));
    if (frame->empty()) {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        m_frame_index++;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        m_merged_cloud->reserve(m_merged_cloud->size() + frame->size());
        *m_merged_cloud += *frame;
        m_frame_index++;
    }
    if (!g_refresh_pending.exchange(true)) {
        QMetaObject::invokeMethod(this, "refreshMergedView", Qt::QueuedConnection);
    }
}

/** @brief 将 m_captured_cloud 复制加入点云树并清空预览。 */
void SD_M::add()
{
    if (m_captured_cloud->empty())
    {
        printW("Please capture a cloud first!");
        return;
    }
    m_cloudview->removePointCloud(RVS_PRE_FLAG);
    ct::Cloud::Ptr new_cloud = m_captured_cloud->makeShared();
    new_cloud->setId(RVS_ADD_FLAG);
    new_cloud->setInfo(QFileInfo(QString(DATA_PATH) + QString(RVS_ADD_FLAG)));
    new_cloud->update(false);
    m_cloudtree->appendCloud(new_cloud, true);
    m_captured_cloud->clear();
    printI(QString("Add captured cloud[id:%1] done.").arg(m_captured_cloud->id()));
}

/**
 * @brief 拼接板卡/序列/名称/日期并 emit sendCamInfo。
 * @param index 设备索引，小于 0 直接返回。
 */
void SD_M::updateCamInfo(int index)
{
    if (index < 0) return;
    if (info.name == "") {
        printW("Device List is empty!");
        return;
    }
    QString camInfo = tr("Model:%1 ").arg(info.boardmodel) + tr("Serial:%1 ").arg(info.firmware_version)
        + tr("Name:%1 ").arg(info.name) + tr("Date:%1").arg(info.factroydate);

    emit sendCamInfo(camInfo);
}
 
/**
 * @brief 点图转 PCL（米→毫米），可选 2D 图克隆、深度图与法向/逐像素 RGB。
 * @param pm 点图。
 * @param SDImage 彩色图。
 * @param CvColor 输出彩色 Mat。
 * @param SDDepth 深度图。
 * @param CvDepth 输出深度 Mat。
 * @param Get2DImage 是否生成 2D。
 * @param pclCloud 输出点云。
 * @param saveNormal 是否写法向。
 */
void SD_M::convertToPCL(SDCamera::PointMap& pm, SDCamera::Image& SDImage,
    cv::Mat& CvColor, SDCamera::DepthMap& SDDepth, cv::Mat& CvDepth,
    bool Get2DImage, ct::Cloud::Ptr& pclCloud, bool saveNormal)
{
    if (!pm.IsValid()) {
        printW("point map is invalid!");
        return;
    }

    const double* pm_data = pm.GetPointDataPtr();
    if (pm_data == nullptr) {
        printW("point map data is empty!");
        return;
    }

    const SDCamera::Size pm_size = pm.GetSize();
    const int pm_rows = pm_size.rows;
    const int pm_cols = pm_size.cols;
    const int pm_num = pm_rows * pm_cols;
    const double* normal_data = saveNormal ? pm.GetNormalDataPtr() : nullptr;

    CvColor.release();
    CvDepth.release();

    // ---------- 2D 彩色图：必须用 Image 自己的尺寸，不能用点云尺寸 ----------
    const unsigned char* img_data = nullptr;
    SDCamera::ImageType::Enum image_type = SDCamera::ImageType::Mono8;
    int cv_type = CV_8UC1;
    int pix_step = 1;
    int img_rows = 0;
    int img_cols = 0;
    bool pixel_aligned = false;

    if (SDImage.IsValid()) {
        img_data = SDImage.GetDataPtr();
        image_type = SDImage.GetType();
        const SDCamera::Size isz = SDImage.GetSize();
        img_rows = isz.rows;
        img_cols = isz.cols;

        switch (image_type) {
        case SDCamera::ImageType::RGB8:
        case SDCamera::ImageType::BGR8:
            cv_type = CV_8UC3;
            pix_step = 3;
            break;
        default:
            cv_type = CV_8UC1;
            pix_step = 1;
            break;
        }

        if (Get2DImage && img_data != nullptr && img_rows > 0 && img_cols > 0) {
            cv::Mat color_view(img_rows, img_cols, cv_type,
                const_cast<unsigned char*>(img_data));
            CvColor = color_view.clone();
        }

        pixel_aligned = (img_data != nullptr && pm_rows == img_rows && pm_cols == img_cols);
    }

    // ---------- 深度图：double、单通道、用 DepthMap 自己的尺寸 ----------
    if (SDDepth.IsValid() && Get2DImage) {
        const double* depth_data = SDDepth.GetDataPtr();
        const SDCamera::Size dsz = SDDepth.GetSize();
        if (depth_data != nullptr && dsz.rows > 0 && dsz.cols > 0) {
            cv::Mat depth_view(dsz.rows, dsz.cols, CV_64FC1,
                const_cast<double*>(depth_data));
            depth_view.convertTo(CvDepth, CV_32FC1, 1000.0);
        }
    }

    // ---------- 点云：丢掉无效点；仅在与 2D 图逐像素对齐时上色 ----------
    pclCloud->clear();
    pclCloud->reserve(static_cast<std::size_t>(pm_num));

    pcl::PointXYZRGBNormal pt;
    for (int i = 0; i < pm_num; ++i) {
        const double* p = pm_data + static_cast<std::ptrdiff_t>(i) * 3;
        const float x = static_cast<float>(p[0]) * 1000.0f;
        const float y = static_cast<float>(p[1]) * 1000.0f;
        const float z = static_cast<float>(p[2]) * 1000.0f;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z < 10.0f)
            continue;

        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.r = pt.g = pt.b = 0;
        pt.normal_x = pt.normal_y = pt.normal_z = 0.f;

        if (normal_data) {
            const double* n = normal_data + static_cast<std::ptrdiff_t>(i) * 3;
            pt.normal_x = static_cast<float>(n[0]);
            pt.normal_y = static_cast<float>(n[1]);
            pt.normal_z = static_cast<float>(n[2]);
        }

        if (pixel_aligned) {
            const unsigned char* pix = img_data + static_cast<std::ptrdiff_t>(i) * pix_step;
            switch (image_type) {
            case SDCamera::ImageType::Mono8:
                pt.r = pt.g = pt.b = pix[0];
                break;
            case SDCamera::ImageType::RGB8:
                pt.r = pix[0];
                pt.g = pix[1];
                pt.b = pix[2];
                break;
            case SDCamera::ImageType::BGR8:
                pt.r = pix[2];
                pt.g = pix[1];
                pt.b = pix[0];
                break;
            default:
                break;
            }
        }

        pclCloud->push_back(pt);
    }

    pclCloud->width = static_cast<uint32_t>(pclCloud->size());
    pclCloud->height = 1;
    pclCloud->is_dense = true;
}

/**
 * @brief 轻量点图转 PCL，Y 坐标叠加 y_offset_mm（定线扫叠帧）。
 * @param pm 点图。
 * @param pclCloud 输出点云。
 * @param y_offset_mm Y 向平移（毫米）。
 */
void SD_M::convertToPCL(SDCamera::PointMap& pm, ct::Cloud::Ptr& pclCloud, float y_offset_mm)
{
    if (!pm.IsValid()) {
        printW("point map is invalid!");
        return;
    }

    const double* pm_data = pm.GetPointDataPtr();
    const SDCamera::Size pm_size = pm.GetSize();
    const int pm_num = pm_size.rows * pm_size.cols;

    pclCloud->resize(static_cast<std::size_t>(pm_num));
    std::size_t k = 0;

    for (int i = 0; i < pm_num; ++i) {
        const double* p = pm_data + static_cast<std::ptrdiff_t>(i) * 3;
        const float x = static_cast<float>(p[0]) * 1000.0f;
        const float y = static_cast<float>(p[1]) * 1000.0f + y_offset_mm;
        const float z = static_cast<float>(p[2]) * 1000.0f;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z < 10.0f)
            continue;

        pcl::PointXYZRGBNormal& pt = pclCloud->points[k++];
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.r = pt.g = pt.b = 0;
    }

    pclCloud->resize(k);
    pclCloud->width = static_cast<uint32_t>(k);
    pclCloud->height = 1;
    pclCloud->is_dense = true;
}

/**
 * @brief 绑定预览 id、刷新 CloudView 坐标系并上报 CamData。
 * @param captured_cloud_ 采集点云。
 * @param time 耗时（ms）。
 */
void SD_M::updatePointCloud(const ct::Cloud::Ptr& captured_cloud_, float time)
{
    m_captured_cloud = captured_cloud_;

    m_captured_cloud->setId(RVS_PRE_FLAG);
    printI(tr("Number of points in PCL Cloud : %1").arg(m_captured_cloud->size()));
    printI(tr("Capture cloud[id:%1] done, take time %2 ms.").arg(m_captured_cloud->id()).arg(time));
    m_cloudview->addPointCloud(m_captured_cloud);
    m_cloudview->addCoordinateSystem(ct::Coord("cloud", 100.0, Eigen::Affine3f::Identity()));
    m_cloudview->resetCamera();
    m_cloudview->update();
    rvsCamData.cloud = m_captured_cloud;

    emit sendCamCloud(rvsCamData, QString::fromStdString(RVS_PRE_FLAG));
}

/** @brief 拷贝融合点云快照到 CloudView（定线扫过程中节流调用）。 */
void SD_M::refreshMergedView()
{
    g_refresh_pending.store(false);
    ct::Cloud::Ptr snapshot;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        if (m_merged_cloud->empty()) return;
        snapshot.reset(new ct::Cloud(*m_merged_cloud));
    }
    snapshot->setId(RVS_PRE_FLAG);
    m_captured_cloud = snapshot;
    m_cloudview->removePointCloud(RVS_PRE_FLAG);
    m_cloudview->addPointCloud(m_captured_cloud);
}

/** @brief 已连接时调用 X2 打开防护罩。 */
void SD_M::opencover() {
    if (!connected || !device.IsValid()) {
        printE("Please connect a device first!");
        return;
    }
    if (x2.OpenProtectiveCover()) {
        printI("Camera Protective Cover open done!");
    }
    else {
        printE("Camera Protective Cover open failed!");
    }
    return;
}

/** @brief 已连接时调用 X2 关闭防护罩。 */
void SD_M::closecover() {
    if (!connected || !device.IsValid()) {
        printE("Please connect a device first!");
        return;
    }
    if (x2.CloseProtectiveCover()) {
        printI("Camera Protective Cover close done!");
    }
    else {
        printE("Camera Protective Cover close failed!");
    }
    return;
}