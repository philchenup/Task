/**
 * @file SD_M.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 * @brief 深视智能 SD_M 相机驱动（Qt/ICamera），支持面阵采集与硬件触发定长线扫融合。
 */
#ifndef DEVICE_SD_M_H
#define DEVICE_SD_M_H

#include "ICamera.h"
#include <SDCamera/SDCamera.h>
#include <mutex> 
#include <iostream>
#include <string>

/**
 * @brief 深视 SD_M 相机设备实现，封装 SDCamera SDK 的点云、定线扫与护罩控制。
 */
class SD_M : public ICamera
{
    Q_OBJECT
public:
    /**
     * @brief 构造相机对象并初始化 SDCamera 系统。
     * @param parent 父 QWidget，传递给 ICamera。
     */
    explicit SD_M(QWidget* parent = nullptr);

    /** @brief 关闭 X2 会话并调用 SystemShutdown。 */
    ~SD_M();

    /**
     * @brief 根据设备信息索引刷新并上报相机型号/序列等描述。
     * @param index 设备列表索引，小于 0 时忽略。
     */
    void updateCamInfo(int index) override;

    /**
     * @brief 将 SDK 点图、彩色图、深度图转换为 PCL 点云及可选 OpenCV 图像。
     * @param pm SDK 点图。
     * @param SDImage 彩色/灰度图。
     * @param CvColor 输出的 OpenCV 彩色图。
     * @param SDDepth SDK 深度图。
     * @param CvDepth 输出的 OpenCV 深度图（毫米，float）。
     * @param Get2DImage 为 true 时填充 CvColor/CvDepth。
     * @param pclCloud 输出的 PCL 点云（含可选法向与逐像素上色）。
     * @param saveNormal 为 true 时写入点法向。
     */
    void convertToPCL(SDCamera::PointMap& pm, SDCamera::Image& SDImage,
        cv::Mat& CvColor, SDCamera::DepthMap& SDDepth, cv::Mat& CvDepth,
        bool Get2DImage, ct::Cloud::Ptr& pclCloud, bool saveNormal);

    /**
     * @brief 将点图转为 PCL，并在 Y 方向施加扫描偏移（定线扫叠帧）。
     * @param pm SDK 点图。
     * @param pclCloud 输出点云。
     * @param y_offset_mm 本帧沿 Y 的平移量（毫米）。
     */
    void convertToPCL(SDCamera::PointMap& pm, ct::Cloud::Ptr& pclCloud, float y_offset_mm);

    /**
     * @brief 将采集点云显示到 CloudView 并通过信号上报。
     * @param cloud 已填充的点云。
     * @param time 采集耗时（毫秒）。
     */
    void updatePointCloud(const ct::Cloud::Ptr& cloud, float time);

public slots:
    /** @brief 枚举系统中的 SD 相机，取首台有效设备并上报型号。 */
    void searchDevice() override;

    /**
     * @brief 单次触发采集（面阵或仅点云）。
     * @param IsFace 为 true 时同时采集彩色/深度并写入 CamData。
     */
    void captureDevice(const bool& IsFace) override;

    /**
     * @brief 连接或断开相机，连接时从 JSON 加载参数。
     * @param json_path 相机配置文件路径。
     */
    void connectDevice(const std::string& json_path) override;

    /** @brief 将预览点云追加到点云树并重置预览缓存。 */
    void add() override;

    /** @brief 清空预览点云、坐标系及 CloudView 中的预览图层。 */
    void reset() override;

    /**
     * @brief 启动硬件触发定长线扫模式并注册回调。
     * @param jsonPath 采集参数 JSON，可为空则沿用当前设置。
     */
    void captureFixedLineScan(const std::string& jsonPath) override;

    /** @brief 停止定线扫，合并点云并发送 sendCamCloud。 */
    void stopFixedLineScan() override;

    /** @brief 定线扫过程中节流刷新 CloudView 上的融合预览。 */
    void refreshMergedView();

    /** @brief 打开相机防护罩。 */
    void opencover() override;

    /** @brief 关闭相机防护罩。 */
    void closecover() override;

private:
    /**
     * @brief 定线扫 SDK 静态回调，转发至 onFixedScanFrame。
     * @param data 回调携带的点图等信息。
     * @param pUser 用户指针（SD_M*）。
     */
    static void FixedLineScanCB(SDCamera::X2::FixedLineScanCallBackInfo data, SDCamera::UserPtr pUser);

    /**
     * @brief 处理定线扫单帧：转 PCL、Y 向平移后并入 m_merged_cloud。
     * @param pm 当前帧点图。
     */
    void onFixedScanFrame(SDCamera::PointMap& pm);

    SDCamera::Device device;           /**< 当前选中的 SDK 设备句柄。 */
    ct::Cloud::Ptr m_captured_cloud;   /**< 最近一次预览/停止线扫后的点云。 */
    SDCamera::DeviceInfo info;        /**< 设备信息（型号、序列等）。 */
    bool connected = false;           /**< 是否已通过 X2 打开设备。 */
    CamData rvsCamData;               /**< 上报给 UI 的相机数据结构。 */

    SDCamera::X2 x2;                  /**< X2 采集会话对象。 */

    ct::Cloud::Ptr m_merged_cloud;    /**< 定线扫累计融合点云。 */
    std::mutex     m_scan_mutex;      /**< 保护 m_merged_cloud 与 m_frame_index。 */
    int            m_frame_index = 0; /**< 定线扫已接收帧计数。 */
    double         m_y_step_mm = 2.0; /**< 相邻触发帧的 Y 步距（毫米）。 */
    bool           m_scanning = false;/**< 定线扫是否正在运行。 */
};

#endif // DEVICE_SD_M_H