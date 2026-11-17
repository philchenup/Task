/**
 * @file IntellgentLaser.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 * @brief 智能线激光 PF400 设备驱动，实现 ILaser 接口与 SDK 回调。
 */
#ifndef CT_DEVICE_IL_H
#define CT_DEVICE_IL_H

#include "ILaser.h"
#include <QMutex>
#include <QObject>
#include <QTimer>
#include "IntelligentLaser_export.h"
#include "IntelligentLaser_define.h"
#include <iostream>
#include <string>

/**
 * @brief Intelligent Laser PF400 驱动：设备发现、多端口连接、出光/传感器/采集与轮廓回调。
 */
class ILaser_PF400 : public ILaser
{
    Q_OBJECT
public:
    /**
     * @brief 构造并调用 Laser_Init 注册 SDK 回调。
     * @param parent Qt 父对象。
     */
    explicit ILaser_PF400(QObject* parent = nullptr);

    /** @brief 断开传感器并 UnInitSDK。 */
    ~ILaser_PF400();

    /**
     * @brief 初始化 Intelligent Laser SDK 并注册轮廓/采集/监控 YUV 回调。
     * @return SDK 初始化成功为 true。
     */
    bool Laser_Init();

    /**
     * @brief 将 SDK YUV420P 帧转为 OpenCV BGR Mat。
     * @param pBitmapData SDK 图像平面数据。
     * @return BGR 图像；数据无效时返回空 Mat。
     */
    cv::Mat convertYUV420PtoMat(IntelligentYUV* pBitmapData);

    /**
     * @brief SDK 毫米轮廓回调：节流后组装点云并 send3DContour。
     * @param pMMContourData 轮廓与检测结果。
     * @param pUser 用户指针（ILaser_PF400*）。
     * @param dwInstanceId SDK 实例 ID。
     */
    static void sdkCallbackMMProfile(IntelligentMillionOutlineInspect* pMMContourData, void* pUser, uint32_t dwInstanceId);

    /**
     * @brief SDK 采集相机 YUV 回调：转 BGR 后 sendCaptureImage。
     * @param pBitmapData YUV 帧。
     * @param pUser ILaser_PF400 实例。
     * @param dwInstanceId 实例 ID。
     */
    static void sdkCallbackScatterBitmap1(IntelligentYUV* pBitmapData, void* pUser, uint32_t dwInstanceId);

    /**
     * @brief SDK 监控相机 YUV 回调：转 BGR 后 sendMonitorImage。
     * @param pBitmapData YUV 帧。
     * @param pUser ILaser_PF400 实例。
     * @param dwInstanceId 实例 ID。
     */
    static void sdkCallbackScatterBitmap2(IntelligentYUV* pBitmapData, void* pUser, uint32_t dwInstanceId);

public slots:
    /** @brief 探测局域网传感器，最多重试 6 次并 sendLaserIp。 */
    void searchDevice() override;

    /**
     * @brief 开关轮廓/图像采集标志并上报 sendCaptureStatus。
     * @param bEnable true 开始采集，false 停止。
     */
    void captureDevice(const bool& bEnable) override;

    /**
     * @brief 按 IP 连接或断开多服务端口传感器。
     * @param camIp 点分十进制 IP 字符串。
     */
    void connectDevice(const std::string& camIp) override;

    /**
     * @brief 启用/关闭激光出光（SetLaserEnable）。
     * @param bEnable true 开光，false 关光。
     */
    void openLaser(const bool& bEnable) override;

    /**
     * @brief 启用/关闭传感器、采集相机与监控相机。
     * @param bEnable true 全开，false 全关。
     */
    void openSensor(const bool& bEnable) override;

private:
    bool connected = false;   /**< 是否已与传感器建立连接。 */
    bool m_capture = false; /**< 是否处于数据采集中（回调闸门）。 */
    QTimer* m_pTimer;         /**< 预留定时器指针。 */
    int m_timerIndex = 0;     /**< 预留定时器索引。 */
};

#endif // CT_DEVICE_IL_H