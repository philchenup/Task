/**
 * @file meltCam.h
 * @brief 海康工业相机熔池取流 Worker（MVS SDK），运行在独立线程，通过信号向 UI 交付图像。
 */
#pragma once

#include <opencv2/core.hpp>

#ifdef signals
#pragma push_macro("signals")
#undef signals
#define MVS_RESTORE_SIGNALS
#endif
#ifdef slots
#pragma push_macro("slots")
#undef slots
#define MVS_RESTORE_SLOTS
#endif
#ifdef emit
#pragma push_macro("emit")
#undef emit
#define MVS_RESTORE_EMIT
#endif
#include "MvCameraControl.h"
#ifdef MVS_RESTORE_EMIT
#pragma pop_macro("emit")
#undef MVS_RESTORE_EMIT
#endif
#ifdef MVS_RESTORE_SLOTS
#pragma pop_macro("slots")
#undef MVS_RESTORE_SLOTS
#endif
#ifdef MVS_RESTORE_SIGNALS
#pragma pop_macro("signals")
#undef MVS_RESTORE_SIGNALS
#endif

#include <QElapsedTimer>
#include <QMetaType>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <vector>

Q_DECLARE_METATYPE(cv::Mat)

/**
 * @brief 熔池相机取流对象：枚举/连接/采图，定时 grabFrame 并通过 imageReady 深拷贝图像到主线程。
 */
class MvCameraWorker : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 构造 Worker 并连接定时器到 grabFrame。
     * @param parent Qt 父对象。
     */
    explicit MvCameraWorker(QObject* parent = nullptr);

    /** @brief 释放设备资源（不向外发送断开通知）。 */
    ~MvCameraWorker() override;

public slots:
    /** @brief 枚举 GigE 与 USB 相机并发送 devicesFound。 */
    void searchDevices();

    /**
     * @brief 按上次搜索结果中的序号打开相机。
     * @param index 设备列表下标。
     */
    void connectDevice(int index);

    /** @brief 开始采集并启动 100ms 定时取帧。 */
    void startGrabbing();

    /** @brief 停止采集，保持设备已打开。 */
    void stopGrabbing();

    /** @brief 停止采集并关闭、销毁句柄。 */
    void disconnectDevice();

    /** @brief 单次超时取一帧，转换后 emit imageReady。 */
    void grabFrame();

signals:
    /**
     * @brief 供主线程显示的 OpenCV 图像（已深拷贝）。
     * @param image BGR 或灰度 Mat。
     */
    void imageReady(const cv::Mat& image);

    /**
     * @brief 搜索到的设备型号列表。
     * @param devices 各设备 model 名称字符串。
     */
    void devicesFound(const std::vector<QString>& devices);

    /**
     * @brief 成功打开设备。
     * @param index 已连接的设备序号。
     */
    void deviceConnected(int index);

    /** @brief 设备已关闭。 */
    void deviceDisconnected();

    /** @brief 已开始取流。 */
    void grabbingStarted();

    /** @brief 已停止取流。 */
    void grabbingStopped();

    /**
     * @brief 普通状态提示。
     * @param message 提示文本。
     */
    void statusMessage(const QString& message);

    /**
     * @brief 错误信息。
     * @param message 错误描述。
     */
    void errorOccurred(const QString& message);

private:
    /**
     * @brief 关流、关设备、销毁句柄并清理缓冲。
     * @param notify 为 true 时发送 deviceDisconnected 等信号。
     */
    void releaseDevice(bool notify);

    /**
     * @brief 停止 MVS 取流并停定时器。
     * @param notify 为 true 时发送 grabbingStopped / 错误。
     * @return 曾处于取流状态并已处理时为 true。
     */
    bool endGrabbing(bool notify);

    /**
     * @brief 将一帧原始数据转为 OpenCV Mat。
     * @param info 帧信息（宽高、像素格式）。
     * @param data 帧缓冲指针。
     * @param out 输出 Mat。
     * @return 转换成功为 true。
     */
    bool frameToMat(const MV_FRAME_OUT_INFO_EX* info, unsigned char* data, cv::Mat& out);

    /**
     * @brief 使用 MVS ConvertPixelType 转为目标格式后再包装 Mat。
     * @param info 源帧信息。
     * @param data 源数据。
     * @param dstType 目标像素类型（Mono8 或 BGR8）。
     * @param out 输出 Mat。
     * @return 成功为 true。
     */
    bool convertWithSdk(const MV_FRAME_OUT_INFO_EX* info, unsigned char* data,
        MvGvspPixelType dstType, cv::Mat& out);

    void* m_handle = nullptr;                              /**< MVS 设备句柄。 */
    MV_CC_DEVICE_INFO_LIST m_deviceList{};                   /**< 最近一次枚举结果。 */
    std::vector<unsigned char> m_payload;                    /**< 单帧原始缓冲。 */
    std::vector<unsigned char> m_convertBuffer;              /**< SDK 像素转换缓冲。 */
    unsigned int m_payloadSize = 0;                          /**< PayloadSize 字节数。 */
    bool m_opened = false;                                   /**< 设备是否已 OpenDevice。 */
    bool m_grabbingStarted = false;                          /**< 是否已 StartGrabbing。 */
    bool m_loggedFirstFrame = false;                         /**< 预留：首帧日志标志。 */
    bool m_hasFailedPixel = false;                           /**< 是否已上报过像素格式错误。 */
    int m_connectedIndex = -1;                               /**< 当前连接在列表中的下标。 */
    unsigned int m_lastFailedPixel = 0;                      /**< 上次失败的像素格式值。 */
    QElapsedTimer m_grabErrorTimer;                          /**< 取帧失败日志节流。 */

    QTimer* m_timer;                                         /**< 周期性调用 grabFrame。 */

    Q_DISABLE_COPY(MvCameraWorker)
};