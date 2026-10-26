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

#include <vector>

Q_DECLARE_METATYPE(cv::Mat)

// 海康相机取流对象。由 MainWindow 移入独立线程后，通过槽控制、通过信号把 cv::Mat 送回界面线程。
// 槽：searchDevices 搜索，connectDevice 连接，startGrabbing 开始取图，
//     stopGrabbing 停止取图，disconnectDevice 断开连接。
class MvCameraWorker : public QObject
{
    Q_OBJECT

public:
    explicit MvCameraWorker(QObject *parent = nullptr);
    ~MvCameraWorker() override;

public slots:
    // ch:外部触发相机搜索 | en:Enumerate GigE and USB cameras.
    void searchDevices();
    // ch:按搜索结果序号连接 | en:Open the device selected from the last search.
    void connectDevice(int index);
    // ch:开始取图，并持续把图像发给界面 | en:Start acquisition and emit frames.
    void startGrabbing();
    // ch:停止取图，保持连接 | en:Stop acquisition and keep the device open.
    void stopGrabbing();
    // ch:停止取图并关闭相机 | en:Stop acquisition and close the device.
    void disconnectDevice();
    // ch:界面显示完一帧后调用，允许发送下一帧 | en:Let the worker emit the next frame.
    void notifyFrameDisplayed();

signals:
    // ch:发送到主线程显示的 OpenCV 图像，已深拷贝 | en:Deep-copied image for the UI thread.
    void imageReady(cv::Mat image);
    void devicesFound(const QStringList &devices);
    void deviceConnected(int index);
    void deviceDisconnected();
    void grabbingStarted();
    void grabbingStopped();
    void statusMessage(const QString &message);
    void errorOccurred(const QString &message);

private:
    void grabFrame();
    void releaseDevice(bool notify);
    bool endGrabbing(bool notify);
    bool frameToMat(const MV_FRAME_OUT_INFO_EX *info, unsigned char *data, cv::Mat &out);
    bool convertWithSdk(const MV_FRAME_OUT_INFO_EX *info, unsigned char *data,
                        MvGvspPixelType dstType, cv::Mat &out);
    QString deviceText(MV_CC_DEVICE_INFO *info, unsigned int index, bool markOccupied) const;

    void *m_handle = nullptr;
    MV_CC_DEVICE_INFO_LIST m_deviceList{};
    std::vector<unsigned char> m_payload;
    std::vector<unsigned char> m_convertBuffer;
    unsigned int m_payloadSize = 0;
    bool m_opened = false;
    bool m_grabbing = false;
    bool m_grabbingStarted = false;
    bool m_displayBusy = false;
    bool m_loggedFirstFrame = false;
    bool m_hasFailedPixel = false;
    int m_connectedIndex = -1;
    unsigned int m_lastFailedPixel = 0;
    QElapsedTimer m_grabErrorTimer;

    Q_DISABLE_COPY(MvCameraWorker)
};
