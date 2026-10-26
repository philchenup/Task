#pragma once

#include <opencv2/core.hpp>

#include <QImage>
#include <QMainWindow>
#include <QStringList>

class QCloseEvent;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QResizeEvent;
class QThread;
class MvCameraWorker;

// 界面线程。startCameraThread() 创建相机线程，并把按钮接到 MvCameraWorker 的槽。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

signals:
    void connectRequested(int index);
    void frameShown();

protected:
    void closeEvent(QCloseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void connectSelectedDevice();
    void showFrame(cv::Mat image);
    void onDevicesFound(const QStringList &devices);
    void onDeviceConnected(int index);
    void onDeviceDisconnected();
    void onGrabbingStarted();
    void onGrabbingStopped();
    void onStatusMessage(const QString &message);
    void onErrorOccurred(const QString &message);

private:
    void buildUi();
    void startCameraThread();
    void shutdownCameraThread();
    void updateButtons();
    void updateImageLabel();
    void appendLog(const QString &message);
    QImage matToQImage(const cv::Mat &image) const;

    QThread *m_thread = nullptr;
    MvCameraWorker *m_worker = nullptr;

    QLabel *m_imageLabel = nullptr;
    QComboBox *m_deviceCombo = nullptr;
    QPushButton *m_btnSearch = nullptr;
    QPushButton *m_btnConnect = nullptr;
    QPushButton *m_btnStart = nullptr;
    QPushButton *m_btnStop = nullptr;
    QPushButton *m_btnDisconnect = nullptr;
    QPlainTextEdit *m_log = nullptr;

    QImage m_lastImage;
    bool m_connected = false;
    bool m_grabbing = false;

    Q_DISABLE_COPY(MainWindow)
};
