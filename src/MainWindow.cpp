#include <opencv2/imgproc.hpp>

#include "MvCameraWorker.h"
#include "MainWindow.h"

#include <QCloseEvent>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QStatusBar>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    buildUi();
    startCameraThread();
}

MainWindow::~MainWindow()
{
    shutdownCameraThread();
}

void MainWindow::buildUi()
{
    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    auto *bar = new QHBoxLayout();

    m_btnSearch = new QPushButton(QStringLiteral("搜索相机"), central);
    m_deviceCombo = new QComboBox(central);
    m_btnConnect = new QPushButton(QStringLiteral("连接"), central);
    m_btnStart = new QPushButton(QStringLiteral("开始取图"), central);
    m_btnStop = new QPushButton(QStringLiteral("停止取图"), central);
    m_btnDisconnect = new QPushButton(QStringLiteral("断开连接"), central);
    m_deviceCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    bar->addWidget(m_btnSearch);
    bar->addWidget(m_deviceCombo, 1);
    bar->addWidget(m_btnConnect);
    bar->addWidget(m_btnStart);
    bar->addWidget(m_btnStop);
    bar->addWidget(m_btnDisconnect);

    m_imageLabel = new QLabel(QStringLiteral("无图像"), central);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setMinimumSize(640, 480);
    m_imageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_imageLabel->setStyleSheet(QStringLiteral("background-color: #111111; color: #dddddd;"));

    m_log = new QPlainTextEdit(central);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(200);
    m_log->setFixedHeight(96);

    root->addLayout(bar);
    root->addWidget(m_imageLabel, 1);
    root->addWidget(m_log);

    setCentralWidget(central);
    setWindowTitle(QStringLiteral("海康相机 OpenCV 预览"));
    resize(960, 720);
    statusBar()->showMessage(QStringLiteral("就绪"));
    updateButtons();
}

void MainWindow::startCameraThread()
{
    qRegisterMetaType<cv::Mat>("cv::Mat");

    m_worker = new MvCameraWorker();
    m_thread = new QThread();
    m_thread->setObjectName(QStringLiteral("MvCameraThread"));
    m_worker->moveToThread(m_thread);

    connect(m_btnSearch, &QPushButton::clicked, m_worker, &MvCameraWorker::searchDevices);
    connect(m_btnStart, &QPushButton::clicked, m_worker, &MvCameraWorker::startGrabbing);
    connect(m_btnStop, &QPushButton::clicked, m_worker, &MvCameraWorker::stopGrabbing);
    connect(m_btnDisconnect, &QPushButton::clicked, m_worker, &MvCameraWorker::disconnectDevice);
    connect(m_btnConnect, &QPushButton::clicked, this, &MainWindow::connectSelectedDevice);
    connect(this, &MainWindow::connectRequested, m_worker, &MvCameraWorker::connectDevice);
    connect(this, &MainWindow::frameShown, m_worker, &MvCameraWorker::notifyFrameDisplayed);

    connect(m_worker, &MvCameraWorker::imageReady, this, &MainWindow::showFrame);
    connect(m_worker, &MvCameraWorker::devicesFound, this, &MainWindow::onDevicesFound);
    connect(m_worker, &MvCameraWorker::deviceConnected, this, &MainWindow::onDeviceConnected);
    connect(m_worker, &MvCameraWorker::deviceDisconnected, this, &MainWindow::onDeviceDisconnected);
    connect(m_worker, &MvCameraWorker::grabbingStarted, this, &MainWindow::onGrabbingStarted);
    connect(m_worker, &MvCameraWorker::grabbingStopped, this, &MainWindow::onGrabbingStopped);
    connect(m_worker, &MvCameraWorker::statusMessage, this, &MainWindow::onStatusMessage);
    connect(m_worker, &MvCameraWorker::errorOccurred, this, &MainWindow::onErrorOccurred);

    m_thread->start();
    if (!m_thread->isRunning()) {
        appendLog(QStringLiteral("相机线程启动失败"));
        delete m_worker;
        delete m_thread;
        m_worker = nullptr;
        m_thread = nullptr;
    }
}

void MainWindow::shutdownCameraThread()
{
    if (m_thread == nullptr) {
        return;
    }

    if (m_thread->isRunning() && m_worker != nullptr) {
        disconnect(m_worker, nullptr, this, nullptr);
        disconnect(this, nullptr, m_worker, nullptr);
        QMetaObject::invokeMethod(m_worker, "disconnectDevice", Qt::BlockingQueuedConnection);
        m_thread->quit();
        m_thread->wait();
    }

    delete m_worker;
    m_worker = nullptr;
    delete m_thread;
    m_thread = nullptr;
}

void MainWindow::connectSelectedDevice()
{
    if (m_deviceCombo->currentIndex() < 0) {
        return;
    }
    emit connectRequested(m_deviceCombo->currentData().toInt());
}

void MainWindow::showFrame(cv::Mat image)
{
    if (!image.empty()) {
        m_lastImage = matToQImage(image);
        updateImageLabel();
    }
    emit frameShown();
}

void MainWindow::onDevicesFound(const QStringList &devices)
{
    m_deviceCombo->clear();
    for (int i = 0; i < devices.size(); ++i) {
        m_deviceCombo->addItem(devices.at(i), i);
    }
    updateButtons();
}

void MainWindow::onDeviceConnected(int index)
{
    m_connected = true;
    m_grabbing = false;
    if (index >= 0 && index < m_deviceCombo->count()) {
        m_deviceCombo->setCurrentIndex(index);
    }
    updateButtons();
}

void MainWindow::onDeviceDisconnected()
{
    m_connected = false;
    m_grabbing = false;
    updateButtons();
}

void MainWindow::onGrabbingStarted()
{
    m_grabbing = true;
    updateButtons();
}

void MainWindow::onGrabbingStopped()
{
    m_grabbing = false;
    updateButtons();
}

void MainWindow::onStatusMessage(const QString &message)
{
    appendLog(message);
}

void MainWindow::onErrorOccurred(const QString &message)
{
    appendLog(QString("错误: %1").arg(message));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    shutdownCameraThread();
    QMainWindow::closeEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    updateImageLabel();
}

void MainWindow::updateButtons()
{
    const bool hasDevice = m_deviceCombo != nullptr && m_deviceCombo->count() > 0;
    m_btnSearch->setEnabled(true);
    m_btnConnect->setEnabled(hasDevice && !m_connected);
    m_btnStart->setEnabled(m_connected && !m_grabbing);
    m_btnStop->setEnabled(m_connected && m_grabbing);
    m_btnDisconnect->setEnabled(m_connected);
}

void MainWindow::updateImageLabel()
{
    if (m_lastImage.isNull() || m_imageLabel == nullptr || m_imageLabel->size().isEmpty()) {
        return;
    }
    m_imageLabel->setPixmap(QPixmap::fromImage(m_lastImage).scaled(
        m_imageLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

void MainWindow::appendLog(const QString &message)
{
    if (m_log != nullptr) {
        m_log->appendPlainText(message);
    }
    statusBar()->showMessage(message, 5000);
}

QImage MainWindow::matToQImage(const cv::Mat &image) const
{
    if (image.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data,
                      rgb.cols,
                      rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGB888)
            .copy();
    }
    if (image.type() == CV_8UC1) {
        return QImage(image.data,
                      image.cols,
                      image.rows,
                      static_cast<int>(image.step),
                      QImage::Format_Grayscale8)
            .copy();
    }
    return {};
}
