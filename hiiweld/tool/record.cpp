/**
 * @file record.cpp
 * @brief 屏幕录制与 ffmpeg 管道：开始/暂停/ROI/退出，停止时另存 MP4。
 */
#include "record.h"
#include "ui_record.h"

#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QProcess>
#include <QScreen>
#include <QStandardPaths>
#include <QTimer>

#include <cstring>

namespace
{

    constexpr int kFps = 15;
    constexpr int kMinSize = 16;

    QPoint eventPos(const QMouseEvent* event)
    {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        return event->globalPosition().toPoint();
#else
        return event->globalPos();
#endif
    }

    QString ffmpegPath()
    {
        const QString fromEnv = qEnvironmentVariable("FFMPEG");
        if (!fromEnv.isEmpty() && QFileInfo(fromEnv).isExecutable())
            return fromEnv;
        return QStandardPaths::findExecutable(QStringLiteral("ffmpeg"));
    }

} // namespace

class RoiPicker : public QWidget
{
public:
    explicit RoiPicker(Record* owner)
        : m_owner(owner)
    {
        setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
        setAttribute(Qt::WA_TranslucentBackground);
        setCursor(Qt::CrossCursor);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0, 0, 0, 0));
        const QRect hole = m_band.translated(-geometry().topLeft()).intersected(rect());
        const QColor dim(0, 0, 0, 90);
        if (hole.width() < 2 || hole.height() < 2)
        {
            painter.fillRect(rect(), dim);
        }
        else
        {
            QRegion outside(rect());
            outside -= QRegion(hole);
            painter.setClipRegion(outside);
            painter.fillRect(rect(), dim);
            painter.setClipping(false);
            painter.setPen(QPen(Qt::white, 1, Qt::DashLine));
            painter.drawRect(hole.adjusted(0, 0, -1, -1));
        }
        painter.setPen(Qt::white);
        painter.drawText(QRect(0, 24, width(), 28), Qt::AlignCenter,
            m_dragging
            ? QStringLiteral("松开确认    %1 × %2    Esc 取消").arg(qMax(0, m_band.width())).arg(qMax(0, m_band.height()))
            : QStringLiteral("拖拽划定录制区域，Esc 取消"));
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() != Qt::LeftButton)
            return;
        m_dragging = true;
        m_origin = eventPos(event);
        m_band = QRect(m_origin, m_origin);
        update();
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        if (!m_dragging)
            return;
        m_band = QRect(m_origin, eventPos(event)).normalized();
        update();
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (!m_dragging || event->button() != Qt::LeftButton)
            return;
        m_dragging = false;
        m_band = QRect(m_origin, eventPos(event)).normalized();
        hide();
        m_owner->applyRoi(m_band);
    }

    void keyPressEvent(QKeyEvent* event) override
    {
        if (event->key() == Qt::Key_Escape)
        {
            m_dragging = false;
            hide();
            m_owner->cancelRoi();
            return;
        }
        QWidget::keyPressEvent(event);
    }

    void showEvent(QShowEvent* event) override
    {
        QWidget::showEvent(event);
        m_dragging = false;
        m_band = QRect();
        activateWindow();
        setFocus();
    }

private:
    Record* m_owner = nullptr;
    QPoint m_origin;
    QRect m_band;
    bool m_dragging = false;
};

/** @brief 创建 ROI 选择器与定时器。 */
Record::Record(QWidget* parent)
    :CustomDialog(parent),
    ui(new Ui::Record)
{
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("屏幕录制"));
    setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

    m_picker = new RoiPicker(this);
    m_timer = new QTimer(this);
    m_timer->setInterval(1000 / kFps);
    m_ffmpeg = new QProcess(this);
    m_ffmpeg->setProcessChannelMode(QProcess::SeparateChannels);

    connect(m_timer, &QTimer::timeout, this, &Record::grabFrame);
    connect(ui->startRecordBtn, &QPushButton::clicked, this, &Record::start);
    connect(ui->pauseRecordBtn, &QPushButton::clicked, this, &Record::pause);
    connect(ui->roiRecordBtn, &QPushButton::clicked, this, &Record::selectRoi);
    connect(ui->exitRecordBtn, &QPushButton::clicked, this, &Record::exitRecord);

    if (QScreen* screen = QGuiApplication::primaryScreen())
        m_roi = screen->geometry();
    updateButtons();
}

/** @brief 停止 ffmpeg 并释放资源。 */
Record::~Record()
{
    m_timer->stop();
    if (m_ffmpeg->state() != QProcess::NotRunning)
    {
        m_ffmpeg->kill();
        m_ffmpeg->waitForFinished(1000);
    }
    if (!m_tempFile.isEmpty())
        QFile::remove(m_tempFile);
    delete m_picker;
    delete ui;
}

/** @brief 对话框打开时初始化。 */
void Record::init()
{
    if (!m_recording && !m_paused)
    {
        if (QScreen* screen = QGuiApplication::primaryScreen())
            m_roi = screen->geometry();
    }
    updateButtons();
}

/** @brief 恢复默认 UI 状态。 */
void Record::reset()
{
    m_timer->stop();
    if (m_ffmpeg->state() != QProcess::NotRunning)
    {
        m_ffmpeg->kill();
        m_ffmpeg->waitForFinished(1500);
    }
    if (!m_tempFile.isEmpty())
        QFile::remove(m_tempFile);
    m_tempFile.clear();
    m_recording = false;
    m_paused = false;
    m_saving = false;
    if (m_picker)
        m_picker->hide();
    updateButtons();
}

/** @brief 停止抓取。
void Record::deinit()
{
    if (!m_saving)
        reset();
    if (m_picker)
        m_picker->hide();
}

/** @brief 开始或结束一次录制。 */
void Record::start()
{
    if (m_saving)
        return;
    if (m_recording || m_paused) {
        ui->startRecordBtn->setIcon(QIcon(":/res/icon/start_.svg"));
        stop();
    }
    else {
        ui->startRecordBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
        begin();
    } 
}

/** @brief 暂停/继续。
void Record::pause()
{
    if (m_saving)
        return;
    if (m_recording)
    {
        m_recording = false;
        m_paused = true;
        m_timer->stop();
    }
    else if (m_paused)
    {
        m_paused = false;
        m_recording = true;
        m_timer->start();
    }
    updateButtons();
}

/** @brief 停止并弹出保存。 */
void Record::stop()
{
    if (m_saving || (!m_recording && !m_paused))
        return;

    m_timer->stop();
    m_recording = false;
    m_paused = false;
    m_saving = true;
    updateButtons();

    if (!finishWriter())
    {
        const QString detail = QString::fromLocal8Bit(m_ffmpeg->readAllStandardError()).trimmed();
        QFile::remove(m_tempFile);
        m_tempFile.clear();
        m_saving = false;
        updateButtons();
        QMessageBox::warning(this, QStringLiteral("屏幕录制"),
            detail.isEmpty() ? QStringLiteral("保存视频失败。") : detail);
        return;
    }

    QTimer::singleShot(0, this, [this]() { saveAs(); });
}

/** @brief 全屏 ROI 选取。 */
void Record::selectRoi()
{
    if (m_recording || m_paused || m_saving)
        return;
    QScreen* screen = QGuiApplication::screenAt(frameGeometry().center());
    if (!screen)
        screen = QGuiApplication::primaryScreen();
    if (!screen)
        return;
    m_picker->setGeometry(screen->geometry());
    m_picker->show();
    m_picker->raise();
    m_picker->activateWindow();
}

/** @brief 退出录制工具。 */
void Record::exitRecord()
{
    if (m_saving)
        return;
    deinit();
    close();
}

/** @brief 启动 ffmpeg 子进程与帧定时器。 */
void Record::begin()
{
    if (m_roi.width() < kMinSize || m_roi.height() < kMinSize)
    {
        QMessageBox::warning(this, QStringLiteral("屏幕录制"), QStringLiteral("录制区域太小，请重新划定 ROI。"));
        return;
    }
    const QString ffmpeg = "./ffmpeg.exe";
    if (ffmpeg.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("屏幕录制"),
            QStringLiteral("未找到 ffmpeg，无法保存 MP4。"));
        return;
    }

    const bool covered = isVisible() && frameGeometry().intersects(m_roi);
    if (covered)
        hide();
    QImage first = capture();
    if (covered)
        show();
    if (first.isNull())
    {
        QMessageBox::warning(this, QStringLiteral("屏幕录制"), QStringLiteral("无法抓取屏幕。"));
        return;
    }

    m_size = QSize(first.width() & ~1, first.height() & ~1);
    if (m_size.width() < 2 || m_size.height() < 2)
    {
        QMessageBox::warning(this, QStringLiteral("屏幕录制"), QStringLiteral("录制区域太小。"));
        return;
    }
    first = first.copy(0, 0, m_size.width(), m_size.height()).convertToFormat(QImage::Format_RGB888);

    const QString dir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    m_tempFile = QDir(dir).filePath(QStringLiteral("screenrecord-%1.mp4").arg(QDateTime::currentMSecsSinceEpoch()));
    QStringList args;
    args << QStringLiteral("-hide_banner") << QStringLiteral("-loglevel") << QStringLiteral("error")
        << QStringLiteral("-y")
        << QStringLiteral("-f") << QStringLiteral("rawvideo")
        << QStringLiteral("-pixel_format") << QStringLiteral("rgb24")
        << QStringLiteral("-video_size") << QString("%1x%2").arg(m_size.width()).arg(m_size.height())
        << QStringLiteral("-framerate") << QString::number(kFps)
        << QStringLiteral("-i") << QStringLiteral("pipe:0")
        << QStringLiteral("-an")
        << QStringLiteral("-c:v") << QStringLiteral("libx264")
        << QStringLiteral("-preset") << QStringLiteral("veryfast")
        << QStringLiteral("-crf") << QStringLiteral("23")
        << QStringLiteral("-pix_fmt") << QStringLiteral("yuv420p")
        << QStringLiteral("-movflags") << QStringLiteral("+faststart")
        << m_tempFile;
    m_ffmpeg->start(ffmpeg, args);
    if (!m_ffmpeg->waitForStarted(5000))
    {
        m_tempFile.clear();
        QMessageBox::warning(this, QStringLiteral("屏幕录制"), QStringLiteral("无法启动 ffmpeg。"));
        return;
    }

    writeFrame(first);
    m_recording = true;
    m_paused = false;
    m_timer->start();
    updateButtons();
}

/** @brief 关闭管道并等待编码结束。 */
bool Record::finishWriter()
{
    if (m_ffmpeg->state() == QProcess::NotRunning)
        return QFileInfo::exists(m_tempFile) && QFileInfo(m_tempFile).size() > 0;

    m_ffmpeg->closeWriteChannel();
    if (!m_ffmpeg->waitForFinished(20000))
    {
        m_ffmpeg->kill();
        m_ffmpeg->waitForFinished(2000);
        return false;
    }
    return m_ffmpeg->exitStatus() == QProcess::NormalExit
        && m_ffmpeg->exitCode() == 0
        && QFileInfo(m_tempFile).size() > 0;
}

/** @brief 将临时 MP4 移到用户路径。 */
void Record::saveAs()
{
    const QString temp = m_tempFile;
    const QString suggested = QStringLiteral("./屏幕录制_%1.mp4").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));

    QString path = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("保存录制视频"),
        suggested,
        QStringLiteral("MP4 视频 (*.mp4)"),
        nullptr,
        QFileDialog::DontUseNativeDialog);
    if (!path.isEmpty())
    {
        if (!path.endsWith(QStringLiteral(".mp4"), Qt::CaseInsensitive))
            path += QStringLiteral(".mp4");
        QFile::remove(path);
        if (!QFile::copy(temp, path))
            QMessageBox::warning(this, QStringLiteral("屏幕录制"), QStringLiteral("保存 MP4 失败。"));
    }

    QFile::remove(temp);
    if (m_tempFile == temp)
        m_tempFile.clear();
    m_saving = false;
    updateButtons();
}

/** @brief 定时抓屏回调。 */
void Record::grabFrame()
{
    if (!m_recording || m_grabbing)
        return;
    m_grabbing = true;
    const bool covered = isVisible() && frameGeometry().intersects(m_roi);
    if (covered)
    {
        hide();
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    }
    const QImage image = capture();
    if (covered && m_recording)
        show();
    m_grabbing = false;
    if (!image.isNull())
        writeFrame(image);
}

/** @brief 裁剪 ROI 并写入 stdin。 */
void Record::writeFrame(const QImage& source)
{
    QImage image = source;
    if (image.size() != m_size)
        image = image.scaled(m_size, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    if (image.format() != QImage::Format_RGB888)
        image = image.convertToFormat(QImage::Format_RGB888);
    if (image.size() != m_size || m_ffmpeg->state() != QProcess::Running)
        return;

    const int rowBytes = m_size.width() * 3;
    m_buffer.resize(rowBytes * m_size.height());
    for (int y = 0; y < m_size.height(); ++y)
        std::memcpy(m_buffer.data() + y * rowBytes, image.constScanLine(y), size_t(rowBytes));
    m_ffmpeg->write(m_buffer);
}

/** @brief 截取当前屏幕图像。 */
QImage Record::capture() const
{
    QScreen* screen = QGuiApplication::screenAt(m_roi.center());
    if (!screen)
        screen = QGuiApplication::primaryScreen();
    if (!screen)
        return QImage();

    const QRect local = m_roi.translated(-screen->geometry().topLeft())
        .intersected(QRect(QPoint(0, 0), screen->geometry().size()));
    if (local.width() < 2 || local.height() < 2)
        return QImage();

    const QPixmap pixmap = screen->grabWindow(0, local.x(), local.y(), local.width(), local.height());
    return pixmap.isNull() ? QImage() : pixmap.toImage();
}

/** @brief 应用 ROI 矩形。 */
void Record::applyRoi(const QRect& roi)
{
    QRect rect = roi.normalized();
    QScreen* screen = QGuiApplication::screenAt(rect.center());
    if (!screen)
        screen = QGuiApplication::primaryScreen();
    if (screen)
        rect = rect.intersected(screen->geometry());
    if (rect.width() >= kMinSize && rect.height() >= kMinSize)
        m_roi = rect;
    show();
    raise();
    updateButtons();
}

/** @brief 取消 ROI。
void Record::cancelRoi()
{
    show();
    raise();
}

/** @brief 同步按钮使能状态。 */
void Record::updateButtons()
{
    const bool busy = m_recording || m_paused;
    ui->startRecordBtn->setText(busy ? QStringLiteral("停止") : QStringLiteral("开始"));
    ui->pauseRecordBtn->setText(m_paused ? QStringLiteral("继续") : QStringLiteral("暂停"));
    ui->pauseRecordBtn->setEnabled(busy && !m_saving);
    ui->roiRecordBtn->setEnabled(!busy && !m_saving);
    ui->startRecordBtn->setEnabled(!m_saving);
    ui->exitRecordBtn->setEnabled(!m_saving);
    ui->roiRecordBtn->setToolTip(m_roi.isValid()
        ? QString("ROI %1×%2").arg(m_roi.width()).arg(m_roi.height())
        : QStringLiteral("用鼠标划定录制区域"));
}

/** @brief 关闭窗口前安全停止录制。 */
void Record::closeEvent(QCloseEvent* event)
{
    if (m_saving)
    {
        event->ignore();
        return;
    }
    deinit();
    QDialog::closeEvent(event);
}