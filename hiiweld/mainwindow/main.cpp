#include <QApplication>
#include <QDesktopWidget>
#include "MainWindow.h"
#include <vtkOutputWindow.h>
#include <QSplashScreen>
#include <QSharedMemory>
#include <QScreen>
#include <QPainter>
#include <QLinearGradient>
#include <QLabel>
#include <QProgressBar>
#include <QEventLoop>
#include <QTimer>
#include <QMessageBox>
#include <QGuiApplication>

#include <glog/logging.h>
#include "utils/GlogDailyRollover.h"

/** @brief MainWindow 单例指针的全局存储，由构造/析构维护。 */
MainWindow* MainWindow::singleton = nullptr;

/**
 * @brief 启动阶段事件过滤器，禁止用户在加载完成前操作界面。
 */
class EventFilter : public QObject {
public:
    /**
     * @brief 过滤鼠标相关事件。
     * @param obj 事件目标对象。
     * @param event 待处理事件。
     * @return 鼠标类事件返回 true（已处理）；否则委托基类。
     */
    bool eventFilter(QObject* obj, QEvent* event) override {
        switch (event->type()) {
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::MouseMove:
        case QEvent::Wheel:
            return true; // 屏蔽鼠标事件
        default:
            return QObject::eventFilter(obj, event);
        }
    }
};

/**
 * @brief 应用程序入口。
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return 事件循环退出码；若已有实例运行则直接返回 0。
 */
int main(int argc, char* argv[])
{
    QSharedMemory memory("HSIWELDSYSTEM");
    if (!memory.create(1))
    {
        return 0;
    }

    QDir().mkpath(QStringLiteral("./logs"));

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./logs";
    FLAGS_alsologtostderr = false;
    google::SetLogFilenameExtension(".txt");

    // 午夜切目录；logs 下删除早于 14 天的 yyyy-MM-dd 文件夹
    GlogDailyRollover::instance().start(14);



    vtkOutputWindow::SetGlobalWarningDisplay(0);

    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<rl::plan::VectorList>("rl::plan::VectorList");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<ct::Cloud::Ptr>("ct::Cloud::Ptr");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<TopoDS_Shape>("TopoDS_Shape");
    qRegisterMetaType<std::vector<TopoDS_Edge>>("std::vector<TopoDS_Edge>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
    qRegisterMetaType<IKReturnHomeParams>("IKReturnHomeParams");
    qRegisterMetaType<DiscretePoint>("DiscretePoint");
    qRegisterMetaType<IKGoToStartParams>("IKGoToStartParams");
    qRegisterMetaType<Eigen::Affine3f>("Eigen::Affine3f");
    qRegisterMetaType<PlanToPreWeldParams>("PlanToPreWeldParams");

    QApplication a(argc, argv);
    EventFilter filter;
    a.installEventFilter(&filter);

    constexpr int W = 1080, H = 540, CX = W / 2;
    QPixmap canvas(W, H);
    {
        QPainter p(&canvas);
        p.setRenderHint(QPainter::Antialiasing);
        QLinearGradient bg(0, 0, 0, H);
        bg.setColorAt(0, QColor("#0b121c"));
        bg.setColorAt(1, QColor("#111b2a"));
        p.fillRect(canvas.rect(), bg);

        const int m = 18, len = 22;
        p.setPen(QPen(QColor(0, 112, 210, 140), 1.5));
        p.drawLine(m, m, m + len, m);                 p.drawLine(m, m, m, m + len);
        p.drawLine(W - m - len, m, W - m, m);         p.drawLine(W - m, m, W - m, m + len);
        p.drawLine(m, H - m, m + len, H - m);         p.drawLine(m, H - m - len, m, H - m);
        p.drawLine(W - m - len, H - m, W - m, H - m); p.drawLine(W - m, H - m - len, W - m, H - m);

        QLinearGradient line(300, 0, W - 300, 0);
        line.setColorAt(0, QColor(0, 112, 210, 0));
        line.setColorAt(0.5, QColor(0, 112, 210, 120));
        line.setColorAt(1, QColor(0, 112, 210, 0));
        p.setPen(QPen(QBrush(line), 1));
        p.drawLine(300, 318, W - 300, 318);
    }

    QSplashScreen* splash = new QSplashScreen(canvas);
    splash->setWindowFlags(splash->windowFlags() | Qt::WindowStaysOnTopHint | Qt::FramelessWindowHint);
    splash->setFixedSize(W, H);

    QLabel* logo = new QLabel(splash);
    QPixmap logoPm(QStringLiteral("./icon/preview.png"));
    if (!logoPm.isNull())
        logoPm = logoPm.scaled(860, 860 * 468 / 1490, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    logo->setPixmap(logoPm);
    logo->setAttribute(Qt::WA_TranslucentBackground);
    logo->adjustSize();
    logo->move(CX - logo->width() / 2, 72);

    QLabel* version = new QLabel(QStringLiteral("Version 1.0.0"), splash);
    version->setStyleSheet(QStringLiteral(
        "QLabel{color:#8fa3b8;font-size:20px;letter-spacing:4px;background:transparent;}"));
    version->adjustSize();
    version->move(CX - version->width() / 2, 335);

    QLabel* loading = new QLabel(QString::fromUtf8(u8"正在启动..."), splash);
    loading->setStyleSheet(QStringLiteral(
        "QLabel{color:#c5d4e3;font-size:20px;background:transparent;}"));
    loading->adjustSize();

    QLabel* percent = new QLabel(QStringLiteral("0%"), splash);
    percent->setStyleSheet(QStringLiteral(
        "QLabel{color:#4aa3e8;font-size:20px;font-weight:600;background:transparent;}"));
    percent->adjustSize();

    QProgressBar* progress = new QProgressBar(splash);
    progress->setRange(0, 100);
    progress->setTextVisible(false);
    progress->setFixedSize(680, 4);
    progress->setStyleSheet(QStringLiteral(
        "QProgressBar{border:none;background:rgba(255,255,255,18);}"
        "QProgressBar::chunk{background:#0070D2;}"));
    progress->move(CX - 340, 455);
    loading->move(CX - 340, 418);
    percent->move(CX + 340 - percent->width(), 418);

    splash->show();
    splash->raise();
    if (QScreen* screen = QGuiApplication::primaryScreen())
        splash->move(screen->geometry().center() - splash->rect().center());

    const QString stages[] = {
        QString::fromUtf8(u8"初始化 UI 界面..."),
        QString::fromUtf8(u8"初始化仿真引擎..."),
        QString::fromUtf8(u8"加载场景模型..."),
        QString::fromUtf8(u8"准备渲染资源..."),
        QString::fromUtf8(u8"正在启动用户界面..."),
        QString::fromUtf8(u8"即将完成，请稍候...")
    };
    const int thresholds[] = { 0, 15, 35, 55, 80, 95 };

    QEventLoop loop;
    QTimer timeoutTimer;
    QTimer progressTimer;
    int progressValue = 0;

    QObject::connect(&progressTimer, &QTimer::timeout, [&]() {
        progressValue = qMin(100, progressValue + (progressValue < 25 ? 6 : progressValue < 90 ? 3 : 2));
        progress->setValue(progressValue);

        int i = 5;
        while (i > 0 && progressValue < thresholds[i]) --i;
        loading->setText(stages[i]);
        loading->adjustSize();
        loading->move(CX - 340, 418);

        percent->setText(QStringLiteral("%1%").arg(progressValue));
        percent->adjustSize();
        percent->move(CX + 340 - percent->width(), 418);

        if (progressValue >= 100) {
            progressTimer.stop();
            timeoutTimer.stop();
            loop.quit();
        }
        });

    progressTimer.start(100);
    timeoutTimer.setSingleShot(true);
    QObject::connect(&timeoutTimer, &QTimer::timeout, [&]() {
        progressTimer.stop();
        loop.quit();
        });
    timeoutTimer.start(10000);

    MainWindow w;
    loop.exec();
    splash->finish(&w);
    delete splash;
    a.removeEventFilter(&filter);
    w.showMaximized();
    const int ret = a.exec();
    google::ShutdownGoogleLogging();
    return ret;
}

