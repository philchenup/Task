#include "MeltCamZoom.h"

#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QThread>
#include <QPixmap>
#include <QVBoxLayout>
#include <QtTest>

namespace {

QWidget *findZoomWindow(const QString &title = QString())
{
    const QWidgetList tops = QApplication::topLevelWidgets();
    for (QWidget *widget : tops) {
        if (widget->objectName() != QLatin1String("imageZoomWindow") || !widget->isVisible())
            continue;
        if (title.isEmpty() || widget->windowTitle() == title)
            return widget;
    }
    return nullptr;
}

int visibleZoomCount()
{
    int count = 0;
    const QWidgetList tops = QApplication::topLevelWidgets();
    for (QWidget *widget : tops) {
        if (widget->objectName() == QLatin1String("imageZoomWindow") && widget->isVisible())
            ++count;
    }
    return count;
}

int dipWidth(const QPixmap &pixmap)
{
    if (pixmap.isNull())
        return 0;
    return qRound(pixmap.deviceIndependentSize().width());
}

QLabel *zoomView(QWidget *zoom)
{
    return zoom == nullptr ? nullptr : zoom->findChild<QLabel *>();
}

} // namespace

class MeltCamZoomTest : public QObject
{
    Q_OBJECT

private slots:
    void doubleClickOpensZoomAndCloseRestores();
    void secondDoubleClickReusesTheSameWindow();
    void emptyImageDoesNotOpenZoom();
    void nullLabelIsIgnored();
    void frameFromWorkerThreadReachesLabel();
    void deletingHostRemovesZoom();
    void twoLabelsZoomIndependently();
    void titleFallsBackToObjectName();
};

void MeltCamZoomTest::doubleClickOpensZoomAndCloseRestores()
{
    QWidget window;
    auto *layout = new QVBoxLayout(&window);
    auto *label = new QLabel(&window);
    label->setObjectName(QStringLiteral("meltCamLabel"));
    label->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    label->setFixedSize(160, 120);
    layout->addWidget(label);
    window.show();
    QApplication::processEvents();

    QImage green(800, 600, QImage::Format_RGB32);
    green.fill(Qt::green);
    setMeltCamImage(label, green);

    const QPixmap thumb = label->pixmap();
    QVERIFY(!thumb.isNull());
    QVERIFY(dipWidth(thumb) > 0);
    QVERIFY(dipWidth(thumb) <= label->width());
    QCOMPARE(thumb.toImage().pixelColor(thumb.width() / 2, thumb.height() / 2), QColor(Qt::green));
    QCOMPARE(label->toolTip(), QStringLiteral("双击放大"));
    QVERIFY(!label->testAttribute(Qt::WA_TransparentForMouseEvents));

    QTest::mouseDClick(label, Qt::LeftButton);
    QApplication::processEvents();

    QWidget *zoom = findZoomWindow();
    QVERIFY(zoom != nullptr);
    QCOMPARE(zoom->windowTitle(), QStringLiteral("熔池相机"));
    QLabel *view = zoomView(zoom);
    QVERIFY(view != nullptr);
    const QPixmap enlarged = view->pixmap();
    QVERIFY(dipWidth(enlarged) > dipWidth(label->pixmap()));
    QCOMPARE(enlarged.toImage().pixelColor(enlarged.width() / 2, enlarged.height() / 2), QColor(Qt::green));

    QImage blue(800, 600, QImage::Format_RGB32);
    blue.fill(Qt::blue);
    setMeltCamImage(label, blue);
    QApplication::processEvents();

    QVERIFY(dipWidth(label->pixmap()) <= label->width());
    QCOMPARE(label->pixmap().toImage().pixelColor(label->pixmap().width() / 2, label->pixmap().height() / 2),
             QColor(Qt::blue));
    const QPixmap enlargedBlue = view->pixmap();
    QCOMPARE(enlargedBlue.toImage().pixelColor(enlargedBlue.width() / 2, enlargedBlue.height() / 2),
             QColor(Qt::blue));

    zoom->close();
    QTRY_VERIFY(findZoomWindow() == nullptr);
    QVERIFY(!label->pixmap().isNull());
    QVERIFY(dipWidth(label->pixmap()) <= label->width());
    QCOMPARE(label->pixmap().toImage().pixelColor(label->pixmap().width() / 2, label->pixmap().height() / 2),
             QColor(Qt::blue));
}

void MeltCamZoomTest::secondDoubleClickReusesTheSameWindow()
{
    QWidget window;
    auto *label = new QLabel(&window);
    label->setObjectName(QStringLiteral("meltCamLabel"));
    label->setFixedSize(160, 120);
    window.show();
    QApplication::processEvents();

    QImage image(640, 480, QImage::Format_RGB32);
    image.fill(Qt::white);
    setMeltCamImage(label, image);

    QTest::mouseDClick(label, Qt::LeftButton);
    QApplication::processEvents();
    QWidget *first = findZoomWindow();
    QVERIFY(first != nullptr);

    QTest::mouseDClick(label, Qt::LeftButton);
    QApplication::processEvents();
    QCOMPARE(findZoomWindow(), first);

    QCOMPARE(visibleZoomCount(), 1);

    QLabel *view = zoomView(first);
    QVERIFY(view != nullptr);
    QTest::mouseDClick(view, Qt::LeftButton);
    QTRY_VERIFY(findZoomWindow() == nullptr);
    QVERIFY(dipWidth(label->pixmap()) <= label->width());
}

void MeltCamZoomTest::emptyImageDoesNotOpenZoom()
{
    QWidget window;
    auto *label = new QLabel(&window);
    label->setObjectName(QStringLiteral("meltCamLabel"));
    label->setFixedSize(160, 120);
    window.show();
    setMeltCamImage(label, QImage());
    QVERIFY(label->pixmap().isNull());

    QTest::mouseDClick(label, Qt::LeftButton);
    QApplication::processEvents();
    QVERIFY(findZoomWindow() == nullptr);
}

void MeltCamZoomTest::nullLabelIsIgnored()
{
    QImage image(8, 8, QImage::Format_RGB32);
    image.fill(Qt::black);
    setMeltCamImage(nullptr, image);
}

void MeltCamZoomTest::frameFromWorkerThreadReachesLabel()
{
    QLabel label;
    label.setObjectName(QStringLiteral("meltCamLabel"));
    label.setFixedSize(160, 120);
    label.show();

    QImage red(80, 60, QImage::Format_RGB32);
    red.fill(Qt::red);

    QThread *worker = QThread::create([&label, red]() {
        setMeltCamImage(&label, red);
    });
    worker->start();
    QVERIFY(worker->wait(5000));
    delete worker;

    QTRY_VERIFY(!label.pixmap().isNull());
    const QImage shown = label.pixmap().toImage();
    QCOMPARE(shown.pixelColor(shown.width() / 2, shown.height() / 2), QColor(Qt::red));

    QTest::mouseDClick(&label, Qt::LeftButton);
    QApplication::processEvents();
    QWidget *zoom = findZoomWindow(QStringLiteral("熔池相机"));
    QVERIFY(zoom != nullptr);
    zoom->close();
    QTRY_VERIFY(findZoomWindow() == nullptr);
}

void MeltCamZoomTest::deletingHostRemovesZoom()
{
    auto *window = new QWidget;
    auto *label = new QLabel(window);
    label->setObjectName(QStringLiteral("meltCamLabel"));
    label->setFixedSize(160, 120);
    window->show();
    QApplication::processEvents();

    QImage image(64, 48, QImage::Format_RGB32);
    image.fill(Qt::white);
    setMeltCamImage(label, image);
    QTest::mouseDClick(label, Qt::LeftButton);
    QApplication::processEvents();
    QVERIFY(findZoomWindow() != nullptr);

    delete window;
    QTRY_VERIFY(findZoomWindow() == nullptr);
}

void MeltCamZoomTest::twoLabelsZoomIndependently()
{
    QWidget window;
    auto *layout = new QHBoxLayout(&window);
    auto *melt = new QLabel(&window);
    auto *weld = new QLabel(&window);
    melt->setObjectName(QStringLiteral("meltCamLabel"));
    weld->setObjectName(QStringLiteral("weldCamLabel"));
    melt->setFixedSize(160, 120);
    weld->setFixedSize(160, 120);
    layout->addWidget(melt);
    layout->addWidget(weld);
    window.show();
    QApplication::processEvents();

    QImage green(800, 600, QImage::Format_RGB32);
    green.fill(Qt::green);
    QImage red(800, 600, QImage::Format_RGB32);
    red.fill(Qt::red);
    setMeltCamImage(melt, green);
    setZoomableImage(weld, red, QStringLiteral("焊缝相机"));

    QTest::mouseDClick(melt, Qt::LeftButton);
    QTest::mouseDClick(weld, Qt::LeftButton);
    QApplication::processEvents();

    QWidget *meltZoom = findZoomWindow(QStringLiteral("熔池相机"));
    QWidget *weldZoom = findZoomWindow(QStringLiteral("焊缝相机"));
    QVERIFY(meltZoom != nullptr);
    QVERIFY(weldZoom != nullptr);
    QVERIFY(meltZoom != weldZoom);
    QCOMPARE(visibleZoomCount(), 2);

    const QPixmap meltView = zoomView(meltZoom)->pixmap();
    const QPixmap weldView = zoomView(weldZoom)->pixmap();
    QCOMPARE(meltView.toImage().pixelColor(meltView.width() / 2, meltView.height() / 2), QColor(Qt::green));
    QCOMPARE(weldView.toImage().pixelColor(weldView.width() / 2, weldView.height() / 2), QColor(Qt::red));
    QVERIFY(dipWidth(meltView) > dipWidth(melt->pixmap()));
    QVERIFY(dipWidth(weldView) > dipWidth(weld->pixmap()));

    QImage yellow(800, 600, QImage::Format_RGB32);
    yellow.fill(Qt::yellow);
    setZoomableImage(weld, yellow, QStringLiteral("焊缝相机"));
    const QPixmap weldUpdated = zoomView(weldZoom)->pixmap();
    QCOMPARE(weldUpdated.toImage().pixelColor(weldUpdated.width() / 2, weldUpdated.height() / 2),
             QColor(Qt::yellow));
    const QPixmap meltStill = zoomView(meltZoom)->pixmap();
    QCOMPARE(meltStill.toImage().pixelColor(meltStill.width() / 2, meltStill.height() / 2), QColor(Qt::green));

    weldZoom->close();
    QTRY_VERIFY(findZoomWindow(QStringLiteral("焊缝相机")) == nullptr);
    QVERIFY(findZoomWindow(QStringLiteral("熔池相机")) == meltZoom);
    QVERIFY(dipWidth(weld->pixmap()) <= weld->width());
    QCOMPARE(weld->pixmap().toImage().pixelColor(weld->pixmap().width() / 2, weld->pixmap().height() / 2),
             QColor(Qt::yellow));
    QCOMPARE(melt->pixmap().toImage().pixelColor(melt->pixmap().width() / 2, melt->pixmap().height() / 2),
             QColor(Qt::green));

    meltZoom->close();
    QTRY_VERIFY(findZoomWindow() == nullptr);
}

void MeltCamZoomTest::titleFallsBackToObjectName()
{
    QWidget window;
    auto *named = new QLabel(&window);
    auto *untitled = new QLabel(&window);
    named->setObjectName(QStringLiteral("wireCamLabel"));
    named->setFixedSize(160, 120);
    untitled->setFixedSize(160, 120);
    window.show();
    QApplication::processEvents();

    QImage image(320, 240, QImage::Format_RGB32);
    image.fill(Qt::cyan);
    setZoomableImage(named, image);
    setZoomableImage(untitled, image);

    QTest::mouseDClick(named, Qt::LeftButton);
    QTest::mouseDClick(untitled, Qt::LeftButton);
    QApplication::processEvents();

    QVERIFY(findZoomWindow(QStringLiteral("wireCamLabel")) != nullptr);
    QVERIFY(findZoomWindow(QStringLiteral("图像")) != nullptr);

    setZoomableImage(named, image, QStringLiteral("送丝相机"));
    QVERIFY(findZoomWindow(QStringLiteral("送丝相机")) != nullptr);
    QVERIFY(findZoomWindow(QStringLiteral("wireCamLabel")) == nullptr);

    findZoomWindow(QStringLiteral("送丝相机"))->close();
    findZoomWindow(QStringLiteral("图像"))->close();
    QTRY_VERIFY(visibleZoomCount() == 0);
}

QTEST_MAIN(MeltCamZoomTest)
#include "meltcam_zoom_test.moc"
