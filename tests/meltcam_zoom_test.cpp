#include "MeltCamZoom.h"

#include <QApplication>
#include <QLabel>
#include <QThread>
#include <QPixmap>
#include <QVBoxLayout>
#include <QtTest>

namespace {

QWidget *findZoomWindow()
{
    const QWidgetList tops = QApplication::topLevelWidgets();
    for (QWidget *widget : tops) {
        if (widget->objectName() == QLatin1String("meltCamZoomWindow") && widget->isVisible())
            return widget;
    }
    return nullptr;
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

    int zoomCount = 0;
    const QWidgetList tops = QApplication::topLevelWidgets();
    for (QWidget *widget : tops) {
        if (widget->objectName() == QLatin1String("meltCamZoomWindow") && widget->isVisible())
            ++zoomCount;
    }
    QCOMPARE(zoomCount, 1);

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

QTEST_MAIN(MeltCamZoomTest)
#include "meltcam_zoom_test.moc"
