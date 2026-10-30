#include "MeltCamZoom.h"

#include <QDialog>
#include <QGuiApplication>
#include <QHash>
#include <QLabel>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPixmap>
#include <QPointer>
#include <QScreen>
#include <QShowEvent>
#include <QThread>
#include <QVBoxLayout>
#include <QtGlobal>

namespace {

class LabelZoomState;

QHash<QLabel *, LabelZoomState *> &zoomStates()
{
    // 故意不在进程退出时析构，避免标签析构顺序和函数内静态对象打架。
    static auto *table = new QHash<QLabel *, LabelZoomState *>();
    return *table;
}

QScreen *screenFor(QWidget *widget)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    if (widget != nullptr && widget->screen() != nullptr)
        return widget->screen();
#else
    Q_UNUSED(widget);
#endif
    return QGuiApplication::primaryScreen();
}

QSize availableSize(QWidget *widget)
{
    QSize avail(1280, 720);
    if (QScreen *screen = screenFor(widget)) {
        const QSize got = screen->availableGeometry().size();
        if (!got.isEmpty())
            avail = got;
    }
    return avail;
}

QPixmap scaledPixmap(const QImage &image, const QSize &targetDips, qreal dpr, Qt::TransformationMode mode)
{
    if (image.isNull() || targetDips.width() <= 0 || targetDips.height() <= 0)
        return {};

    const qreal ratio = dpr > 0.0 ? dpr : 1.0;
    const QSize pixels(qMax(1, qRound(targetDips.width() * ratio)),
                       qMax(1, qRound(targetDips.height() * ratio)));
    QPixmap pixmap = QPixmap::fromImage(image.scaled(pixels, Qt::KeepAspectRatio, mode));
    pixmap.setDevicePixelRatio(ratio);
    return pixmap;
}

QSize chooseZoomSize(const QImage &image, const QSize &labelSize, const QSize &avail)
{
    const QSize cap(qMax(1, avail.width() * 9 / 10), qMax(1, avail.height() * 9 / 10));
    QSize desired = image.size();
    if (desired.isEmpty())
        desired = QSize(960, 720);
    desired = desired.boundedTo(cap);

    const QSize floorSize(qMin(cap.width(), qMax(labelSize.width() * 2, 480)),
                          qMin(cap.height(), qMax(labelSize.height() * 2, 360)));
    if (desired.width() < floorSize.width() || desired.height() < floorSize.height())
        desired = floorSize;
    return desired.boundedTo(cap);
}

// 放大窗口里的图。sizeHint 固定，避免原图尺寸把窗口撑满屏幕。
class ZoomView final : public QLabel
{
public:
    explicit ZoomView(QWidget *parent)
        : QLabel(parent)
    {
        setAlignment(Qt::AlignCenter);
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        setMinimumSize(0, 0);
        setText(QStringLiteral("无图像"));
    }

    QSize sizeHint() const override
    {
        return QSize(320, 240);
    }

    QSize minimumSizeHint() const override
    {
        return QSize(0, 0);
    }
};

class ImageZoomWindow final : public QDialog
{
public:
    explicit ImageZoomWindow(QWidget *parent)
        : QDialog(parent, Qt::Dialog | Qt::WindowCloseButtonHint | Qt::WindowMinMaxButtonsHint)
        , m_view(new ZoomView(this))
    {
        setAttribute(Qt::WA_DeleteOnClose);
        setAttribute(Qt::WA_QuitOnClose, false);
        setModal(false);
        setObjectName(QStringLiteral("imageZoomWindow"));
        setStyleSheet(QStringLiteral(
            "QDialog { background-color: #101010; }"
            "QLabel { background-color: #101010; color: #d0d0d0; }"));

        auto *layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(m_view);
        m_view->installEventFilter(this);

        const QSize avail = availableSize(parent);
        setMinimumSize(QSize(320, 240).boundedTo(avail));
    }

    void showImage(const QImage &image, const QSize &labelSize, QWidget *anchor)
    {
        m_image = image;
        m_fit = QSize();
        resize(chooseZoomSize(image, labelSize, availableSize(anchor)));
        if (QScreen *screen = screenFor(anchor)) {
            const QRect area = screen->availableGeometry();
            move(area.topLeft() + QPoint((area.width() - width()) / 2,
                                         (area.height() - height()) / 2));
        }
        show();
        raise();
        activateWindow();
        refit();
    }

    void setImage(const QImage &image)
    {
        m_image = image;
        m_fit = QSize();
        refit();
    }

protected:
    bool eventFilter(QObject *watched, QEvent *event) override
    {
        if (watched != m_view)
            return QDialog::eventFilter(watched, event);

        if (event->type() == QEvent::Resize) {
            refit();
        } else if (event->type() == QEvent::MouseButtonPress) {
            auto *mouse = static_cast<QMouseEvent *>(event);
            if (mouse->button() == Qt::LeftButton)
                mouse->accept();
        } else if (event->type() == QEvent::MouseButtonDblClick) {
            auto *mouse = static_cast<QMouseEvent *>(event);
            if (mouse->button() == Qt::LeftButton) {
                close();
                return true;
            }
        }
        return QDialog::eventFilter(watched, event);
    }

    void showEvent(QShowEvent *event) override
    {
        QDialog::showEvent(event);
        refit();
    }

private:
    void refit()
    {
        if (m_refitting)
            return;
        m_refitting = true;

        const QSize target = m_view->contentsRect().size();
        if (m_image.isNull()) {
            m_view->setPixmap(QPixmap());
            m_view->setText(QStringLiteral("无图像"));
            m_fit = QSize();
        } else if (!target.isEmpty() && target != m_fit) {
            m_view->setText(QString());
            m_view->setPixmap(scaledPixmap(m_image, target, m_view->devicePixelRatioF(),
                                           Qt::SmoothTransformation));
            m_fit = target;
        }

        m_refitting = false;
    }

    ZoomView *m_view = nullptr;
    QImage m_image;
    QSize m_fit;
    bool m_refitting = false;
};

class LabelZoomState final : public QObject
{
public:
    explicit LabelZoomState(QLabel *label)
        : QObject(label)
        , m_label(label)
    {
        zoomStates().insert(m_label, this);
        m_label->installEventFilter(this);
        m_label->setAlignment(Qt::AlignCenter);
        m_label->setScaledContents(false);
        if (m_label->testAttribute(Qt::WA_TransparentForMouseEvents))
            m_label->setAttribute(Qt::WA_TransparentForMouseEvents, false);
        if (m_label->cursor().shape() == Qt::ArrowCursor)
            m_label->setCursor(Qt::PointingHandCursor);
        if (m_label->toolTip().isEmpty())
            m_label->setToolTip(QStringLiteral("双击放大"));
    }

    ~LabelZoomState() override
    {
        zoomStates().remove(m_label);
        if (!m_zoom)
            return;
        QWidget *zoom = m_zoom.data();
        disconnect(zoom, nullptr, this, nullptr);
        m_zoom = nullptr;
        zoom->deleteLater();
    }

    void setImage(const QImage &image, const QString &zoomTitle)
    {
        if (zoomTitle != m_title) {
            m_title = zoomTitle;
            if (m_zoom)
                m_zoom->setWindowTitle(m_title);
        }
        if (!image.isNull() && image.cacheKey() == m_image.cacheKey())
            return;
        m_image = image;
        m_labelFit = QSize();
        applyToLabel();
        if (m_zoom)
            m_zoom->setImage(m_image);
    }

protected:
    bool eventFilter(QObject *watched, QEvent *event) override
    {
        if (watched != m_label)
            return QObject::eventFilter(watched, event);

        if (event->type() == QEvent::MouseButtonPress) {
            auto *mouse = static_cast<QMouseEvent *>(event);
            if (mouse->button() == Qt::LeftButton)
                mouse->accept();
        } else if (event->type() == QEvent::MouseButtonDblClick) {
            auto *mouse = static_cast<QMouseEvent *>(event);
            if (mouse->button() == Qt::LeftButton) {
                openZoom();
                return true;
            }
        } else if (event->type() == QEvent::Resize) {
            applyToLabel();
        }
        return QObject::eventFilter(watched, event);
    }

private:
    void applyToLabel()
    {
        if (m_image.isNull()) {
            m_label->setPixmap(QPixmap());
            m_labelFit = QSize();
            return;
        }

        const QSize target = m_label->contentsRect().size();
        if (target.isEmpty() || target == m_labelFit)
            return;
        // 小图用快速缩放，连续帧还能跟上界面刷新。
        m_label->setPixmap(scaledPixmap(m_image, target, m_label->devicePixelRatioF(),
                                        Qt::FastTransformation));
        m_labelFit = target;
    }

    void restoreLabel()
    {
        m_labelFit = QSize();
        applyToLabel();
        if (QWidget *host = m_label->window()) {
            if (host->isVisible())
                host->activateWindow();
        }
    }

    void openZoom()
    {
        if (m_image.isNull())
            return;
        if (m_zoom) {
            m_zoom->raise();
            m_zoom->activateWindow();
            m_zoom->setImage(m_image);
            return;
        }

        auto *zoom = new ImageZoomWindow(m_label->window());
        zoom->setWindowTitle(m_title);
        m_zoom = zoom;
        connect(zoom, &QObject::destroyed, this, [this]() {
            restoreLabel();
        });
        zoom->showImage(m_image, m_label->size(), m_label);
    }

    QLabel *m_label = nullptr;
    QImage m_image;
    QString m_title = QStringLiteral("图像");
    QSize m_labelFit;
    QPointer<ImageZoomWindow> m_zoom;
};

QString resolvedZoomTitle(QLabel *label, const QString &zoomTitle)
{
    if (!zoomTitle.isEmpty())
        return zoomTitle;
    if (!label->objectName().isEmpty())
        return label->objectName();
    return QStringLiteral("图像");
}

} // namespace

void setZoomableImage(QLabel *label, const QImage &image, const QString &zoomTitle)
{
    if (label == nullptr)
        return;

    if (QThread::currentThread() != label->thread()) {
        QImage copy(image);
        copy.detach();
        const QString title = zoomTitle;
        const bool posted = QMetaObject::invokeMethod(label, [label, copy, title]() {
            setZoomableImage(label, copy, title);
        }, Qt::QueuedConnection);
        if (!posted)
            qWarning("setZoomableImage: 无法把图像投递到标签所在的界面线程");
        return;
    }

    LabelZoomState *state = zoomStates().value(label, nullptr);
    if (state == nullptr)
        state = new LabelZoomState(label);
    state->setImage(image, resolvedZoomTitle(label, zoomTitle));
}

void setMeltCamImage(QLabel *meltCamLabel, const QImage &image)
{
    setZoomableImage(meltCamLabel, image, QStringLiteral("熔池相机"));
}
