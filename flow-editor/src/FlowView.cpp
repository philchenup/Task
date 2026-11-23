#include "FlowView.h"

#include "FlowScene.h"
#include "Types.h"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QGraphicsItem>
#include <QKeyEvent>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollBar>
#include <QWheelEvent>

FlowView::FlowView(FlowScene *scene, QWidget *parent)
    : QGraphicsView(scene, parent)
    , m_scene(scene)
{
    setAcceptDrops(true);
    viewport()->setAcceptDrops(true);
    viewport()->installEventFilter(this);
    setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setDragMode(QGraphicsView::RubberBandDrag);
    setBackgroundBrush(QColor("#1c1e20"));
    setFrameShape(QFrame::NoFrame);
    setFocusPolicy(Qt::StrongFocus);
}

QPointF FlowView::centerScenePos() const
{
    return mapToScene(viewport()->rect().center());
}

void FlowView::frameItems()
{
    const QRectF bounds = m_scene->nodesBoundingRect();
    if (bounds.isNull())
        centerOn(0, 0);
    else
        centerOn(bounds.center());
}

bool FlowView::acceptStepDrag(QDropEvent *event) const
{
    return event->mimeData() && event->mimeData()->hasFormat(QLatin1String(kStepMime));
}

void FlowView::dragEnterEvent(QDragEnterEvent *event)
{
    if (acceptStepDrag(event))
        event->acceptProposedAction();
    else
        event->ignore();
}

void FlowView::dragMoveEvent(QDragMoveEvent *event)
{
    if (acceptStepDrag(event))
        event->acceptProposedAction();
    else
        event->ignore();
}

void FlowView::dropEvent(QDropEvent *event)
{
    if (!acceptStepDrag(event)) {
        event->ignore();
        return;
    }
    const QString typeId = QString::fromUtf8(event->mimeData()->data(QLatin1String(kStepMime))).trimmed();
    m_scene->addNode(typeId, mapToScene(event->pos()));
    event->acceptProposedAction();
    setFocus();
}

bool FlowView::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == viewport()) {
        switch (event->type()) {
        case QEvent::DragEnter:
            dragEnterEvent(static_cast<QDragEnterEvent *>(event));
            return true;
        case QEvent::DragMove:
            dragMoveEvent(static_cast<QDragMoveEvent *>(event));
            return true;
        case QEvent::Drop:
            dropEvent(static_cast<QDropEvent *>(event));
            return true;
        default:
            break;
        }
    }
    return QGraphicsView::eventFilter(watched, event);
}

void FlowView::wheelEvent(QWheelEvent *event)
{
    const QPoint delta = event->angleDelta();
    const int steps = delta.y() != 0 ? delta.y() : delta.x();
    if (steps == 0) {
        event->ignore();
        return;
    }
    const qreal factor = steps > 0 ? 1.12 : 1.0 / 1.12;
    const qreal next = transform().m11() * factor;
    if (next < 0.35 || next > 2.4) {
        event->accept();
        return;
    }
    scale(factor, factor);
    emit zoomChanged(transform().m11());
    event->accept();
}

void FlowView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::MiddleButton || (event->button() == Qt::LeftButton && event->modifiers() & Qt::AltModifier)) {
        m_panning = true;
        m_panAnchor = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QGraphicsView::mousePressEvent(event);
}

void FlowView::mouseMoveEvent(QMouseEvent *event)
{
    if (m_panning) {
        const QPoint delta = event->pos() - m_panAnchor;
        m_panAnchor = event->pos();
        horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
        verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
        event->accept();
        return;
    }
    QGraphicsView::mouseMoveEvent(event);
}

void FlowView::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_panning && (event->button() == Qt::MiddleButton || event->button() == Qt::LeftButton)) {
        m_panning = false;
        unsetCursor();
        event->accept();
        return;
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void FlowView::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
        m_scene->deleteSelection();
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_Escape) {
        m_scene->cancelDraft();
        event->accept();
        return;
    }
    if (event->modifiers() == Qt::ControlModifier && event->key() == Qt::Key_0) {
        resetTransform();
        emit zoomChanged(1.0);
        event->accept();
        return;
    }
    if (event->matches(QKeySequence::SelectAll)) {
        for (QGraphicsItem *item : m_scene->items()) {
            if (item->flags() & QGraphicsItem::ItemIsSelectable)
                item->setSelected(true);
        }
        event->accept();
        return;
    }
    QGraphicsView::keyPressEvent(event);
}

void FlowView::paintEvent(QPaintEvent *event)
{
    QGraphicsView::paintEvent(event);
    if (!m_scene->nodes().isEmpty())
        return;
    QPainter painter(viewport());
    painter.setPen(QColor("#8b939a"));
    painter.setFont(uiFont(14));
    painter.drawText(viewport()->rect().adjusted(24, 24, -24, -24),
                     Qt::AlignCenter,
                     QStringLiteral("从左侧步骤库把步骤拖到画布\n按住输出端口，拖到输入端口完成连线"));
}
