#include "EdgeItem.h"

#include "FlowScene.h"
#include "PortItem.h"

#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QPainter>
#include <QPainterPathStroker>
#include <QTimer>
#include <QtMath>

EdgeItem::EdgeItem()
{
    setZValue(-1);
    setTemporary(true);
}

EdgeItem::EdgeItem(PortItem *from, PortItem *to)
    : m_from(from)
    , m_to(to)
{
    setZValue(-1);
    setFlag(ItemIsSelectable);
    setAcceptHoverEvents(true);
    if (m_from)
        m_from->addEdge(this);
    if (m_to)
        m_to->addEdge(this);
    updatePen();
    updatePath();
}

EdgeItem::~EdgeItem()
{
    detach();
}

QRectF EdgeItem::boundingRect() const
{
    return QGraphicsPathItem::boundingRect().adjusted(-14, -14, 14, 14);
}

QPainterPath EdgeItem::shape() const
{
    QPainterPathStroker stroker;
    stroker.setWidth(14);
    return stroker.createStroke(path());
}

void EdgeItem::setFrom(PortItem *port)
{
    m_from = port;
    updatePath();
}

void EdgeItem::setTo(PortItem *port)
{
    m_to = port;
    updatePath();
}

void EdgeItem::setFreeEnd(const QPointF &scenePos)
{
    m_freeEnd = scenePos;
    updatePath();
}

void EdgeItem::setTemporary(bool temporary)
{
    m_temporary = temporary;
    setFlag(ItemIsSelectable, !temporary);
    updatePen();
}

void EdgeItem::forget(PortItem *port)
{
    if (m_from == port)
        m_from = nullptr;
    if (m_to == port)
        m_to = nullptr;
}

void EdgeItem::detach()
{
    if (m_from) {
        PortItem *port = m_from;
        m_from = nullptr;
        port->removeEdge(this);
    }
    if (m_to) {
        PortItem *port = m_to;
        m_to = nullptr;
        port->removeEdge(this);
    }
}

void EdgeItem::updatePen()
{
    QColor color = isSelected() ? QColor("#8af3ff") : QColor("#3ec6e0");
    if (m_temporary)
        color = QColor("#8fd4e4");
    m_pen = QPen(color, isSelected() ? 2.6 : 2.0);
    m_pen.setCosmetic(true);
    m_pen.setCapStyle(Qt::RoundCap);
    m_pen.setJoinStyle(Qt::RoundJoin);
    if (m_temporary)
        m_pen.setStyle(Qt::DashLine);
    setPen(m_pen);
}

void EdgeItem::updatePath()
{
    const QPointF start = m_from ? m_from->anchorScene() : m_freeEnd;
    const QPointF end = m_to ? m_to->anchorScene() : m_freeEnd;
    QPainterPath curve(start);
    const qreal dy = qMax(qreal(48), qAbs(end.y() - start.y()) * qreal(0.45));
    curve.cubicTo(QPointF(start.x(), start.y() + dy),
                  QPointF(end.x(), end.y() - dy),
                  end);
    setPath(curve);
    m_arrow = makeArrow(start, end);
}

QPolygonF EdgeItem::makeArrow(const QPointF &from, const QPointF &to) const
{
    const QPointF delta = to - from;
    const qreal length = qSqrt(delta.x() * delta.x() + delta.y() * delta.y());
    if (length < 8.0)
        return QPolygonF();
    const QPointF unit = delta / length;
    const QPointF normal(-unit.y(), unit.x());
    const QPointF base = to - unit * 10.0;
    QPolygonF arrow;
    arrow << to << (base + normal * 4.5) << (base - normal * 4.5);
    return arrow;
}

void EdgeItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(m_pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path());
    if (!m_arrow.isEmpty()) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(m_pen.color());
        painter->drawPolygon(m_arrow);
    }
}

QVariant EdgeItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemSelectedHasChanged)
        updatePen();
    return QGraphicsPathItem::itemChange(change, value);
}

void EdgeItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
    QMenu menu;
    QAction *removeAction = menu.addAction(QStringLiteral("删除连线"));
    event->accept();
    if (menu.exec(event->screenPos()) != removeAction)
        return;
    auto *flow = qobject_cast<FlowScene *>(scene());
    if (!flow)
        return;
    EdgeItem *self = this;
    QTimer::singleShot(0, flow, [flow, self]() {
        flow->removeEdge(self, true);
    });
}
