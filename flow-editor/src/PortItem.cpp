#include "PortItem.h"

#include "EdgeItem.h"
#include "FlowScene.h"
#include "NodeItem.h"

#include <QCursor>
#include <QFontMetrics>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

PortItem::PortItem(const PortDesc &desc, PortDir direction, NodeItem *parent)
    : QGraphicsObject(parent)
    , m_desc(desc)
    , m_dir(direction)
    , m_font(uiFont(12))
{
    QFontMetrics metrics(m_font);
    const int textWidth = qMax(metrics.horizontalAdvance(desc.typeLabel),
                               metrics.horizontalAdvance(desc.name));
    m_chip = QRectF(0, 0, qBound(118, textWidth + 18, 320), 36);
    setAcceptHoverEvents(true);
    setCursor(direction == PortDir::Output ? Qt::CrossCursor : Qt::PointingHandCursor);
    setToolTip(desc.typeLabel + QLatin1Char('\n') + desc.name);
}

PortItem::~PortItem()
{
    const QVector<EdgeItem *> snapshot = m_edges;
    m_edges.clear();
    for (EdgeItem *edge : snapshot)
        edge->forget(this);
}

QRectF PortItem::boundingRect() const
{
    return m_chip.adjusted(-4, -8, 4, 8);
}

QPainterPath PortItem::shape() const
{
    QPainterPath path;
    path.addRoundedRect(m_chip, 3, 3);
    path.addEllipse(dotRect());
    return path;
}

QRectF PortItem::dotRect() const
{
    const QPointF anchor = anchorLocal();
    return QRectF(anchor.x() - 5, anchor.y() - 5, 10, 10);
}

QPointF PortItem::anchorLocal() const
{
    if (m_dir == PortDir::Output)
        return QPointF(m_chip.center().x(), m_chip.bottom());
    return QPointF(m_chip.center().x(), m_chip.top());
}

QPointF PortItem::anchorScene() const
{
    return mapToScene(anchorLocal());
}

NodeItem *PortItem::node() const
{
    return qgraphicsitem_cast<NodeItem *>(parentItem());
}

void PortItem::addEdge(EdgeItem *edge)
{
    if (edge && !m_edges.contains(edge))
        m_edges.append(edge);
}

void PortItem::removeEdge(EdgeItem *edge)
{
    m_edges.removeAll(edge);
}

void PortItem::updateEdges()
{
    for (EdgeItem *edge : m_edges)
        edge->updatePath();
}

FlowScene *PortItem::flowScene() const
{
    return qobject_cast<FlowScene *>(scene());
}

void PortItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);

    QColor fill = portColor(m_desc.typeId);
    if (m_hover)
        fill = fill.lighter(115);

    painter->setPen(Qt::NoPen);
    painter->setBrush(fill);
    painter->drawRoundedRect(m_chip, 3, 3);

    painter->setFont(m_font);
    painter->setPen(portTextColor(m_desc.typeId));
    const QRectF top = m_chip.adjusted(8, 1, -8, -m_chip.height() / 2.0);
    const QRectF bottom = m_chip.adjusted(8, m_chip.height() / 2.0 - 1, -8, -1);
    painter->drawText(top, Qt::AlignLeft | Qt::AlignVCenter, m_desc.typeLabel);
    painter->drawText(bottom, Qt::AlignLeft | Qt::AlignVCenter, m_desc.name);

    painter->setBrush(QColor("#d8f6ff"));
    painter->setPen(QPen(QColor("#1a1c1e"), 1));
    painter->drawEllipse(dotRect());
}

void PortItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        if (FlowScene *flow = flowScene()) {
            flow->beginDraft(this);
            m_dragging = true;
        }
        if (NodeItem *owner = node())
            owner->setSelected(true);
        event->accept();
        return;
    }
    QGraphicsObject::mousePressEvent(event);
}

void PortItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_dragging) {
        if (FlowScene *flow = flowScene())
            flow->moveDraft(event->scenePos());
        event->accept();
        return;
    }
    QGraphicsObject::mouseMoveEvent(event);
}

void PortItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_dragging && event->button() == Qt::LeftButton) {
        m_dragging = false;
        if (FlowScene *flow = flowScene())
            flow->endDraft(event->scenePos());
        event->accept();
        return;
    }
    QGraphicsObject::mouseReleaseEvent(event);
}

void PortItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event)
{
    m_hover = true;
    update();
    QGraphicsObject::hoverEnterEvent(event);
}

void PortItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    m_hover = false;
    update();
    QGraphicsObject::hoverLeaveEvent(event);
}
