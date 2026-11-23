#include "NodeItem.h"

#include "FlowScene.h"

#include <QFontMetrics>
#include <QGraphicsSceneContextMenuEvent>
#include <QGraphicsSceneMouseEvent>
#include <QMenu>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QTimer>

NodeItem::NodeItem(const StepDesc &desc, int index, const QString &id)
    : m_desc(desc)
    , m_id(id)
    , m_index(index)
{
    for (const PortDesc &port : desc.inputs)
        m_inputs.append(new PortItem(port, PortDir::Input, this));
    for (const PortDesc &port : desc.outputs)
        m_outputs.append(new PortItem(port, PortDir::Output, this));

    relayout();
    setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges | ItemIsFocusable);
    setAcceptHoverEvents(true);
    setZValue(1);
    setToolTip(desc.summary);
}

QString NodeItem::caption() const
{
    return m_desc.title + QStringLiteral(" (") + QString::number(m_index) + QStringLiteral(")");
}

QVector<PortItem *> NodeItem::ports() const
{
    QVector<PortItem *> all = m_inputs;
    all += m_outputs;
    return all;
}

PortItem *NodeItem::findPort(const QString &key, PortDir direction) const
{
    const QVector<PortItem *> &list = direction == PortDir::Input ? m_inputs : m_outputs;
    for (PortItem *port : list) {
        if (port->key() == key)
            return port;
    }
    return nullptr;
}

void NodeItem::relayout()
{
    const QFontMetrics metrics(uiFont(13));
    qreal width = metrics.horizontalAdvance(caption()) + 64;
    for (PortItem *port : ports())
        width = qMax(width, port->chipWidth() + 20);
    m_width = qMax(width, qreal(230));

    qreal y = 0;
    if (!m_inputs.isEmpty()) {
        for (PortItem *port : m_inputs) {
            port->setPos((m_width - port->chipWidth()) / 2.0, y);
            y += port->chipHeight() + 6;
        }
        y += 2;
    }
    m_bodyTop = y;
    y += m_bodyHeight;
    if (!m_outputs.isEmpty()) {
        y += 8;
        for (PortItem *port : m_outputs) {
            port->setPos((m_width - port->chipWidth()) / 2.0, y);
            y += port->chipHeight() + 6;
        }
    }
}

QRectF NodeItem::bodyRect() const
{
    return QRectF(0, m_bodyTop, m_width, m_bodyHeight);
}

QRectF NodeItem::boundingRect() const
{
    return bodyRect().adjusted(-4, -4, 4, 8);
}

QPainterPath NodeItem::shape() const
{
    QPainterPath path;
    path.addRoundedRect(bodyRect(), 8, 8);
    return path;
}

QRectF NodeItem::eyeRect() const
{
    return QRectF(m_width - 66, m_bodyTop + 8, 24, 24);
}

QRectF NodeItem::saveRect() const
{
    return QRectF(m_width - 36, m_bodyTop + 8, 24, 24);
}

QRectF NodeItem::helpRect() const
{
    const QFontMetrics metrics(uiFont(13));
    const qreal textWidth = metrics.horizontalAdvance(caption());
    const qreal x = m_width / 2.0 + textWidth / 2.0 + 6;
    const qreal y = m_bodyTop + 48;
    return QRectF(x, y, 16, 16);
}

NodeItem::Hot NodeItem::hitButton(const QPointF &pos) const
{
    if (eyeRect().contains(pos))
        return Hot::Eye;
    if (saveRect().contains(pos))
        return Hot::Save;
    if (helpRect().contains(pos))
        return Hot::Help;
    return Hot::None;
}

void NodeItem::paintToolButton(QPainter *painter, const QRectF &rect, bool hot) const
{
    painter->setPen(Qt::NoPen);
    painter->setBrush(hot ? QColor("#2f8f88") : QColor("#2a333b"));
    painter->drawRoundedRect(rect, 4, 4);
}

void NodeItem::paintIcon(QPainter *painter, const QRectF &rect) const
{
    painter->save();
    painter->setRenderHint(QPainter::Antialiasing);
    const QString &kind = m_desc.icon;
    if (kind == QLatin1String("cloud")) {
        painter->setPen(QPen(QColor("#9ecbff"), 1.3));
        painter->setBrush(QColor(90, 150, 190, 50));
        QPainterPath folder;
        folder.moveTo(rect.left() + 3, rect.top() + 9);
        folder.lineTo(rect.left() + 9, rect.top() + 9);
        folder.lineTo(rect.left() + 11, rect.top() + 6);
        folder.lineTo(rect.right() - 3, rect.top() + 6);
        folder.lineTo(rect.right() - 3, rect.bottom() - 5);
        folder.lineTo(rect.left() + 3, rect.bottom() - 5);
        folder.closeSubpath();
        painter->drawPath(folder);
        painter->setPen(Qt::NoPen);
        painter->setBrush(QColor("#7dcea0"));
        painter->drawEllipse(QPointF(rect.left() + 8, rect.bottom() - 10), 1.6, 1.6);
        painter->drawEllipse(QPointF(rect.left() + 13, rect.bottom() - 12), 1.5, 1.5);
        painter->drawEllipse(QPointF(rect.left() + 17, rect.bottom() - 9), 1.4, 1.4);
    } else if (kind == QLatin1String("camera")) {
        painter->setPen(QPen(QColor("#9ec0e0"), 1.4));
        painter->setBrush(Qt::NoBrush);
        painter->drawRoundedRect(rect.adjusted(2, 5, -2, -4), 2, 2);
        painter->drawEllipse(rect.center(), 4, 4);
    } else if (kind == QLatin1String("image")) {
        painter->setPen(QPen(QColor("#9ec0e0"), 1.4));
        painter->drawRect(rect.adjusted(3, 4, -3, -4));
        painter->drawLine(rect.left() + 5, rect.bottom() - 7, rect.center().x(), rect.center().y());
        painter->drawLine(rect.center().x(), rect.center().y(), rect.right() - 5, rect.bottom() - 8);
    } else if (kind == QLatin1String("normal")) {
        painter->setPen(QPen(QColor("#7dcea0"), 1.6, Qt::SolidLine, Qt::RoundCap));
        for (int i = 0; i < 5; ++i) {
            const qreal x = rect.left() + 4 + i * 4;
            const qreal lean = (i - 2) * 1.4;
            painter->drawLine(QPointF(x, rect.bottom() - 5), QPointF(x + lean, rect.top() + 4));
        }
    } else if (kind == QLatin1String("group")) {
        painter->setPen(QPen(QColor("#9ecbff"), 1.4));
        painter->drawRect(QRectF(rect.left() + 3, rect.top() + 8, 8, 6));
        painter->drawRect(QRectF(rect.right() - 12, rect.top() + 4, 8, 6));
        painter->drawRect(QRectF(rect.right() - 12, rect.bottom() - 11, 8, 6));
        painter->drawLine(QPointF(rect.left() + 11, rect.top() + 11), QPointF(rect.right() - 12, rect.top() + 7));
        painter->drawLine(QPointF(rect.left() + 11, rect.top() + 12), QPointF(rect.right() - 12, rect.bottom() - 8));
    } else if (kind == QLatin1String("pose") || kind == QLatin1String("sort")) {
        painter->setPen(QPen(QColor("#d2e29a"), 1.4));
        QPolygonF star;
        const QPointF c = rect.center();
        star << QPointF(c.x(), rect.top() + 3) << QPointF(c.x() + 3, c.y() - 1)
             << QPointF(rect.right() - 3, c.y()) << QPointF(c.x() + 3, c.y() + 3)
             << QPointF(c.x(), rect.bottom() - 3) << QPointF(c.x() - 3, c.y() + 3)
             << QPointF(rect.left() + 3, c.y()) << QPointF(c.x() - 3, c.y() - 1);
        painter->drawPolygon(star);
    } else if (kind == QLatin1String("file") || kind == QLatin1String("script")) {
        painter->setPen(QPen(QColor("#d0d0d0"), 1.4));
        painter->drawRect(rect.adjusted(6, 3, -6, -3));
        painter->drawLine(rect.left() + 8, rect.top() + 9, rect.right() - 8, rect.top() + 9);
        painter->drawLine(rect.left() + 8, rect.top() + 13, rect.right() - 8, rect.top() + 13);
    } else {
        painter->setPen(QPen(QColor("#d0d0d0"), 1.4));
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(rect.adjusted(4, 4, -4, -4));
    }
    painter->restore();
}

void NodeItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing);
    const QRectF body = bodyRect();
    const bool selected = option->state & QStyle::State_Selected;
    const bool hovered = option->state & QStyle::State_MouseOver;

    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(0, 0, 0, 70));
    painter->drawRoundedRect(body.translated(0, 3), 8, 8);

    QColor fill("#3c454e");
    if (m_desc.icon == QLatin1String("group"))
        fill = QColor("#343c46");
    if (hovered && !selected)
        fill = fill.lighter(112);
    painter->setBrush(fill);
    painter->setPen(QPen(QColor("#56616b"), 1));
    painter->drawRoundedRect(body, 8, 8);

    if (selected) {
        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(QColor(46, 224, 208, 70), 5));
        painter->drawRoundedRect(body, 8, 8);
        painter->setPen(QPen(QColor("#2ee0d0"), 1.7));
        painter->drawRoundedRect(body.adjusted(0.5, 0.5, -0.5, -0.5), 8, 8);
    }

    paintIcon(painter, QRectF(12, m_bodyTop + 8, 26, 26));
    paintToolButton(painter, eyeRect(), m_hot == Hot::Eye);
    paintToolButton(painter, saveRect(), m_hot == Hot::Save);

    painter->setPen(QPen(QColor("#d5dde3"), 1.3));
    const QRectF eye = eyeRect();
    painter->drawEllipse(eye.center(), 5.5, 4.0);
    painter->setBrush(QColor("#d5dde3"));
    painter->drawEllipse(eye.center(), 1.6, 1.6);

    painter->setBrush(Qt::NoBrush);
    const QRectF save = saveRect();
    painter->drawLine(QPointF(save.center().x(), save.top() + 5),
                      QPointF(save.center().x(), save.bottom() - 8));
    painter->drawLine(QPointF(save.center().x() - 4, save.bottom() - 12),
                      QPointF(save.center().x(), save.bottom() - 8));
    painter->drawLine(QPointF(save.center().x() + 4, save.bottom() - 12),
                      QPointF(save.center().x(), save.bottom() - 8));
    painter->drawLine(QPointF(save.left() + 6, save.bottom() - 6),
                      QPointF(save.right() - 6, save.bottom() - 6));

    painter->setPen(QColor("#f2f5f7"));
    QFont titleFont = uiFont(13);
    painter->setFont(titleFont);
    const QFontMetrics metrics(titleFont);
    const int maxTitle = qMax(40, int(m_width - 48));
    const QString shown = metrics.elidedText(caption(), Qt::ElideRight, maxTitle);
    painter->drawText(QRectF(8, m_bodyTop + 42, m_width - 16, 28),
                      Qt::AlignHCenter | Qt::AlignVCenter, shown);

    const QRectF help = helpRect();
    if (help.right() < m_width - 6) {
        painter->setPen(QPen(m_hot == Hot::Help ? QColor("#ffffff") : QColor("#9aa3ab"), 1));
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(help);
        painter->setPen(QColor("#d5dde3"));
        painter->setFont(uiFont(10));
        painter->drawText(help, Qt::AlignCenter, QStringLiteral("?"));
    }
}

QVariant NodeItem::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemPositionHasChanged) {
        for (PortItem *port : ports())
            port->updateEdges();
    } else if (change == ItemSelectedHasChanged) {
        setZValue(value.toBool() ? 8 : 1);
    }
    return QGraphicsObject::itemChange(change, value);
}

void NodeItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_pressed = hitButton(event->pos());
        if (m_pressed != Hot::None) {
            event->accept();
            return;
        }
    }
    QGraphicsObject::mousePressEvent(event);
}

void NodeItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_pressed != Hot::None) {
        event->accept();
        return;
    }
    QGraphicsObject::mouseMoveEvent(event);
}

void NodeItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_pressed != Hot::None) {
        const Hot pressed = m_pressed;
        m_pressed = Hot::None;
        if (hitButton(event->pos()) == pressed) {
            if (pressed == Hot::Eye)
                emit visualizeRequested(caption());
            else if (pressed == Hot::Save)
                emit exportRequested(caption());
            else if (pressed == Hot::Help)
                emit helpRequested(caption(), m_desc.summary);
        }
        event->accept();
        return;
    }
    QGraphicsObject::mouseReleaseEvent(event);
}

void NodeItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    emit helpRequested(caption(), m_desc.summary);
    QGraphicsObject::mouseDoubleClickEvent(event);
}

void NodeItem::hoverMoveEvent(QGraphicsSceneHoverEvent *event)
{
    const Hot hot = hitButton(event->pos());
    if (hot != m_hot) {
        m_hot = hot;
        update();
    }
    setCursor(hot == Hot::None ? Qt::SizeAllCursor : Qt::PointingHandCursor);
    QGraphicsObject::hoverMoveEvent(event);
}

void NodeItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event)
{
    m_hot = Hot::None;
    unsetCursor();
    update();
    QGraphicsObject::hoverLeaveEvent(event);
}

void NodeItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
    QMenu menu;
    QAction *removeAction = menu.addAction(QStringLiteral("删除步骤"));
    event->accept();
    if (menu.exec(event->screenPos()) != removeAction)
        return;
    auto *flow = qobject_cast<FlowScene *>(scene());
    if (!flow)
        return;
    const QString nodeId = m_id;
    QTimer::singleShot(0, flow, [flow, nodeId]() {
        if (NodeItem *node = flow->nodeById(nodeId))
            flow->deleteNode(node, true);
    });
}
