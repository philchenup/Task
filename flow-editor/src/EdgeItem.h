#ifndef FLOWEDITOR_EDGEITEM_H
#define FLOWEDITOR_EDGEITEM_H

#include <QGraphicsPathItem>
#include <QPen>

class PortItem;

class EdgeItem : public QGraphicsPathItem
{
public:
    enum { Type = UserType + 3 };

    EdgeItem();
    EdgeItem(PortItem *from, PortItem *to);
    ~EdgeItem() override;

    int type() const override { return Type; }
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    PortItem *from() const { return m_from; }
    PortItem *to() const { return m_to; }

    void setFrom(PortItem *port);
    void setTo(PortItem *port);
    void setFreeEnd(const QPointF &scenePos);
    void setTemporary(bool temporary);
    void updatePath();
    void detach();
    void forget(PortItem *port);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
    void contextMenuEvent(QGraphicsSceneContextMenuEvent *event) override;

private:
    void updatePen();
    QPolygonF makeArrow(const QPointF &from, const QPointF &to) const;

    PortItem *m_from = nullptr;
    PortItem *m_to = nullptr;
    QPointF m_freeEnd;
    QPolygonF m_arrow;
    QPen m_pen;
    bool m_temporary = false;
};

#endif
