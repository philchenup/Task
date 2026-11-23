#ifndef FLOWEDITOR_PORTITEM_H
#define FLOWEDITOR_PORTITEM_H

#include "Types.h"

#include <QGraphicsObject>
#include <QVector>

class EdgeItem;
class FlowScene;
class NodeItem;

class PortItem : public QGraphicsObject
{
public:
    enum { Type = UserType + 2 };

    PortItem(const PortDesc &desc, PortDir direction, NodeItem *parent);
    ~PortItem() override;

    int type() const override { return Type; }
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    PortDir direction() const { return m_dir; }
    QString key() const { return m_desc.key; }
    QString typeId() const { return m_desc.typeId; }
    QString typeLabel() const { return m_desc.typeLabel; }
    QString name() const { return m_desc.name; }
    qreal chipWidth() const { return m_chip.width(); }
    qreal chipHeight() const { return m_chip.height(); }

    NodeItem *node() const;
    QPointF anchorScene() const;
    QVector<EdgeItem *> edges() const { return m_edges; }

    void addEdge(EdgeItem *edge);
    void removeEdge(EdgeItem *edge);
    void updateEdges();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

private:
    QPointF anchorLocal() const;
    QRectF dotRect() const;
    FlowScene *flowScene() const;

    PortDesc m_desc;
    PortDir m_dir;
    QRectF m_chip;
    QFont m_font;
    QVector<EdgeItem *> m_edges;
    bool m_hover = false;
    bool m_dragging = false;
};

#endif
