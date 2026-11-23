#ifndef FLOWEDITOR_NODEITEM_H
#define FLOWEDITOR_NODEITEM_H

#include "PortItem.h"

#include <QGraphicsObject>

class NodeItem : public QGraphicsObject
{
    Q_OBJECT
public:
    enum { Type = UserType + 1 };

    NodeItem(const StepDesc &desc, int index, const QString &id);

    int type() const override { return Type; }
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    QString id() const { return m_id; }
    QString typeId() const { return m_desc.typeId; }
    int index() const { return m_index; }
    QString caption() const;
    QString summary() const { return m_desc.summary; }
    qreal width() const { return m_width; }
    qreal bodyHeight() const { return m_bodyHeight; }

    QVector<PortItem *> inputs() const { return m_inputs; }
    QVector<PortItem *> outputs() const { return m_outputs; }
    QVector<PortItem *> ports() const;
    PortItem *findPort(const QString &key, PortDir direction) const;

signals:
    void visualizeRequested(const QString &caption);
    void exportRequested(const QString &caption);
    void helpRequested(const QString &caption, const QString &summary);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
    void contextMenuEvent(QGraphicsSceneContextMenuEvent *event) override;

private:
    enum class Hot { None, Eye, Save, Help };

    void relayout();
    QRectF bodyRect() const;
    QRectF eyeRect() const;
    QRectF saveRect() const;
    QRectF helpRect() const;
    Hot hitButton(const QPointF &pos) const;
    void paintIcon(QPainter *painter, const QRectF &rect) const;
    void paintToolButton(QPainter *painter, const QRectF &rect, bool hot) const;

    StepDesc m_desc;
    QString m_id;
    int m_index = 0;
    qreal m_width = 230;
    qreal m_bodyHeight = 86;
    qreal m_bodyTop = 0;
    QVector<PortItem *> m_inputs;
    QVector<PortItem *> m_outputs;
    Hot m_hot = Hot::None;
    Hot m_pressed = Hot::None;
};

#endif
