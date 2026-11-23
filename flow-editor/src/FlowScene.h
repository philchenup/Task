#ifndef FLOWEDITOR_FLOWSCENE_H
#define FLOWEDITOR_FLOWSCENE_H

#include "Types.h"

#include <QGraphicsScene>

class EdgeItem;
class NodeItem;
class PortItem;
class StepRegistry;

class FlowScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit FlowScene(const StepRegistry *registry, QObject *parent = nullptr);

    NodeItem *addNode(const QString &typeId, const QPointF &scenePos);
    NodeItem *nodeById(const QString &id) const;
    QVector<NodeItem *> nodes() const;
    QRectF nodesBoundingRect() const;

    void load(const GraphDocument &document);
    GraphDocument capture() const;
    void clearGraph();
    void deleteSelection();
    void deleteNode(NodeItem *node, bool notify);
    void removeEdge(EdgeItem *edge, bool notify);
    void execute(bool continuous);

    void beginDraft(PortItem *port);
    void moveDraft(const QPointF &scenePos);
    void endDraft(const QPointF &scenePos);
    void cancelDraft();

signals:
    void userMessage(const QString &channel, const QString &level, const QString &message);

protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;

private:
    NodeItem *insertNode(const QString &typeId, const QPointF &topLeft, int index, const QString &id);
    bool connectPorts(PortItem *from, PortItem *to, bool notify);
    PortItem *portAt(const QPointF &scenePos, PortItem *ignore) const;
    QVector<EdgeItem *> edges() const;

    const StepRegistry *m_registry = nullptr;
    int m_nextIndex = 0;
    EdgeItem *m_draft = nullptr;
    PortItem *m_draftOrigin = nullptr;
};

#endif
