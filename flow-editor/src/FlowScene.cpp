#include "FlowScene.h"

#include "EdgeItem.h"
#include "NodeItem.h"
#include "PortItem.h"
#include "StepRegistry.h"

#include <QPainter>
#include <QSet>
#include <QUuid>
#include <QtMath>
#include <algorithm>
#include <cmath>

FlowScene::FlowScene(const StepRegistry *registry, QObject *parent)
    : QGraphicsScene(parent)
    , m_registry(registry)
{
    setBackgroundBrush(QColor("#1c1e20"));
    setSceneRect(-5000, -5000, 10000, 10000);
}

NodeItem *FlowScene::addNode(const QString &typeId, const QPointF &scenePos)
{
    NodeItem *node = insertNode(typeId, QPointF(0, 0), -1, QString());
    if (!node)
        return nullptr;
    node->setPos(scenePos.x() - node->width() / 2.0, scenePos.y() - node->bodyHeight() / 2.0);
    emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                     QStringLiteral("添加步骤 ") + node->caption());
    return node;
}

NodeItem *FlowScene::insertNode(const QString &typeId, const QPointF &topLeft,
                                int index, const QString &id)
{
    const StepDesc *desc = m_registry ? m_registry->find(typeId) : nullptr;
    if (!desc) {
        emit userMessage(QStringLiteral("操作"), QStringLiteral("错误"),
                         QStringLiteral("未知步骤：") + typeId);
        return nullptr;
    }

    int resolved = index;
    if (resolved > 0)
        m_nextIndex = qMax(m_nextIndex, resolved);
    else
        resolved = ++m_nextIndex;

    const QString nodeId = id.isEmpty()
        ? QUuid::createUuid().toString(QUuid::WithoutBraces)
        : id;
    auto *node = new NodeItem(*desc, resolved, nodeId);
    connect(node, &NodeItem::visualizeRequested, this, [this](const QString &caption) {
        emit userMessage(QStringLiteral("控制台"), QStringLiteral("信息"),
                         QStringLiteral("预览数据：") + caption);
    });
    connect(node, &NodeItem::exportRequested, this, [this](const QString &caption) {
        emit userMessage(QStringLiteral("控制台"), QStringLiteral("信息"),
                         QStringLiteral("导出数据：") + caption);
    });
    connect(node, &NodeItem::helpRequested, this, [this](const QString &caption, const QString &summary) {
        emit userMessage(QStringLiteral("控制台"), QStringLiteral("信息"),
                         caption + QStringLiteral("：") + summary);
    });
    addItem(node);
    node->setPos(topLeft);
    return node;
}

NodeItem *FlowScene::nodeById(const QString &id) const
{
    for (NodeItem *node : nodes()) {
        if (node->id() == id)
            return node;
    }
    return nullptr;
}

QVector<NodeItem *> FlowScene::nodes() const
{
    QVector<NodeItem *> result;
    const QList<QGraphicsItem *> all = items();
    for (QGraphicsItem *item : all) {
        if (item->type() == NodeItem::Type)
            result.append(static_cast<NodeItem *>(item));
    }
    return result;
}

QVector<EdgeItem *> FlowScene::edges() const
{
    QVector<EdgeItem *> result;
    const QList<QGraphicsItem *> all = items();
    for (QGraphicsItem *item : all) {
        if (item->type() == EdgeItem::Type && item != m_draft)
            result.append(static_cast<EdgeItem *>(item));
    }
    return result;
}

QRectF FlowScene::nodesBoundingRect() const
{
    QRectF bounds;
    bool any = false;
    for (NodeItem *node : nodes()) {
        const QRectF itemBounds = node->sceneBoundingRect();
        bounds = any ? bounds.united(itemBounds) : itemBounds;
        any = true;
    }
    return bounds;
}

void FlowScene::load(const GraphDocument &document)
{
    clearGraph();
    m_nextIndex = 0;

    QHash<QString, NodeItem *> byId;
    for (const GraphNode &nodeData : document.nodes) {
        NodeItem *node = insertNode(nodeData.typeId, nodeData.pos, nodeData.index, nodeData.id);
        if (node)
            byId.insert(node->id(), node);
    }
    for (const GraphEdge &edgeData : document.edges) {
        NodeItem *fromNode = byId.value(edgeData.fromId);
        NodeItem *toNode = byId.value(edgeData.toId);
        if (!fromNode || !toNode)
            continue;
        PortItem *from = fromNode->findPort(edgeData.fromKey, PortDir::Output);
        PortItem *to = toNode->findPort(edgeData.toKey, PortDir::Input);
        if (!from || !to)
            continue;
        connectPorts(from, to, false);
    }
}

GraphDocument FlowScene::capture() const
{
    GraphDocument document;
    for (NodeItem *node : nodes()) {
        GraphNode data;
        data.id = node->id();
        data.typeId = node->typeId();
        data.pos = node->pos();
        data.index = node->index();
        document.nodes.append(data);
    }
    for (EdgeItem *edge : edges()) {
        if (!edge->from() || !edge->to() || !edge->from()->node() || !edge->to()->node())
            continue;
        GraphEdge data;
        data.fromId = edge->from()->node()->id();
        data.fromKey = edge->from()->key();
        data.toId = edge->to()->node()->id();
        data.toKey = edge->to()->key();
        document.edges.append(data);
    }
    return document;
}

void FlowScene::clearGraph()
{
    cancelDraft();
    const QVector<NodeItem *> nodeList = nodes();
    for (NodeItem *node : nodeList)
        deleteNode(node, false);
    const QVector<EdgeItem *> leftover = edges();
    for (EdgeItem *edge : leftover)
        removeEdge(edge, false);
}

void FlowScene::deleteSelection()
{
    const QList<QGraphicsItem *> selected = selectedItems();
    QVector<NodeItem *> nodeList;
    QVector<EdgeItem *> edgeList;
    for (QGraphicsItem *item : selected) {
        if (item->type() == NodeItem::Type)
            nodeList.append(static_cast<NodeItem *>(item));
        else if (item->type() == EdgeItem::Type && item != m_draft)
            edgeList.append(static_cast<EdgeItem *>(item));
    }
    if (nodeList.isEmpty() && edgeList.isEmpty())
        return;

    QSet<EdgeItem *> uniqueEdges;
    for (EdgeItem *edge : edgeList)
        uniqueEdges.insert(edge);
    for (NodeItem *node : nodeList) {
        for (PortItem *port : node->ports()) {
            for (EdgeItem *edge : port->edges())
                uniqueEdges.insert(edge);
        }
    }
    const int nodeCount = nodeList.size();
    const int edgeCount = uniqueEdges.size();
    for (EdgeItem *edge : uniqueEdges)
        removeEdge(edge, false);
    for (NodeItem *node : nodeList)
        deleteNode(node, false);

    emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                     QStringLiteral("已删除 %1 个步骤、%2 条连线").arg(nodeCount).arg(edgeCount));
}

void FlowScene::deleteNode(NodeItem *node, bool notify)
{
    if (!node || node->scene() != this)
        return;
    const QString caption = node->caption();
    QSet<EdgeItem *> related;
    for (PortItem *port : node->ports()) {
        for (EdgeItem *edge : port->edges())
            related.insert(edge);
    }
    for (EdgeItem *edge : related)
        removeEdge(edge, false);
    delete node;
    if (notify) {
        emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                         QStringLiteral("已删除步骤 ") + caption);
    }
}

void FlowScene::removeEdge(EdgeItem *edge, bool notify)
{
    if (!edge || edge->scene() != this)
        return;
    if (edge == m_draft) {
        cancelDraft();
        return;
    }
    QString text;
    if (edge->from() && edge->to() && edge->from()->node() && edge->to()->node()) {
        text = edge->from()->node()->caption() + QStringLiteral(" → ")
               + edge->to()->node()->caption();
    }
    delete edge;
    if (notify) {
        emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                         text.isEmpty() ? QStringLiteral("已删除连线")
                                        : QStringLiteral("已删除连线 ") + text);
    }
}

bool FlowScene::connectPorts(PortItem *from, PortItem *to, bool notify)
{
    if (!from || !to)
        return false;
    if (from->direction() != PortDir::Output || to->direction() != PortDir::Input) {
        if (notify) {
            emit userMessage(QStringLiteral("操作"), QStringLiteral("警告"),
                             QStringLiteral("只能从输出端口连到输入端口"));
        }
        return false;
    }
    if (from->node() == to->node()) {
        if (notify) {
            emit userMessage(QStringLiteral("操作"), QStringLiteral("警告"),
                             QStringLiteral("不能把步骤连到自身"));
        }
        return false;
    }
    if (from->typeId() != to->typeId()) {
        if (notify) {
            emit userMessage(QStringLiteral("操作"), QStringLiteral("警告"),
                             QStringLiteral("端口类型不兼容：%1 不能连到 %2")
                                 .arg(from->typeLabel(), to->typeLabel()));
        }
        return false;
    }
    for (EdgeItem *edge : from->edges()) {
        if (edge->to() == to) {
            if (notify) {
                emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                                 QStringLiteral("这两个端口已经连接"));
            }
            return false;
        }
    }

    const bool replaced = !to->edges().isEmpty();
    const QVector<EdgeItem *> existing = to->edges();
    for (EdgeItem *edge : existing)
        removeEdge(edge, false);

    auto *edge = new EdgeItem(from, to);
    addItem(edge);
    if (notify) {
        const QString verb = replaced ? QStringLiteral("已替换连线 ") : QStringLiteral("已连接 ");
        emit userMessage(QStringLiteral("操作"), QStringLiteral("信息"),
                         verb + from->node()->caption() + QLatin1Char('.') + from->name()
                             + QStringLiteral(" → ") + to->node()->caption() + QLatin1Char('.')
                             + to->name());
    }
    return true;
}

void FlowScene::beginDraft(PortItem *port)
{
    if (!port)
        return;
    cancelDraft();
    m_draftOrigin = port;
    m_draft = new EdgeItem();
    if (port->direction() == PortDir::Output)
        m_draft->setFrom(port);
    else
        m_draft->setTo(port);
    m_draft->setFreeEnd(port->anchorScene());
    addItem(m_draft);
}

void FlowScene::moveDraft(const QPointF &scenePos)
{
    if (m_draft)
        m_draft->setFreeEnd(scenePos);
}

void FlowScene::endDraft(const QPointF &scenePos)
{
    if (!m_draft || !m_draftOrigin) {
        cancelDraft();
        return;
    }
    PortItem *origin = m_draftOrigin;
    const PortDir originDir = origin->direction();
    PortItem *hit = portAt(scenePos, origin);
    cancelDraft();
    if (!hit || hit == origin)
        return;

    PortItem *from = originDir == PortDir::Output ? origin : hit;
    PortItem *to = originDir == PortDir::Output ? hit : origin;
    if (from->direction() != PortDir::Output || to->direction() != PortDir::Input) {
        emit userMessage(QStringLiteral("操作"), QStringLiteral("警告"),
                         QStringLiteral("请把输出端口拖到输入端口"));
        return;
    }
    connectPorts(from, to, true);
}

void FlowScene::cancelDraft()
{
    EdgeItem *draft = m_draft;
    m_draft = nullptr;
    m_draftOrigin = nullptr;
    delete draft;
}

PortItem *FlowScene::portAt(const QPointF &scenePos, PortItem *ignore) const
{
    const QRectF area(scenePos.x() - 14, scenePos.y() - 14, 28, 28);
    const QList<QGraphicsItem *> hits = items(area, Qt::IntersectsItemShape, Qt::DescendingOrder);
    PortItem *fallback = nullptr;
    for (QGraphicsItem *item : hits) {
        if (item->type() != PortItem::Type)
            continue;
        auto *port = static_cast<PortItem *>(item);
        if (port == ignore)
            continue;
        if (!fallback)
            fallback = port;
        if (m_draftOrigin && port->direction() != m_draftOrigin->direction())
            return port;
    }
    return fallback;
}

void FlowScene::execute(bool continuous)
{
    const QVector<NodeItem *> nodeList = nodes();
    if (nodeList.isEmpty()) {
        emit userMessage(QStringLiteral("视觉"), QStringLiteral("警告"),
                         QStringLiteral("画布上没有步骤"));
        return;
    }

    QHash<NodeItem *, int> indegree;
    QHash<NodeItem *, QVector<NodeItem *>> next;
    QHash<NodeItem *, QVector<NodeItem *>> previous;
    for (NodeItem *node : nodeList)
        indegree.insert(node, 0);

    for (EdgeItem *edge : edges()) {
        if (!edge->from() || !edge->to())
            continue;
        NodeItem *source = edge->from()->node();
        NodeItem *target = edge->to()->node();
        if (!source || !target)
            continue;
        next[source].append(target);
        previous[target].append(source);
        indegree[target] += 1;
    }

    QVector<NodeItem *> ready;
    for (NodeItem *node : nodeList) {
        if (indegree.value(node) == 0)
            ready.append(node);
    }
    const auto byIndex = [](NodeItem *left, NodeItem *right) {
        return left->index() < right->index();
    };
    std::sort(ready.begin(), ready.end(), byIndex);

    emit userMessage(QStringLiteral("视觉"), QStringLiteral("信息"),
                     continuous ? QStringLiteral("连续运行开始") : QStringLiteral("运行开始"));

    QHash<NodeItem *, bool> success;
    while (!ready.isEmpty()) {
        NodeItem *node = ready.takeFirst();
        bool inputsReady = true;
        for (PortItem *input : node->inputs()) {
            if (input->edges().isEmpty())
                inputsReady = false;
        }
        bool upstreamReady = true;
        for (NodeItem *parent : previous.value(node)) {
            if (!success.value(parent, false))
                upstreamReady = false;
        }

        if (!inputsReady) {
            success.insert(node, false);
            emit userMessage(QStringLiteral("视觉"), QStringLiteral("警告"),
                             node->caption() + QStringLiteral(" 缺少输入，已跳过"));
        } else if (!upstreamReady) {
            success.insert(node, false);
            emit userMessage(QStringLiteral("视觉"), QStringLiteral("警告"),
                             node->caption() + QStringLiteral(" 上游未完成，已跳过"));
        } else {
            success.insert(node, true);
            QStringList produced;
            for (PortItem *output : node->outputs())
                produced << output->name();
            QString line = node->caption();
            if (!produced.isEmpty())
                line += QStringLiteral("  →  ") + produced.join(QStringLiteral("，"));
            emit userMessage(QStringLiteral("视觉"), QStringLiteral("信息"), line);
        }

        for (NodeItem *child : next.value(node)) {
            indegree[child] -= 1;
            if (indegree[child] == 0) {
                ready.append(child);
                std::sort(ready.begin(), ready.end(), byIndex);
            }
        }
    }

    if (success.size() < nodeList.size()) {
        emit userMessage(QStringLiteral("视觉"), QStringLiteral("错误"),
                         QStringLiteral("流程中存在环，未执行剩余步骤"));
    } else {
        emit userMessage(QStringLiteral("视觉"), QStringLiteral("信息"),
                         continuous ? QStringLiteral("连续运行完成") : QStringLiteral("运行完成"));
    }
}

void FlowScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->fillRect(rect, QColor("#1c1e20"));
    const int step = 22;
    const int firstX = static_cast<int>(std::floor(rect.left() / step));
    const int lastX = static_cast<int>(std::ceil(rect.right() / step));
    const int firstY = static_cast<int>(std::floor(rect.top() / step));
    const int lastY = static_cast<int>(std::ceil(rect.bottom() / step));

    QPen minor(QColor(255, 255, 255, 18));
    minor.setCosmetic(true);
    QPen major(QColor(255, 255, 255, 32));
    major.setCosmetic(true);

    for (int i = firstX; i <= lastX; ++i) {
        const bool majorLine = ((i % 5) + 5) % 5 == 0;
        painter->setPen(majorLine ? major : minor);
        const qreal x = i * step;
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
    }
    for (int i = firstY; i <= lastY; ++i) {
        const bool majorLine = ((i % 5) + 5) % 5 == 0;
        painter->setPen(majorLine ? major : minor);
        const qreal y = i * step;
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
    }
}
