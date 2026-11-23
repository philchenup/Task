#ifndef FLOWEDITOR_TYPES_H
#define FLOWEDITOR_TYPES_H

#include <QColor>
#include <QFont>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QVector>

// Drag payload from the step library. Only the step type id is carried.
constexpr char kStepMime[] = "application/x-flow-step";

enum class PortDir { Input, Output };

struct PortDesc {
    QString key;
    QString typeId;
    QString typeLabel;
    QString name;
};

struct StepDesc {
    QString typeId;
    QString title;
    QString category;
    QString summary;
    QString icon;
    QVector<PortDesc> inputs;
    QVector<PortDesc> outputs;
};

struct GraphNode {
    QString id;
    QString typeId;
    QPointF pos;
    int index = 0;
};

struct GraphEdge {
    QString fromId;
    QString fromKey;
    QString toId;
    QString toKey;
};

struct GraphDocument {
    QVector<GraphNode> nodes;
    QVector<GraphEdge> edges;
};

inline QFont uiFont(int pixelSize)
{
    QFont font;
    font.setFamilies(QStringList()
                     << QStringLiteral("WenQuanYi Micro Hei")
                     << QStringLiteral("Noto Sans CJK SC")
                     << QStringLiteral("Microsoft YaHei")
                     << QStringLiteral("sans-serif"));
    font.setPixelSize(pixelSize);
    return font;
}

inline QColor portColor(const QString &typeId)
{
    if (typeId == QLatin1String("CloudXYZ"))
        return QColor("#3aa3d6");
    if (typeId == QLatin1String("CloudXYZNormal"))
        return QColor("#c6be45");
    if (typeId == QLatin1String("Pose"))
        return QColor("#8aaa55");
    if (typeId == QLatin1String("Image"))
        return QColor("#6a93c4");
    if (typeId == QLatin1String("Path"))
        return QColor("#d0924a");
    if (typeId == QLatin1String("Number"))
        return QColor("#8d8d8d");
    if (typeId == QLatin1String("Bool"))
        return QColor("#5dba8a");
    return QColor("#5aa0b0");
}

inline QColor portTextColor(const QString &typeId)
{
    if (typeId == QLatin1String("CloudXYZNormal") || typeId == QLatin1String("Pose"))
        return QColor("#1c1c12");
    return QColor("#ffffff");
}

#endif
