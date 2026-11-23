#include "StepLibrary.h"

#include "StepRegistry.h"
#include "Types.h"

#include <QDrag>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QPainter>
#include <QVBoxLayout>

StepTree::StepTree(QWidget *parent)
    : QTreeWidget(parent)
{
    setHeaderHidden(true);
    setDragEnabled(true);
    setDragDropMode(QAbstractItemView::DragOnly);
    setDefaultDropAction(Qt::CopyAction);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setExpandsOnDoubleClick(true);
    setIconSize(QSize(16, 16));
    setIndentation(16);
    setAnimated(false);
    setFrameShape(QFrame::NoFrame);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    connect(this, &QTreeWidget::itemDoubleClicked, this, &StepTree::onDoubleClicked);
}

QMimeData *StepTree::mimeData(const QList<QTreeWidgetItem *> items) const
{
    if (items.isEmpty())
        return nullptr;
    const QString typeId = items.first()->data(0, kStepTypeRole).toString();
    if (typeId.isEmpty())
        return nullptr;
    auto *mime = new QMimeData;
    mime->setData(QLatin1String(kStepMime), typeId.toUtf8());
    return mime;
}

QStringList StepTree::mimeTypes() const
{
    return QStringList() << QLatin1String(kStepMime);
}

void StepTree::onDoubleClicked(QTreeWidgetItem *item, int column)
{
    Q_UNUSED(column);
    if (!item)
        return;
    const QString typeId = item->data(0, kStepTypeRole).toString();
    if (!typeId.isEmpty())
        emit stepActivated(typeId);
}

StepLibrary::StepLibrary(const StepRegistry *registry, QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto *title = new QLabel(QStringLiteral("步骤库"));
    title->setObjectName(QStringLiteral("panelTitle"));
    title->setContentsMargins(12, 8, 12, 8);
    layout->addWidget(title);

    auto *search = new QLineEdit;
    search->setPlaceholderText(QStringLiteral("请输入关键词"));
    search->setClearButtonEnabled(true);
    search->setObjectName(QStringLiteral("stepSearch"));
    layout->addWidget(search);

    m_tree = new StepTree;
    layout->addWidget(m_tree, 1);

    const QStringList expanded = QStringList()
        << QStringLiteral("采集")
        << QStringLiteral("预处理")
        << QStringLiteral("数据处理");

    for (const QString &category : registry->categories()) {
        auto *group = new QTreeWidgetItem(m_tree);
        group->setText(0, category);
        group->setIcon(0, categoryIcon(category));
        group->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        for (const StepDesc &step : registry->stepsInCategory(category)) {
            auto *item = new QTreeWidgetItem(group);
            item->setText(0, step.title);
            item->setData(0, kStepTypeRole, step.typeId);
            item->setToolTip(0, step.summary + QStringLiteral("\n拖到画布，或双击添加"));
            item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled);
        }
        group->setExpanded(expanded.contains(category));
    }

    connect(search, &QLineEdit::textChanged, this, &StepLibrary::applyFilter);
    connect(m_tree, &StepTree::stepActivated, this, &StepLibrary::stepActivated);
}

void StepLibrary::applyFilter(const QString &text)
{
    const QString key = text.trimmed();
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        QTreeWidgetItem *group = m_tree->topLevelItem(i);
        bool any = false;
        for (int child = 0; child < group->childCount(); ++child) {
            QTreeWidgetItem *step = group->child(child);
            const bool match = key.isEmpty() || step->text(0).contains(key, Qt::CaseInsensitive);
            step->setHidden(!match);
            any = any || match;
        }
        group->setHidden(!any);
        if (!key.isEmpty() && any)
            group->setExpanded(true);
    }
}

QIcon StepLibrary::categoryIcon(const QString &category) const
{
    QPixmap pixmap(16, 16);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QPen(QColor("#c5c8cc"), 1.3));
    painter.setBrush(Qt::NoBrush);
    if (category == QStringLiteral("采集")) {
        painter.drawEllipse(QRectF(1.5, 1.5, 13, 13));
        painter.drawEllipse(QRectF(6, 6, 4, 4));
    } else if (category == QStringLiteral("预处理") || category == QStringLiteral("数据处理")) {
        painter.drawRoundedRect(QRectF(2, 2, 12, 12), 2, 2);
        painter.drawLine(QPointF(4, 6), QPointF(12, 6));
        painter.drawLine(QPointF(4, 10), QPointF(10, 10));
    } else if (category == QStringLiteral("路径规划") || category == QStringLiteral("传输")) {
        painter.drawLine(QPointF(2, 8), QPointF(12, 8));
        painter.drawLine(QPointF(9, 4), QPointF(13, 8));
        painter.drawLine(QPointF(9, 12), QPointF(13, 8));
    } else if (category == QStringLiteral("位姿") || category == QStringLiteral("位姿处理")) {
        painter.drawLine(QPointF(8, 2), QPointF(8, 14));
        painter.drawLine(QPointF(2, 8), QPointF(14, 8));
    } else {
        painter.drawRoundedRect(QRectF(2, 2, 12, 12), 2, 2);
    }
    return QIcon(pixmap);
}
