#include "ProcedureList.h"

#include "Types.h"

#include <QLabel>
#include <QListWidget>
#include <QPainter>
#include <QSignalBlocker>
#include <QStyledItemDelegate>
#include <QVBoxLayout>

namespace {

class ProcedureDelegate : public QStyledItemDelegate
{
public:
    explicit ProcedureDelegate(QObject *parent = nullptr)
        : QStyledItemDelegate(parent)
    {
    }

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override
    {
        painter->save();
        painter->setRenderHint(QPainter::Antialiasing);
        if (option.state & QStyle::State_Selected)
            painter->fillRect(option.rect, QColor("#3a3d41"));
        else if (option.state & QStyle::State_MouseOver)
            painter->fillRect(option.rect, QColor("#2e3236"));

        const QString name = index.data(Qt::DisplayRole).toString();
        const int number = index.data(Qt::UserRole).toInt();
        painter->setFont(uiFont(13));

        QRect textRect = option.rect.adjusted(10, 0, -8, 0);
        painter->setPen(QColor("#8b9096"));
        painter->drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, QString::number(number));

        textRect.setLeft(option.rect.left() + 36);
        painter->setPen(QColor("#43c46a"));
        painter->drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, QStringLiteral("✓  ") + name);
        painter->restore();
    }

    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const override
    {
        QSize size = QStyledItemDelegate::sizeHint(option, index);
        size.setHeight(26);
        return size;
    }
};

} // namespace

ProcedureList::ProcedureList(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto *title = new QLabel(QStringLiteral("拼接程序"));
    title->setObjectName(QStringLiteral("panelTitle"));
    title->setContentsMargins(12, 8, 12, 8);
    layout->addWidget(title);

    m_list = new QListWidget;
    m_list->setFrameShape(QFrame::NoFrame);
    m_list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_list->setItemDelegate(new ProcedureDelegate(m_list));
    m_list->setMouseTracking(true);
    layout->addWidget(m_list, 1);

    connect(m_list, &QListWidget::currentItemChanged, this, [this](QListWidgetItem *current) {
        if (!current)
            return;
        emit procedureSelected(current->data(Qt::DisplayRole).toString());
    });
}

void ProcedureList::setProcedures(const QStringList &names)
{
    m_list->clear();
    int number = 6;
    for (const QString &name : names) {
        auto *item = new QListWidgetItem(name);
        item->setData(Qt::UserRole, number);
        m_list->addItem(item);
        ++number;
    }
}

void ProcedureList::setCurrent(const QString &name)
{
    QSignalBlocker blocker(m_list);
    for (int row = 0; row < m_list->count(); ++row) {
        QListWidgetItem *item = m_list->item(row);
        if (item->text() == name) {
            m_list->setCurrentItem(item);
            return;
        }
    }
}

QString ProcedureList::currentName() const
{
    QListWidgetItem *item = m_list->currentItem();
    return item ? item->text() : QString();
}
