#ifndef FLOWEDITOR_STEPLIBRARY_H
#define FLOWEDITOR_STEPLIBRARY_H

#include <QTreeWidget>

class StepRegistry;

constexpr int kStepTypeRole = Qt::UserRole + 1;

class StepTree : public QTreeWidget
{
    Q_OBJECT
public:
    explicit StepTree(QWidget *parent = nullptr);

signals:
    void stepActivated(const QString &typeId);

protected:
    QMimeData *mimeData(const QList<QTreeWidgetItem *> items) const override;
    QStringList mimeTypes() const override;

private slots:
    void onDoubleClicked(QTreeWidgetItem *item, int column);
};

class StepLibrary : public QWidget
{
    Q_OBJECT
public:
    explicit StepLibrary(const StepRegistry *registry, QWidget *parent = nullptr);

signals:
    void stepActivated(const QString &typeId);

private slots:
    void applyFilter(const QString &text);

private:
    QIcon categoryIcon(const QString &category) const;

    StepTree *m_tree = nullptr;
};

#endif
