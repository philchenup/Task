#ifndef FLOWEDITOR_PROCEDURELIST_H
#define FLOWEDITOR_PROCEDURELIST_H

#include <QWidget>

class QListWidget;

class ProcedureList : public QWidget
{
    Q_OBJECT
public:
    explicit ProcedureList(QWidget *parent = nullptr);

    void setProcedures(const QStringList &names);
    void setCurrent(const QString &name);
    QString currentName() const;

signals:
    void procedureSelected(const QString &name);

private:
    QListWidget *m_list = nullptr;
};

#endif
