#ifndef FLOWEDITOR_MAINWINDOW_H
#define FLOWEDITOR_MAINWINDOW_H

#include "StepRegistry.h"
#include "Types.h"

#include <QHash>
#include <QMainWindow>

class FlowScene;
class FlowView;
class ProcedureList;
class QLabel;
class QTableWidget;
class QTabWidget;
class StepLibrary;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void showEvent(QShowEvent *event) override;

private:
    void buildUi();
    void loadInitial();
    GraphDocument demoDocument() const;
    void switchProcedure(const QString &name);
    void addStepFromLibrary(const QString &typeId);
    void clearCanvas();
    void appendLog(const QString &channel, const QString &level, const QString &message);
    QTableWidget *makeLogTable();

    StepRegistry m_registry;
    QHash<QString, GraphDocument> m_docs;
    QString m_current;
    FlowScene *m_scene = nullptr;
    FlowView *m_view = nullptr;
    StepLibrary *m_library = nullptr;
    ProcedureList *m_procedures = nullptr;
    QLabel *m_zoom = nullptr;
    QHash<QString, QTableWidget *> m_logs;
    bool m_framed = false;
};

#endif
