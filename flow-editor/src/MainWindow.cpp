#include "MainWindow.h"

#include "FlowScene.h"
#include "FlowView.h"
#include "NodeItem.h"
#include "ProcedureList.h"
#include "StepLibrary.h"

#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QShowEvent>
#include <QSplitter>
#include <QTableWidget>
#include <QTabWidget>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>

namespace {

QString darkStyle()
{
    return QStringLiteral(
        "QWidget { background: #1e1e1e; color: #d8d8d8; }"
        "QLabel#panelTitle { background: #2a2d31; color: #e8e8e8; font-size: 13px; }"
        "QLabel#zoomLabel { color: #9aa0a6; padding-right: 8px; }"
        "QSplitter::handle { background: #121314; }"
        "QTreeWidget, QListWidget, QTableWidget { background: #252628; border: none; outline: none; }"
        "QTreeWidget::item { height: 26px; }"
        "QTreeWidget::item:selected { background: #3a3d41; color: #ffffff; }"
        "QTreeWidget::item:hover { background: #2f3338; }"
        "QListWidget::item:selected { background: transparent; }"
        "QLineEdit { background: #1b1c1e; border: 1px solid #3a3d41; border-radius: 3px;"
        "  padding: 4px 8px; margin: 8px; color: #e0e0e0; }"
        "QHeaderView::section { background: #2a2d31; color: #a0a4a8; border: none; padding: 4px 8px; }"
        "QTableWidget { alternate-background-color: #222426; selection-background-color: #314448; }"
        "QTabWidget::pane { border: none; background: #1e1e1e; }"
        "QTabBar::tab { background: #2a2d31; color: #a0a4a8; padding: 6px 16px; border: none; }"
        "QTabBar::tab:selected { background: #1e1e1e; color: #ffffff; }"
        "QPushButton { background: #2d3034; border: 1px solid #3c4044; border-radius: 3px; padding: 4px 14px; }"
        "QPushButton:hover { background: #3a3e44; }"
        "QPushButton#runButton { background: #1f7a43; color: white; border: none; }"
        "QPushButton#runButton:hover { background: #259352; }"
        "QPushButton#loopButton { background: #1a5c38; color: white; border: none; }"
        "QPushButton#loopButton:hover { background: #217448; }"
        "QScrollBar:vertical { background: #1e1e1e; width: 10px; margin: 0; }"
        "QScrollBar::handle:vertical { background: #3a3e44; min-height: 24px; border-radius: 4px; }"
        "QScrollBar::add-line, QScrollBar::sub-line { height: 0; }"
        "QMenu { background: #2a2d31; border: 1px solid #3c4044; }"
        "QMenu::item { padding: 6px 18px; }"
        "QMenu::item:selected { background: #245e66; }"
        "QToolTip { background: #2a2d31; color: #e8e8e8; border: 1px solid #3c4044; }"
        "QRubberBand { background: rgba(46, 204, 196, 40); border: 1px solid #2ee0d0; }");
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setStyleSheet(darkStyle());
    buildUi();
    loadInitial();
    resize(1360, 860);
    setMinimumSize(1024, 640);
}

void MainWindow::buildUi()
{
    m_scene = new FlowScene(&m_registry, this);
    m_view = new FlowView(m_scene, this);
    m_library = new StepLibrary(&m_registry, this);
    m_procedures = new ProcedureList(this);

    auto *clearButton = new QPushButton(QStringLiteral("清空"));
    clearButton->setToolTip(QStringLiteral("删除画布上的全部步骤"));
    auto *runButton = new QPushButton(QStringLiteral("运行"));
    runButton->setObjectName(QStringLiteral("runButton"));
    runButton->setToolTip(QStringLiteral("按连线的依赖顺序执行一次 (F5)"));
    runButton->setShortcut(Qt::Key_F5);
    auto *loopButton = new QPushButton(QStringLiteral("连续运行"));
    loopButton->setObjectName(QStringLiteral("loopButton"));
    loopButton->setToolTip(QStringLiteral("按当前连线执行一轮"));
    auto *fitButton = new QPushButton(QStringLiteral("适配"));
    fitButton->setToolTip(QStringLiteral("把流程移到视口中心"));

    m_zoom = new QLabel(QStringLiteral("100%"));
    m_zoom->setObjectName(QStringLiteral("zoomLabel"));

    auto *toolbar = new QWidget;
    auto *toolbarLayout = new QHBoxLayout(toolbar);
    toolbarLayout->setContentsMargins(8, 6, 8, 6);
    toolbarLayout->addWidget(clearButton);
    toolbarLayout->addStretch();
    toolbarLayout->addWidget(runButton);
    toolbarLayout->addWidget(loopButton);
    toolbarLayout->addStretch();
    toolbarLayout->addWidget(fitButton);
    toolbarLayout->addWidget(m_zoom);

    auto *canvas = new QWidget;
    auto *canvasLayout = new QVBoxLayout(canvas);
    canvasLayout->setContentsMargins(0, 0, 0, 0);
    canvasLayout->setSpacing(0);
    canvasLayout->addWidget(toolbar);
    canvasLayout->addWidget(m_view, 1);

    auto *logTitle = new QLabel(QStringLiteral("日志"));
    logTitle->setObjectName(QStringLiteral("panelTitle"));
    logTitle->setContentsMargins(12, 6, 12, 6);

    auto *tabs = new QTabWidget;
    const QStringList channels = QStringList()
        << QStringLiteral("视觉")
        << QStringLiteral("控制台")
        << QStringLiteral("操作");
    for (const QString &channel : channels) {
        QTableWidget *table = makeLogTable();
        m_logs.insert(channel, table);
        tabs->addTab(table, channel);
    }

    auto *logPanel = new QWidget;
    auto *logLayout = new QVBoxLayout(logPanel);
    logLayout->setContentsMargins(0, 0, 0, 0);
    logLayout->setSpacing(0);
    logLayout->addWidget(logTitle);
    logLayout->addWidget(tabs, 1);
    logPanel->setMinimumHeight(150);

    auto *rightSplit = new QSplitter(Qt::Vertical);
    rightSplit->addWidget(canvas);
    rightSplit->addWidget(logPanel);
    rightSplit->setStretchFactor(0, 1);
    rightSplit->setStretchFactor(1, 0);
    rightSplit->setSizes(QList<int>() << 640 << 180);
    rightSplit->setChildrenCollapsible(false);

    auto *leftSplit = new QSplitter(Qt::Vertical);
    leftSplit->addWidget(m_procedures);
    leftSplit->addWidget(m_library);
    leftSplit->setStretchFactor(0, 1);
    leftSplit->setStretchFactor(1, 2);
    leftSplit->setSizes(QList<int>() << 280 << 460);
    leftSplit->setChildrenCollapsible(false);

    auto *split = new QSplitter;
    split->addWidget(leftSplit);
    split->addWidget(rightSplit);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);
    split->setSizes(QList<int>() << 260 << 1080);
    split->setChildrenCollapsible(false);
    setCentralWidget(split);

    connect(m_scene, &FlowScene::userMessage, this, &MainWindow::appendLog);
    connect(m_library, &StepLibrary::stepActivated, this, &MainWindow::addStepFromLibrary);
    connect(m_procedures, &ProcedureList::procedureSelected, this, &MainWindow::switchProcedure);
    connect(runButton, &QPushButton::clicked, this, [this]() { m_scene->execute(false); });
    connect(loopButton, &QPushButton::clicked, this, [this]() { m_scene->execute(true); });
    connect(clearButton, &QPushButton::clicked, this, &MainWindow::clearCanvas);
    connect(fitButton, &QPushButton::clicked, m_view, &FlowView::frameItems);
    connect(m_view, &FlowView::zoomChanged, this, [this](qreal factor) {
        m_zoom->setText(QString::number(qRound(factor * 100)) + QLatin1Char('%'));
    });
}

QTableWidget *MainWindow::makeLogTable()
{
    auto *table = new QTableWidget(0, 3);
    table->setHorizontalHeaderLabels(QStringList()
                                     << QStringLiteral("时间")
                                     << QStringLiteral("等级")
                                     << QStringLiteral("消息"));
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    table->horizontalHeader()->setStretchLastSection(true);
    table->horizontalHeader()->setHighlightSections(false);
    table->verticalHeader()->setVisible(false);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setShowGrid(false);
    table->setAlternatingRowColors(true);
    table->setFocusPolicy(Qt::NoFocus);
    return table;
}

GraphDocument MainWindow::demoDocument() const
{
    GraphDocument document;

    GraphNode read;
    read.id = QStringLiteral("n-read");
    read.typeId = QStringLiteral("read_cloud_v2");
    read.pos = QPointF(470, 30);
    read.index = 1;

    GraphNode filter;
    filter.id = QStringLiteral("n-filter");
    filter.typeId = QStringLiteral("calc_normals");
    filter.pos = QPointF(780, 270);
    filter.index = 2;

    GraphNode group;
    group.id = QStringLiteral("n-group");
    group.typeId = QStringLiteral("step_group");
    group.pos = QPointF(230, 300);
    group.index = 3;

    document.nodes << read << filter << group;

    GraphEdge toFilter;
    toFilter.fromId = read.id;
    toFilter.fromKey = QStringLiteral("cloud");
    toFilter.toId = filter.id;
    toFilter.toKey = QStringLiteral("cloud");

    GraphEdge toGroup;
    toGroup.fromId = read.id;
    toGroup.fromKey = QStringLiteral("cloud");
    toGroup.toId = group.id;
    toGroup.toKey = QStringLiteral("cloud");
    document.edges << toFilter << toGroup;
    return document;
}

void MainWindow::loadInitial()
{
    const QStringList names = QStringList()
        << QStringLiteral("拼接主程序")
        << QStringLiteral("7")
        << QStringLiteral("工程8")
        << QStringLiteral("9")
        << QStringLiteral("拍一打多")
        << QStringLiteral("拍一打多 副本")
        << QStringLiteral("demo")
        << QStringLiteral("测试")
        << QStringLiteral("算子库测试");
    for (const QString &name : names)
        m_docs.insert(name, GraphDocument());
    m_current = QStringLiteral("算子库测试");
    m_docs[m_current] = demoDocument();
    m_procedures->setProcedures(names);
    m_procedures->setCurrent(m_current);
    m_scene->load(m_docs.value(m_current));
    for (NodeItem *node : m_scene->nodes()) {
        if (node->typeId() == QLatin1String("calc_normals"))
            node->setSelected(true);
    }
    setWindowTitle(QStringLiteral("流程编辑器 - ") + m_current);
    appendLog(QStringLiteral("视觉"), QStringLiteral("信息"),
              QStringLiteral("已载入示例流程「算子库测试」"));
    appendLog(QStringLiteral("操作"), QStringLiteral("信息"),
              QStringLiteral("从步骤库拖拽步骤到画布。从输出端口拖到输入端口完成连线。Delete 删除，滚轮缩放，中键平移。"));
}

void MainWindow::switchProcedure(const QString &name)
{
    if (name.isEmpty() || name == m_current)
        return;
    m_docs[m_current] = m_scene->capture();
    m_current = name;
    m_scene->load(m_docs.value(name));
    setWindowTitle(QStringLiteral("流程编辑器 - ") + m_current);
    appendLog(QStringLiteral("操作"), QStringLiteral("信息"),
              QStringLiteral("已切换工程 ") + name);
    m_view->frameItems();
}

void MainWindow::addStepFromLibrary(const QString &typeId)
{
    m_scene->addNode(typeId, m_view->centerScenePos());
    m_view->setFocus();
}

void MainWindow::clearCanvas()
{
    m_scene->clearGraph();
    appendLog(QStringLiteral("操作"), QStringLiteral("信息"), QStringLiteral("已清空画布"));
    m_view->viewport()->update();
}

void MainWindow::appendLog(const QString &channel, const QString &level, const QString &message)
{
    QTableWidget *table = m_logs.value(channel);
    if (!table)
        table = m_logs.value(QStringLiteral("操作"));
    if (!table)
        return;
    const int row = table->rowCount();
    table->insertRow(row);
    auto *timeItem = new QTableWidgetItem(QTime::currentTime().toString(QStringLiteral("HH:mm:ss")));
    auto *levelItem = new QTableWidgetItem(level);
    auto *messageItem = new QTableWidgetItem(message);
    if (level == QStringLiteral("警告"))
        levelItem->setForeground(QColor("#e0b15a"));
    else if (level == QStringLiteral("错误"))
        levelItem->setForeground(QColor("#e06a6a"));
    else
        levelItem->setForeground(QColor("#8b939a"));
    table->setItem(row, 0, timeItem);
    table->setItem(row, 1, levelItem);
    table->setItem(row, 2, messageItem);
    table->scrollToBottom();
}

void MainWindow::showEvent(QShowEvent *event)
{
    QMainWindow::showEvent(event);
    if (!m_framed) {
        m_framed = true;
        QTimer::singleShot(0, this, [this]() { m_view->frameItems(); });
    }
}
