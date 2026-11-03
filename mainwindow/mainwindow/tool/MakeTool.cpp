#include "MakeTool.h"

#include <QAbstractItemView>
#include <QByteArray>
#include <QColor>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QFont>
#include <QMessageBox>
#include <QPalette>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextCodec>
#include <QVBoxLayout>

namespace {

QString decodeClassFile(const QByteArray &raw)
{
    QByteArray data = raw;
    if (data.startsWith("\xEF\xBB\xBF"))
        data.remove(0, 3);

    const QString utf8 = QString::fromUtf8(data);
    if (!utf8.contains(QChar(0xFFFD)))
        return utf8;

    // 中文 Windows 下的 class.txt 常见编码是 GBK / GB18030。
    if (QTextCodec *codec = QTextCodec::codecForName("GB18030"))
        return codec->toUnicode(data);
    return utf8;
}

QStringList classNamesFromText(const QString &text)
{
    QStringList names;
    const QStringList lines = text.split(QLatin1Char('\n'));
    for (QString line : lines) {
        if (line.endsWith(QLatin1Char('\r')))
            line.chop(1);
        line = line.trimmed();
        if (!line.isEmpty())
            names.append(line);
    }
    return names;
}

QTableWidgetItem *makeCell(const QString &text, Qt::Alignment alignment)
{
    auto *item = new QTableWidgetItem(text);
    item->setTextAlignment(alignment);
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    return item;
}

} // namespace

MakeTool::MakeTool(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
}

void MakeTool::setupUi()
{
    templateTableWidget = new QTableWidget(this);
    templateTableWidget->setObjectName(QStringLiteral("templateTableWidget"));

    const QString buttonStyle = QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #334155;"
        "  border: 1px solid #d0d7e2;"
        "  border-radius: 4px;"
        "  padding: 4px 16px;"
        "}"
        "QPushButton:hover { background-color: #f8fafc; }"
        "QPushButton:pressed { background-color: #eef2f7; }");

    openfileBtn = new QPushButton(QStringLiteral("打开"), this);
    openfileBtn->setObjectName(QStringLiteral("openfileBtn"));
    openfileBtn->setMinimumWidth(88);
    openfileBtn->setStyleSheet(buttonStyle);

    clearfileBtn = new QPushButton(QStringLiteral("清除"), this);
    clearfileBtn->setObjectName(QStringLiteral("clearfileBtn"));
    clearfileBtn->setMinimumWidth(88);
    clearfileBtn->setStyleSheet(buttonStyle);

    QPalette pagePalette = palette();
    pagePalette.setColor(QPalette::Window, QColor(0xf3, 0xf5, 0xf8));
    setPalette(pagePalette);
    setAutoFillBackground(true);

    auto *buttonLayout = new QHBoxLayout;
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->addStretch();
    buttonLayout->addWidget(openfileBtn);
    buttonLayout->addWidget(clearfileBtn);

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);
    layout->addWidget(templateTableWidget, 1);
    layout->addLayout(buttonLayout);

    setWindowTitle(QStringLiteral("模板列表"));
    resize(860, 380);
}

void MakeTool::applyTableStyle()
{
    QTableWidget *table = templateTableWidget;
    table->setColumnCount(3);
    table->setHorizontalHeaderLabels(QStringList()
                                     << QStringLiteral("序号")
                                     << QStringLiteral("识别类别")
                                     << QStringLiteral("文件地址"));
    table->verticalHeader()->setVisible(false);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setDefaultSectionSize(28);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setShowGrid(false);
    table->setWordWrap(false);
    table->setTextElideMode(Qt::ElideMiddle);
    table->setAlternatingRowColors(true);
    table->setCornerButtonEnabled(false);
    table->setFocusPolicy(Qt::StrongFocus);

    QPalette tablePalette = table->palette();
    tablePalette.setColor(QPalette::Base, QColor(0xff, 0xff, 0xff));
    tablePalette.setColor(QPalette::AlternateBase, QColor(0xf8, 0xfa, 0xfc));
    tablePalette.setColor(QPalette::Highlight, QColor(0xdb, 0xea, 0xfe));
    tablePalette.setColor(QPalette::HighlightedText, QColor(0x1e, 0x29, 0x3b));
    tablePalette.setColor(QPalette::Text, QColor(0x1e, 0x29, 0x3b));
    table->setPalette(tablePalette);

    QFont font;
#if QT_VERSION >= QT_VERSION_CHECK(5, 13, 0)
    font.setFamilies(QStringList()
                     << QStringLiteral("Microsoft YaHei")
                     << QStringLiteral("微软雅黑")
                     << QStringLiteral("WenQuanYi Micro Hei")
                     << QStringLiteral("Noto Sans CJK SC")
                     << QStringLiteral("Droid Sans Fallback"));
#else
    font.setFamily(QStringLiteral("Microsoft YaHei"));
#endif
    font.setPixelSize(13);
    font.setBold(false);
    table->setFont(font);
    table->horizontalHeader()->setFont(font);
    openfileBtn->setFont(font);
    clearfileBtn->setFont(font);

    QHeaderView *header = table->horizontalHeader();
    header->setHighlightSections(false);
    header->setSectionsClickable(false);
    header->setStretchLastSection(true);
    header->setMinimumSectionSize(56);
    header->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    header->setSectionResizeMode(0, QHeaderView::Fixed);
    header->setSectionResizeMode(1, QHeaderView::Interactive);
    header->setSectionResizeMode(2, QHeaderView::Stretch);
    header->setFixedHeight(32);
    table->setColumnWidth(0, 72);
    table->setColumnWidth(1, 150);

    table->setStyleSheet(QStringLiteral(
        "QTableWidget {"
        "  background-color: #ffffff;"
        "  alternate-background-color: #f8fafc;"
        "  border: 1px solid #e6ebf2;"
        "  selection-background-color: #dbeafe;"
        "  selection-color: #1e293b;"
        "  outline: none;"
        "}"
        "QTableWidget::item {"
        "  padding: 0px 8px;"
        "  border: none;"
        "  border-bottom: 1px solid #eef2f6;"
        "  color: #1e293b;"
        "}"
        "QTableWidget::item:selected,"
        "QTableWidget::item:selected:active,"
        "QTableWidget::item:selected:!active {"
        "  background-color: #dbeafe;"
        "  color: #1e293b;"
        "}"
        "QHeaderView::section {"
        "  background-color: #eef2f7;"
        "  color: #334155;"
        "  padding: 0px 8px;"
        "  border: none;"
        "  border-bottom: 1px solid #e6ebf2;"
        "  border-right: 1px solid #eef2f6;"
        "  font-weight: 400;"
        "}"));
}

void MakeTool::bindFileButtons()
{
    if (m_fileBtnsBound)
        return;

    connect(openfileBtn, &QPushButton::clicked, this, [this]() { openSelectedFile(); });
    connect(clearfileBtn, &QPushButton::clicked, this, [this]() { clearSelectedFile(); });
    m_fileBtnsBound = true;
}

void MakeTool::loadClassTable(const QString &classTxtPath)
{
    if (!templateTableWidget || !openfileBtn || !clearfileBtn)
        return;

    applyTableStyle();
    bindFileButtons();

    QFile file(classTxtPath);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this,
                             QStringLiteral("提示"),
                             QStringLiteral("无法打开 class.txt：\n%1").arg(classTxtPath));
        return;
    }

    const QByteArray raw = file.readAll();
    file.close();
    m_classTxtDir = QFileInfo(classTxtPath).absolutePath();

    const QStringList classes = classNamesFromText(decodeClassFile(raw));
    QTableWidget *table = templateTableWidget;
    table->clearContents();
    table->setRowCount(classes.size());

    for (int i = 0; i < classes.size(); ++i) {
        table->setItem(i, 0, makeCell(QString::number(i + 1), Qt::AlignCenter));
        table->setItem(i, 1, makeCell(classes.at(i), Qt::AlignLeft | Qt::AlignVCenter));
        table->setItem(i, 2, makeCell(QString(), Qt::AlignLeft | Qt::AlignVCenter));
    }

    if (table->rowCount() > 0)
        table->selectRow(0);
    else
        table->clearSelection();
}

void MakeTool::openSelectedFile()
{
    const int row = templateTableWidget->currentRow();
    if (row < 0) {
        QMessageBox::information(this,
                                 QStringLiteral("提示"),
                                 QStringLiteral("请先选择一行"));
        return;
    }

    QString startDir = m_classTxtDir;
    if (QTableWidgetItem *existing = templateTableWidget->item(row, 2)) {
        const QString current = existing->text().trimmed();
        if (!current.isEmpty()) {
            QFileInfo info(current);
            if (info.isRelative())
                info.setFile(QDir(m_classTxtDir).filePath(current));
            if (!info.absolutePath().isEmpty())
                startDir = info.absolutePath();
        }
    }

    const QString path = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("选择文件"),
        startDir,
        QStringLiteral("文本文件 (*.txt);;所有文件 (*.*)"));
    if (path.isEmpty())
        return;

    QTableWidgetItem *item = templateTableWidget->item(row, 2);
    if (!item) {
        item = makeCell(QString(), Qt::AlignLeft | Qt::AlignVCenter);
        templateTableWidget->setItem(row, 2, item);
    }
    item->setText(path);
    item->setToolTip(path);
}

void MakeTool::clearSelectedFile()
{
    const int row = templateTableWidget->currentRow();
    if (row < 0) {
        QMessageBox::information(this,
                                 QStringLiteral("提示"),
                                 QStringLiteral("请先选择一行"));
        return;
    }

    if (QTableWidgetItem *item = templateTableWidget->item(row, 2)) {
        item->setText(QString());
        item->setToolTip(QString());
    }
}
