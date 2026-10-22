/**
 * @file ui_TcpCalib.h
 * Hand-built Qt layout for TcpCalib.
 *
 * Control object names match the previous Designer form so TcpCalib.cpp
 * can keep the same slots and table reads. Layout follows the workspace
 * redesign: mode controls on the left, preview above, P1-P4 and P5-P6
 * side by side underneath.
 */
#ifndef UI_TCPCALIB_H
#define UI_TCPCALIB_H

#include <QVTKOpenGLNativeWidget.h>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TcpCalib
{
public:
    QVBoxLayout *verticalLayout = nullptr;
    QVBoxLayout *verticalLayout_2 = nullptr;
    QHBoxLayout *horizontalLayout_2 = nullptr;
    QLabel *label = nullptr;
    QPushButton *checkPdfBtn = nullptr;
    QFrame *line_6 = nullptr;
    QHBoxLayout *horizontalLayout_3 = nullptr;
    QRadioButton *eulerBtn = nullptr;
    QComboBox *eulerorderComb = nullptr;
    QFrame *line = nullptr;
    QRadioButton *quatBtn = nullptr;
    QFrame *line_3 = nullptr;
    QRadioButton *rotateVecBtn = nullptr;
    QLabel *label_3 = nullptr;
    QStackedWidget *panelWidget = nullptr;
    QWidget *page = nullptr;
    QVBoxLayout *verticalLayout_4 = nullptr;
    QVBoxLayout *verticalLayout_3 = nullptr;
    QTableWidget *fourPTable = nullptr;
    QVBoxLayout *verticalLayout_9 = nullptr;
    QTableWidget *sixPTable = nullptr;
    QWidget *page_2 = nullptr;
    QVBoxLayout *verticalLayout_8 = nullptr;
    QVBoxLayout *verticalLayout_7 = nullptr;
    QVBoxLayout *verticalLayout_5 = nullptr;
    QTableWidget *quatFourTable = nullptr;
    QFrame *line_4 = nullptr;
    QVBoxLayout *verticalLayout_6 = nullptr;
    QTableWidget *quatSixTable = nullptr;
    QFrame *line_2 = nullptr;
    QVTKOpenGLNativeWidget *qvtkWidget = nullptr;
    QLabel *label_2 = nullptr;
    QHBoxLayout *horizontalLayout = nullptr;
    QPushButton *Calib4pTcpBtn = nullptr;
    QPushButton *Calib6pTcpBtn = nullptr;
    QSpacerItem *horizontalSpacer = nullptr;
    QPushButton *saveTcpDataBtn = nullptr;

    void setupUi(QDialog *TcpCalib)
    {
        if (TcpCalib->objectName().isEmpty())
            TcpCalib->setObjectName(QStringLiteral("TcpCalib"));
        TcpCalib->resize(1280, 760);
        TcpCalib->setMinimumSize(QSize(1180, 680));

        verticalLayout = new QVBoxLayout(TcpCalib);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setSpacing(8);
        verticalLayout->setContentsMargins(12, 10, 12, 10);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setSpacing(12);
        label = new QLabel(TcpCalib);
        label->setObjectName(QStringLiteral("label"));
        label->setWordWrap(true);
        checkPdfBtn = new QPushButton(TcpCalib);
        checkPdfBtn->setObjectName(QStringLiteral("checkPdfBtn"));
        checkPdfBtn->setMinimumSize(QSize(96, 30));
        horizontalLayout_2->addWidget(label, 1);
        horizontalLayout_2->addWidget(checkPdfBtn, 0, Qt::AlignRight | Qt::AlignVCenter);
        verticalLayout->addLayout(horizontalLayout_2);

        line_6 = makeHLine(TcpCalib, QStringLiteral("line_6"));
        verticalLayout->addWidget(line_6);

        auto *body = new QHBoxLayout();
        body->setSpacing(12);

        auto *side = new QWidget(TcpCalib);
        side->setObjectName(QStringLiteral("sidePanel"));
        side->setFixedWidth(230);
        auto *sideLayout = new QVBoxLayout(side);
        sideLayout->setContentsMargins(0, 0, 0, 0);
        sideLayout->setSpacing(8);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setSpacing(8);
        eulerBtn = makeModeButton(side, QStringLiteral("eulerBtn"));
        eulerorderComb = new QComboBox(side);
        eulerorderComb->setObjectName(QStringLiteral("eulerorderComb"));
        eulerorderComb->addItem(QString());
        eulerorderComb->addItem(QString());
        eulerorderComb->addItem(QString());
        eulerorderComb->setMinimumWidth(72);
        horizontalLayout_3->addWidget(eulerBtn, 1);
        horizontalLayout_3->addWidget(eulerorderComb);
        sideLayout->addLayout(horizontalLayout_3);

        line = makeHLine(side, QStringLiteral("line"));
        sideLayout->addWidget(line);

        quatBtn = makeModeButton(side, QStringLiteral("quatBtn"));
        sideLayout->addWidget(quatBtn);

        line_3 = makeHLine(side, QStringLiteral("line_3"));
        sideLayout->addWidget(line_3);

        rotateVecBtn = makeModeButton(side, QStringLiteral("rotateVecBtn"));
        sideLayout->addWidget(rotateVecBtn);

        label_3 = new QLabel(side);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setWordWrap(true);
        sideLayout->addWidget(label_3);
        sideLayout->addStretch(1);

        Calib4pTcpBtn = makeActionButton(side, QStringLiteral("Calib4pTcpBtn"));
        Calib6pTcpBtn = makeActionButton(side, QStringLiteral("Calib6pTcpBtn"));
        sideLayout->addWidget(Calib4pTcpBtn);
        sideLayout->addWidget(Calib6pTcpBtn);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 4, 0, 0);
        saveTcpDataBtn = makeActionButton(side, QStringLiteral("saveTcpDataBtn"));
        horizontalSpacer = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Minimum);
        horizontalLayout->addWidget(saveTcpDataBtn, 1);
        horizontalLayout->addItem(horizontalSpacer);
        sideLayout->addLayout(horizontalLayout);
        body->addWidget(side);

        auto *stageColumn = new QWidget(TcpCalib);
        verticalLayout_2 = new QVBoxLayout(stageColumn);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2->setSpacing(6);

        qvtkWidget = new QVTKOpenGLNativeWidget(stageColumn);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        qvtkWidget->setMinimumHeight(260);
        verticalLayout_2->addWidget(qvtkWidget, 1);

        label_2 = new QLabel(stageColumn);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setWordWrap(true);
        verticalLayout_2->addWidget(label_2);

        line_2 = makeHLine(stageColumn, QStringLiteral("line_2"));
        verticalLayout_2->addWidget(line_2);

        panelWidget = new QStackedWidget(stageColumn);
        panelWidget->setObjectName(QStringLiteral("panelWidget"));
        panelWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        page = new QWidget();
        page->setObjectName(QStringLiteral("page"));
        verticalLayout_4 = new QVBoxLayout(page);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        auto *eulerRow = new QHBoxLayout();
        eulerRow->setSpacing(12);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        fourPTable = new QTableWidget(page);
        fourPTable->setObjectName(QStringLiteral("fourPTable"));
        preparePoseTable(fourPTable, 6, 4);
        verticalLayout_3->addWidget(fourPTable, 0, Qt::AlignTop);
        eulerRow->addLayout(verticalLayout_3, 1);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        verticalLayout_9->setContentsMargins(0, 0, 0, 0);
        sixPTable = new QTableWidget(page);
        sixPTable->setObjectName(QStringLiteral("sixPTable"));
        preparePoseTable(sixPTable, 6, 2);
        verticalLayout_9->addWidget(sixPTable, 0, Qt::AlignTop);
        eulerRow->addLayout(verticalLayout_9, 1);
        verticalLayout_4->addLayout(eulerRow);
        panelWidget->addWidget(page);

        page_2 = new QWidget();
        page_2->setObjectName(QStringLiteral("page_2"));
        verticalLayout_8 = new QVBoxLayout(page_2);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        verticalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        auto *quatRow = new QHBoxLayout();
        quatRow->setSpacing(12);
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        quatFourTable = new QTableWidget(page_2);
        quatFourTable->setObjectName(QStringLiteral("quatFourTable"));
        preparePoseTable(quatFourTable, 7, 4);
        verticalLayout_5->addWidget(quatFourTable, 0, Qt::AlignTop);
        quatRow->addLayout(verticalLayout_5, 1);

        line_4 = new QFrame(page_2);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::VLine);
        line_4->setFrameShadow(QFrame::Sunken);
        quatRow->addWidget(line_4);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        quatSixTable = new QTableWidget(page_2);
        quatSixTable->setObjectName(QStringLiteral("quatSixTable"));
        preparePoseTable(quatSixTable, 7, 2);
        verticalLayout_6->addWidget(quatSixTable, 0, Qt::AlignTop);
        quatRow->addLayout(verticalLayout_6, 1);
        verticalLayout_7->addLayout(quatRow);
        verticalLayout_8->addLayout(verticalLayout_7);
        panelWidget->addWidget(page_2);

        const int dockHeight = qMax(fourPTable->minimumHeight(), quatFourTable->minimumHeight());
        panelWidget->setFixedHeight(dockHeight + 8);
        verticalLayout_2->addWidget(panelWidget);
        body->addWidget(stageColumn, 1);
        verticalLayout->addLayout(body, 1);

        retranslateUi(TcpCalib);
        panelWidget->setCurrentIndex(0);
        eulerBtn->setChecked(true);

        QWidget::setTabOrder(checkPdfBtn, eulerBtn);
        QWidget::setTabOrder(eulerBtn, eulerorderComb);
        QWidget::setTabOrder(eulerorderComb, quatBtn);
        QWidget::setTabOrder(quatBtn, rotateVecBtn);
        QWidget::setTabOrder(rotateVecBtn, fourPTable);
        QWidget::setTabOrder(fourPTable, sixPTable);
        QWidget::setTabOrder(sixPTable, quatFourTable);
        QWidget::setTabOrder(quatFourTable, quatSixTable);
        QWidget::setTabOrder(quatSixTable, Calib4pTcpBtn);
        QWidget::setTabOrder(Calib4pTcpBtn, Calib6pTcpBtn);
        QWidget::setTabOrder(Calib6pTcpBtn, saveTcpDataBtn);

        QMetaObject::connectSlotsByName(TcpCalib);
    }

    void retranslateUi(QDialog *TcpCalib)
    {
        TcpCalib->setWindowTitle(QCoreApplication::translate("TcpCalib", "Tcp\346\240\207\345\256\232\345\267\245\345\205\267", nullptr));
        label->setText(QCoreApplication::translate("TcpCalib", "\345\217\257\344\273\245\347\202\271\345\207\273\346\237\245\347\234\213\346\211\213\345\206\214\346\214\211\351\222\256\346\211\223\345\274\200Tcp\346\240\207\345\256\232\346\265\201\347\250\213", nullptr));
        checkPdfBtn->setText(QCoreApplication::translate("TcpCalib", "\346\237\245\347\234\213\346\211\213\345\206\214", nullptr));
        eulerBtn->setText(QCoreApplication::translate("TcpCalib", "\346\254\247\346\213\211\350\247\222", nullptr));
        eulerorderComb->setItemText(0, QCoreApplication::translate("TcpCalib", "ZYX", nullptr));
        eulerorderComb->setItemText(1, QCoreApplication::translate("TcpCalib", "XYZ", nullptr));
        eulerorderComb->setItemText(2, QCoreApplication::translate("TcpCalib", "YZY", nullptr));
        quatBtn->setText(QCoreApplication::translate("TcpCalib", "\345\233\233\345\205\203\346\225\260", nullptr));
        rotateVecBtn->setText(QCoreApplication::translate("TcpCalib", "\346\227\213\350\275\254\347\237\242\351\207\217", nullptr));
        label_3->setText(QCoreApplication::translate("TcpCalib", "\346\217\220\347\244\272:\350\276\223\345\205\245\346\225\260\346\215\256\344\270\255\350\267\235\347\246\273\345\215\225\344\275\215\344\270\272mm\357\274\214\350\247\222\345\272\246\345\215\225\344\275\215\344\270\272Degree", nullptr));

        setPoseHeaders(fourPTable,
                       {QCoreApplication::translate("TcpCalib", "X(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Y(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Z(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Rx(\302\260)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Ry(\302\260)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Rz(\302\260)", nullptr)},
                       {QCoreApplication::translate("TcpCalib", "P1", nullptr),
                        QCoreApplication::translate("TcpCalib", "P2", nullptr),
                        QCoreApplication::translate("TcpCalib", "P3", nullptr),
                        QCoreApplication::translate("TcpCalib", "P4", nullptr)});
        setPoseHeaders(sixPTable,
                       {QCoreApplication::translate("TcpCalib", "X(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Y(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Z(mm)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Rx(\302\260)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Ry(\302\260)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Rz(\302\260)", nullptr)},
                       {QCoreApplication::translate("TcpCalib", "P5", nullptr),
                        QCoreApplication::translate("TcpCalib", "P6", nullptr)});
        setPoseHeaders(quatFourTable,
                       {QCoreApplication::translate("TcpCalib", "X(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Y(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Z(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "w", nullptr),
                        QCoreApplication::translate("TcpCalib", "x", nullptr),
                        QCoreApplication::translate("TcpCalib", "y", nullptr),
                        QCoreApplication::translate("TcpCalib", "z", nullptr)},
                       {QCoreApplication::translate("TcpCalib", "P1", nullptr),
                        QCoreApplication::translate("TcpCalib", "P2", nullptr),
                        QCoreApplication::translate("TcpCalib", "P3", nullptr),
                        QCoreApplication::translate("TcpCalib", "P4", nullptr)});
        setPoseHeaders(quatSixTable,
                       {QCoreApplication::translate("TcpCalib", "X(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Y(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "Z(m)", nullptr),
                        QCoreApplication::translate("TcpCalib", "w", nullptr),
                        QCoreApplication::translate("TcpCalib", "x", nullptr),
                        QCoreApplication::translate("TcpCalib", "y", nullptr),
                        QCoreApplication::translate("TcpCalib", "z", nullptr)},
                       {QCoreApplication::translate("TcpCalib", "P5", nullptr),
                        QCoreApplication::translate("TcpCalib", "P6", nullptr)});

        label_2->setText(QCoreApplication::translate("TcpCalib", "\346\217\220\347\244\272:\345\260\272\345\257\270\346\233\264\345\244\247\347\232\204\344\270\272\346\263\225\345\205\260\345\235\220\346\240\207\347\263\273", nullptr));
        Calib4pTcpBtn->setText(QCoreApplication::translate("TcpCalib", "4\347\202\271\346\263\225\346\240\207\345\256\232", nullptr));
        Calib6pTcpBtn->setText(QCoreApplication::translate("TcpCalib", "6\347\202\271\346\263\225\346\240\207\345\256\232", nullptr));
        saveTcpDataBtn->setText(QCoreApplication::translate("TcpCalib", "\344\277\235\345\255\230\346\240\207\345\256\232\346\225\260\346\215\256", nullptr));
    }

private:
    static QFrame *makeHLine(QWidget *parent, const QString &name)
    {
        auto *rule = new QFrame(parent);
        rule->setObjectName(name);
        rule->setFrameShape(QFrame::HLine);
        rule->setFrameShadow(QFrame::Sunken);
        return rule;
    }

    static QRadioButton *makeModeButton(QWidget *parent, const QString &name)
    {
        auto *button = new QRadioButton(parent);
        button->setObjectName(name);
        button->setAutoExclusive(true);
        button->setMinimumHeight(28);
        return button;
    }

    static QPushButton *makeActionButton(QWidget *parent, const QString &name)
    {
        auto *button = new QPushButton(parent);
        button->setObjectName(name);
        button->setMinimumHeight(32);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        return button;
    }

    static void preparePoseTable(QTableWidget *table, int columns, int rows)
    {
        table->setColumnCount(columns);
        table->setRowCount(rows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setSelectionBehavior(QAbstractItemView::SelectItems);
        table->setCornerButtonEnabled(false);
        table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        table->horizontalHeader()->setHighlightSections(false);
        table->horizontalHeader()->setMinimumSectionSize(52);
        table->horizontalHeader()->setTextElideMode(Qt::ElideNone);
        table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
        table->verticalHeader()->setDefaultSectionSize(28);
        table->verticalHeader()->setFixedWidth(40);
        table->verticalHeader()->setHighlightSections(false);
        table->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        table->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        table->setMinimumWidth(columns * 52 + 44);
        const int height = 30 + rows * 28 + 4;
        table->setMinimumHeight(height);
        table->setMaximumHeight(height);
        for (int column = 0; column < columns; ++column)
            table->setHorizontalHeaderItem(column, new QTableWidgetItem());
        for (int row = 0; row < rows; ++row)
            table->setVerticalHeaderItem(row, new QTableWidgetItem());
    }

    static void setPoseHeaders(QTableWidget *table, const QStringList &columns, const QStringList &rows)
    {
        for (int column = 0; column < columns.size(); ++column) {
            if (auto *item = table->horizontalHeaderItem(column))
                item->setText(columns.at(column));
        }
        for (int row = 0; row < rows.size(); ++row) {
            if (auto *item = table->verticalHeaderItem(row))
                item->setText(rows.at(row));
        }
    }
};

namespace Ui {
class TcpCalib : public Ui_TcpCalib {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TCPCALIB_H
