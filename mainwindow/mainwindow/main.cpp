#include "tool/MakeTool.h"

#include <cstdio>

#include <QApplication>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QPushButton>
#include <QTableWidget>
#include <QTextCodec>
#include <QTimer>

#include <nlohmann/json.hpp>

namespace {

int fail(const char *message)
{
    fprintf(stderr, "self-test failed: %s\n", message);
    return 1;
}

void closeModalDialog()
{
    if (QWidget *dialog = QApplication::activeModalWidget())
        dialog->close();
}

int runSelfTest()
{
    MakeTool tool;
    const QString classPath =
        QFileInfo(QStringLiteral("tool/class.txt")).absoluteFilePath();
    if (!QFile::exists(classPath))
        return fail("tool/class.txt is missing");

    tool.loadClassTable(classPath);

    auto *table = tool.findChild<QTableWidget *>(QStringLiteral("templateTableWidget"));
    auto *openBtn = tool.findChild<QPushButton *>(QStringLiteral("openfileBtn"));
    auto *clearBtn = tool.findChild<QPushButton *>(QStringLiteral("clearfileBtn"));
    if (!table || !openBtn || !clearBtn)
        return fail("widgets were not created");
    if (table->columnCount() != 3)
        return fail("column count");
    if (table->horizontalHeaderItem(0)->text() != QStringLiteral("序号")
        || table->horizontalHeaderItem(1)->text() != QStringLiteral("识别类别")
        || table->horizontalHeaderItem(2)->text() != QStringLiteral("文件地址"))
        return fail("headers");
    if (table->rowCount() != 5)
        return fail("row count");

    const QStringList names = {QStringLiteral("shield"),
                               QStringLiteral("stator"),
                               QStringLiteral("shell"),
                               QStringLiteral("bearing"),
                               QStringLiteral("sbearing")};
    for (int row = 0; row < names.size(); ++row) {
        if (!table->item(row, 0) || table->item(row, 0)->text() != QString::number(row + 1))
            return fail("index column");
        if (table->item(row, 0)->textAlignment() != Qt::AlignCenter)
            return fail("index alignment");
        if (!table->item(row, 1) || table->item(row, 1)->text() != names.at(row))
            return fail("class name");
        if (!table->item(row, 2) || !table->item(row, 2)->text().isEmpty())
            return fail("file address should start empty");
    }
    if (table->currentRow() != 0)
        return fail("first row should be selected");

    table->item(0, 2)->setText(QStringLiteral("targets/shield.txt"));
    table->item(1, 2)->setText(QStringLiteral("targets/stator.txt"));
    table->selectRow(1);
    clearBtn->click();
    if (!table->item(1, 2)->text().isEmpty())
        return fail("clearfileBtn did not clear the selected row");
    if (table->item(0, 2)->text() != QStringLiteral("targets/shield.txt"))
        return fail("clearfileBtn cleared a row that was not selected");

    table->clearSelection();
    table->setCurrentCell(-1, -1);
    QTimer::singleShot(0, closeModalDialog);
    clearBtn->click();
    if (table->item(0, 2)->text() != QStringLiteral("targets/shield.txt"))
        return fail("clear with no selection changed a row");

    const int rowsBefore = table->rowCount();
    QTimer::singleShot(0, closeModalDialog);
    tool.loadClassTable(QStringLiteral("/no/such/class.txt"));
    if (table->rowCount() != rowsBefore)
        return fail("failed reload cleared the table");

    QFile gbkFile(QStringLiteral("/tmp/class_gbk.txt"));
    if (!gbkFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return fail("could not write gbk sample");
    QTextCodec *gbk = QTextCodec::codecForName("GB18030");
    if (!gbk)
        return fail("GB18030 codec is unavailable");
    gbkFile.write(gbk->fromUnicode(QStringLiteral("划痕\r\n污渍\r\n")));
    gbkFile.close();
    tool.loadClassTable(gbkFile.fileName());
    if (table->rowCount() != 2
        || !table->item(0, 1) || table->item(0, 1)->text() != QStringLiteral("划痕")
        || !table->item(1, 1) || table->item(1, 1)->text() != QStringLiteral("污渍"))
        return fail("gbk class names");

    tool.loadClassTable(classPath);
    if (table->rowCount() != 5 || table->item(0, 1)->text() != QStringLiteral("shield"))
        return fail("reload class.txt");
    table->item(0, 2)->setText(QStringLiteral("targets/shield.txt"));
    table->item(2, 2)->setText(QStringLiteral("targets/壳.txt"));

    const QString jsonPath = QStringLiteral("/tmp/class_table.json");
    if (!tool.saveClassJson(jsonPath))
        return fail("saveClassJson");
    QFile jsonFile(jsonPath);
    if (!jsonFile.open(QIODevice::ReadOnly))
        return fail("read saved json");
    const QByteArray rawJson = jsonFile.readAll();
    jsonFile.close();
    const auto saved = nlohmann::json::parse(rawJson.constData(),
                                             rawJson.constData() + rawJson.size());
    if (!saved.is_array() || saved.size() != 5)
        return fail("json size");
    if (saved[0].at("识别类别") != "shield"
        || saved[0].at("文件地址") != "targets/shield.txt")
        return fail("json first row");
    if (saved[1].at("识别类别") != "stator" || saved[1].at("文件地址") != "")
        return fail("json empty address");
    if (saved[2].at("识别类别") != "shell"
        || saved[2].at("文件地址") != "targets/壳.txt")
        return fail("json chinese address");
    const int namePos = rawJson.indexOf(QStringLiteral("识别类别").toUtf8());
    const int pathPos = rawJson.indexOf(QStringLiteral("文件地址").toUtf8());
    if (namePos < 0 || pathPos < 0 || namePos > pathPos)
        return fail("json key order");

    table->selectRow(0);
    tool.resize(860, 380);
    tool.show();
    QApplication::processEvents();
    const QPixmap shot = tool.grab();
    if (shot.isNull() || !shot.save(QStringLiteral("/tmp/maketool_table.png")))
        return fail("screenshot");

    fprintf(stdout, "self-test passed\n");
    return 0;
}

} // namespace

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setStyle(QStringLiteral("Fusion"));

    const QStringList args = QCoreApplication::arguments();
    if (args.contains(QStringLiteral("--self-test")))
        return runSelfTest();

    MakeTool tool;
    const QString classPath = args.size() > 1
        ? args.at(1)
        : QFileInfo(QStringLiteral("tool/class.txt")).absoluteFilePath();
    tool.loadClassTable(classPath);
    tool.show();
    return app.exec();
}
