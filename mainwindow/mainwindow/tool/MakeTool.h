#ifndef MAKETOOL_H
#define MAKETOOL_H

#include <QString>
#include <QWidget>

class QPushButton;
class QTableWidget;

// 类别模板页。
// templateTableWidget 按「序号 / 识别类别 / 文件地址」列出 class.txt。
// 选中一行后，openfileBtn 选择文件写入该行文件地址，clearfileBtn 清除该地址。
class MakeTool : public QWidget
{
public:
    explicit MakeTool(QWidget *parent = nullptr);

    // 输入 class.txt 路径。文件中一行一个识别类别。
    // 重复调用会按新文件重填表格，已填写的文件地址会被清空。
    void loadClassTable(const QString &classTxtPath);

    // 把表格里的识别类别和文件地址保存成 JSON 数组。成功返回 true。
    // [
    //   { "识别类别": "shield", "文件地址": "targets/shield.txt" }
    // ]
    bool saveClassJson(const QString &jsonPath);

private:
    void setupUi();
    void applyTableStyle();
    void bindFileButtons();
    void openSelectedFile();
    void clearSelectedFile();

    QTableWidget *templateTableWidget = nullptr;
    QPushButton *openfileBtn = nullptr;
    QPushButton *clearfileBtn = nullptr;
    QString m_classTxtDir;
    bool m_fileBtnsBound = false;
};

#endif
