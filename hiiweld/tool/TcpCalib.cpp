/**
 * @file TcpCalib.cpp
 * @brief TCP 标定实现。
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "TcpCalib.h"
#include "ui_TcpCalib.h"
#include <QDesktopServices> 
#include <QUrl>
#include "MathUtils.h"
#include <fstream>
#include <QDateTime>
#include <QHeaderView>
#include <QFileDialog>

/** @brief 初始化 VTK 与欧拉/四元数面板。 */
TcpCalib::TcpCalib(QWidget* parent) :CustomDialog(parent), 
ui(new Ui::TcpCalib),
m_viewer(nullptr)
{
    ui->setupUi(this);

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    ui->qvtkWidget->setRenderWindow(renderWindow);
    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    m_viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    m_viewer->setBackgroundColor(0.8, 0.8, 0.8);
    m_viewer->resetCamera();
    ui->qvtkWidget->update();

    ui->eulerBtn->setChecked(true);
    ui->panelWidget->setCurrentIndex(0);

    connect(ui->eulerBtn, &QRadioButton::toggled, this, [&](bool checked) {
        if (checked) { 
            ui->panelWidget->setCurrentIndex(0);
        }
    });

    connect(ui->quatBtn, &QRadioButton::toggled, this, [&](bool checked) {
        if (checked) {
            ui->panelWidget->setCurrentIndex(1);
        }
    });

    connect(ui->rotateVecBtn, &QRadioButton::toggled, this, [&](bool checked) {
        if (checked) {
            ui->panelWidget->setCurrentIndex(0);
        }
    });

    ui->fourPTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->sixPTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

/** @brief 析构。
TcpCalib::~TcpCalib()
{
    delete ui;
}

/** @brief 浮点格式化为 4 位小数字符串。 */
QString TcpCalib::doubleToQString(double val) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4) << val; // 保留4位小数
    return QString::fromStdString(ss.str());
}

/** @brief 用系统默认程序打开标定说明 PDF。 */
void TcpCalib::on_checkPdfBtn_clicked() {
    QString currentAppDir = QApplication::applicationDirPath();
    QString filePath = currentAppDir + "/doc/TcpCalibration.pdf";
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        printE("Tcp Calibration file not found!");
        return;
    }
    QUrl fileUrl = QUrl::fromLocalFile(fileInfo.absoluteFilePath());
    bool isOpened = QDesktopServices::openUrl(fileUrl);
    if (!isOpened) {
        printE("Tcp Calibration file open failed!");
        return;
    }
}

/** @brief 读取并校验 4/6 点表格数据。 */
bool TcpCalib::ReadTableData(std::vector<std::vector<double>>& inputData, int line, RotateType rt) {
    int frowCount, fcolCount = 0;
    int srowCount, scolCount = 0;
    bool isQuat = false;

    switch (rt)
    {
    case TcpCalib::RotateType::Euler:
    case TcpCalib::RotateType::RotateVec:
        frowCount = ui->fourPTable->rowCount();
        fcolCount = ui->fourPTable->columnCount();
        srowCount = ui->sixPTable->rowCount();
        scolCount = ui->sixPTable->columnCount();
        break;
    case TcpCalib::RotateType::Quat:
        frowCount = ui->quatFourTable->rowCount();
        fcolCount = ui->quatFourTable->columnCount();
        srowCount = ui->quatSixTable->rowCount();
        scolCount = ui->quatSixTable->columnCount();
        isQuat = true;
        break;
    default:
        break;
    }

    std::vector<std::vector<double>> allRowData;
    for (int row = 0; row < frowCount; ++row) {
        std::vector<double> currentRow;
        currentRow.reserve(fcolCount);

        for (int col = 0; col < fcolCount; ++col) {
            QTableWidgetItem* item;;
            if (isQuat) {
                item = ui->quatFourTable->item(row, col);
            }
            else {
                item = ui->fourPTable->item(row, col);
            }
            double value = 0.0;
            if (item != nullptr) {

                bool ok;
                double cellValue = item->text().toDouble(&ok);

                if (ok) {
                    value = cellValue;
                }
            }
            if (value == 0) {
                printE("Please input non zero number!");
                return false;
            }
            currentRow.push_back(value);
        }
        inputData.push_back(currentRow);
    }
    
    if (line != 6 && line == 4) {
        return true;
    }

    for (int row = 0; row < srowCount; ++row) {
        std::vector<double> currentRow;
        currentRow.reserve(scolCount);

        for (int col = 0; col < scolCount; ++col) {
            QTableWidgetItem* item;;
            if (isQuat) {
                item = ui->quatSixTable->item(row, col);
            }
            else {
                item = ui->sixPTable->item(row, col);
            }
            double value = 0.0;
            if (item != nullptr) {

                bool ok;
                double cellValue = item->text().toDouble(&ok);

                if (ok) {
                    value = cellValue;
                }
            }
            if (value == 0) {
                printE("Please input non zero number!");
                return false;
            }
            currentRow.push_back(value);
        }
        inputData.push_back(currentRow);
    }

    return true;
}

/** @brief 触发四点 TCP 计算。 */
void TcpCalib::on_Calib4pTcpBtn_clicked() {
    TcpCalc(4);
}

/** @brief 触发六点 TCP 计算。 */
void TcpCalib::on_Calib6pTcpBtn_clicked() {
    TcpCalc(6);
}

/** @brief 组装位姿并调用 MathUtils::TcpCalc。 */
void TcpCalib::TcpCalc(int line) {
    if (line != 4 && line != 6) return;

    RotateType rt;
    std::vector<std::vector<double>> TableData;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->rotateVecBtn->isChecked()) {
        rt = RotateType::RotateVec;
    }
    else {
        rt = RotateType::Quat;
    }

    if (!ReadTableData(TableData, line, rt)) {
        printE("Input Normal Tcp Calibration Data!");
        return;
    }

    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Matrix3d> rot;
    for (int i = 0; i < TableData.size(); ++i) {
        pos.push_back(Eigen::Vector3d(TableData[i][0], TableData[i][1], TableData[i][2]));

        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == TcpCalib::RotateType::Euler) {
            assert(TableData[i].size() == 6);
            Eigen::Vector3d euler_rad = mu->deg2radVec(Eigen::Vector3d(TableData[i][3], TableData[i][4], TableData[i][5]));
            matrix = mu->eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == TcpCalib::RotateType::Quat) {
            assert(TableData[i].size() == 7); // w x y z
            Eigen::Quaterniond q = Eigen::Quaterniond(TableData[i][3], TableData[i][4], TableData[i][5], TableData[i][6]);
            matrix = mu->quaternionToMatrix(q);
        }
        else if(rt == TcpCalib::RotateType::RotateVec){
            assert(TableData[i].size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(TableData[i][3], TableData[i][4], TableData[i][5]);
            Eigen::Vector3d rot_vec_rad = mu->deg2radVec(rotate_vec);
            matrix = mu->rotVecToMatrix(rot_vec_rad);
        }
        rot.push_back(matrix);
    }

    Eigen::Matrix4d ret;
    if (mu->TcpCalc(pos, rot, ret) != -1) {
        transform = Eigen::Affine3f(ret.cast<float>());
        showTcpCoor();
    }
}

/** @brief 打印 TCP 并在视图中绘制坐标系。 */
void TcpCalib::showTcpCoor() {
    printI("Six Point Tcp Calibration calc done!");

    translation_vec = transform.translation();
    Eigen::Quaternionf quaternion(transform.rotation());
    quatTcp = quaternion;
    quatTcp.normalize();

    printI(QString("Translation [x:%1, y:%2, z:%3] Quaternion [w:%4, x:%5, y:%6, z:%7] done.")
        .arg(translation_vec.x())
        .arg(translation_vec.y())
        .arg(translation_vec.z())
        .arg(quatTcp.w())
        .arg(quatTcp.x())
        .arg(quatTcp.y())
        .arg(quatTcp.z()));

    m_viewer->removeAllCoordinateSystems();

    double scale = 100; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale, "coordinate_base");

    double scale_tcp = 50; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale_tcp, transform, "coordinate_tcp");

    m_viewer->resetCamera();
    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}

/** @brief 将 TCP 保存为 JSON 文件。 */
void TcpCalib::on_saveTcpDataBtn_clicked() {
    QString filename = "TcpCalib-" + QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QString filepath = QFileDialog::getSaveFileName(this, tr("Save Tcp"), filename, "JSON(*.json)");
    if (filepath.isEmpty()) return;

    std::vector<double> translation = { translation_vec.x(), translation_vec.y(), translation_vec.z() };
    nlohmann::json j;
    j["Translation"] = translation;
    std::vector<double> quat = { quatTcp.w(), quatTcp.x(), quatTcp.y(), quatTcp.z() };
    j["Quaternion"] = quat;
    std::ofstream file(filepath.toStdString());
    if (!file.is_open()) {
        printE(tr("The TCP Calibration data (%1) save failed!").arg(filepath));
        return;
    }
    file << j.dump(4);
    file.close();

    printI(tr("The TCP Calibration data (%1) save done!").arg(filepath));
}