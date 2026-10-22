/**
 * @file cutting.cpp
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
#include "vtk_nlohmannjson.h"
#include VTK_NLOHMANN_JSON(json.hpp)
#include <QDateTime>
#include <QHeaderView>
#include <QFileDialog>

using json = nlohmann::json;

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

    //Eigen::MatrixXd t(6, 6);
    //t << -485.687, -35.2817, 354.2837, -76.0034, 1.7698, 89.7785,
    //    -632.1971, -138.5274, 367.3399, -101.1107, -16.2200, 89.0011,
    //    -644.2368, 53.9687, 360.3177, -103.7742, 16.2670, 89.3453,
    //    -559.7433, -40.3720, 375.0590, -88.8739, -0.0107, 92.8546,
    //    -360.9080, -40.3720, 375.0590, -88.8739, -0.0107, 92.8546,
    //    -559.7964, -40.3720, 505, -88.8739, -0.0107, 92.8546;
    //for (int i = 0; i < t.rows(); ++i) {
    //    for (int j = 0; j < t.cols(); ++j) {
    //        QTableWidgetItem* item = new QTableWidgetItem(doubleToQString(t(i, j)));
    //        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter); // 右对齐，美观
    //        if (i < 4) {
    //            ui->fourPTable->setItem(i, j, item);
    //        }
    //        else {
    //            int targetRow = i - 4;
    //            if (targetRow >= 0 && targetRow < 2) {
    //                ui->sixPTable->setItem(targetRow, j, item);
    //            }
    //        }
    //    }
    //}

    //Eigen::MatrixXd t(6, 6);
    //t << 0.81784271 * 1000.0, -0.11455369 * 1000.0, 0.29121033 * 1000.0, 34.04496999, 58.77172911, 53.37233642,
    //    0.8374768 * 1000.0, -0.28040563 * 1000.0, 0.34194714 * 1000.0, 87.31024237, 20.42946622, 58.33350291,
    //    1.01447711 * 1000.0, -0.26121916 * 1000.0, 0.36923868 * 1000.0, 90.50105521, 78.07269078, 54.2911917,
    //    0.90790113 * 1000.0, -0.24320044 * 1000.0, 0.36929397 * 1000.0, 67.9371052,  69.36401534, 69.16493141,
    //    1.0953894700000002 * 1000.0, -0.2431951 * 1000.0, 0.36927847 * 1000.0, 67.93884527, 69.37141623, 69.16146043,
    //    0.90789905 * 1000.0, -0.243231 * 1000.0, 0.56353583 * 1000.0, 67.94116575, 69.38270579, 69.155626;
    //for (int i = 0; i < t.rows(); ++i) {
    //    for (int j = 0; j < t.cols(); ++j) {
    //        QTableWidgetItem* item = new QTableWidgetItem(doubleToQString(t(i, j)));
    //        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter); // 右对齐，美观
    //        if (i < 4) {
    //            ui->fourPTable->setItem(i, j, item);
    //        }
    //        else {
    //            int targetRow = i - 4;
    //            if (targetRow >= 0 && targetRow < 2) {
    //                ui->sixPTable->setItem(targetRow, j, item);
    //            }
    //        }
    //    }
    //}
}

TcpCalib::~TcpCalib()
{
    delete ui;
}

QString TcpCalib::doubleToQString(double val) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4) << val; // 保留4位小数
    return QString::fromStdString(ss.str());
}

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

void TcpCalib::on_Calib4pTcpBtn_clicked() {
    TcpCalc(4);
}

void TcpCalib::on_Calib6pTcpBtn_clicked() {
    TcpCalc(6);
}

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