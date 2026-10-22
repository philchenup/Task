/**
 * @file TcpCalib.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#ifndef CT_TOOL_TCPCALIB_H
#define CT_TOOL_TCPCALIB_H

#include "base/customdialog.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <vtkGenericOpenGLRenderWindow.h>

namespace Ui
{
    class TcpCalib;
}

class MathUtils;

class TcpCalib : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit TcpCalib(QWidget* parent = nullptr);
    ~TcpCalib();

    // 定义输入数据类型
    enum class RotateType
    {
        Euler,
        Quat,
        RotateVec
    };

    bool ReadTableData(std::vector<std::vector<double>>& inputData, int line, RotateType rt);

    void TcpCalc(int line);

    QString doubleToQString(double val);

    void showTcpCoor();

private slots:

    void on_Calib4pTcpBtn_clicked();
    void on_Calib6pTcpBtn_clicked();
    void on_saveTcpDataBtn_clicked();

    void on_checkPdfBtn_clicked();

private:
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    Eigen::Quaternionf quatTcp;

    Eigen::Vector3f translation_vec;

    pcl::visualization::PCLVisualizer::Ptr m_viewer;

    Ui::TcpCalib* ui;

    MathUtils* mu;
};

#endif // TCPCALIB_H
