/**
 * @file TcpCalib.h
 * @brief 机器人 TCP 标定对话框：四点/六点法，欧拉/四元数/旋转向量姿态输入。
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
#include "vtk_nlohmannjson.h"
#include VTK_NLOHMANN_JSON(json.hpp)

using json = nlohmann::json;

namespace Ui
{
    class TcpCalib;
}

class MathUtils;

/**
 * @brief 法兰位姿表格 → MathUtils::TcpCalc → VTK 显示 TCP 坐标系。
 */
class TcpCalib : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit TcpCalib(QWidget* parent = nullptr);
    ~TcpCalib();

    /** @brief 表格中姿态的表示方式。 */
    enum class RotateType
    {
        Euler,
        Quat,
        RotateVec
    };

    /**
     * @brief 从 UI 表格读取标定点位姿。
     * @param inputData 输出各行 [x,y,z, 姿态分量…]
     * @param line 4 或 6 点标定
     * @param rt 姿态类型
     */
    bool ReadTableData(std::vector<std::vector<double>>& inputData, int line, RotateType rt);

    /** @brief 执行 TCP 解算并刷新显示。@param line 4 或 6。 */
    void TcpCalc(int line);

    /** @brief 双精度转字符串（保留 4 位小数）。 */
    QString doubleToQString(double val);

    /** @brief 控制台打印并在 PCL 中绘制基座与 TCP 坐标系。 */
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

#endif // CT_TOOL_TCPCALIB_H