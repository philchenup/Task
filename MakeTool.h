/**
 * @file MakeTool.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#ifndef MAKETOOL_H
#define MAKETOOL_H

#include "base/customdialog.h"
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <QListWidget>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QTimer>
#include "CylindricalHoleDetector.h"

struct SphereData {
    double X, Y, Z;       // 位置
    double w, x, y, z; // 欧拉角旋转 (度数)

    SphereData() : X(0), Y(0), Z(0), w(0), x(0), y(0), z(0) {}
    SphereData(double _X, double _Y, double _Z, double _w, double _x, double _y, double _z)
        : X(_X), Y(_Y), Z(_Z), w(_w), x(_x), y(_y), z(_z) {}
};

struct offsetData
{
    double X_Offset;
    double Y_Offset;
    double Z_Offset;
    Eigen::Quaterniond quat;
};

Q_DECLARE_METATYPE(SphereData)

namespace Ui
{
    class MakeTool;
}

class MathUtils;
class QVTKOpenGLNativeWidget;

class MakeTool : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit MakeTool(QWidget* parent = nullptr);
    ~MakeTool();

    // 定义输入数据类型
    enum class RotateType
    {
        Euler,
        Quat,
        RotateVec
    };

    enum class ViewType { 
        Front, 
        Top, 
        Left 
    };

    void initVtkWindow();

    void initDragPoint(const vtkNew<vtkMatrix4x4>& matrix, vtkNew<vtkActor>& actor);

    void initPcd(const ct::Cloud::Ptr& cloud, vtkNew<vtkActor>& pointCloudActor);

    void initMesh(const pcl::PolygonMesh& mesh, vtkNew<vtkActor>& meshActor);

    void GetTargetInCam(vtkNew<vtkMatrix4x4>& vtk_matrix);

    Eigen::Affine3d GetFlange2TcpMatrix();

    Eigen::Affine3d GetGraspBase2FlangeMatrix();

    Eigen::Affine3d GetCapBase2FlangeMatrix();

    Eigen::Affine3d GetFlange2CamMatrix();

    void GetSingleGraspPoint(SphereData& data);

    void createSphereActor(const SphereData& data, vtkNew<vtkActor>& actor);

    void updateSphereActor(const SphereData& data, vtkActor* actor);

    void updateSpinboxes(const SphereData& data);

    void updateCadSpinboxes(const SphereData& data);

    void loadVisionConfig(const std::string& visionconfig_path);

    void autoAddGraspPoint(const SphereData& data);

    void autoCadAddGraspPoint(const SphereData& data);

    Eigen::Quaterniond dirToQuat(const gp_Dir& direction);

    void showPcd(const ct::Cloud::Ptr& cloud_in, const pcl::PolygonMesh mesh_in, const std::vector<HoleInfo>& holeInfo, int coorSize);

    Eigen::Affine3d readQuatPoseFromJson(const std::string& filepath);
private slots:
    void on_loadToolkitBtn_clicked();

    void on_loadPcdBtn_clicked();

    void timerHandle();

    void on_loadTcpBtn_clicked();

    void on_loadHandeyePathBtn_clicked();

    void on_flushBtn_clicked();

    void on_newGraspBtn_clicked();

    void onListItemClicked(QListWidgetItem* item);

    void onCadListItemClicked(QListWidgetItem* item);

    void timer1Handle();

    void timer2Handle();

    void onDeleteActionTriggered();

    void onCustomContextMenuRequested(const QPoint& pos);

    void on_saveMultiGraspDataBtn_clicked();

    void on_updateTcpBtn_clicked();

    /*CAD model*/
    void on_handExecBtn_clicked();

    void on_pcdProcessBtn_clicked();

    void on_paraSaveBtn_clicked();

    void on_loadCadModelBtn_clicked();

    void on_loadCadTcpBtn_clicked();

signals:
    void camTrigger();

private:
    void updateGraspCloudActor();

    void activateVtkWidget(QVTKOpenGLNativeWidget* widget);

    void activateVisibleVtkWidgets();

    int max_count = 0;
    int last_max_count = 0;

    bool IsFlushFirst = false;

    vtkNew<vtkActor> cones;

    vtkNew<vtkActor> pointCloudActor;

    vtkNew<vtkActor> graspCloudActor;

    vtkNew<vtkActor> cadMeshActor;

    vtkNew<vtkCallbackCommand> transformCallback_multi;
    
    QListWidgetItem* current_item;

    vtkActor* currentSelectedActor;

    std::vector<vtkSmartPointer<vtkActor>> allActors;

    Eigen::Affine3d Flange2Cam;

    Eigen::Affine3d CapBase2Flange;

    Eigen::Affine3d GraspBase2Flange;

    Eigen::Affine3d Flange2Tcp;

    QTimer* m_timer;

    QTimer* m_timer_1;

    QTimer* m_timer_2;

    MathUtils* mu;

    vtkNew<vtkRenderer> render_tool;

    vtkNew<vtkRenderer> renderer_grasp;

    vtkNew<vtkRenderer> renderer_cad_grasp;

    pcl::visualization::PCLVisualizer::Ptr m_viewer;

    ct::Cloud::Ptr tool_cloud;

    ct::Cloud::Ptr cad_cloud;

    ct::Cloud::Ptr post_cloud;

    pcl::PolygonMesh m_mesh;

    std::vector<HoleInfo> holes;
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    Ui::MakeTool* ui;
};

#endif // TCPCALIB_H
