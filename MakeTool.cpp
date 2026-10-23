/**
 * @file HandEyeCalib.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "MakeTool.h"
#include "ui_MakeTool.h"
#include "MathUtils.h"
#include <QDesktopServices> 
#include <QUrl>
#include <fstream>
#include "vtk_nlohmannjson.h"
#include VTK_NLOHMANN_JSON(json.hpp)
#include <QDateTime>
#include <QEvent>
#include <QFileDialog>
#include <QApplication>
#include <QScreen>
#include <QMessageBox>
#include <random>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>
#include <vtkAutoInit.h>
#include "vtkMove/mySWInteractorStyle.h"
#include <vtkCallbackCommand.h>
#include <vtkConeSource.h>
#include <vtkTransform.h>
#include <vtkProperty.h>
#include <vtkTransformFilter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <vtkVertexGlyphFilter.h>
#include <pcl/common/common.h>
#include <vtkMath.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include "utils/utils.h"

namespace {

class VtkWidgetGlStateFilter : public QObject
{
public:
    explicit VtkWidgetGlStateFilter(QVTKOpenGLNativeWidget* widget)
        : QObject(widget), widget(widget) {}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        const auto type = event->type();
        if (type == QEvent::MouseButtonPress || type == QEvent::Wheel || type == QEvent::Show) {
            if (widget->isValid()) {
                widget->makeCurrent();
            }
            if (auto* renWin = vtkOpenGLRenderWindow::SafeDownCast(widget->renderWindow())) {
                if (renWin->GetState()) {
                    renWin->GetState()->Reset();
                }
            }
        }
        return QObject::eventFilter(watched, event);
    }

private:
    QVTKOpenGLNativeWidget* widget;
};

} // namespace

#define Create(type,name) vtkNew<type> name

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

#define ACTOR_POINTER_ROLE Qt::UserRole + 1
#define GRASP_DATA_ROLE   Qt::UserRole + 2

using json = nlohmann::json;

Eigen::Matrix4d graspInCamMatrix;
Eigen::Matrix4d multiGraspInCamMatrix;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SphereData, X, Y, Z, w, x, y, z)

auto printTransformCallback = [](vtkObject* caller, long unsigned int eventId, void* clientData, void* callData) {
    auto actor = static_cast<vtkActor*>(clientData);
    double finalMatrixArray[16];
    actor->GetMatrix(finalMatrixArray);

    static double lastMatrixArray[16] = { 0 };
    bool hasChanged = false;
    const double tolerance = 1e-4;

    for (int i = 0; i < 16; ++i) {
        if (std::abs(lastMatrixArray[i] - finalMatrixArray[i]) > tolerance) {
            hasChanged = true;
            break;
        }
    }

    if (hasChanged) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                graspInCamMatrix(i, j) = finalMatrixArray[i * 4 + j];
            }
        }
        std::memcpy(lastMatrixArray, finalMatrixArray, sizeof(lastMatrixArray));
    }
};

auto printTransformCallback_multi = [](vtkObject* caller, long unsigned int eventId, void* clientData, void* callData) {
    auto actor = static_cast<vtkActor*>(clientData);
    double finalMatrixArray[16];
    actor->GetMatrix(finalMatrixArray);

    static double lastMatrixArray[16] = { 0 };
    bool hasChanged = false;
    const double tolerance = 1e-4;

    for (int i = 0; i < 16; ++i) {
        if (std::abs(lastMatrixArray[i] - finalMatrixArray[i]) > tolerance) {
            hasChanged = true;
            break;
        }
    }

    if (hasChanged) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                multiGraspInCamMatrix(i, j) = finalMatrixArray[i * 4 + j];
            }
        }
        std::memcpy(lastMatrixArray, finalMatrixArray, sizeof(lastMatrixArray));
    }
};

MakeTool::MakeTool(QWidget* parent) :CustomDialog(parent),
ui(new Ui::MakeTool),
tool_cloud(new ct::Cloud()),
post_cloud(new ct::Cloud()),
m_timer(new QTimer(this)),
m_timer_1(new QTimer(this)),
mu(new MathUtils),
currentSelectedActor(nullptr),
m_viewer(nullptr),
current_item(nullptr)
{
    ui->setupUi(this);

    if (ui == nullptr) {
        ui = new Ui::MakeTool();
    }

    ui->stackedWidget->setContentsMargins(0, 0, 3, 3);
    ui->stackedWidget->setCurrentIndex(0);
    ui->tcpStackWidget->setCurrentIndex(0);

    // toolSettingWidget

    ui->capEulerBtn->setChecked(true);
    ui->capStackWidget->setCurrentIndex(0);

    ui->gEulerBtn->setChecked(true);
    ui->gStackWidget->setCurrentIndex(0);

    ui->step1Btn->setChecked(true);
    ui->eulerTcpBtn->setChecked(true);

    connect(ui->step1Btn, &QPushButton::clicked, this, [&]() { 
        ui->stackedWidget->setCurrentIndex(0);  
        m_timer->stop(); 
        m_timer_1->stop();
        });
    connect(ui->step2Btn, &QPushButton::clicked, this, [this]() {
        ui->qvtkWidget->releaseMouse();
        ui->stackedWidget->setCurrentIndex(1);
        m_timer->start(100);
        m_timer_1->stop();
        QTimer::singleShot(0, this, [this]() { activateVtkWidget(ui->toolqvtkWidget); });
    });
    connect(ui->step3Btn, &QPushButton::clicked, this, [this]() {
        ui->toolqvtkWidget->releaseMouse();
        ui->stackedWidget->setCurrentIndex(2);
        m_timer->stop();
        m_timer_1->start(100);
        QTimer::singleShot(0, this, [this]() { activateVtkWidget(ui->qvtkWidget); });
    });

    connect(ui->eulerTcpBtn, &QPushButton::clicked, this, [&]() { ui->tcpStackWidget->setCurrentIndex(0); });
    connect(ui->quatTcpBtn, &QPushButton::clicked, this, [&]() { ui->tcpStackWidget->setCurrentIndex(1); });
    connect(ui->rotvecTcpBtn, &QPushButton::clicked, this, [&]() { ui->tcpStackWidget->setCurrentIndex(2); });

    connect(ui->gEulerBtn, &QPushButton::clicked, this, [&]() { ui->gStackWidget->setCurrentIndex(0); });
    connect(ui->gQuatBtn, &QPushButton::clicked, this, [&]() { ui->gStackWidget->setCurrentIndex(1); });
    connect(ui->gRotBtn, &QPushButton::clicked, this, [&]() { ui->gStackWidget->setCurrentIndex(2); });

    connect(ui->capEulerBtn, &QPushButton::clicked, this, [&]() { ui->capStackWidget->setCurrentIndex(0); });
    connect(ui->quatcapBtn, &QPushButton::clicked, this, [&]() { ui->capStackWidget->setCurrentIndex(1); });
    connect(ui->rotveccapBtn, &QPushButton::clicked, this, [&]() { ui->capStackWidget->setCurrentIndex(2); });

    connect(ui->IsExistTcpBtn, &QPushButton::clicked, this, [&]() { ui->tcpExistWidget->setCurrentIndex(0); });
    connect(ui->NotExistTcpBtn, &QPushButton::clicked, this, [&]() { ui->tcpExistWidget->setCurrentIndex(1); });


    ui->IsExistTcpBtn->setChecked(true);
    ui->tcpExistWidget->setCurrentIndex(0);

    graspInCamMatrix = Eigen::Matrix4d::Identity();
    multiGraspInCamMatrix = Eigen::Matrix4d::Identity();

    ui->graspListWidget->setStyleSheet(
        "QListWidget { background-color: white; }"
        "QListWidget::item { background-color: white; color: black; border: 1px solid #ddd; }"
        "QListWidget::item:selected { background-color: #cce0ff; color: black; }"
        "QListWidget::item:hover { background-color: #f0f0f0; }"
    );
    ui->graspListWidget->setAutoScroll(false);

    connect(ui->graspListWidget, &QListWidget::itemClicked, this, &MakeTool::onListItemClicked);

    connect(m_timer, &QTimer::timeout, this, &MakeTool::timerHandle);
    connect(m_timer_1, &QTimer::timeout, this, &MakeTool::timer1Handle);

    ui->graspListWidget->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->graspListWidget, &QListWidget::customContextMenuRequested, this, &MakeTool::onCustomContextMenuRequested);

    initVtkWindow();
}

MakeTool::~MakeTool()
{
    m_timer->stop();
    if (m_timer != nullptr) {
        delete m_timer;
    }

    m_timer_1->stop();
    if (m_timer_1 != nullptr) {
        delete m_timer_1;
    }

    if (ui != nullptr) {
        delete ui;
    }
    if (mu != nullptr) {
        delete mu;
    }
}

void MakeTool::initVtkWindow() {

    ui->toolqvtkWidget->setEnabled(true);
    ui->toolqvtkWidget->setFocusPolicy(Qt::StrongFocus);
    ui->toolqvtkWidget->renderWindow()->AddRenderer(render_tool);
    ui->toolqvtkWidget->installEventFilter(new VtkWidgetGlStateFilter(ui->toolqvtkWidget));

    // 控件自己已经创建并绑定了 QVTKInteractor。这里控件还在隐藏页，
    // 再调用 SetRenderWindow / Initialize 会立刻 Render，把 FBO 建到错误的上下文上。
    vtkNew<mySWInteractorStyle> style;
    ui->toolqvtkWidget->interactor()->SetInteractorStyle(style);

    ui->qvtkWidget->setEnabled(true);
    ui->qvtkWidget->setFocusPolicy(Qt::StrongFocus);
    ui->qvtkWidget->renderWindow()->AddRenderer(renderer_grasp);
    ui->qvtkWidget->installEventFilter(new VtkWidgetGlStateFilter(ui->qvtkWidget));

    vtkNew<mySWInteractorStyle> styleGrasp;
    ui->qvtkWidget->interactor()->SetInteractorStyle(styleGrasp);

    transformCallback_multi->SetCallback(printTransformCallback_multi);
}

void MakeTool::activateVtkWidget(QVTKOpenGLNativeWidget* widget)
{
    if (!widget || !widget->renderWindow() || !widget->isVisible()) {
        return;
    }

    if (widget->isValid()) {
        widget->makeCurrent();
    }

    if (auto* renWin = vtkOpenGLRenderWindow::SafeDownCast(widget->renderWindow())) {
        renWin->SetForceMakeCurrent();
        if (renWin->GetState()) {
            renWin->GetState()->Reset();
        }
    }

    if (auto* iren = widget->interactor()) {
        if (!iren->GetInitialized()) {
            iren->Initialize();
        }
        iren->Enable();
        const QSize sz = widget->size() * widget->devicePixelRatioF();
        if (sz.width() > 0 && sz.height() > 0) {
            iren->UpdateSize(sz.width(), sz.height());
        }
    }

    widget->renderWindow()->Render();
    widget->update();
}

void MakeTool::updateGraspCloudActor()
{
    if (!tool_cloud || tool_cloud->size() == 0) {
        return;
    }

    initPcd(tool_cloud, graspCloudActor);
    graspCloudActor->SetPickable(0);
    if (!renderer_grasp->HasViewProp(graspCloudActor)) {
        renderer_grasp->AddActor(graspCloudActor);
    }
}

void MakeTool::initDragPoint(const vtkNew<vtkMatrix4x4>& matrix, vtkNew<vtkActor>& actor) {

    Create(vtkSphereSource, source);
    source->SetCenter(0.0, 0.0, 0.0);
    source->SetRadius(3.0);
    source->SetPhiResolution(100);
    source->SetThetaResolution(100);
    source->Update();

    vtkNew<vtkTransform> transform;
    transform->SetMatrix(matrix);

    Create(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(source->GetOutputPort());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->SetUserTransform(transform);

    return;
}

void MakeTool::initPcd(const ct::Cloud::Ptr& cloud, vtkNew<vtkActor>& pointCloudActor) {
    if (cloud->size() < 0) {
        printE("Input Make Tool Template Pcd is Null, please check the input!");
        return;
    }

    vtkNew<vtkPoints> vtk_points;
    for (const auto& point : cloud->points)
    {
        vtk_points->InsertNextPoint(point.x, point.y, point.z);
    }

    vtkNew<vtkPolyData> vtk_cloud_data;
    vtk_cloud_data->SetPoints(vtk_points);

    vtkNew<vtkVertexGlyphFilter> glyphFilter;
    glyphFilter->SetInputData(vtk_cloud_data);
    glyphFilter->Update();

    // 2c. 创建点云的 Mapper 和 Actor
    vtkNew<vtkPolyDataMapper> pointCloudMapper;
    pointCloudMapper->SetInputData(glyphFilter->GetOutput());

    pointCloudActor->SetMapper(pointCloudMapper);
    pointCloudActor->GetProperty()->SetPointSize(2.0); // 设置点的大小
    pointCloudActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 设置点云颜色为绿色
}

void MakeTool::initMesh(const pcl::PolygonMesh& mesh, vtkNew<vtkActor>& meshActor) {
    // Direct VTK surface from PolygonMesh topology — do NOT use VertexGlyphFilter / point cloud.
    const auto& cloud = mesh.cloud;
    const std::size_t n_points = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (mesh.polygons.empty() || n_points == 0 || cloud.data.empty()) {
        printE("Input PolygonMesh is Null, please check the input!");
        return;
    }

    const int x_idx = pcl::getFieldIndex(cloud, "x");
    const int y_idx = pcl::getFieldIndex(cloud, "y");
    const int z_idx = pcl::getFieldIndex(cloud, "z");
    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
        printE("PolygonMesh cloud missing x/y/z fields!");
        return;
    }

    const std::size_t x_off = cloud.fields[x_idx].offset;
    const std::size_t y_off = cloud.fields[y_idx].offset;
    const std::size_t z_off = cloud.fields[z_idx].offset;
    const std::size_t step = cloud.point_step;

    vtkNew<vtkPoints> vtk_points;
    vtk_points->SetDataTypeToFloat();
    vtk_points->SetNumberOfPoints(static_cast<vtkIdType>(n_points));
    for (std::size_t i = 0; i < n_points; ++i) {
        const std::uint8_t* ptr = cloud.data.data() + i * step;
        float x = 0.f, y = 0.f, z = 0.f;
        std::memcpy(&x, ptr + x_off, sizeof(float));
        std::memcpy(&y, ptr + y_off, sizeof(float));
        std::memcpy(&z, ptr + z_off, sizeof(float));
        vtk_points->SetPoint(static_cast<vtkIdType>(i), x, y, z);
    }

    vtkNew<vtkCellArray> vtk_polygons;
    for (const auto& poly : mesh.polygons) {
        if (poly.vertices.size() < 3) {
            continue;
        }
        // Fan-triangulate n-gons → triangles (surface cells, not points).
        for (std::size_t k = 1; k + 1 < poly.vertices.size(); ++k) {
            vtkNew<vtkTriangle> triangle;
            triangle->GetPointIds()->SetId(0, static_cast<vtkIdType>(poly.vertices[0]));
            triangle->GetPointIds()->SetId(1, static_cast<vtkIdType>(poly.vertices[k]));
            triangle->GetPointIds()->SetId(2, static_cast<vtkIdType>(poly.vertices[k + 1]));
            vtk_polygons->InsertNextCell(triangle);
        }
    }

    if (vtk_polygons->GetNumberOfCells() == 0) {
        printE("PolygonMesh has no valid triangles!");
        return;
    }

    vtkNew<vtkPolyData> vtk_mesh_data;
    vtk_mesh_data->SetPoints(vtk_points);
    vtk_mesh_data->SetPolys(vtk_polygons);  // surface mesh

    vtkNew<vtkPolyDataMapper> meshMapper;
    meshMapper->SetInputData(vtk_mesh_data);
    meshMapper->ScalarVisibilityOff();

    meshActor->SetMapper(meshMapper);
    meshActor->GetProperty()->SetRepresentationToSurface();
    meshActor->GetProperty()->SetColor(0.75, 0.75, 0.8);
    meshActor->GetProperty()->SetOpacity(1.0);
    meshActor->GetProperty()->SetInterpolationToFlat();
}

void MakeTool::on_loadToolkitBtn_clicked() {
    QString filepath = QFileDialog::getOpenFileName(this, tr("Get Json"), "./config/toolkit/", "Json(*.Json)");
    if (filepath.isEmpty()) {
        printE("Load toolkit Path File Failed!");
        return;
    }

    ui->toolkitPath->setText(filepath);

    loadVisionConfig(filepath.toStdString());

    return;
}

void MakeTool::on_loadPcdBtn_clicked() {
    QString filepath = QFileDialog::getOpenFileName(this, tr("Get PointCloud"), "", "ply(*.ply)");
    if (filepath.isEmpty()) {
        printE("Load Pcd Path File Failed!");
        return;
    }

    if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(filepath.toStdString(), *tool_cloud) == -1)  {
        printE("Load Pcd Failed!");
        return;
    }

    ui->pcdPathEdit->setText(filepath);
    return;
}

void MakeTool::on_flushBtn_clicked() {
    if (tool_cloud->size() == 0) {
        printE("Load Pcd Failed!");
        return;
    }

    if (cones != nullptr) {
        render_tool->RemoveActor(cones);
    }
    if (pointCloudActor != nullptr) {
        render_tool->RemoveActor(pointCloudActor);
    }

    vtkNew<vtkMatrix4x4> vtk_matrix;
    GetTargetInCam(vtk_matrix);

    initDragPoint(vtk_matrix, cones);
    render_tool->AddActor(cones);
    cones->AddPosition(0, 0, 0.0001);

    initPcd(tool_cloud, pointCloudActor);
    pointCloudActor->SetPickable(0);
    render_tool->AddActor(pointCloudActor);

    vtkNew<vtkCallbackCommand> transformCallback;
    transformCallback->SetCallback(printTransformCallback);
    transformCallback->SetClientData(cones);
    if (render_tool) {
        render_tool->RemoveObserver(transformCallback);
        render_tool->AddObserver(vtkCommand::EndEvent, transformCallback);
    }

    if (renderer_grasp->HasViewProp(graspCloudActor)) {
        updateGraspCloudActor();
    }

    render_tool->ResetCamera();
    ui->toolqvtkWidget->renderWindow()->Render();

    return;
}

void MakeTool::GetTargetInCam(vtkNew<vtkMatrix4x4>& vtk_matrix) {
    Flange2Cam = GetFlange2CamMatrix();

    CapBase2Flange = GetCapBase2FlangeMatrix();

    GraspBase2Flange = GetGraspBase2FlangeMatrix();

    Flange2Tcp = GetFlange2TcpMatrix();

	Eigen::Affine3d targetInCam = Eigen::Affine3d::Identity();
    if(ui->handEyeComb->currentIndex() == 0) {
        targetInCam = Flange2Cam.inverse() * CapBase2Flange.inverse() * GraspBase2Flange * Flange2Tcp;
	}
    else {
        targetInCam = Flange2Cam.inverse() * GraspBase2Flange * Flange2Tcp;
    }

    const Eigen::Matrix4d& eigen_matrix = targetInCam.matrix();

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtk_matrix->SetElement(i, j, eigen_matrix(i, j));
        }
    }

    return;
}

Eigen::Affine3d MakeTool::GetFlange2TcpMatrix() {

    if (ui->IsExistTcpBtn->isChecked()) {
        QString filename = ui->tcpCalibEdit->text();
        Eigen::Affine3d transform_tcp = readQuatPoseFromJson(filename.toStdString());
        return transform_tcp;
    }

    double tcpX = ui->tcpXspin->value();
    double tcpY = ui->tcpYspin->value();
    double tcpZ = ui->tcpZspin->value();

    RotateType rt;
    if (ui->eulerTcpBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->quatTcpBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }
    
    Eigen::Vector3d pos = Eigen::Vector3d(tcpX, tcpY, tcpZ);
    QString order = ui->tcpEulerComb->currentText();
    Eigen::Matrix3d matrix;
    if (rt == RotateType::Euler) {
        Eigen::Vector3d euler_angle = Eigen::Vector3d(ui->tcpAngleX->value(), ui->tcpAngleY->value(), ui->tcpAngleZ->value());
        Eigen::Vector3d euler_rad = euler_angle;
        if (ui->tcpeulerTypeComb->currentIndex() == 0) {
            euler_rad = mu->deg2radVec(euler_angle);
        }
        matrix = mu->eulerToMatrix(euler_rad, order.toStdString());
    }
    else if (rt == RotateType::Quat) {
        // w x y z
        Eigen::Quaterniond q = Eigen::Quaterniond(ui->tcpwspin->value(), ui->tcpxspin->value(),
            ui->tcpyspin->value(), ui->tcpzspin->value());
        matrix = mu->quaternionToMatrix(q);
    }
    else if (rt == RotateType::RotateVec) {
        Eigen::Vector3d rotate_vec = Eigen::Vector3d(ui->rotxSpin->value(), ui->rotySpin->value(), ui->rotzSpin->value());
        Eigen::Vector3d rot_vec_rad = rotate_vec;
        if (ui->tcpRotTypeComb->currentIndex() == 0) {
            rot_vec_rad = mu->deg2radVec(rotate_vec);
        }
        matrix = mu->rotVecToMatrix(rot_vec_rad);
    }
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = matrix;
    transform.translation() = pos;

    return transform;
}

Eigen::Affine3d MakeTool::GetGraspBase2FlangeMatrix() {
    double gXSpin = ui->gXSpin->value();
    double gYSpin = ui->gYSpin->value();
    double gZSpin = ui->gZSpin->value();

    RotateType rt;
    if (ui->gEulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->gQuatBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

    Eigen::Vector3d pos = Eigen::Vector3d(gXSpin, gYSpin, gZSpin);
    QString order = ui->gEulerComb->currentText();
    Eigen::Matrix3d matrix;
    if (rt == RotateType::Euler) {
        Eigen::Vector3d euler_angle = Eigen::Vector3d(ui->geulerxspin->value(), ui->geuleryspin->value(), ui->geulerzspin->value());
        Eigen::Vector3d euler_rad = euler_angle;
        if (ui->geulertypeComb->currentIndex() == 0) {
            euler_rad = mu->deg2radVec(euler_angle);
        }
        matrix = mu->eulerToMatrix(euler_rad, order.toStdString());
    }
    else if (rt == RotateType::Quat) {
        // w x y z
        Eigen::Quaterniond q = Eigen::Quaterniond(ui->gquatwspin->value(), ui->gquatxspin->value(),
            ui->gquatyspin->value(), ui->gquatzspin->value());
        matrix = mu->quaternionToMatrix(q);
    }
    else if (rt == RotateType::RotateVec) {
        Eigen::Vector3d rotate_vec = Eigen::Vector3d(ui->grotxspin->value(), ui->grotyspin->value(), ui->grotzspin->value());
        Eigen::Vector3d rot_vec_rad = rotate_vec;
        if (ui->grotTypeComb->currentIndex() == 0) {
            rot_vec_rad = mu->deg2radVec(rotate_vec);
        }
        matrix = mu->rotVecToMatrix(rot_vec_rad);
    }
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = matrix;
    transform.translation() = pos;

    return transform;
}

Eigen::Affine3d MakeTool::GetCapBase2FlangeMatrix() {
    double capXspin = ui->capXspin->value();
    double capYspin = ui->capYspin->value();
    double capZspin = ui->capZspin->value();

    RotateType rt;
    if (ui->capEulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->quatcapBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

    Eigen::Vector3d pos = Eigen::Vector3d(capXspin, capYspin, capZspin);
    QString order = ui->capEulerComb->currentText();
    Eigen::Matrix3d matrix;
    if (rt == RotateType::Euler) {
        Eigen::Vector3d euler_angle = Eigen::Vector3d(ui->capAngleX->value(), ui->capAngleY->value(), ui->capAngleZ->value());
        Eigen::Vector3d euler_rad = euler_angle;
        if (ui->capeulerTypeComb->currentIndex() == 0) {
            euler_rad = mu->deg2radVec(euler_angle);
        }
        matrix = mu->eulerToMatrix(euler_rad, order.toStdString());
    }
    else if (rt == RotateType::Quat) {
        // w x y z
        Eigen::Quaterniond q = Eigen::Quaterniond(ui->capwspin->value(), ui->capxspin->value(),
            ui->capyspin->value(), ui->capzspin->value());
        matrix = mu->quaternionToMatrix(q);
    }
    else if (rt == RotateType::RotateVec) {
        Eigen::Vector3d rotate_vec = Eigen::Vector3d(ui->caprotxSpin->value(), ui->caprotySpin->value(), ui->caprotzSpin->value());
        Eigen::Vector3d rot_vec_rad = rotate_vec;
        if (ui->caprotTypeComb->currentIndex() == 0) {
            rot_vec_rad = mu->deg2radVec(rotate_vec);
        }
        matrix = mu->rotVecToMatrix(rot_vec_rad);
    }
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = matrix;
    transform.translation() = pos;

    return transform;
}

Eigen::Affine3d MakeTool::GetFlange2CamMatrix() {
    QString filename = ui->calibEyeFilePath->text();

	Eigen::Affine3d transform_handeye = readQuatPoseFromJson(filename.toStdString());

    return transform_handeye;
}

void MakeTool::timerHandle() {
    Eigen::Matrix3d rotation_matrix = graspInCamMatrix.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);

    ui->toolXspin->setValue(graspInCamMatrix(0, 3));
    ui->toolYspin->setValue(graspInCamMatrix(1, 3));
    ui->toolZspin->setValue(graspInCamMatrix(2, 3));

    ui->toolwspin->setValue(quaternion.w());
    ui->toolxspin->setValue(quaternion.x());
    ui->toolyspin->setValue(quaternion.y());
    ui->toolzspin->setValue(quaternion.z());
}

void MakeTool::timer1Handle() {

    QListWidgetItem* item = nullptr;
    for (int i = 0; i < ui->graspListWidget->count(); ++i) {
        if (ui->graspListWidget->item(i) == current_item) {
            item = current_item;
            break;
        }
    }
    if (!item) {
        current_item = nullptr;
        return;
    }

    Eigen::Matrix3d rotation_matrix = multiGraspInCamMatrix.block<3, 3>(0, 0);
    Eigen::Quaterniond quaternion = Eigen::Quaterniond(rotation_matrix);

    // if (quaternion.w() == 1 && quaternion.x() == 0 && quaternion.y() == 0 && quaternion.z() == 0) return;

    SphereData data;
    data.X = multiGraspInCamMatrix(0, 3);
    data.Y = multiGraspInCamMatrix(1, 3);
    data.Z = multiGraspInCamMatrix(2, 3);

    data.w = quaternion.w();
    data.x = quaternion.x();
    data.y = quaternion.y();
    data.z = quaternion.z();

    QVariantList pose;
    pose << data.X << data.Y << data.Z << data.w << data.x << data.y << data.z;

    current_item->setData(GRASP_DATA_ROLE, pose);
    updateSpinboxes(data);
}

void MakeTool::on_updateTcpBtn_clicked() {
    // Flange2Tcp
    Eigen::Affine3d cam2target = Eigen::Affine3d::Identity();
    Eigen::Quaterniond q = Eigen::Quaterniond(ui->toolwspin->value(), ui->toolxspin->value(), ui->toolyspin->value(), ui->toolzspin->value());
    cam2target.linear() = mu->quaternionToMatrix(q);
    cam2target.translation() = Eigen::Vector3d(ui->toolXspin->value(), ui->toolYspin->value(), ui->toolZspin->value());
    
    Flange2Tcp = GraspBase2Flange.inverse() * CapBase2Flange * Flange2Cam * cam2target;

    Eigen::Quaterniond new_q = mu->matrixToQuaternion(Flange2Tcp.linear());

    ui->tcpXspin->setValue(Flange2Tcp.translation().x());
    ui->tcpYspin->setValue(Flange2Tcp.translation().y());
    ui->tcpZspin->setValue(Flange2Tcp.translation().z());

    ui->tcpwspin->setValue(new_q.w());
    ui->tcpxspin->setValue(new_q.x());
    ui->tcpyspin->setValue(new_q.y());
    ui->tcpzspin->setValue(new_q.z());
}

void MakeTool::on_loadTcpBtn_clicked() {
    QString filepath = QFileDialog::getOpenFileName(this, tr("Get Tcp CalibFile"), "./config/tcp/", "Json (*.json)");
    if (filepath.isEmpty()) {
        printE("Load Tcp calibration json file failed!");
        return;
    }
    ui->tcpCalibEdit->setText(filepath);
}

void MakeTool::on_loadHandeyePathBtn_clicked() {
    QString filepath = QFileDialog::getOpenFileName(this, tr("Get HandEye CalibFile"), "./config/handeye/", "Json (*.json)");
    if (filepath.isEmpty()) {
        printE("Load Tcp calibration json file failed!");
        return;
    }

    ui->calibEyeFilePath->setText(filepath);
}

void MakeTool::GetSingleGraspPoint(SphereData& data) {

    if (ui->gpointXspin->value() == 0 && ui->gpointYspin->value() == 0 && ui->gpointZspin->value() == 0) {
        data.X = ui->toolXspin->value();
        data.Y = ui->toolYspin->value();
        data.Z = ui->toolZspin->value();

        data.w = ui->toolwspin->value();
        data.x = ui->toolxspin->value();
        data.y = ui->toolyspin->value();
        data.z = ui->toolzspin->value();
    }
    else {
        data.X = ui->gpointXspin->value();
        data.Y = ui->gpointYspin->value();
        data.Z = ui->gpointZspin->value();
        data.w = ui->gpointwspin->value();
        data.x = ui->gpointxspin->value();
        data.y = ui->gpointyspin->value();
        data.z = ui->gpointzspin->value();
    }
    
    return;
}

void MakeTool::autoAddGraspPoint(const SphereData& data) {
    int count = ui->graspListWidget->count();

    vtkNew<vtkActor> new_actor;
    createSphereActor(data, new_actor);

    if (count == 0) {
        updateGraspCloudActor();
    }
    renderer_grasp->AddActor(new_actor);
    allActors.push_back(new_actor);

    max_count = count + 1;
    if (last_max_count >= max_count) {
        max_count = last_max_count + 1;
    }
    last_max_count = max_count;

    auto* item = new QListWidgetItem(QString(u8"GraspPoint_%1").arg(max_count));
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    item->setData(ACTOR_POINTER_ROLE, QVariant::fromValue<void*>(new_actor));
    item->setData(GRASP_DATA_ROLE, QVariant::fromValue(data));

    ui->graspListWidget->addItem(item);
    ui->graspListWidget->setCurrentItem(item);
    onListItemClicked(item);

    renderer_grasp->ResetCamera();
    ui->qvtkWidget->renderWindow()->Render();
}

void MakeTool::on_newGraspBtn_clicked() {
    int count = ui->graspListWidget->count();

    std::random_device rd;
    std::uniform_real_distribution<double> dist(2.0, 8.0);
    double random_double_1 = dist(rd);
    double random_double_2 = dist(rd);
    double random_double_3 = dist(rd);

    SphereData singleToolPoint;
    if (count == 0) {
        GetSingleGraspPoint(singleToolPoint);
    }
    else {
        GetSingleGraspPoint(singleToolPoint);
        singleToolPoint.X += random_double_1;
        singleToolPoint.Y += random_double_2;
        singleToolPoint.Z += random_double_3;
    }
    
    vtkNew<vtkActor> new_actor;
    createSphereActor(singleToolPoint, new_actor);

    if (count == 0) {
        updateGraspCloudActor();
    }
    renderer_grasp->AddActor(new_actor);
    allActors.push_back(new_actor);

    max_count = count + 1;
    if (last_max_count >= max_count) {
        max_count = last_max_count + 1;
    }
    last_max_count = max_count;
    
    auto* item = new QListWidgetItem(QString(u8"GraspPoint_%1").arg(max_count));
    item->setFlags(item->flags() | Qt::ItemIsDragEnabled | Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    item->setData(ACTOR_POINTER_ROLE, QVariant::fromValue<void*>(new_actor));
    item->setData(GRASP_DATA_ROLE, QVariant::fromValue(singleToolPoint));

    ui->graspListWidget->addItem(item);
    ui->graspListWidget->setCurrentItem(item);
    onListItemClicked(item);

    renderer_grasp->ResetCamera();
    ui->qvtkWidget->renderWindow()->Render();
}

void MakeTool::createSphereActor(const SphereData& data, vtkNew<vtkActor>& actor) {

    Eigen::Quaterniond q = Eigen::Quaterniond(data.w, data.x, data.y, data.z);
    Eigen::Matrix3d matrix = mu->quaternionToMatrix(q);
    Eigen::Vector3d pos = Eigen::Vector3d(data.X, data.Y, data.Z);
    Eigen::Affine3d transform_actor = Eigen::Affine3d::Identity();
    transform_actor.linear() = matrix;
    transform_actor.translation() = pos;

    vtkNew<vtkMatrix4x4> vtk_matrix;
    const Eigen::Matrix4d& eigen_matrix = transform_actor.matrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtk_matrix->SetElement(i, j, eigen_matrix(i, j));
        }
    }

    Create(vtkSphereSource, source);
    source->SetCenter(0.0, 0.0, 0.0);
    source->SetRadius(3.0);
    source->SetPhiResolution(100);
    source->SetThetaResolution(100);
    source->Update();

    vtkNew<vtkTransform> transform;
    transform->SetMatrix(vtk_matrix);

    Create(vtkPolyDataMapper, mapper);
    mapper->SetInputConnection(source->GetOutputPort());
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    actor->SetUserTransform(transform);

    return;
}

void MakeTool::updateSphereActor(const SphereData& data, vtkActor* actor) {

    Eigen::Quaterniond q = Eigen::Quaterniond(data.w, data.x, data.y, data.z);
    Eigen::Matrix3d matrix = mu->quaternionToMatrix(q);
    Eigen::Vector3d pos = Eigen::Vector3d(data.X, data.Y, data.Z);
    Eigen::Affine3d transform_actor = Eigen::Affine3d::Identity();
    transform_actor.linear() = matrix;
    transform_actor.translation() = pos;

    vtkNew<vtkMatrix4x4> vtk_matrix;
    const Eigen::Matrix4d& eigen_matrix = transform_actor.matrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtk_matrix->SetElement(i, j, eigen_matrix(i, j));
        }
    }

    vtkNew<vtkTransform> transform;
    transform->SetMatrix(vtk_matrix);
    actor->SetUserTransform(transform);

    return;
}

void MakeTool::onListItemClicked(QListWidgetItem* item) {
    if (!item) return;

    if (!m_timer_1->isActive()) {
        m_timer_1->start(100);
    }

    if (currentSelectedActor) {
        currentSelectedActor->GetProperty()->SetColor(1.0, 1.0, 1.0); // White
    }

    for (auto& actor : allActors) {
        actor->SetPickable(0);
    }
    current_item = item;
    QVariant actorVar = item->data(ACTOR_POINTER_ROLE);
    if (!actorVar.isNull()) {
        vtkActor* clickedActor = static_cast<vtkActor*>(actorVar.value<void*>());
        if (clickedActor) {
            clickedActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
            currentSelectedActor = clickedActor;
            clickedActor->SetPickable(1);
        }
    }

    transformCallback_multi->SetClientData(currentSelectedActor);
    if (renderer_grasp) {
        renderer_grasp->RemoveObserver(transformCallback_multi);
        renderer_grasp->AddObserver(vtkCommand::EndEvent, transformCallback_multi);
    }

    QVariant dataVar = item->data(GRASP_DATA_ROLE);
    if (!dataVar.isNull()) {
        SphereData data = dataVar.value<SphereData>();
        updateSpinboxes(data);
    }
    ui->qvtkWidget->renderWindow()->Render();
}

void MakeTool::updateSpinboxes(const SphereData& data) {
    ui->gpointXspin->blockSignals(true);
    ui->gpointYspin->blockSignals(true);
    ui->gpointZspin->blockSignals(true);
    ui->gpointwspin->blockSignals(true);
    ui->gpointxspin->blockSignals(true);
    ui->gpointyspin->blockSignals(true);
    ui->gpointzspin->blockSignals(true);

    ui->gpointXspin->setValue(data.X);
    ui->gpointYspin->setValue(data.Y);
    ui->gpointZspin->setValue(data.Z);
    ui->gpointwspin->setValue(data.w);
    ui->gpointxspin->setValue(data.x);
    ui->gpointyspin->setValue(data.y);
    ui->gpointzspin->setValue(data.z);

    ui->gpointXspin->blockSignals(false);
    ui->gpointYspin->blockSignals(false);
    ui->gpointZspin->blockSignals(false);
    ui->gpointwspin->blockSignals(false);
    ui->gpointxspin->blockSignals(false);
    ui->gpointyspin->blockSignals(false);
    ui->gpointzspin->blockSignals(false);
}

void MakeTool::onDeleteActionTriggered()
{
    m_timer_1->stop();

    // 获取当前所有选中的 items
    QList<QListWidgetItem*> selectedItems = ui->graspListWidget->selectedItems();

    if (selectedItems.isEmpty()) {
        return; 
    }

    QString confirmationText;
    if (selectedItems.size() == 1) {
        confirmationText = tr(u8"确认删除抓取点 '%1'?").arg(selectedItems.first()->text());
    }

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr(u8"确认 删除"),
        confirmationText,
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        for (auto item : selectedItems) {
            delete ui->graspListWidget->takeItem(ui->graspListWidget->row(item));
            renderer_grasp->RemoveActor(currentSelectedActor);
        }
    }
    ui->qvtkWidget->renderWindow()->Render();

	current_item = nullptr;
}

void MakeTool::onCustomContextMenuRequested(const QPoint& pos)
{
    
    QPoint globalPos = ui->graspListWidget->mapToGlobal(pos);

    if (ui->graspListWidget->selectedItems().isEmpty()) {
        return; // 直接返回，不显示菜单
    }

    QMenu contextMenu(tr("Context menu"), this);

    QAction deleteAction(tr("&Delete"), this);
    deleteAction.setShortcut(QKeySequence::Delete);

    connect(&deleteAction, &QAction::triggered, this, &MakeTool::onDeleteActionTriggered);

    contextMenu.addAction(&deleteAction);

    contextMenu.exec(globalPos);
}

void MakeTool::on_saveMultiGraspDataBtn_clicked() {

    if (!ui->graspListWidget) {
        return;
    }

    json jArray = json::array();
    int count = ui->graspListWidget->count();
    for (int i = 0; i < count; ++i) {
        QListWidgetItem* item = ui->graspListWidget->item(i);

        if (!item) continue;

        QVariant dataVar = item->data(GRASP_DATA_ROLE);
        SphereData data;
        const QVariantList list = dataVar.toList();
        if (list.size() == 7) {
            data.X = list.at(0).toDouble();
            data.Y = list.at(1).toDouble();
            data.Z = list.at(2).toDouble();
            data.w = list.at(3).toDouble();
            data.x = list.at(4).toDouble();
            data.y = list.at(5).toDouble();
            data.z = list.at(6).toDouble();
        }
        else if (dataVar.canConvert<SphereData>()) {
            data = dataVar.value<SphereData>();
        }
        else {
            continue;
        }

        jArray.push_back(data);
    }   

    QString filename = QString("multiGrasp_") + QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QString filepath = QFileDialog::getSaveFileName(this, tr("Save multiGrasp"), filename, "JSON(*.json)");
    if (filepath.isEmpty()) return;

    // 偏移位置
    nlohmann::json j;
    j["multiGrasp"] = jArray;

    //手眼标定
    Eigen::Quaterniond quaternion = mu->matrixToQuaternion(Flange2Cam.linear());
    std::vector<double> flange2cam = { Flange2Cam.translation().x(), Flange2Cam.translation().y(), Flange2Cam.translation().z(), quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["Flange2Cam"] = flange2cam;

    // 拍照位法兰位置
    quaternion = mu->matrixToQuaternion(CapBase2Flange.linear());
    std::vector<double> capflange = { CapBase2Flange.translation().x(), CapBase2Flange.translation().y(), CapBase2Flange.translation().z(),
        quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["capFlange"] = capflange;

    // 抓取位置
    quaternion = mu->matrixToQuaternion(GraspBase2Flange.linear());
    std::vector<double> graspflange = { GraspBase2Flange.translation().x(), GraspBase2Flange.translation().y(), GraspBase2Flange.translation().z(),
        quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["graspFlange"] = graspflange;

    // tcp位置
    quaternion = mu->matrixToQuaternion(Flange2Tcp.linear());
    std::vector<double> flange2tcp = { Flange2Tcp.translation().x(), Flange2Tcp.translation().y(), Flange2Tcp.translation().z(),
        quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["tcp"] = flange2tcp;

    j["calib"] = ui->calibEyeFilePath->text().toStdString();
    j["template"] = ui->pcdPathEdit->text().toStdString();
	j["handeye"] = ui->handEyeComb->currentIndex();

    std::ofstream file(filepath.toStdString());
    if (!file.is_open()) {
        printE(tr("Data (%1) save failed!").arg(filepath));
        return;
    }
    printI(tr("Data (%1) save done!").arg(filepath));
    file << j.dump(4);
    file.close();

    return;
}

void MakeTool::loadVisionConfig(const std::string& toolkit_path) {
    std::ifstream file(toolkit_path);
    if (!file.is_open()) {
        return;
    }

    json json_data;
    try {
        file >> json_data;
        utils::MultiGraspConfig vc = json_data.get<utils::MultiGraspConfig>();

        ui->quatcapBtn->clicked();
        ui->quatcapBtn->setChecked(true);
        ui->capXspin->setValue(vc.capFlange[0]);
        ui->capYspin->setValue(vc.capFlange[1]);
        ui->capZspin->setValue(vc.capFlange[2]);
        ui->capwspin->setValue(vc.capFlange[3]);
        ui->capxspin->setValue(vc.capFlange[4]);
        ui->capyspin->setValue(vc.capFlange[5]);
        ui->capzspin->setValue(vc.capFlange[6]);

        ui->quatTcpBtn->clicked();
        ui->quatTcpBtn->setChecked(true);
        ui->tcpXspin->setValue(vc.tcp[0]);
        ui->tcpYspin->setValue(vc.tcp[1]);
        ui->tcpZspin->setValue(vc.tcp[2]);
        ui->tcpwspin->setValue(vc.tcp[3]);
        ui->tcpxspin->setValue(vc.tcp[4]);
        ui->tcpyspin->setValue(vc.tcp[5]);
        ui->tcpzspin->setValue(vc.tcp[6]);

        ui->gQuatBtn->clicked();
        ui->gQuatBtn->setChecked(true);
        ui->gXSpin->setValue(vc.graspFlange[0]);
        ui->gYSpin->setValue(vc.graspFlange[1]);
        ui->gZSpin->setValue(vc.graspFlange[2]);
        ui->gquatwspin->setValue(vc.graspFlange[3]);
        ui->gquatxspin->setValue(vc.graspFlange[4]);
        ui->gquatyspin->setValue(vc.graspFlange[5]);
        ui->gquatzspin->setValue(vc.graspFlange[6]);

        ui->calibEyeFilePath->setText(QString::fromStdString(vc.calib_path));
        ui->pcdPathEdit->setText(QString::fromStdString(vc.template_pcd_path));

        ui->NotExistTcpBtn->clicked();
        ui->NotExistTcpBtn->setChecked(true);

		ui->handEyeComb->setCurrentIndex(static_cast<int>(vc.handEyeSetting));

        if (pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>(vc.template_pcd_path, *tool_cloud) == -1) {
            printE("Load Pcd Failed!");
            return;
        }

        max_count = 0;
        ui->graspListWidget->clear();
        for (auto pose : vc.multiGrasp) {
            SphereData data;
            data.X = pose.X;
            data.Y = pose.Y;
            data.Z = pose.Z;
            data.w = pose.w;
            data.x = pose.x;
            data.y = pose.y;
            data.z = pose.z;
            autoAddGraspPoint(data);
        }
    }
    catch (const std::exception& e) {
        file.close();
        return;
    }
    file.close();

    return;
}

Eigen::Affine3d MakeTool::readQuatPoseFromJson(const std::string& filepath) {
    try
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            printE("file open failed!");
            return Eigen::Affine3d::Identity();
        }

        json jsonData;
        file >> jsonData;
        file.close();

        std::vector<double> quaternion = jsonData["Quaternion"].get<std::vector<double>>();
        std::vector<double> translation = jsonData["Translation"].get<std::vector<double>>();

        Eigen::Quaterniond eigen_quat(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3]
        );
        Eigen::Vector3d eigen_translation(
            translation[0],
            translation[1],
            translation[2]
        );

        Eigen::Affine3d transform_tcp = Eigen::Affine3d::Identity();
        transform_tcp.linear() = eigen_quat.toRotationMatrix();
        transform_tcp.translation() = eigen_translation;

        return transform_tcp;
    }
    catch (const std::exception&)
    {
        printE("file parser failed!");
        return Eigen::Affine3d::Identity();
    }
}