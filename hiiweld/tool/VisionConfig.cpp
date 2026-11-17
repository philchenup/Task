/**
 * @file VisionConfig.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "VisionConfig.h"
#include "ui_VisionConfig.h"
#include <QDesktopServices> 
#include <fstream>
#include "vtk_nlohmannjson.h"
#include VTK_NLOHMANN_JSON(json.hpp)
#include <QDateTime>
#include <QFileDialog>
#include <pcl/io/ply_io.h> 
#include <pcl/common/transforms.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/filters/passthrough.h>
#include "utils/pcd_utils.h"

/** @brief 创建点云成员并初始化 PCL/VTK 视图。 */
VisionConfig::VisionConfig(QWidget* parent) :CustomDialog(parent), 
m_viewer(nullptr),
ui(new Ui::VisionConfig), 
m_view_cloud(new ct::Cloud), 
m_model_cloud(new ct::Cloud),
m_background_cloud(new ct::Cloud)
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

    ui->tabWidget->setCurrentIndex(0);
}

/** @brief 释放 UI。
VisionConfig::~VisionConfig()
{
    delete ui;
}

/** @brief 选择并加载场景点云到视图。 */
void VisionConfig::on_loadPcdBtn_clicked() {
    QString PoseFilePath = QFileDialog::getOpenFileName(
        this,                                    // 父窗口
        tr("Open PCD File"),                      // 对话框标题
        "",
        tr("PCD File (*.ply | *.pcd);;所有文件 (*)")   // 文件过滤器
    );
    if (!PoseFilePath.isEmpty()) {
        ui->pcdPathEdit->setText(PoseFilePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }

    if (pcl::io::loadPLYFile(PoseFilePath.toStdString(), *m_view_cloud) == -1) // 如果返回值为-1，表示加载失败
    {
        printE(tr("Point Cloud file read failed!"));
        return;
    }

    QString info = QString("PCD file load successfully,Points num is %1!").arg(m_view_cloud->size());
    printI(info);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*m_view_cloud, *cloud_src);

    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();
    m_viewer->addPointCloud(cloud_src, "cloud_src");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_src");
    m_viewer->resetCamera();

    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}

/** @brief 选择并加载背景点云。 */
void VisionConfig::on_loadBackBtn_clicked() {
    QString PoseFilePath = QFileDialog::getOpenFileName(
        this,                                    // 父窗口
        tr("Open PCD File"),                      // 对话框标题
        "",
        tr("PCD File (*.ply | *.pcd);;所有文件 (*)")   // 文件过滤器
    );
    if (!PoseFilePath.isEmpty()) {
        ui->backGroundPcdPath->setText(PoseFilePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }

    if (pcl::io::loadPLYFile(PoseFilePath.toStdString(), *m_background_cloud) == -1) // 如果返回值为-1，表示加载失败
    {
        printE(tr("Point Cloud file read failed!"));
        return;
    }

    QString info = QString("PCD file load successfully,Points num is %1!").arg(m_background_cloud->size());
    printI(info);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*m_background_cloud, *cloud_back);

    m_viewer->addPointCloud(cloud_back, "cloud_back");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud_back");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_back");
    m_viewer->resetCamera();

    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}

/** @brief 执行配准处理链。 */
void VisionConfig::on_runChainBtn_clicked() {
    // 1 下采样
    ct::Cloud::Ptr cloud_filtered(new ct::Cloud);
    PcdUtils::pcd_voxel_down(m_view_cloud, cloud_filtered, ui->downsampleRadiusSpin->value());

    // 2 直通滤波
    ct::Cloud::Ptr cloud_pass(new ct::Cloud);
    PcdUtils::pcd_pass_filters(cloud_filtered, cloud_pass, ui->minZSpin->value(), ui->maxZSpin->value());

    // 3 地面分割
    ct::Cloud::Ptr cloud_target(new ct::Cloud);
    PcdUtils::removeBackground(m_background_cloud, cloud_pass, cloud_target, ui->segBackResSpin->value());

    // 4 杂点过滤
    ct::Cloud::Ptr cloud_radius_filter(new ct::Cloud);
    PcdUtils::pcd_radius_filter(cloud_target, cloud_radius_filter, ui->filterRadiusSpin->value(), ui->minPtsSpin->value());

    if (cloud_radius_filter->size() < 100) {
        printE(tr("The cloud points num is too low, check the input sample parameter!"));
        return;
    }

    if (ui->regCheck->isChecked()) {
        VisionReg(cloud_radius_filter);
    }
    else {
        this->showPcd(cloud_radius_filter);
    }
}

/** @brief 保存配准结果或配置 JSON。 */
void VisionConfig::on_saveBtn_clicked() {
    QString filename = "VisionConfig_" + QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");
    
    QString filepath = QFileDialog::getSaveFileName(this, tr("Save VisionConfig"), filename, "JSON(*.json)");
    if (filepath.isEmpty()) return;

    nlohmann::json j;

    j["Filter"]["DownSampleRadius"] = ui->downsampleRadiusSpin->value();
    j["Filter"]["PassMinZ"] = ui->minZSpin->value();
    j["Filter"]["PassMaxZ"] = ui->maxZSpin->value();
    j["Filter"]["groundModelPath"] = ui->backGroundPcdPath->text().toStdString();
    j["Filter"]["groundRadius"] = ui->segBackResSpin->value();
    j["Filter"]["filterRadius"] = ui->filterRadiusSpin->value();
    j["Filter"]["minPts"] = ui->minPtsSpin->value();

    j["Reg"]["modelPath"] = ui->modelPcdPath->text().toStdString();
    j["Reg"]["regfitness"] = ui->regfitness->value();
    j["Reg"]["maxIter"] = ui->regMaxIter->value();
    j["Reg"]["regPriority"] = ui->regPriCheck->currentIndex();

    std::ofstream file(filepath.toStdString());
    if (!file.is_open()) {
        printE(tr("The Vision Config (%1) save failed!").arg(filepath));
        return;
    }
    file << j.dump(4);
    file.close();
    
    printI(tr("The Vision Config (%1) save done!").arg(filepath));
}

/** @brief 加载模型点云。 */
void VisionConfig::on_loadModelPcdBtn_clicked() {
    QString PoseFilePath = QFileDialog::getOpenFileName(
        this,                                    // 父窗口
        tr("Open PCD File"),                      // 对话框标题
        "",
        tr("PCD File (*.ply | *.pcd);;所有文件 (*)")   // 文件过滤器
    );
    if (!PoseFilePath.isEmpty()) {
        ui->modelPcdPath->setText(PoseFilePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }

    if (pcl::io::loadPLYFile(PoseFilePath.toStdString(), *m_model_cloud) == -1) // 如果返回值为-1，表示加载失败
    {
        printE(tr("Point Cloud file read failed!"));
        return;
    }

    this->showPcd(m_model_cloud);
}

/** @brief 将输入点云与模型/背景配准。 */
void VisionConfig::VisionReg(const ct::Cloud::Ptr& cloud_in) {
    if (m_model_cloud->size() < 100 || cloud_in->size() < 100) {
        printE(tr("model cloud or scene cloud no enough points!"));
        return;
    }
    Eigen::Affine3f transform_;

    REG::Parameters pars;
    pars.max_icp = ui->regMaxIter->value();
    PcdUtils::FRReg(m_model_cloud, cloud_in, pars, PcdUtils::Method(ui->regPriCheck->currentIndex()), transform_);

    ct::Cloud::Ptr transformPcd(new ct::Cloud);
    pcl::transformPointCloud(*m_model_cloud, *transformPcd, transform_);
    double fitness = calculateFitness(transformPcd, cloud_in, 3.0);

    if (fitness < ui->regfitness->value()) {
        printE(tr("registration is failed!"));
        return;
    }
    this->showPcd(transformPcd);
}

/** @brief 评估配准质量。 */
double VisionConfig::calculateFitness(const ct::Cloud::Ptr& source, const  ct::Cloud::Ptr& target,
    double max_correspondence_distance) {
    if (source->empty() || target->empty()) {
        return 0.0;
    }
    pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
    kdtree.setInputCloud(target);

    int inlier_count = 0;
    for (const auto& point : source->points) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < max_correspondence_distance * max_correspondence_distance) {
                inlier_count++;
            }
        }
    }
    double fitness = static_cast<double>(inlier_count) / static_cast<double>(source->size());

    return fitness;
}

/** @brief 在视图中绘制点云。 */
void VisionConfig::showPcd(const ct::Cloud::Ptr& cloud_in) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_in, *cloud_out);

    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();
    m_viewer->addPointCloud(cloud_out, "cloud");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*m_view_cloud, *cloud_src);
    if (!ui->showSrcPcdCheck->isChecked()) {
        m_viewer->addPointCloud(cloud_src, "cloud_src");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_src");
    }
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    m_viewer->resetCamera();

    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}