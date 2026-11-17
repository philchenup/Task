/**
 * @file VisionConfig.h
 * @brief 视觉点云配准配置对话框：加载场景/背景/模型点云，ICP 链式配准与拟合度评估。
 */
#ifndef VISIONCONFIG_H
#define VISIONCONFIG_H
#include <cstdio>
#include "base/customdialog.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <QLabel>

namespace Ui
{
    class VisionConfig;
}

/**
 * @brief 点云视觉标定与配准 UI，维护视图点云、模型与背景。
 */
class VisionConfig : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit VisionConfig(QWidget* parent = nullptr);
    ~VisionConfig();

    /** @brief 在 PCL 视图中显示点云。 @param cloud_in 待显示点云。 */
    void showPcd(const ct::Cloud::Ptr& cloud_in);

    /** @brief 对输入点云执行配准链（相对模型/背景）。 */
    void VisionReg(const ct::Cloud::Ptr& cloud_in);

    /**
     * @brief ICP 拟合度（内点比例或均方距离指标，取决于实现）。
     * @param source 源点云
     * @param target 目标点云
     * @param max_correspondence_distance 最大对应距离
     */
    double calculateFitness(const ct::Cloud::Ptr& source, const  ct::Cloud::Ptr& target,
        double max_correspondence_distance);

public slots:
    /** @brief 加载当前视图/场景点云文件。 */
    void on_loadPcdBtn_clicked();
    /** @brief 加载背景点云。 */
    void on_loadBackBtn_clicked();
    /** @brief 执行完整配准流程。 */
    void on_runChainBtn_clicked();
    /** @brief 保存配准结果或参数。 */
    void on_saveBtn_clicked();
    /** @brief 加载模型点云。 */
    void on_loadModelPcdBtn_clicked();

private:
    ct::Cloud::Ptr m_view_cloud;       ///< 当前场景/视图点云
    ct::Cloud::Ptr m_model_cloud;      ///< 模型点云
    ct::Cloud::Ptr m_background_cloud; ///< 背景点云
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    Ui::VisionConfig* ui;
};

#endif // VISIONCONFIG_H
