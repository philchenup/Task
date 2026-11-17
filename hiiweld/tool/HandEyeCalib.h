/**
 * @file HandEyeCalib.h
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#ifndef HANDEYECALIB_H
#define HANDEYECALIB_H

#include "base/customdialog.h"
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
#include <vtkGenericOpenGLRenderWindow.h>

namespace Ui
{
    class HandEyeCalib;
}

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eigenVector;

enum Col { COL_INDEX = 0, COL_CHECK, COL_X, COL_Y, COL_Z, COL_COUNT };

/** @brief 3D 一致性评估结果（各姿态/点位误差统计）。 */
struct Point3DConsistency
{
    Eigen::Vector3d meanAbsError = Eigen::Vector3d::Zero(); // 全局 X/Y/Z 平均绝对误差
    double reproject_error_pixel = 0.0;
    int    numPoses = 0;

    std::vector<Eigen::Vector3d> perPoseMeanAbsError; // [i] 该位姿 X/Y/Z 平均绝对误差
    std::vector<double>          perPoseError;        // [i] 该位姿标量误差（欧氏距离均值）

    std::vector<std::vector<Eigen::Vector3d>> perPointDev;

    std::vector<Eigen::Vector3d> pointMeanInBase;
};

/** @brief TCP/点触式手眼标定结果与 RMSE。 */
struct EyeHandCalibResult {
    bool success = false;
    Eigen::Affine3d T_calib = Eigen::Affine3d::Identity();  // ^{F}T_{C}
    double rmse = 0.0;
    std::vector<double> per_sample_error;
    Eigen::Matrix<double, 1, 7> quat_vec;
};

/**
 * @brief 手眼标定对话框：标定板图像法与 TCP 点触法，眼在手上/眼在手外。
 */
class HandEyeCalib : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit HandEyeCalib(QWidget* parent = nullptr);
    ~HandEyeCalib();

    /** @brief 标定板角点/圆点 + OpenCV 手眼标定主流程。 */
    void markCalibMethod();

    /** @brief TCP 点触 + 刚体 SVD 手眼标定。 */
    void tcpCalibMethod();

    /** @brief 位姿 JSON 中旋转表示方式。 */
    enum class RotateType
    {
        Euler,
        Quat,
        RotateVec
    };


    std::vector<Eigen::Vector3d> readXYZFromJson(const std::string& pose_path);

    std::vector<Eigen::Affine3d> readFlangePoseFromJson(const std::string& pose_path);

    void writeCalibration(const Eigen::Affine3f& result);

    double AssessImageQuality(const cv::Mat& image);

    void GenerateW3DPoints(bool symmetric, bool chess_board, int pattern_x, int pattern_y, double square_size, std::vector<cv::Point3f>& objp);

    void updateVtkWindow(const Eigen::Affine3f& transform, const Point3DConsistency& cons, bool handineye);

    void updateVtkWindow(const Eigen::Affine3f& transform, const EyeHandCalibResult& cons, bool handineye);

    void updateTable(QTableWidget* table, std::vector<bool>& validIndex, const Point3DConsistency& res);
    /**
     * @brief 眼在手外标定（Eye-to-Hand；函数名 eye_on_hand）。
     * @param pose_path 机器人位姿 CSV/JSON 路径
     * @param image_path 标定图像文件夹路径
     * @param square_size 棋盘格方格边长
     * @param pattern_x 棋盘格横向角点数
     * @param pattern_y 棋盘格纵向角点数
     * @param chess_board 是否为棋盘格
     * @param symmetric 是否为对称圆点网格
     * @param save_path 处理后图像保存路径
     * @param cons 输出 3D 一致性评估
     * @return 相机相对基座的仿射变换
     */
    Eigen::Affine3f eye_on_hand(
        const std::string& pose_path,
        const std::string& image_path,
        double square_size,
        int pattern_x,
        int pattern_y,
        bool chess_board,
        bool symmetric,
        const std::string& save_path,
        Point3DConsistency& cons);


    /**
     * @brief 眼在手上标定（Eye-in-Hand；函数名 eye_in_hand）。
     * @param pose_path 机器人位姿 CSV/JSON 路径
     * @param image_path 标定图像文件夹路径
     * @param square_size 棋盘格方格边长
     * @param pattern_x 棋盘格横向角点数
     * @param pattern_y 棋盘格纵向角点数
     * @param chess_board 是否为棋盘格
     * @param symmetric 是否为对称圆点网格
     * @param save_path 处理后图像保存路径
     * @param cons 输出 3D 一致性评估
     * @return 相机相对法兰/末端的仿射变换
     */
    Eigen::Affine3f eye_in_hand(
        const std::string& pose_path,
        const std::string& image_path,
        double square_size,
        int pattern_x,
        int pattern_y,
        bool chess_board,
        bool symmetric,
        const std::string& save_path,
        Point3DConsistency& cons);

    /**
     * @brief 眼在手上 3D 一致性：p_base = T_end2base * T_cam2end * T_board2cam * p
     *        全部有效位姿参与均值，不跳过第 0 帧。
     */
    Point3DConsistency evaluate3DPointConsistencyEyeInHand(
        const std::vector<cv::Mat>& R_board2cams,
        const std::vector<cv::Mat>& t_board2cams,
        const std::vector<cv::Mat>& R_end2bases,
        const std::vector<cv::Mat>& t_end2bases,
        const cv::Mat& T_cam2end,
        const std::vector<cv::Point3f>& objp);

    /**
     * @brief 眼在手外 3D 一致性：p_end = T_end2base^{-1} * T_cam2base * T_board2cam * p
     *        输入位姿约定与眼在手上相同（end→base）；全部有效位姿参与均值。
     */
    Point3DConsistency evaluate3DPointConsistencyEyeOnHand(
        const std::vector<cv::Mat>& R_board2cams,
        const std::vector<cv::Mat>& t_board2cams,
        const std::vector<cv::Mat>& R_end2bases,
        const std::vector<cv::Mat>& t_end2bases,
        const cv::Mat& T_cam2base,
        const std::vector<cv::Point3f>& objp);

    EyeHandCalibResult CalibrateEyeInHand(
        const std::vector<Eigen::Vector3d>& points_in_camera,
        const std::vector<Eigen::Affine3d>& flanges_in_base,
        const std::vector<Eigen::Vector3d>& tcps_in_base);

    EyeHandCalibResult CalibrateEyeOnHand(
        const std::vector<Eigen::Vector3d>& points_in_camera,
        const std::vector<Eigen::Vector3d>& tcps_in_base);


private slots:
    /** @brief 选择标定图像目录并初始化有效位姿表。 */
    void on_loadImageBtn_clicked();

    /** @brief 选择机器人位姿 JSON。 */
    void on_loadPoseBtn_clicked();

    /** @brief 执行标定板或 TCP 点触标定。 */
    void on_execCalibBtn_clicked();

    /** @brief 保存标定结果 JSON。 */
    void on_saveCalibBtn_clicked();

    /** @brief 加载相机目标点云/点列。 */
    void on_loadCamTgtPointBtn_clicked();

    /** @brief 加载法兰标定采集 JSON。 */
    void on_loadCapFlangeBtn_clicked();

    /** @brief 加载 TCP 点触坐标数据。 */
    void on_loadTcpTouchBtn_clicked();
private:

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    Ui::HandEyeCalib* ui;

    pcl::visualization::PCLVisualizer::Ptr m_viewer;
};

#endif // TCPCALIB_H
