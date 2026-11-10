#ifndef SCENE_SEAM_EXTRACTOR_H
#define SCENE_SEAM_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

/**
 * @brief 整场点云初始焊缝提取参数。所有长度单位为毫米。
 *
 * 适用场景：3D 相机光轴垂直向下拍摄，组立工件底板平放，立板垂直于底板。
 * 点云里可以有地面或工作台，也可以只有放在工作台上的工件。
 * 整场点云已在同一坐标系。输入 Z 轴指向远离相机的方向，支撑面 Z 最大，
 * 立板顶面 Z 更小。计算和输出都留在这个坐标系里，不再做 Z 向翻转。
 * 平面法向指向相机（Z 减小方向）。高出某个平面的高度 = 该平面 Z - 点的 Z。
 *
 * 焊缝只由立板顶面的投影轮廓决定，不估计板厚、也不做直线投票：
 *   平角焊缝 = 顶面轮廓投到底板上的拟合直线段；
 *   立角焊缝 = 轮廓凹拐角处，从底板指向顶面（Z 减小）的垂线段。
 * 地面拟合容差、工件掩码闭运算、顶面邻域这些内部常数不对外暴露。
 * 换一批组立件时，通常只需要按板厚和筋高调整 ribMinHeight，按工件外形调整
 * minWorkpieceArea / minWorkpieceSize，按最短焊缝调整 minSeamLength。
 */
struct SceneSeamParams {
    // 预处理与分割
    float voxelSize = 2.0f;             ///< 体素下采样边长，<= 0 关闭
    float sceneResolution = 4.0f;       ///< 整场高度图栅格边长，用于分割工件
    float workpieceResolution = 2.0f;   ///< 单个工件高度图栅格边长，用于底板和顶面轮廓
    float groundHeight = std::numeric_limits<float>::quiet_NaN(); ///< 已标定地面或工作台高度（输入坐标系的 Z，支撑面 Z 最大）；非有限值时自动估计。标定后始终按该高度分割
    float groundThreshold = 6.0f;       ///< 比地面更靠近相机超过该值的栅格视为物体，应小于最薄底板厚度
    float minWorkpieceArea = 10000.0f;  ///< 工件最小投影面积 mm^2
    float minWorkpieceSize = 100.0f;    ///< 工件外接框短边最小长度

    // 顶面与焊缝
    float ribMinHeight = 30.0f;         ///< 比底板更靠近相机超过该值才可能是立板顶面，应大于底板厚度、小于最矮立板
    float minSeamLength = 30.0f;        ///< 平角焊缝最短长度，短于该值的轮廓边（板厚端面）丢弃
};

/// 顶面轮廓上的一条直线段，投影在底板平面上。
struct RibSegment {
    int id = -1;
    Eigen::Vector2f start = Eigen::Vector2f::Zero(); ///< 场景 XY 坐标
    Eigen::Vector2f end = Eigen::Vector2f::Zero();
    float thickness = 0.0f;   ///< 轮廓定义下不再估计板厚，保留字段为 0
    float height = 0.0f;      ///< 该段相邻立板顶面高出底板的高度
    float confidence = 0.0f;  ///< 0~1

    Eigen::Vector2f direction() const;
    float length() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class SeamType {
    FlatFillet,     ///< 立板与底板之间的平角焊缝
    VerticalFillet  ///< 两块立板凹拐角处的立角焊缝
};

/// 初始焊缝，用于引导后续激光或接触寻位。
struct InitialSeam {
    int workpieceId = -1;
    SeamType type = SeamType::FlatFillet;
    Eigen::Vector3f start = Eigen::Vector3f::Zero(); ///< 场景坐标。立焊缝从底板指向顶部（Z 减小）
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    Eigen::Vector3f approachSide = Eigen::Vector3f::Zero(); ///< 焊枪接近方向的单位向量，位于底板平面内，指向板外
    float ribHeight = 0.0f;   ///< 相邻立板高度，用于焊枪避让
    float confidence = 0.0f;  ///< 0~1
    int ribA = -1;            ///< 所属轮廓边
    int ribB = -1;            ///< 立焊缝的另一条轮廓边，平焊缝为 -1

    float length() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 一个组立工件。
struct Workpiece {
    int id = -1;
    Eigen::Vector4f basePlane = Eigen::Vector4f::Zero(); ///< 底板上表面 ax + by + cz + d = 0，法向指向相机
    float baseHeight = 0.0f;  ///< 底板上表面在工件中心处高出外部地面或工作台的值；没有外部支撑面时接近 0
    Eigen::Vector2f center = Eigen::Vector2f::Zero();
    Eigen::Vector2f minXY = Eigen::Vector2f::Zero();
    Eigen::Vector2f maxXY = Eigen::Vector2f::Zero();
    float yawRad = 0.0f;      ///< 最长轮廓边的方向角，范围 [0, pi)
    std::vector<RibSegment, Eigen::aligned_allocator<RibSegment>> ribs;
    std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>> seams; ///< 含低置信度焊缝
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;                              ///< 属于该工件的点

    Workpiece();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SceneSeamResult {
    bool success = false;
    std::string message;
    Eigen::Vector4f groundPlane = Eigen::Vector4f::Zero(); ///< 外部地面或工作台；没有外部支撑面时为整场最低大平面（通常是底板）
    std::vector<Workpiece, Eigen::aligned_allocator<Workpiece>> workpieces;
    std::vector<InitialSeam, Eigen::aligned_allocator<InitialSeam>> seams; ///< 置信度达标的全部焊缝
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;                              ///< 预处理后的整场点云

    SceneSeamResult();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 平面上 (x, y) 处的 z 值。
float PlaneZ(const Eigen::Vector4f& plane, float x, float y);

/**
 * @brief 从整场点云中提取每个组立工件的初始焊缝。
 *
 * 输入点云的 Z 轴指向远离相机的方向，地面、工作台或底板的 Z 最大。
 *
 * 1. 去无效点、体素下采样；
 * 2. 建整场俯视高度图（每格保留最靠近相机的 Z），估计最低的大平面；
 * 3. 该平面之上若有成片底板，则把它当作地面或工作台，高出它的栅格是工件；
 *    若没有，说明点云里没有外部支撑面，参考面就是底板，全部有数据的栅格都是工件。
 *    再做闭运算和连通域，得到单个工件；
 * 4. 每个工件内按高度直方图找底板平面；
 * 5. 高出底板且为局部最高的栅格构成立板顶面，取其投影轮廓（外轮廓和孔洞）；
 * 6. 轮廓上的直线段投到底板，得到平角焊缝；凹拐角得到立角焊缝。
 *
 * 全流程确定性，同一输入得到同一输出。失败时 success 为 false 并在 message 中说明。
 * 输出与输入点云处于同一坐标系。
 */
SceneSeamResult ExtractSceneSeams(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const SceneSeamParams& params = SceneSeamParams());

#endif // SCENE_SEAM_EXTRACTOR_H
