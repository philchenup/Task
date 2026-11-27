#include "retreatPlanePose.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace {

using PointT = pcl::PointXYZRGBNormal;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxNormalAngleDeg = 10.0;
// 恰好 10° 时给浮点留一点余量，真实超过 10.05° 的平面仍然拒绝。
constexpr double kAngleSlackDeg = 0.05;
constexpr double kRansacAngleMarginDeg = 0.5;
constexpr double kRetreatMm = 300.0;
constexpr double kRansacDistanceMm = 2.0;
constexpr double kFineDistanceMinMm = 0.2;
constexpr double kFineDistanceMaxMm = 1.0;
constexpr int kRefineIterations = 3;
constexpr int kRansacIterations = 1000;
constexpr std::size_t kMinPlanePoints = 30;

double degreesToRadians(double degrees)
{
    return degrees * kPi / 180.0;
}

double medianOf(std::vector<double> values)
{
    const std::size_t count = values.size();
    if (count == 0) {
        return 0.0;
    }
    const auto mid = static_cast<std::ptrdiff_t>(count / 2);
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    if (count % 2 == 1) {
        return values[static_cast<std::size_t>(mid)];
    }
    const double upper = values[static_cast<std::size_t>(mid)];
    std::nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid);
    return 0.5 * (upper + values[static_cast<std::size_t>(mid - 1)]);
}

bool normalWithinCameraZ(const Eigen::Vector3d& normal)
{
    const double length = normal.norm();
    if (!(length > 1e-9) || !std::isfinite(length)) {
        return false;
    }
    const double aligned = std::min(1.0, std::abs(normal.z()) / length);
    const double angle_deg = std::acos(aligned) * 180.0 / kPi;
    return angle_deg <= kMaxNormalAngleDeg + kAngleSlackDeg;
}

bool normalsUsable(const pcl::PointCloud<PointT>& cloud)
{
    if (cloud.empty()) {
        return false;
    }
    const std::size_t step = std::max<std::size_t>(1, cloud.size() / 64);
    int valid = 0;
    int checked = 0;
    for (std::size_t i = 0; i < cloud.size(); i += step) {
        const auto& point = cloud.points[i];
        ++checked;
        if (!std::isfinite(point.normal_x) || !std::isfinite(point.normal_y) || !std::isfinite(point.normal_z)) {
            continue;
        }
        const float norm_sq = point.normal_x * point.normal_x
            + point.normal_y * point.normal_y
            + point.normal_z * point.normal_z;
        if (norm_sq > 0.25f && norm_sq < 4.0f) {
            ++valid;
        }
    }
    return valid * 2 >= checked;
}

bool estimateNormals(pcl::PointCloud<PointT>& cloud)
{
    if (cloud.size() < 6) {
        return false;
    }
    const int neighbor_count = static_cast<int>(std::min<std::size_t>(30, cloud.size() - 1));
    if (neighbor_count < 5) {
        return false;
    }

    // 别名指针只在本次计算期间使用，不拷贝点云，也不改调用方的寿命。
    pcl::PointCloud<PointT>::ConstPtr view(&cloud, [](const pcl::PointCloud<PointT>*) {});
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::NormalEstimation<PointT, pcl::Normal> estimator;
    estimator.setInputCloud(view);
    estimator.setSearchMethod(tree);
    estimator.setKSearch(neighbor_count);
    estimator.setViewPoint(0.0f, 0.0f, 0.0f);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimator.compute(*normals);
    if (normals->size() < cloud.size()) {
        return false;
    }
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        cloud.points[i].normal_x = normals->points[i].normal_x;
        cloud.points[i].normal_y = normals->points[i].normal_y;
        cloud.points[i].normal_z = normals->points[i].normal_z;
    }
    return true;
}

pcl::PointCloud<PointT>::Ptr finiteCloud(const pcl::PointCloud<PointT>& cloud)
{
    auto work = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    work->reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        work->push_back(point);
    }
    work->width = static_cast<std::uint32_t>(work->size());
    work->height = 1;
    work->is_dense = true;
    return work;
}

pcl::PointCloud<PointT>::Ptr facingPoints(const pcl::PointCloud<PointT>& cloud)
{
    auto facing = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    facing->reserve(cloud.size());
    for (const auto& point : cloud.points) {
        const Eigen::Vector3d normal(point.normal_x, point.normal_y, point.normal_z);
        // 点法向可以朝向或背向相机，夹角按与 Z 轴的锐角计算。
        if (!normalWithinCameraZ(normal)) {
            continue;
        }
        facing->push_back(point);
    }
    facing->width = static_cast<std::uint32_t>(facing->size());
    facing->height = 1;
    facing->is_dense = true;
    return facing;
}

bool segmentFacingPlane(const pcl::PointCloud<PointT>::Ptr& cloud, pcl::Indices& inliers)
{
    pcl::SACSegmentation<PointT> segmentation;
    // 粗分割只收内点。PCL 自带的系数优化可能把法向推出夹角阈值，随后把内点清空。
    segmentation.setOptimizeCoefficients(false);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(degreesToRadians(kMaxNormalAngleDeg + kRansacAngleMarginDeg));
    segmentation.setDistanceThreshold(kRansacDistanceMm);
    segmentation.setMaxIterations(kRansacIterations);
    segmentation.setProbability(0.99);
    segmentation.setInputCloud(cloud);

    pcl::PointIndices inlier_indices;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inlier_indices, coefficients);
    if (inlier_indices.indices.size() < kMinPlanePoints || coefficients.values.size() < 4) {
        return false;
    }
    inliers.swap(inlier_indices.indices);
    return true;
}

struct PlaneModel {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

Eigen::Vector3d pointAt(const pcl::PointCloud<PointT>& cloud, pcl::index_t index)
{
    const auto& point = cloud.points[static_cast<std::size_t>(index)];
    return Eigen::Vector3d(point.x, point.y, point.z);
}

bool fitPlane(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& indices, PlaneModel& model)
{
    if (indices.size() < 3) {
        return false;
    }
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Eigen::Vector4d centroid = Eigen::Vector4d::Zero();
    const unsigned int used = pcl::computeMeanAndCovarianceMatrix(cloud, indices, covariance, centroid);
    if (used < 3) {
        return false;
    }
    covariance = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    if (!(eigenvalues[1] > 1e-8) || eigenvalues[0] > eigenvalues[1] * 0.25) {
        return false;
    }
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0.0) {
        normal = -normal;
    }
    if (!normalWithinCameraZ(normal)) {
        return false;
    }
    model.centroid = centroid.head<3>();
    model.normal = normal.normalized();
    return true;
}

double fineDistanceThreshold(const pcl::PointCloud<PointT>& cloud,
                             const pcl::Indices& indices,
                             const PlaneModel& model)
{
    std::vector<double> distances;
    distances.reserve(indices.size());
    for (const pcl::index_t index : indices) {
        distances.push_back(std::abs(model.normal.dot(pointAt(cloud, index) - model.centroid)));
    }
    const double median = medianOf(distances);
    std::vector<double> deviation;
    deviation.reserve(distances.size());
    for (const double distance : distances) {
        deviation.push_back(std::abs(distance - median));
    }
    double threshold = 2.5 * 1.4826 * medianOf(deviation);
    if (threshold < kFineDistanceMinMm) {
        threshold = kFineDistanceMinMm;
    }
    if (threshold > kFineDistanceMaxMm) {
        threshold = kFineDistanceMaxMm;
    }
    return threshold;
}

bool refinePlane(const pcl::PointCloud<PointT>& cloud, pcl::Indices indices, PlaneModel& model)
{
    if (indices.size() < kMinPlanePoints || !fitPlane(cloud, indices, model)) {
        return false;
    }
    for (int iteration = 0; iteration < kRefineIterations; ++iteration) {
        const double threshold = fineDistanceThreshold(cloud, indices, model);
        pcl::Indices kept;
        kept.reserve(indices.size());
        for (const pcl::index_t index : indices) {
            const double distance = std::abs(model.normal.dot(pointAt(cloud, index) - model.centroid));
            if (distance <= threshold) {
                kept.push_back(index);
            }
        }
        if (kept.size() < kMinPlanePoints || kept.size() == indices.size()) {
            break;
        }
        PlaneModel refined;
        if (!fitPlane(cloud, kept, refined)) {
            break;
        }
        indices.swap(kept);
        model = refined;
    }
    return normalWithinCameraZ(model.normal);
}

// 把相机 X 投到平面上作为位姿 X，Y 由 Z×X 得到，使两轴同时朝向相机坐标轴。
Eigen::Matrix3d cameraAlignedRotation(const Eigen::Vector3d& plane_normal)
{
    const Eigen::Vector3d camera_x(1.0, 0.0, 0.0);
    const Eigen::Vector3d camera_y(0.0, 1.0, 0.0);
    Eigen::Vector3d z_axis = plane_normal.normalized();
    if (z_axis.z() < 0.0) {
        z_axis = -z_axis;
    }

    Eigen::Vector3d x_axis = camera_x - z_axis * z_axis.dot(camera_x);
    if (x_axis.squaredNorm() < 1e-12) {
        x_axis = camera_y - z_axis * z_axis.dot(camera_y);
    }
    x_axis.normalize();
    if (x_axis.dot(camera_x) < 0.0) {
        x_axis = -x_axis;
    }
    const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    return rotation;
}

}  // namespace

bool computeRetreatPlanePose(const pcl::PointCloud<PointT>& cloud, Eigen::Affine3d& pose)
{
    if (cloud.size() < kMinPlanePoints) {
        return false;
    }

    pcl::PointCloud<PointT>::Ptr work = finiteCloud(cloud);
    if (work->size() < kMinPlanePoints) {
        return false;
    }
    if (!normalsUsable(*work) && !estimateNormals(*work)) {
        return false;
    }

    pcl::PointCloud<PointT>::Ptr facing = facingPoints(*work);
    if (facing->size() < kMinPlanePoints) {
        return false;
    }

    pcl::Indices inliers;
    if (!segmentFacingPlane(facing, inliers)) {
        return false;
    }

    PlaneModel model;
    if (!refinePlane(*facing, inliers, model)) {
        return false;
    }

    pose = Eigen::Affine3d::Identity();
    pose.linear() = cameraAlignedRotation(model.normal);
    // 沿位姿 Z 轴反向离开平面 300mm。Z 与相机 +Z 同向，因此退向相机一侧。
    pose.translation() = model.centroid - kRetreatMm * pose.linear().col(2);
    return true;
}
