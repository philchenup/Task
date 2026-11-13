#include "plane_level/plane_level.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <cmath>

namespace plane_level {
namespace {

pcl::PointCloud<pcl::PointXYZ> toCloud(const std::vector<cv::Vec3d>& points) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(points.size());
    for (const cv::Vec3d& p : points) {
        cloud.push_back(pcl::PointXYZ(
            static_cast<float>(p[0]),
            static_cast<float>(p[1]),
            static_cast<float>(p[2])));
    }
    return cloud;
}

}

AlignResult absoluteOrientationPcl(const std::vector<cv::Vec3d>& source, const std::vector<cv::Vec3d>& target) {
    // pcl::registration::TransformationEstimationSVD is the same Kabsch
    // estimator as absoluteOrientation: centroids, H = sum p' q'^T, SVD.
    // Correspondences are index-wise, which is what the resampled cross uses.
    const pcl::PointCloud<pcl::PointXYZ> src = toCloud(source);
    const pcl::PointCloud<pcl::PointXYZ> dst = toCloud(target);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    estimator.estimateRigidTransformation(src, dst, transform);

    cv::Matx33d R;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R(r, c) = transform(r, c);
    const cv::Vec3d t(transform(0, 3), transform(1, 3), transform(2, 3));
    AlignResult result;
    result.sourceToTarget = Rigid3d::fromRotationTranslation(R, t);
    double acc = 0;
    for (size_t i = 0; i < source.size(); ++i) {
        const cv::Vec3d e = target[i] - result.sourceToTarget.apply(source[i]);
        acc += e.dot(e);
    }
    result.rms = std::sqrt(acc / static_cast<double>(source.size()));
    return result;
}

}  // namespace plane_level
