#include "plane_level/plane_level.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <stdexcept>

namespace plane_level {
namespace {

cv::Vec3d meanOf(const std::vector<cv::Vec3d>& points) {
    cv::Vec3d c(0, 0, 0);
    for (const cv::Vec3d& p : points) c += p;
    return c * (1.0 / static_cast<double>(points.size()));
}

}

cv::Matx33d Intrinsics::K() const {
    return cv::Matx33d(fx, skew, cx, 0, fy, cy, 0, 0, 1);
}

cv::Vec2d distortNormalized(const cv::Vec2d& xy, const DistortCoeffs& d) {
    const double x = xy[0];
    const double y = xy[1];
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;
    const double xTangential = 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x);
    const double yTangential = d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y;
    return cv::Vec2d(x * radial + xTangential, y * radial + yTangential);
}

cv::Vec2d undistortNormalized(const cv::Vec2d& distorted, const DistortCoeffs& d, int iterations) {
    // Fixed-point form of distort(x) = xd. Each step freezes the radial
    // factor and the tangential shift at the current estimate, which is the
    // iteration OpenCV uses inside undistortPoints.
    double x = distorted[0];
    double y = distorted[1];
    for (int i = 0; i < iterations; ++i) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + d.k1 * r2 + d.k2 * r4 + d.k3 * r6;
        if (std::abs(radial) < 1e-15) break;
        const double dx = 2.0 * d.p1 * x * y + d.p2 * (r2 + 2.0 * x * x);
        const double dy = d.p1 * (r2 + 2.0 * y * y) + 2.0 * d.p2 * x * y;
        x = (distorted[0] - dx) / radial;
        y = (distorted[1] - dy) / radial;
    }
    return cv::Vec2d(x, y);
}

cv::Vec2d pixelToNormalizedDistorted(const Intrinsics& K, double u, double v) {
    // Inverse of [u v 1]^T = K [x y 1]^T, before undistortion.
    const double y = (v - K.cy) / K.fy;
    const double x = (u - K.cx - K.skew * y) / K.fx;
    return cv::Vec2d(x, y);
}

cv::Vec3d pixelRay(const Intrinsics& K, double u, double v) {
    const cv::Vec2d distorted = pixelToNormalizedDistorted(K, u, v);
    const cv::Vec2d ideal = undistortNormalized(distorted, K.dist);
    return cv::Vec3d(ideal[0], ideal[1], 1);
}

double Plane3::signedDistance(const cv::Vec3d& p) const {
    return n.dot(p) + d;
}

Plane3 Plane3::fit(const std::vector<cv::Vec3d>& points) {
    if (points.size() < 3) throw std::invalid_argument("a plane needs at least 3 points");
    const cv::Vec3d c = meanOf(points);
    cv::Matx33d cov = cv::Matx33d::zeros();
    for (const cv::Vec3d& p : points) {
        const cv::Vec3d e = p - c;
        cov += e * e.t();
    }
    cv::SVD svd(cv::Mat(cov), cv::SVD::FULL_UV);
    const cv::Mat vt = svd.vt;
    cv::Vec3d n(vt.at<double>(2, 0), vt.at<double>(2, 1), vt.at<double>(2, 2));
    const double nn = cv::norm(n);
    if (nn < 1e-15) throw std::runtime_error("plane fit degenerated");
    n *= 1.0 / nn;
    // Point the normal toward the camera origin when the cloud is in the camera frame.
    if (n.dot(c) > 0) n = -n;
    Plane3 plane;
    plane.n = n;
    plane.d = -n.dot(c);
    return plane;
}

cv::Vec3d laserPointOnBoard(const Intrinsics& K, const Rigid3d& boardInCamera, double u, double v) {
    const cv::Vec3d ray = pixelRay(K, u, v);
    const double x = ray[0];
    const double y = ray[1];
    const cv::Matx33d& R = boardInCamera.R;
    const cv::Vec3d& T = boardInCamera.t;
    // [ r11 r12 -x ] [Xw]   [-tx]
    // [ r21 r22 -y ] [Yw] = [-ty]
    // [ r31 r32 -1 ] [s ]   [-tz]
    const cv::Matx33d A(
        R(0, 0), R(0, 1), -x,
        R(1, 0), R(1, 1), -y,
        R(2, 0), R(2, 1), -1);
    cv::Vec3d solved;
    if (!cv::solve(A, -T, solved, cv::DECOMP_LU)) {
        throw std::runtime_error("board intersection is singular");
    }
    const cv::Vec3d onBoard(solved[0], solved[1], 0);
    return boardInCamera.apply(onBoard);
}

std::optional<cv::Vec3d> intersectRayPlane(const cv::Vec3d& rayThroughOrigin, const Plane3& plane) {
    const double denom = plane.n.dot(rayThroughOrigin);
    if (std::abs(denom) < 1e-12) return std::nullopt;
    const double s = -plane.d / denom;
    if (s <= 0) return std::nullopt;
    return rayThroughOrigin * s;
}

MonoCalibration calibrateIntrinsics(
    const std::vector<std::vector<cv::Vec3d>>& objectPoints,
    const std::vector<std::vector<cv::Vec2d>>& imagePoints,
    cv::Size imageSize) {
    if (objectPoints.size() != imagePoints.size() || objectPoints.size() < 3) {
        throw std::invalid_argument("calibrateIntrinsics needs at least 3 paired views");
    }
    std::vector<std::vector<cv::Point3f>> objects;
    std::vector<std::vector<cv::Point2f>> images;
    objects.reserve(objectPoints.size());
    images.reserve(imagePoints.size());
    for (size_t v = 0; v < objectPoints.size(); ++v) {
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        obj.reserve(objectPoints[v].size());
        img.reserve(imagePoints[v].size());
        for (const cv::Vec3d& p : objectPoints[v]) obj.emplace_back(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
        for (const cv::Vec2d& p : imagePoints[v]) img.emplace_back(static_cast<float>(p[0]), static_cast<float>(p[1]));
        objects.push_back(std::move(obj));
        images.push_back(std::move(img));
    }
    cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
    const double rms = cv::calibrateCamera(objects, images, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    MonoCalibration out;
    out.rmsPx = rms;
    out.intrinsics.fx = cameraMatrix.at<double>(0, 0);
    out.intrinsics.skew = cameraMatrix.at<double>(0, 1);
    out.intrinsics.cx = cameraMatrix.at<double>(0, 2);
    out.intrinsics.fy = cameraMatrix.at<double>(1, 1);
    out.intrinsics.cy = cameraMatrix.at<double>(1, 2);
    if (distCoeffs.total() >= 5) {
        out.intrinsics.dist.k1 = distCoeffs.at<double>(0);
        out.intrinsics.dist.k2 = distCoeffs.at<double>(1);
        out.intrinsics.dist.p1 = distCoeffs.at<double>(2);
        out.intrinsics.dist.p2 = distCoeffs.at<double>(3);
        out.intrinsics.dist.k3 = distCoeffs.at<double>(4);
    }
    out.boardInCamera.resize(static_cast<size_t>(rvecs.rows));
    for (int i = 0; i < rvecs.rows; ++i) {
        cv::Matx33d R;
        cv::Rodrigues(rvecs.row(i), R);
        cv::Vec3d t(
            tvecs.at<double>(i, 0),
            tvecs.at<double>(i, 1),
            tvecs.at<double>(i, 2));
        out.boardInCamera[static_cast<size_t>(i)] = Rigid3d::fromRotationTranslation(R, t);
    }
    return out;
}

LaserPlanes fitLaserPlanes(const Intrinsics& K, const std::vector<LaserView>& views) {
    std::vector<cv::Vec3d> arm0;
    std::vector<cv::Vec3d> arm1;
    for (const LaserView& view : views) {
        for (const cv::Point2d& px : view.arm0Px) {
            arm0.push_back(laserPointOnBoard(K, view.boardInCamera, px.x, px.y));
        }
        for (const cv::Point2d& px : view.arm1Px) {
            arm1.push_back(laserPointOnBoard(K, view.boardInCamera, px.x, px.y));
        }
    }
    LaserPlanes planes;
    planes.arm0 = Plane3::fit(arm0);
    planes.arm1 = Plane3::fit(arm1);
    return planes;
}

}  // namespace plane_level
