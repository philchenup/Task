#include "plane_level/plane_level.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace plane_level {
namespace {

constexpr double kPi = 3.14159265358979323846;

}

double wrapPi(double rad) {
    while (rad > kPi) rad -= 2 * kPi;
    while (rad < -kPi) rad += 2 * kPi;
    return rad;
}

cv::Vec3d Rigid3d::apply(const cv::Vec3d& p) const {
    return R * p + t;
}

cv::Vec3d Rigid3d::applyRotation(const cv::Vec3d& v) const {
    return R * v;
}

Rigid3d Rigid3d::inverse() const {
    Rigid3d out;
    out.R = R.t();
    out.t = -(out.R * t);
    return out;
}

Rigid3d Rigid3d::operator*(const Rigid3d& rhs) const {
    Rigid3d out;
    out.R = R * rhs.R;
    out.t = R * rhs.t + t;
    return out;
}

Rigid3d Rigid3d::fromRotationTranslation(const cv::Matx33d& rotation, const cv::Vec3d& translation) {
    Rigid3d out;
    out.R = rotation;
    out.t = translation;
    return out;
}

cv::Matx33d eulerToMatrix(const EulerZYX& e) {
    const double cx = std::cos(e.phi);
    const double sx = std::sin(e.phi);
    const double cy = std::cos(e.theta);
    const double sy = std::sin(e.theta);
    const double cz = std::cos(e.psi);
    const double sz = std::sin(e.psi);
    // Rows of Rz(psi) Ry(theta) Rx(phi).
    return cv::Matx33d(
        cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
        -sy, cy * sx, cy * cx);
}

EulerZYX matrixToEuler(const cv::Matx33d& R) {
    EulerZYX e;
    const double r31 = R(2, 0);
    e.theta = std::atan2(-r31, std::hypot(R(2, 1), R(2, 2)));
    if (std::abs(r31) < 0.999999) {
        e.phi = std::atan2(R(2, 1), R(2, 2));
        e.psi = std::atan2(R(1, 0), R(0, 0));
    } else {
        // Gimbal lock: yaw is set to 0 and roll absorbs the remaining rotation.
        e.psi = 0;
        e.phi = std::atan2(-R(0, 1), R(1, 1));
    }
    return e;
}

Rigid3d Rigid3d::fromKukaAbc(const cv::Vec3d& xyzMm, double aDeg, double bDeg, double cDeg) {
    EulerZYX e;
    e.psi = aDeg * kPi / 180.0;
    e.theta = bDeg * kPi / 180.0;
    e.phi = cDeg * kPi / 180.0;
    return fromRotationTranslation(eulerToMatrix(e), xyzMm);
}

void Rigid3d::toKukaAbc(cv::Vec3d& xyzMm, double& aDeg, double& bDeg, double& cDeg) const {
    xyzMm = t;
    const EulerZYX e = matrixToEuler(R);
    aDeg = e.psi * 180.0 / kPi;
    bDeg = e.theta * 180.0 / kPi;
    cDeg = e.phi * 180.0 / kPi;
}

cv::Matx33d scaleRotation(const cv::Matx33d& R, double gain) {
    if (gain == 1.0) return R;
    cv::Vec3d w;
    cv::Rodrigues(R, w);
    cv::Matx33d out;
    cv::Rodrigues(w * gain, out);
    return out;
}

cv::Matx33d rotationAligning(const cv::Vec3d& from, const cv::Vec3d& to) {
    const double nf = cv::norm(from);
    const double nt = cv::norm(to);
    if (nf < 1e-15 || nt < 1e-15) return cv::Matx33d::eye();
    const cv::Vec3d a = from * (1.0 / nf);
    const cv::Vec3d b = to * (1.0 / nt);
    const cv::Vec3d v = a.cross(b);
    const double c = a.dot(b);
    const double s = cv::norm(v);
    if (s < 1e-12) {
        if (c > 0) return cv::Matx33d::eye();
        cv::Vec3d axis = std::abs(a[0]) < 0.9 ? cv::Vec3d(1, 0, 0).cross(a) : cv::Vec3d(0, 1, 0).cross(a);
        axis *= 1.0 / cv::norm(axis);
        cv::Matx33d R;
        cv::Rodrigues(axis * kPi, R);
        return R;
    }
    const cv::Matx33d K(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
    return cv::Matx33d::eye() + K + K * K * ((1.0 - c) / (s * s));
}

}  // namespace plane_level
