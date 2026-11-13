#pragma once

// Automatic plane leveling for a tube-sheet welding robot with a cross laser.
// Algorithm follows CN 112894209 A, with the kinematic errata corrected:
//   * the third unknown in the board intersection is the ray scale, Zw = 0;
//   * AX = XB uses A = Hg2^{-1} Hg1 and B = Hc2 Hc1^{-1} (OpenCV eye-in-hand);
//   * the printed forward Euler matrix does not match the printed inverse;
//     the inverse is the one implemented (phi about X, theta about Y, psi about Z);
//   * Steger's normal is the eigenvector of largest |eigenvalue|, and a bright
//     stripe has a negative second derivative along that normal.
//
// Frames: base b, gripper/tool g, camera c, calibration target t.
// X = ^g T_c maps a camera point into the gripper. Hg = ^b T_g. Hc = ^c T_t.

#include <opencv2/core.hpp>

#include <array>
#include <optional>
#include <vector>

namespace plane_level {

struct EulerZYX {
    double phi = 0;    // rad, about X. KUKA C.
    double theta = 0;  // rad, about Y. KUKA B.
    double psi = 0;    // rad, about Z. KUKA A.
};

struct Rigid3d {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t{0, 0, 0};

    cv::Vec3d apply(const cv::Vec3d& p) const;
    cv::Vec3d applyRotation(const cv::Vec3d& v) const;
    Rigid3d inverse() const;
    // (*this) * rhs applies rhs first.
    Rigid3d operator*(const Rigid3d& rhs) const;

    static Rigid3d fromRotationTranslation(const cv::Matx33d& rotation, const cv::Vec3d& translation);
    static Rigid3d fromKukaAbc(const cv::Vec3d& xyzMm, double aDeg, double bDeg, double cDeg);
    void toKukaAbc(cv::Vec3d& xyzMm, double& aDeg, double& bDeg, double& cDeg) const;
};

// R = Rz(psi) Ry(theta) Rx(phi). Inverse matches the patent:
//   phi   = atan2(r32, r33)
//   theta = atan2(-r31, hypot(r32, r33))
//   psi   = atan2(r21, r11)
cv::Matx33d eulerToMatrix(const EulerZYX& e);
EulerZYX matrixToEuler(const cv::Matx33d& R);
double wrapPi(double rad);

struct DistortCoeffs {
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
};

struct Intrinsics {
    double fx = 1, fy = 1, cx = 0, cy = 0, skew = 0;
    DistortCoeffs dist;
    cv::Matx33d K() const;
};

// Brown-Conrady on normalized camera coordinates. (x, y) ideal, r^2 = x^2 + y^2.
//   x' = x (1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x y + p2 (r^2 + 2 x^2)
//   y' = y (1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y^2) + 2 p2 x y
cv::Vec2d distortNormalized(const cv::Vec2d& xy, const DistortCoeffs& d);
cv::Vec2d undistortNormalized(const cv::Vec2d& distorted, const DistortCoeffs& d, int iterations = 12);

cv::Vec2d pixelToNormalizedDistorted(const Intrinsics& K, double u, double v);
cv::Vec3d pixelRay(const Intrinsics& K, double u, double v);

struct Plane3 {
    cv::Vec3d n{0, 0, 1};  // unit normal
    double d = 0;          // n · x + d = 0
    double signedDistance(const cv::Vec3d& p) const;
    // Smallest eigenvector of the point covariance. Exact minimizer of
    // sum (n · x_i + d)^2 over ||n|| = 1, with d = -n · mean.
    static Plane3 fit(const std::vector<cv::Vec3d>& points);
};

// Patent display writes the third unknown as Zw. On a Z=0 checkerboard the
// third unknown is the ray scale s, and the camera point is s (x, y, 1).
cv::Vec3d laserPointOnBoard(const Intrinsics& K, const Rigid3d& boardInCamera, double u, double v);

std::optional<cv::Vec3d> intersectRayPlane(const cv::Vec3d& rayThroughOrigin, const Plane3& plane);

struct MonoCalibration {
    Intrinsics intrinsics;
    std::vector<Rigid3d> boardInCamera;
    double rmsPx = 0;
};

// Zhang's method via cv::calibrateCamera. objectPoints are checkerboard
// corners in millimetres with Zw = 0.
MonoCalibration calibrateIntrinsics(
    const std::vector<std::vector<cv::Vec3d>>& objectPoints,
    const std::vector<std::vector<cv::Vec2d>>& imagePoints,
    cv::Size imageSize);

struct LaserView {
    Rigid3d boardInCamera;
    std::vector<cv::Point2d> arm0Px;
    std::vector<cv::Point2d> arm1Px;
};

struct LaserPlanes {
    Plane3 arm0;
    Plane3 arm1;
};

// Each stripe is intersected with that view's board, expressed in the camera
// frame, then one plane is fit per arm across all views.
LaserPlanes fitLaserPlanes(const Intrinsics& K, const std::vector<LaserView>& views);

struct HandEyeSample {
    Rigid3d gripperInBase;  // Hg
    Rigid3d targetInCamera; // Hc
};

struct HandEyeMotions {
    std::vector<Rigid3d> A;  // Hg[i+1]^{-1} * Hg[i]
    std::vector<Rigid3d> B;  // Hc[i+1] * Hc[i]^{-1}
};

// Consecutive motions that satisfy A X = X B for X = ^g T_c.
HandEyeMotions relativeMotions(const std::vector<HandEyeSample>& samples);

// Park & Martin: rotation vectors obey alpha = Rx beta, solved by Kabsch,
// then (Ra - I) tx = Rx tb - ta by linear least squares.
Rigid3d solveHandEye(const std::vector<HandEyeSample>& samples);

// cv::calibrateHandEye, method CALIB_HAND_EYE_PARK. Same X.
Rigid3d solveHandEyeOpenCv(const std::vector<HandEyeSample>& samples);

cv::Vec3d cameraToBase(const cv::Vec3d& pCamera, const Rigid3d& gripperInBase, const Rigid3d& cameraInGripper);

struct StegerParams {
    double stripeWidthPx = 5;  // omega in the patent
    double sigma = -1;         // < 0 selects omega / (2 sqrt(3)), inside sigma < omega/sqrt(3)
    double strength = 1;       // accept bright ridges with n^T H n < -strength
    double minIntensity = 0;
};

struct StripePoint {
    double x = 0;
    double y = 0;
    double nx = 0;
    double ny = 0;
    double response = 0;  // n^T H n, negative on a bright ridge
};

// Hessian normal, then a first-order zero of the directional derivative.
// Accept the pixel only when the zero lies inside it.
std::vector<StripePoint> extractStripeCenters(const cv::Mat& gray, const StegerParams& params);

struct CrossSplit {
    std::vector<StripePoint> arm0;
    std::vector<StripePoint> arm1;
};
// Two undirected tangent clusters. A cross has two of them.
CrossSplit splitCross(const std::vector<StripePoint>& points);

struct AlignResult {
    Rigid3d sourceToTarget;
    double rms = 0;
};

// Minimizes sum || target_i - (R source_i + t) ||^2.
// T = mean(target) - R mean(source). R maximizes trace(R H), H = sum p' q'^T,
// via the SVD H = U S V^T, R = V U^T, with a reflection flip when det R < 0.
AlignResult absoluteOrientation(const std::vector<cv::Vec3d>& source, const std::vector<cv::Vec3d>& target);

std::vector<cv::Vec3d> resampleByArcLength(const std::vector<cv::Vec3d>& points, int samples);

// Resampled arms, eight discrete correspondences (arm swap and direction flips).
AlignResult alignCrosses(
    const std::vector<cv::Vec3d>& current0,
    const std::vector<cv::Vec3d>& current1,
    const std::vector<cv::Vec3d>& reference0,
    const std::vector<cv::Vec3d>& reference1,
    int samplesPerArm);

// Optional PCL path. Same estimator as absoluteOrientation.
#ifdef PLANE_LEVEL_HAS_PCL
AlignResult absoluteOrientationPcl(const std::vector<cv::Vec3d>& source, const std::vector<cv::Vec3d>& target);
#endif

struct Plate {
    cv::Vec3d n{0, 0, 1};
    double d = 0;  // n · x + d = 0, in the robot base
};

struct LaserModel {
    Plane3 arm0InCamera;
    Plane3 arm1InCamera;
};

// Intersection of each camera-fixed laser plane with the plate.
std::array<std::vector<cv::Vec3d>, 2> sampleLaserCross(
    const Plate& plateInBase,
    const Rigid3d& gripperInBase,
    const Rigid3d& cameraInGripper,
    const LaserModel& laser,
    double halfLengthMm,
    int samplesPerArm);

struct LevelObservation {
    std::vector<cv::Vec3d> arm0;
    std::vector<cv::Vec3d> arm1;
    Rigid3d gripperInBase;
};

struct LevelCommand {
    cv::Matx33d toolCorrection = cv::Matx33d::eye();  // right-multiplies the gripper pose
    EulerZYX euler;                                   // Euler of toolCorrection
    double kukaADeg = 0;
    double kukaBDeg = 0;
    double kukaCDeg = 0;
    double tiltDeg = 0;  // angle between the two plate normals
    double kabschTiltDeg = 0;
    double rmsMm = 0;
    Rigid3d nextGripperInBase;
    bool withinTolerance = false;
};

struct LevelConfig {
    double toleranceDeg = 0.1;
    double gain = 1;  // scales the correction rotation vector. 1 is the full step.
    int samplesPerArm = 40;
};

// Tool-frame correction that takes the measured plate normal onto the
// reference normal. Kabsch is reported alongside; the commanded rotation is
// the normal alignment, which stays exact when foreshortening makes the
// cross a non-rigid image of the reference cross.
LevelCommand computeLevelCommand(
    const LevelObservation& reference,
    const LevelObservation& current,
    const Rigid3d& cameraInGripper,
    const LevelConfig& config);

cv::Vec3d crossNormal(const std::vector<cv::Vec3d>& arm0, const std::vector<cv::Vec3d>& arm1);
cv::Matx33d rotationAligning(const cv::Vec3d& from, const cv::Vec3d& to);
cv::Matx33d scaleRotation(const cv::Matx33d& R, double gain);

}  // namespace plane_level
