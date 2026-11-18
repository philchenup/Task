#include "plane_level/plane_level.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace plane_level {
namespace {

constexpr double kPi = 3.14159265358979323846;

cv::Vec3d lineOrigin(const Plane3& laser, const Plane3& plate, cv::Vec3d& direction) {
    direction = laser.n.cross(plate.n);
    const double dn = cv::norm(direction);
    if (dn < 1e-9) throw std::runtime_error("laser plane is parallel to the plate");
    direction *= 1.0 / dn;
    const double n11 = laser.n.dot(laser.n);
    const double n12 = laser.n.dot(plate.n);
    const double n22 = plate.n.dot(plate.n);
    const double det = n11 * n22 - n12 * n12;
    if (std::abs(det) < 1e-12) throw std::runtime_error("laser and plate normals are parallel");
    const double alpha = (-laser.d * n22 - (-plate.d) * n12) / det;
    const double beta = (n11 * (-plate.d) - n12 * (-laser.d)) / det;
    return alpha * laser.n + beta * plate.n;
}

std::vector<cv::Vec3d> sampleLine(const cv::Vec3d& origin, const cv::Vec3d& direction, double halfLength, int samples) {
    std::vector<cv::Vec3d> out;
    out.reserve(static_cast<size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const double u = samples == 1 ? 0 : -halfLength + 2.0 * halfLength * static_cast<double>(i) / static_cast<double>(samples - 1);
        out.push_back(origin + direction * u);
    }
    return out;
}

double angleDeg(const cv::Vec3d& a, const cv::Vec3d& b) {
    const double c = std::clamp(a.dot(b) / (cv::norm(a) * cv::norm(b)), -1.0, 1.0);
    return std::acos(c) * 180.0 / kPi;
}

}

std::array<std::vector<cv::Vec3d>, 2> sampleLaserCross(
    const Plate& plateInBase,
    const Rigid3d& gripperInBase,
    const Rigid3d& cameraInGripper,
    const LaserModel& laser,
    double halfLengthMm,
    int samplesPerArm) {
    if (samplesPerArm < 2) throw std::invalid_argument("cross sampling needs at least 2 points per arm");
    const Rigid3d baseFromCamera = gripperInBase * cameraInGripper;
    Plane3 plateInCamera;
    plateInCamera.n = baseFromCamera.R.t() * plateInBase.n;
    const double nn = cv::norm(plateInCamera.n);
    plateInCamera.n *= 1.0 / nn;
    plateInCamera.d = (plateInBase.n.dot(baseFromCamera.t) + plateInBase.d) / nn;
    if (plateInCamera.n.dot(cv::Vec3d(0, 0, 1)) > 0) {
        // Keep the stored samples consistent; the fit later reorients the normal.
    }
    cv::Vec3d dir0, dir1;
    const cv::Vec3d o0 = lineOrigin(laser.arm0InCamera, plateInCamera, dir0);
    const cv::Vec3d o1 = lineOrigin(laser.arm1InCamera, plateInCamera, dir1);
    return {sampleLine(o0, dir0, halfLengthMm, samplesPerArm), sampleLine(o1, dir1, halfLengthMm, samplesPerArm)};
}

cv::Vec3d crossNormal(const std::vector<cv::Vec3d>& arm0, const std::vector<cv::Vec3d>& arm1) {
    std::vector<cv::Vec3d> all = arm0;
    all.insert(all.end(), arm1.begin(), arm1.end());
    return Plane3::fit(all).n;
}

LevelCommand computeLevelCommand(
    const LevelObservation& reference,
    const LevelObservation& current,
    const Rigid3d& cameraInGripper,
    const LevelConfig& config) {
    if (config.gain <= 0 || config.gain > 1) throw std::invalid_argument("level gain must lie in (0, 1]");
    const cv::Vec3d nRef = crossNormal(reference.arm0, reference.arm1);
    cv::Vec3d nCur = crossNormal(current.arm0, current.arm1);
    if (nCur.dot(nRef) < 0) nCur = -nCur;

    // Rc takes the current camera normal onto the reference camera normal.
    const cv::Matx33d Rc = rotationAligning(nCur, nRef);
    const cv::Matx33d Rx = cameraInGripper.R;
    cv::Matx33d Rcorr = Rx * Rc.t() * Rx.t();
    Rcorr = scaleRotation(Rcorr, config.gain);

    const AlignResult kabsch = alignCrosses(
        current.arm0, current.arm1, reference.arm0, reference.arm1, config.samplesPerArm);
    const cv::Vec3d nKabsch = kabsch.sourceToTarget.applyRotation(nCur);
    const double kabschTilt = angleDeg(nKabsch, nRef);

    LevelCommand command;
    command.toolCorrection = Rcorr;
    command.euler = matrixToEuler(Rcorr);
    command.kukaADeg = command.euler.psi * 180.0 / kPi;
    command.kukaBDeg = command.euler.theta * 180.0 / kPi;
    command.kukaCDeg = command.euler.phi * 180.0 / kPi;
    command.tiltDeg = angleDeg(nCur, nRef);
    command.kabschTiltDeg = kabschTilt;
    command.rmsMm = kabsch.rms;
    command.withinTolerance = command.tiltDeg <= config.toleranceDeg;
    Rigid3d correction;
    correction.R = Rcorr;
    command.nextGripperInBase = current.gripperInBase * correction;
    return command;
}

}  // namespace plane_level
