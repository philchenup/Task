#include "plane_level/plane_level.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

plane_level::LaserModel crossLaser() {
    plane_level::LaserModel laser;
    laser.arm0InCamera = {cv::Vec3d(0, 1, 0), 0};
    laser.arm1InCamera = {cv::Vec3d(1, 0, 0), 0};
    return laser;
}

plane_level::LevelObservation observe(
    const plane_level::Plate& plate,
    const plane_level::Rigid3d& gripper,
    const plane_level::Rigid3d& cameraInGripper,
    const plane_level::LaserModel& laser) {
    auto arms = plane_level::sampleLaserCross(plate, gripper, cameraInGripper, laser, 40, 25);
    plane_level::LevelObservation obs;
    obs.arm0 = std::move(arms[0]);
    obs.arm1 = std::move(arms[1]);
    obs.gripperInBase = gripper;
    return obs;
}

}

int main() {
    using namespace plane_level;
    const LaserModel laser = crossLaser();
    // Plate Z = 500 mm, normal pointing back toward a camera that looks along +Z.
    const Plate plate{cv::Vec3d(0, 0, -1), 500};

    EulerZYX mount;
    mount.phi = 0.04;
    mount.theta = -0.03;
    mount.psi = 0.02;
    const Rigid3d cameraInGripper = Rigid3d::fromRotationTranslation(eulerToMatrix(mount), cv::Vec3d(0, 35, 80));

    EulerZYX taught;
    taught.theta = 0;
    const Rigid3d referenceGripper = Rigid3d::fromRotationTranslation(eulerToMatrix(taught), cv::Vec3d(0, 0, 0));
    const LevelObservation reference = observe(plate, referenceGripper, cameraInGripper, laser);

    EulerZYX tilted = taught;
    tilted.phi = 8.0 * CV_PI / 180.0;
    tilted.theta = -5.0 * CV_PI / 180.0;
    Rigid3d gripper = Rigid3d::fromRotationTranslation(eulerToMatrix(tilted), referenceGripper.t);

    LevelConfig config;
    config.toleranceDeg = 0.1;
    config.gain = 0.6;  // a cautious robot takes a fraction of the computed step

    std::cout << "cross-laser plane leveling, gain " << config.gain
              << ", stop at " << config.toleranceDeg << " deg\n";
    for (int step = 1; step <= 8; ++step) {
        const LevelObservation current = observe(plate, gripper, cameraInGripper, laser);
        const LevelCommand command = computeLevelCommand(reference, current, cameraInGripper, config);
        std::cout << "step " << step
                  << "  tilt " << command.tiltDeg << " deg"
                  << "  kabsch residual " << command.rmsMm << " mm"
                  << "  tool dA " << command.kukaADeg
                  << "  dB " << command.kukaBDeg
                  << "  dC " << command.kukaCDeg << "\n";
        if (command.withinTolerance) {
            cv::Vec3d xyz;
            double a = 0, b = 0, c = 0;
            command.nextGripperInBase.toKukaAbc(xyz, a, b, c);
            std::cout << "converged. absolute KUKA ABC A " << a << " B " << b << " C " << c << "\n";
            return 0;
        }
        gripper = command.nextGripperInBase;
    }
    std::cerr << "did not converge\n";
    return 1;
}
