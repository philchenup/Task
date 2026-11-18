#include "plane_level/plane_level.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool cond, const std::string& name) {
    if (cond) {
        std::cout << "  PASS  " << name << "\n";
    } else {
        std::cout << "  FAIL  " << name << "\n";
        ++gFailures;
    }
}

double deg(double rad) { return rad * 180.0 / CV_PI; }

bool nearMat(const cv::Matx33d& A, const cv::Matx33d& B, double tol) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            if (std::abs(A(r, c) - B(r, c)) > tol) return false;
    return true;
}

bool nearVec(const cv::Vec3d& a, const cv::Vec3d& b, double tol) {
    return cv::norm(a - b) < tol;
}

plane_level::LaserModel testLaser() {
    plane_level::LaserModel laser;
    laser.arm0InCamera = {cv::Vec3d(0, 1, 0), 0};
    laser.arm1InCamera = {cv::Vec3d(1, 0, 0), 0};
    return laser;
}

void testEuler() {
    std::cout << "euler\n";
    using namespace plane_level;
    const EulerZYX samples[] = {
        {0.2, -0.35, 0.4},
        {-1.1, 0.7, 0.15},
        {0.01, -0.02, 1.4},
        {0, 0, 0},
    };
    for (const EulerZYX& e : samples) {
        const EulerZYX back = matrixToEuler(eulerToMatrix(e));
        expect(std::abs(wrapPi(back.phi - e.phi)) < 1e-10
                && std::abs(wrapPi(back.theta - e.theta)) < 1e-10
                && std::abs(wrapPi(back.psi - e.psi)) < 1e-10,
            "roundtrip phi/theta/psi");
    }
    // Patent inverse on the matrix we publish: phi = atan2(r32, r33).
    const cv::Matx33d R = eulerToMatrix(EulerZYX{0.3, -0.4, 0.5});
    expect(std::abs(std::atan2(R(2, 1), R(2, 2)) - 0.3) < 1e-12, "patent phi = atan2(r32, r33)");
    expect(std::abs(std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2))) + 0.4) < 1e-12, "patent theta");
    expect(std::abs(std::atan2(R(1, 0), R(0, 0)) - 0.5) < 1e-12, "patent psi");

    cv::Vec3d xyz;
    double A = 0, B = 0, C = 0;
    const Rigid3d pose = Rigid3d::fromKukaAbc(cv::Vec3d(10, -20, 30), 12.5, -7.25, 3.5);
    pose.toKukaAbc(xyz, A, B, C);
    expect(nearVec(xyz, cv::Vec3d(10, -20, 30), 1e-9) && std::abs(A - 12.5) < 1e-8
            && std::abs(B + 7.25) < 1e-8 && std::abs(C - 3.5) < 1e-8,
        "KUKA A/B/C is psi/theta/phi in degrees");
}

void testDistortion() {
    std::cout << "distortion\n";
    using namespace plane_level;
    DistortCoeffs d;
    d.k1 = -0.12;
    d.k2 = 0.03;
    d.k3 = -0.004;
    d.p1 = 0.001;
    d.p2 = -0.002;
    const cv::Vec2d ideal(0.18, -0.07);
    const cv::Vec2d distorted = distortNormalized(ideal, d);
    const cv::Vec2d recovered = undistortNormalized(distorted, d);
    expect(nearVec(cv::Vec3d(recovered[0], recovered[1], 0), cv::Vec3d(ideal[0], ideal[1], 0), 1e-12),
        "fixed-point undistort inverts Brown-Conrady");

    Intrinsics K;
    K.fx = 900;
    K.fy = 910;
    K.cx = 320;
    K.cy = 240;
    K.skew = -0.4;
    K.dist = d;
    const cv::Vec3d ray = pixelRay(K, 400, 260);
    cv::Mat dist(1, 5, CV_64F);
    dist.at<double>(0) = d.k1;
    dist.at<double>(1) = d.k2;
    dist.at<double>(2) = d.p1;
    dist.at<double>(3) = d.p2;
    dist.at<double>(4) = d.k3;
    std::vector<cv::Point2f> src{{400.f, 260.f}}, dst;
    cv::undistortPoints(src, dst, cv::Mat(K.K()), dist);
    expect(std::abs(dst[0].x - ray[0]) < 1e-5 && std::abs(dst[0].y - ray[1]) < 1e-5,
        "pixel ray matches cv::undistortPoints");
}

void testBoardAndPlane() {
    std::cout << "board intersection and light plane\n";
    using namespace plane_level;
    Intrinsics K;
    K.fx = 800;
    K.fy = 800;
    K.cx = 320;
    K.cy = 240;
    EulerZYX e{0.15, -0.2, 0.35};
    const Rigid3d board = Rigid3d::fromRotationTranslation(eulerToMatrix(e), cv::Vec3d(20, -10, 450));
    const cv::Vec3d world(30, -15, 0);
    const cv::Vec3d cam = board.apply(world);
    const double u = K.fx * (cam[0] / cam[2]) + K.cx;
    const double v = K.fy * (cam[1] / cam[2]) + K.cy;
    const cv::Vec3d rebuilt = laserPointOnBoard(K, board, u, v);
    expect(nearVec(rebuilt, cam, 1e-8), "Z=0 board solve returns the camera point");

    std::vector<cv::Vec3d> cloud;
    for (int i = -5; i <= 5; ++i) {
        for (int j = -5; j <= 5; ++j) {
            const double x = 10.0 * i;
            const double y = 12.0 * j;
            cloud.emplace_back(x, y, 0.02 * x - 0.05 * y + 3);
        }
    }
    const Plane3 plane = Plane3::fit(cloud);
    double maxAbs = 0;
    for (const cv::Vec3d& p : cloud) maxAbs = std::max(maxAbs, std::abs(plane.signedDistance(p)));
    expect(maxAbs < 1e-8, "SVD plane fit residual is numerical noise");
}

void testCalibration() {
    std::cout << "zhang intrinsics\n";
    using namespace plane_level;
    Intrinsics truth;
    truth.fx = 1000;
    truth.fy = 980;
    truth.cx = 320;
    truth.cy = 240;
    std::vector<cv::Vec3d> grid;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 8; ++c) grid.emplace_back(20.0 * c, 20.0 * r, 0);
    const EulerZYX poses[] = {
        {0.1, -0.2, 0.05},
        {-0.25, 0.15, 0.2},
        {0.05, 0.3, -0.4},
        {-0.15, -0.1, 0.35},
        {0.3, 0.05, -0.1},
        {-0.05, 0.22, 0.18},
    };
    const cv::Vec3d translations[] = {
        {10, -20, 500}, {30, 15, 470}, {-25, 10, 530},
        {5, -30, 490}, {20, 25, 510}, {-15, -5, 460},
    };
    std::vector<std::vector<cv::Vec3d>> objects;
    std::vector<std::vector<cv::Vec2d>> images;
    for (int i = 0; i < 6; ++i) {
        const Rigid3d board = Rigid3d::fromRotationTranslation(eulerToMatrix(poses[i]), translations[i]);
        std::vector<cv::Vec2d> pix;
        for (const cv::Vec3d& w : grid) {
            const cv::Vec3d cam = board.apply(w);
            pix.emplace_back(truth.fx * cam[0] / cam[2] + truth.cx, truth.fy * cam[1] / cam[2] + truth.cy);
        }
        objects.push_back(grid);
        images.push_back(pix);
    }
    const MonoCalibration cal = calibrateIntrinsics(objects, images, cv::Size(640, 480));
    expect(std::abs(cal.intrinsics.fx - truth.fx) < 1e-3
            && std::abs(cal.intrinsics.fy - truth.fy) < 1e-3
            && std::abs(cal.intrinsics.cx - truth.cx) < 1e-3
            && std::abs(cal.intrinsics.cy - truth.cy) < 1e-3
            && cal.rmsPx < 1e-4,
        "cv::calibrateCamera recovers fx, fy, cx, cy");
}

void testHandEye() {
    std::cout << "hand eye\n";
    using namespace plane_level;
    const Rigid3d X = Rigid3d::fromRotationTranslation(
        eulerToMatrix(EulerZYX{0.12, -0.2, 0.31}), cv::Vec3d(8, -15, 90));
    const Rigid3d boardInBase = Rigid3d::fromRotationTranslation(
        eulerToMatrix(EulerZYX{0.4, 0.25, -0.15}), cv::Vec3d(480, -20, 120));
    const EulerZYX spins[] = {
        {0.4, -0.3, 0.2}, {-0.5, 0.35, -0.4}, {0.2, 0.55, 0.1},
        {-0.35, -0.2, 0.6}, {0.6, 0.1, -0.45}, {-0.15, 0.45, 0.35},
        {0.25, -0.5, -0.2}, {-0.55, 0.15, 0.5},
    };
    std::vector<HandEyeSample> samples;
    for (int i = 0; i < 8; ++i) {
        HandEyeSample s;
        s.gripperInBase = Rigid3d::fromRotationTranslation(
            eulerToMatrix(spins[i]), cv::Vec3d(100.0 + 20 * i, -30.0 + 8 * i, 200.0 - 10 * i));
        s.targetInCamera = X.inverse() * s.gripperInBase.inverse() * boardInBase;
        samples.push_back(s);
    }
    const HandEyeMotions motions = relativeMotions(samples);
    bool axb = true;
    for (size_t i = 0; i < motions.A.size(); ++i) {
        const Rigid3d left = motions.A[i] * X;
        const Rigid3d right = X * motions.B[i];
        if (!nearMat(left.R, right.R, 1e-8) || !nearVec(left.t, right.t, 1e-6)) axb = false;
    }
    expect(axb, "A = Hg2^{-1} Hg1 and B = Hc2 Hc1^{-1} satisfy AX = XB");

    const Rigid3d solved = solveHandEye(samples);
    expect(nearMat(solved.R, X.R, 1e-6) && nearVec(solved.t, X.t, 1e-4), "Park SVD recovers X");
    const Rigid3d opencv = solveHandEyeOpenCv(samples);
    expect(nearMat(opencv.R, X.R, 1e-6) && nearVec(opencv.t, X.t, 1e-4), "cv::calibrateHandEye Park recovers X");

    const cv::Vec3d inCamera(1, 2, 40);
    const cv::Vec3d inBase = cameraToBase(inCamera, samples[0].gripperInBase, X);
    const cv::Vec3d direct = (samples[0].gripperInBase * X).apply(inCamera);
    expect(nearVec(inBase, direct, 1e-8), "cloud goes camera, tool, base");
}

void testSteger() {
    std::cout << "steger\n";
    using namespace plane_level;
    cv::Mat image(80, 120, CV_64F, cv::Scalar(0));
    const double y0 = 40.35;
    const double stripeSigma = 1.6;
    for (int y = 0; y < image.rows; ++y) {
        const double g = std::exp(-0.5 * std::pow(y - y0, 2) / (stripeSigma * stripeSigma));
        for (int x = 10; x < 110; ++x) image.at<double>(y, x) = 220.0 * g;
    }
    StegerParams params;
    params.stripeWidthPx = 6;
    params.sigma = 1.2;
    params.strength = 0.5;
    params.minIntensity = 20;
    const std::vector<StripePoint> centers = extractStripeCenters(image, params);
    expect(centers.size() > 40, "horizontal ridge produces a centerline");
    double bias = 0;
    int used = 0;
    for (const StripePoint& p : centers) {
        if (p.x < 20 || p.x > 100) continue;
        bias += p.y - y0;
        ++used;
    }
    bias /= static_cast<double>(used);
    expect(std::abs(bias) < 0.02, "horizontal subpixel bias stays under 0.02 px");

    cv::Mat vertical(80, 120, CV_64F, cv::Scalar(0));
    const double x0 = 60.25;
    for (int x = 0; x < vertical.cols; ++x) {
        const double g = std::exp(-0.5 * std::pow(x - x0, 2) / (stripeSigma * stripeSigma));
        for (int y = 10; y < 70; ++y) vertical.at<double>(y, x) = 220.0 * g;
    }
    double xbias = 0;
    int xused = 0;
    for (const StripePoint& p : extractStripeCenters(vertical, params)) {
        if (p.y < 20 || p.y > 60) continue;
        xbias += p.x - x0;
        ++xused;
    }
    xbias /= static_cast<double>(xused);
    expect(xused > 20 && std::abs(xbias) < 0.02, "vertical subpixel bias stays under 0.02 px");

    cv::Mat cross = image.clone();
    const double crossX = 60.2;
    for (int y = 10; y < 70; ++y) {
        for (int x = 0; x < cross.cols; ++x) {
            const double g = std::exp(-0.5 * std::pow(x - crossX, 2) / (stripeSigma * stripeSigma));
            cross.at<double>(y, x) = std::max(cross.at<double>(y, x), 220.0 * g);
        }
    }
    const CrossSplit split = splitCross(extractStripeCenters(cross, params));
    expect(split.arm0.size() > 15 && split.arm1.size() > 15, "cross splits into two arms");
}

void testKabsch() {
    std::cout << "kabsch\n";
    using namespace plane_level;
    std::vector<cv::Vec3d> source;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) source.emplace_back(5.0 * i, -3.0 * j, 0.4 * i - 0.2 * j);
    const Rigid3d truth = Rigid3d::fromRotationTranslation(
        eulerToMatrix(EulerZYX{-0.2, 0.35, 0.55}), cv::Vec3d(10, -4, 7));
    std::vector<cv::Vec3d> target;
    for (const cv::Vec3d& p : source) target.push_back(truth.apply(p));
    const AlignResult fit = absoluteOrientation(source, target);
    expect(nearMat(fit.sourceToTarget.R, truth.R, 1e-8) && nearVec(fit.sourceToTarget.t, truth.t, 1e-7),
        "SVD absolute orientation recovers R and t");
    expect(fit.rms < 1e-8, "paired residual is numerical noise");

    // The centered objective depends on R only, and T = q_bar - R p_bar.
    const cv::Vec3d ps = [&] {
        cv::Vec3d c(0, 0, 0);
        for (const cv::Vec3d& p : source) c += p;
        return c * (1.0 / source.size());
    }();
    const cv::Vec3d pt = [&] {
        cv::Vec3d c(0, 0, 0);
        for (const cv::Vec3d& p : target) c += p;
        return c * (1.0 / target.size());
    }();
    expect(nearVec(fit.sourceToTarget.t, pt - fit.sourceToTarget.R * ps, 1e-8),
        "translation is the centroid relation in the patent");
}

void testLeveling() {
    std::cout << "leveling\n";
    using namespace plane_level;
    const LaserModel laser = testLaser();
    const Plate plate{cv::Vec3d(0, 0, -1), 500};
    const Rigid3d cameraInGripper = Rigid3d::fromRotationTranslation(
        eulerToMatrix(EulerZYX{0.05, -0.04, 0.03}), cv::Vec3d(0, 30, 70));
    const Rigid3d referenceGripper = Rigid3d::fromKukaAbc(cv::Vec3d(0, 0, 0), 0, 0, 0);
    auto armsOf = [&](const Rigid3d& gripper) {
        return sampleLaserCross(plate, gripper, cameraInGripper, laser, 40, 30);
    };
    auto observe = [&](const Rigid3d& gripper) {
        auto arms = armsOf(gripper);
        LevelObservation obs;
        obs.arm0 = std::move(arms[0]);
        obs.arm1 = std::move(arms[1]);
        obs.gripperInBase = gripper;
        return obs;
    };
    const LevelObservation reference = observe(referenceGripper);
    const Rigid3d tilted = Rigid3d::fromRotationTranslation(
        eulerToMatrix(EulerZYX{8.0 * CV_PI / 180.0, -5.0 * CV_PI / 180.0, 2.0 * CV_PI / 180.0}),
        referenceGripper.t);
    LevelConfig full;
    full.gain = 1;
    full.toleranceDeg = 0.1;
    const LevelCommand once = computeLevelCommand(reference, observe(tilted), cameraInGripper, full);
    const cv::Vec3d nRef = crossNormal(reference.arm0, reference.arm1);
    auto armsNext = armsOf(once.nextGripperInBase);
    const cv::Vec3d nNext = crossNormal(armsNext[0], armsNext[1]);
    const double residual = std::acos(std::clamp(std::abs(nRef.dot(nNext)), 0.0, 1.0)) * 180.0 / CV_PI;
    expect(once.tiltDeg > 5 && residual < 1e-4, "one full normal step removes a finite tilt");
    expect(once.kabschTiltDeg < 0.2, "Kabsch on the rescanned cross agrees at this tilt");

    LevelConfig damped = full;
    damped.gain = 0.5;
    Rigid3d gripper = tilted;
    double previous = 1e9;
    bool monotone = true;
    bool converged = false;
    for (int i = 0; i < 8; ++i) {
        const LevelCommand step = computeLevelCommand(reference, observe(gripper), cameraInGripper, damped);
        if (!(step.tiltDeg < previous - 1e-9)) monotone = false;
        previous = step.tiltDeg;
        gripper = step.nextGripperInBase;
        if (step.withinTolerance) {
            converged = true;
            break;
        }
    }
    expect(monotone && converged, "half-gain iteration reaches 0.1 deg");
}

void testLightPlaneCalibration() {
    std::cout << "light plane calibration\n";
    using namespace plane_level;
    Intrinsics K;
    K.fx = 700;
    K.fy = 700;
    K.cx = 320;
    K.cy = 240;
    // A laser plane in the camera, y = 0.1 x, not through a trivial axis.
    const Plane3 truth{cv::Vec3d(0.1, -1, 0.02), 15};
    const cv::Vec3d n = truth.n * (1.0 / cv::norm(truth.n));
    const Plane3 truthUnit{n, truth.d / cv::norm(truth.n)};
    std::vector<LaserView> views;
    const EulerZYX spins[] = {{0.2, -0.1, 0.05}, {-0.15, 0.25, 0.1}, {0.05, 0.1, -0.3}, {-0.3, -0.05, 0.2}};
    const cv::Vec3d translations[] = {{0, 0, 400}, {30, -10, 380}, {-20, 15, 430}, {10, 20, 360}};
    for (int v = 0; v < 4; ++v) {
        LaserView view;
        view.boardInCamera = Rigid3d::fromRotationTranslation(eulerToMatrix(spins[v]), translations[v]);
        // Board plane in the camera: points board.apply((X, Y, 0)), normal R * ez.
        const cv::Vec3d boardNormal = view.boardInCamera.R * cv::Vec3d(0, 0, 1);
        const double boardD = -boardNormal.dot(view.boardInCamera.t);
        cv::Vec3d direction = truthUnit.n.cross(boardNormal);
        direction *= 1.0 / cv::norm(direction);
        const double n11 = truthUnit.n.dot(truthUnit.n);
        const double n12 = truthUnit.n.dot(boardNormal);
        const double n22 = boardNormal.dot(boardNormal);
        const double det = n11 * n22 - n12 * n12;
        const double alpha = (-truthUnit.d * n22 + boardD * n12) / det;
        const double beta = (-boardD * n11 + truthUnit.d * n12) / det;
        const cv::Vec3d origin = alpha * truthUnit.n + beta * boardNormal;
        for (int i = -6; i <= 6; ++i) {
            const cv::Vec3d cam = origin + direction * (12.0 * i);
            if (cam[2] < 50) continue;
            const double u = K.fx * cam[0] / cam[2] + K.cx;
            const double v = K.fy * cam[1] / cam[2] + K.cy;
            view.arm0Px.emplace_back(u, v);
        }
        // Second arm is a different known plane, x = 0 shifted.
        views.push_back(view);
    }
    // Fit arm 0 alone by duplicating the same stripe into arm1 with a second plane.
    const Plane3 truth2{cv::Vec3d(1, 0.05, -0.01) * (1.0 / cv::norm(cv::Vec3d(1, 0.05, -0.01))), 8};
    for (LaserView& view : views) {
        const cv::Vec3d boardNormal = view.boardInCamera.R * cv::Vec3d(0, 0, 1);
        const double boardD = -boardNormal.dot(view.boardInCamera.t);
        cv::Vec3d direction = truth2.n.cross(boardNormal);
        direction *= 1.0 / cv::norm(direction);
        const double n11 = 1;
        const double n12 = truth2.n.dot(boardNormal);
        const double n22 = 1;
        const double det = n11 * n22 - n12 * n12;
        const double alpha = (-truth2.d * n22 + boardD * n12) / det;
        const double beta = (-boardD * n11 + truth2.d * n12) / det;
        const cv::Vec3d origin = alpha * truth2.n + beta * boardNormal;
        for (int i = -6; i <= 6; ++i) {
            const cv::Vec3d cam = origin + direction * (12.0 * i);
            if (cam[2] < 50) continue;
            view.arm1Px.emplace_back(K.fx * cam[0] / cam[2] + K.cx, K.fy * cam[1] / cam[2] + K.cy);
        }
    }
    const LaserPlanes fit = fitLaserPlanes(K, views);
    auto planeErr = [](const Plane3& a, const Plane3& b) {
        const double flip = a.n.dot(b.n) < 0 ? -1 : 1;
        return cv::norm(a.n - b.n * flip) + std::abs(a.d - b.d * flip);
    };
    expect(planeErr(fit.arm0, truthUnit) < 1e-6 && planeErr(fit.arm1, truth2) < 1e-6,
        "least squares recovers both laser planes");
}

}

int main() {
    testEuler();
    testDistortion();
    testBoardAndPlane();
    testCalibration();
    testHandEye();
    testSteger();
    testKabsch();
    testLightPlaneCalibration();
    testLeveling();
    if (gFailures) {
        std::cerr << gFailures << " failure(s)\n";
        return 1;
    }
    std::cout << "all checks passed\n";
    return 0;
}
