#include "plane_level.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace tubeplate;

namespace {

int gFails = 0;

void expect(bool cond, const std::string& name)
{
    if (cond) {
        std::cout << "  ok  " << name << "\n";
        return;
    }
    std::cout << "  FAIL " << name << "\n";
    ++gFails;
}

double deg2rad(double d) { return d * 3.14159265358979323846 / 180.0; }

double angleBetween(Vec3 a, Vec3 b)
{
    const double c = std::max(-1.0, std::min(1.0, dot(normalized(a), normalized(b))));
    return std::acos(c) * 180.0 / 3.14159265358979323846;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) < tol; }

bool vecNear(Vec3 a, Vec3 b, double tol)
{
    return norm(sub(a, b)) < tol;
}

Mat3 rotDiff(const Mat3& a, const Mat3& b)
{
    return matMul(a, transpose(b));
}

// Intersection of two planes n·x + d = 0. dir points along the line.
bool planePlane(const LightPlane& p, const LightPlane& q, Vec3& point, Vec3& dir)
{
    const Vec3 n1{p.a, p.b, p.c};
    const Vec3 n2{q.a, q.b, q.c};
    dir = cross(n1, n2);
    const double dd = dot(dir, dir);
    if (dd < 1e-12) return false;
    point = scale(add(scale(cross(n2, dir), -p.d), scale(cross(dir, n1), -q.d)), 1.0 / dd);
    dir = scale(dir, 1.0 / std::sqrt(dd));
    return true;
}

LightPlane makePlane(Vec3 normal, Vec3 point)
{
    const Vec3 n = normalized(normal);
    LightPlane p;
    p.a = n.x; p.b = n.y; p.c = n.z;
    p.d = -dot(n, point);
    return p;
}

LightPlane planeToBase(const LightPlane& cam, const Pose& sensorInBase)
{
    const Vec3 n = matVec(sensorInBase.R, Vec3{cam.a, cam.b, cam.c});
    LightPlane out;
    out.a = n.x; out.b = n.y; out.c = n.z;
    out.d = cam.d - dot(n, sensorInBase.t);
    return out;
}

std::vector<Vec3> sampleCross(const Pose& sensorInBase, const LightPlane laser[2],
    Vec3 plateNormal, Vec3 platePoint, double stepMm, double halfMm)
{
    const LightPlane plate = makePlane(plateNormal, platePoint);
    Vec3 origin[2], dir[2];
    for (int k = 0; k < 2; ++k) {
        const LightPlane inBase = planeToBase(laser[k], sensorInBase);
        if (!planePlane(inBase, plate, origin[k], dir[k])) return {};
    }
    // Both arms are sampled through their intersection, which is the cross
    // the camera actually sees. The plane-plane helper's own origin is the
    // point nearest the base origin and is not that intersection.
    const Vec3 w = sub(origin[0], origin[1]);
    const double a = dot(dir[0], dir[0]);
    const double b = dot(dir[0], dir[1]);
    const double c = dot(dir[1], dir[1]);
    const double d = dot(dir[0], w);
    const double e = dot(dir[1], w);
    const double denom = a * c - b * b;
    if (std::fabs(denom) < 1e-12) return {};
    const double s = (b * e - c * d) / denom;
    const Vec3 center = add(origin[0], scale(dir[0], s));

    std::vector<Vec3> pts;
    const int n = std::max(2, static_cast<int>(halfMm / stepMm));
    for (int k = 0; k < 2; ++k) {
        for (int i = -n; i <= n; ++i) {
            if (i == 0) continue;
            pts.push_back(add(center, scale(dir[k], stepMm * i)));
        }
    }
    return pts;
}

void paintRidge(std::vector<float>& image, int w, int h,
    double u0, double v0, double u1, double v1, double sigma)
{
    const double dx = u1 - u0;
    const double dy = v1 - v0;
    const double L2 = dx * dx + dy * dy;
    if (L2 < 1) return;
    const int rad = static_cast<int>(std::ceil(4.0 * sigma));
    const int xLo = std::max(0, static_cast<int>(std::floor(std::min(u0, u1))) - rad);
    const int xHi = std::min(w - 1, static_cast<int>(std::ceil(std::max(u0, u1))) + rad);
    const int yLo = std::max(0, static_cast<int>(std::floor(std::min(v0, v1))) - rad);
    const int yHi = std::min(h - 1, static_cast<int>(std::ceil(std::max(v0, v1))) + rad);
    for (int y = yLo; y <= yHi; ++y) {
        for (int x = xLo; x <= xHi; ++x) {
            double t = ((x - u0) * dx + (y - v0) * dy) / L2;
            t = std::max(0.0, std::min(1.0, t));
            const double eu = x - (u0 + t * dx);
            const double ev = y - (v0 + t * dy);
            const float val = static_cast<float>(std::exp(-(eu * eu + ev * ev) / (2.0 * sigma * sigma)));
            float& pix = image[y * w + x];
            if (val > pix) pix = val;
        }
    }
}

double median(std::vector<double> v)
{
    if (v.empty()) return 1e9;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

void testHardware()
{
    std::cout << "hardware\n";
    const Hardware hw = selectedHardware();
    const Camera cam = nominalCamera(hw);
    expect(near(cam.fx, 12.0 / 0.00345, 1e-6), "fx = f / pixel size");
    expect(near(cam.fy, cam.fx, 1e-12), "square pixels");
    expect(cam.width == 1440 && cam.height == 1080, "IMX273 raster");
    const double mmPerPx = hw.standoffMm / cam.fx;
    const double fovW = hw.width * mmPerPx;
    const double fovH = hw.height * mmPerPx;
    expect(fovW > 95 && fovW < 110 && fovH > 70 && fovH < 85, "250 mm field covers the cross");
    const double sigma = stripeSigmaPx(hw);
    expect(sigma > 3.5 && sigma < 4.5, "sigma = omega / sqrt(3) is about 4 px");
    const double patternW = (hw.boardInnerCols - 1) * hw.squareMm;
    const double patternH = (hw.boardInnerRows - 1) * hw.squareMm;
    expect(patternW < fovW * 0.8 && patternH < fovH * 0.8, "chessboard fits the field");
    std::cout << "    camera " << hw.width << "x" << hw.height
              << "  fx " << cam.fx
              << "  FOV " << fovW << " x " << fovH << " mm\n";
    std::cout << "    laser " << hw.laserNm << " nm, " << hw.laserPowerMw
              << " mW, line " << hw.lineWidthMm << " mm, sigma " << sigma << " px\n";
    std::cout << "    board " << hw.boardInnerCols << "x" << hw.boardInnerRows
              << " inner corners, " << hw.squareMm << " mm, pattern "
              << patternW << " x " << patternH << " mm\n";
}

void testEuler()
{
    std::cout << "euler\n";
    const double phi = deg2rad(12.0);
    const double theta = deg2rad(-25.0);
    const double psi = deg2rad(40.0);
    const Mat3 r = eulerMatrix(phi, theta, psi);
    double p2, t2, s2;
    eulerAngles(r, p2, t2, s2);
    expect(near(p2, phi, 1e-9) && near(t2, theta, 1e-9) && near(s2, psi, 1e-9),
        "phi theta psi roundtrip");
    // Patent extraction, checked against the matrix elements directly.
    const double phiE = std::atan2(r.m[7], r.m[8]);
    const double thetaE = std::atan2(-r.m[6], std::hypot(r.m[7], r.m[8]));
    const double psiE = std::atan2(r.m[3], r.m[0]);
    expect(near(phiE, phi, 1e-9) && near(thetaE, theta, 1e-9) && near(psiE, psi, 1e-9),
        "atan2(r32,r33), atan2(-r31,hypot), atan2(r21,r11)");

    const Mat3 built = matMul(matMul(eulerMatrix(0, 0, psi), eulerMatrix(0, theta, 0)),
        eulerMatrix(phi, 0, 0));
    expect(rotationAngleDeg(rotDiff(r, built)) < 1e-4, "R = Rz(psi) Ry(theta) Rx(phi)");

    double A, B, C;
    kukaAngles(r, A, B, C);
    expect(near(A, 40.0, 1e-8) && near(B, -25.0, 1e-8) && near(C, 12.0, 1e-8),
        "KUKA A = psi, B = theta, C = phi");
    expect(rotationAngleDeg(rotDiff(kukaMatrix(A, B, C), r)) < 1e-4, "KUKA matrix matches");
}

void testDistortionAndLift()
{
    std::cout << "distortion and board lift\n";
    Camera cam;
    cam.fx = 1200; cam.fy = 1180; cam.cx = 640; cam.cy = 480; cam.skew = 0.4;
    cam.k1 = -0.12; cam.k2 = 0.03; cam.k3 = -0.01; cam.p1 = 0.0015; cam.p2 = -0.002;
    double x = 0.18, y = -0.12, xu, yu;
    double xd, yd;
    distortNormalized(cam, x, y, xd, yd);
    expect(undistortNormalized(cam, xd, yd, xu, yu), "undistort converges");
    expect(near(xu, x, 1e-10) && near(yu, y, 1e-10), "undistort inverts k1..p2");

    Pose board;
    board.R = eulerMatrix(deg2rad(8), deg2rad(-14), deg2rad(20));
    board.t = {12, -20, 280};
    const Vec3 onBoard{24, -15, 0};
    const Vec3 camPt = transformPoint(board, onBoard);
    double u, v;
    project(cam, camPt, u, v);
    Vec3 lifted;
    expect(liftStripeOnBoard(cam, board, u, v, lifted), "lift solves the 3x3");
    expect(vecNear(lifted, camPt, 1e-6), "lifted point is the camera point");
}

void testPlaneFit()
{
    std::cout << "light plane\n";
    const LightPlane truth = makePlane(Vec3{0.2, -0.4, 0.9}, Vec3{10, -5, 200});
    std::vector<Vec3> pts;
    for (int i = -5; i <= 5; ++i) {
        for (int j = -5; j <= 5; ++j) {
            const double x = 4.0 * i;
            const double y = 3.0 * j;
            const double z = -(truth.a * x + truth.b * y + truth.d) / truth.c;
            pts.push_back({x, y, z});
        }
    }
    LightPlane fit;
    expect(fitLightPlane(pts, fit), "fit");
    const double parallel = std::fabs(dot(
        normalized({truth.a, truth.b, truth.c}),
        normalized({fit.a, fit.b, fit.c})));
    expect(parallel > 0.999999, "normal matches");
    double worst = 0;
    for (Vec3 p : pts) {
        worst = std::max(worst, std::fabs(fit.a * p.x + fit.b * p.y + fit.c * p.z + fit.d));
    }
    expect(worst < 1e-6, "points lie on the fitted plane");
}

void testOrientation()
{
    std::cout << "absolute orientation\n";
    const Mat3 r = eulerMatrix(deg2rad(7), deg2rad(-11), deg2rad(4));
    const Vec3 t{5, -3, 8};
    std::vector<Vec3> p, q;
    const double src[5][3] = {
        {0, 0, 0}, {30, 0, 0}, {-30, 0, 0}, {0, 20, 0}, {0, -20, 1}
    };
    for (auto s : src) {
        Vec3 a{s[0], s[1], s[2]};
        p.push_back(a);
        q.push_back(add(matVec(r, a), t));
    }
    Mat3 rg;
    Vec3 tg;
    expect(absoluteOrientation(p, q, rg, tg), "svd");
    expect(rotationAngleDeg(rotDiff(rg, r)) < 1e-4, "recovers R");
    expect(vecNear(tg, t, 1e-6), "recovers T = q_bar - R p_bar");

    // Planar cross: the reflection fix has to keep the normal, not flip it.
    std::vector<Vec3> pref, pcur;
    for (int i = -8; i <= 8; ++i) {
        if (i == 0) continue;
        pref.push_back({3.0 * i, 0, 0});
        pref.push_back({0, 3.0 * i, 0});
    }
    const Mat3 tilt = eulerMatrix(deg2rad(4), deg2rad(-3), 0);
    for (Vec3 a : pref) pcur.push_back(matVec(tilt, a));
    expect(absoluteOrientation(pref, pcur, rg, tg), "planar svd");
    expect(rotationAngleDeg(rotDiff(rg, tilt)) < 1e-4, "planar cross keeps the tilt");
    expect(det(rg) > 0, "proper rotation");
}

void testHandEye()
{
    std::cout << "hand eye\n";
    const Pose truth = [] {
        Pose x;
        x.R = eulerMatrix(deg2rad(6), deg2rad(-9), deg2rad(13));
        x.t = {32, -18, 41};
        return x;
    }();
    const Pose board = [] {
        Pose b;
        b.R = eulerMatrix(deg2rad(15), deg2rad(5), deg2rad(-20));
        b.t = {40, -25, 600};
        return b;
    }();

    std::vector<Pose> hc, hg;
    for (int i = 0; i < 8; ++i) {
        Pose g;
        g.R = eulerMatrix(deg2rad(8.0 * i - 20), deg2rad(-6.0 * i + 10), deg2rad(5.0 * i));
        g.t = {80.0 * i, -40.0 + 15.0 * i, 200.0 + 12.0 * i};
        // T_base_board = Hg * X * Hc  =>  Hc = X^{-1} * Hg^{-1} * T_base_board
        const Pose hcI = poseMul(poseInverse(truth), poseMul(poseInverse(g), board));
        hc.push_back(hcI);
        hg.push_back(g);
    }
    Pose got;
    expect(sensorToTool(hc, hg, got), "solve from 8 poses");
    expect(rotationAngleDeg(rotDiff(got.R, truth.R)) < 1e-6, "sensor-to-tool rotation");
    expect(vecNear(got.t, truth.t, 1e-5), "sensor-to-tool translation");

    std::vector<Pose> hc3(hc.begin(), hc.begin() + 3);
    std::vector<Pose> hg3(hg.begin(), hg.begin() + 3);
    expect(sensorToTool(hc3, hg3, got), "three poses, the patent minimum");
    expect(rotationAngleDeg(rotDiff(got.R, truth.R)) < 1e-6, "three poses recover rotation");
    expect(vecNear(got.t, truth.t, 1e-4), "three poses recover translation");

    // The printed A X = X B, with A the camera motion and B the tool motion,
    // returns the inverse of the matrix used to move points into the tool.
    std::vector<Pose> motionsA, motionsB;
    for (size_t i = 0; i + 1 < hc.size(); ++i) {
        motionsA.push_back(poseMul(hc[i + 1], poseInverse(hc[i])));
        motionsB.push_back(poseMul(poseInverse(hg[i + 1]), hg[i]));
    }
    Pose printed;
    expect(solveAxEqualsXb(motionsA, motionsB, printed), "AX = XB");
    const Pose back = poseInverse(printed);
    expect(rotationAngleDeg(rotDiff(back.R, truth.R)) < 1e-6, "printed X is tool-to-sensor");
}

void testStegerAndReconstruct()
{
    std::cout << "steger and triangulation\n";
    Camera cam;
    cam.fx = 800; cam.fy = 800; cam.cx = 239.5; cam.cy = 179.5;
    cam.width = 480; cam.height = 360;
    LightPlane lasers[2] = {
        makePlane(Vec3{1, 0.02, 0.12}, Vec3{20, 0, 200}),
        makePlane(Vec3{-0.03, 1, -0.05}, Vec3{0, -12, 200})
    };
    const Vec3 plateN = normalized(Vec3{0.08, -0.06, 1});
    const Vec3 plateP{0, 0, 250};
    const Pose eye{}; // camera is the reference frame
    const auto truth = sampleCross(eye, lasers, plateN, plateP, 0.4, 28);
    expect(truth.size() > 40, "synthetic cross has both arms");

    const double sigma = 2.2;
    std::vector<float> image(cam.width * cam.height, 0);
    const int perArm = static_cast<int>(truth.size() / 2);
    auto endpoint = [&](int index, double& u, double& v) {
        project(cam, truth[index], u, v);
    };
    for (int arm = 0; arm < 2; ++arm) {
        double u0, v0, u1, v1;
        endpoint(arm * perArm, u0, v0);
        endpoint(arm * perArm + perArm - 1, u1, v1);
        paintRidge(image, cam.width, cam.height, u0, v0, u1, v1, sigma);
    }
    std::vector<StripeCenter> centers;
    extractStripeCenters(image.data(), cam.width, cam.height, sigma, 0.01f, 0.15f, centers);
    expect(centers.size() > 80, "steger returns a centerline");

    std::vector<double> pixErr;
    for (const auto& s : centers) {
        double best = 1e9;
        for (size_t i = 0; i + 1 < truth.size(); ++i) {
            double u0, v0, u1, v1;
            project(cam, truth[i], u0, v0);
            project(cam, truth[i + 1], u1, v1);
            const double dx = u1 - u0;
            const double dy = v1 - v0;
            const double L2 = dx * dx + dy * dy;
            if (L2 < 1) continue;
            double t = ((s.u - u0) * dx + (s.v - v0) * dy) / L2;
            t = std::max(0.0, std::min(1.0, t));
            const double eu = s.u - (u0 + t * dx);
            const double ev = s.v - (v0 + t * dy);
            best = std::min(best, std::hypot(eu, ev));
        }
        pixErr.push_back(best);
    }
    const double med = median(pixErr);
    std::cout << "    median center error " << med << " px, centers " << centers.size() << "\n";
    expect(med < 0.2, "subpixel center within 0.2 px");

    const Reconstruction rec = reconstructCross(image.data(), cam.width, cam.height,
        cam, lasers, sigma, 250.0, 0.01, 0.15f);
    expect(rec.ok, "both arms triangulate");
    std::vector<double> mm;
    for (Vec3 p : rec.cameraPoints) {
        double best = 1e9;
        for (Vec3 q : truth) best = std::min(best, norm(sub(p, q)));
        mm.push_back(best);
    }
    const double medMm = median(mm);
    std::cout << "    median 3D error " << medMm << " mm\n";
    expect(medMm < 0.3, "ray-plane intersection within 0.3 mm");
}

void testLevelingLoop()
{
    std::cout << "iterative leveling\n";
    const Hardware hw = selectedHardware();
    const Pose handEye = [] {
        Pose x;
        x.R = eulerMatrix(deg2rad(2.0), deg2rad(-3.0), deg2rad(4.0));
        x.t = {35, -15, 25};
        return x;
    }();
    LightPlane lasers[2] = {
        makePlane(Vec3{1, 0.04, 0.10}, Vec3{30, 0, 180}),
        makePlane(Vec3{0.02, 1, -0.06}, Vec3{0, -18, 180})
    };

    RobotPose taught;
    taught.x = 0; taught.y = 0; taught.z = 150;
    taught.a = 5; taught.b = -8; taught.c = 3;
    const Vec3 platePoint{0, 0, 400};
    const Vec3 nRef = matVec(robotPoseToFrame(taught).R, Vec3{0, 0, 1});
    const Pose sensorTaught = poseMul(robotPoseToFrame(taught), handEye);
    const auto reference = sampleCross(sensorTaught, lasers, nRef, platePoint, 3.0, 36);
    expect(reference.size() > 40, "reference cross captured at the taught pose");

    const Mat3 plateTilt = eulerMatrix(deg2rad(3.0), deg2rad(-2.2), deg2rad(0.4));
    const Vec3 nNow = matVec(plateTilt, nRef);
    RobotPose pose = taught;
    const auto referenceTool = basePointsToTool(reference, taught);
    double err = angleBetween(matVec(robotPoseToFrame(pose).R, Vec3{0, 0, 1}), nNow);
    std::cout << "    initial tool-to-plate " << err << " deg\n";
    bool converged = false;
    for (int iter = 0; iter < hw.maxIterations; ++iter) {
        const Pose sensor = poseMul(robotPoseToFrame(pose), handEye);
        const auto current = sampleCross(sensor, lasers, nNow, platePoint, 3.0, 36);
        const auto currentTool = basePointsToTool(current, pose);
        const LevelCommand cmd = levelOnce(referenceTool, currentTool, pose, hw.angleToleranceDeg);
        expect(cmd.ok, "iteration " + std::to_string(iter + 1) + " solved");
        if (!cmd.ok) return;
        pose = cmd.target;
        err = angleBetween(matVec(robotPoseToFrame(pose).R, Vec3{0, 0, 1}), nNow);
        std::cout << "    iter " << iter + 1
                  << "  correction " << cmd.tiltDeg
                  << " deg  phi " << cmd.phiDeg
                  << "  theta " << cmd.thetaDeg
                  << "  psi " << cmd.psiDeg
                  << "  residual " << err << " deg\n";
        if (err < hw.angleToleranceDeg && cmd.withinTolerance) {
            converged = true;
            break;
        }
    }
    expect(converged && err < hw.angleToleranceDeg, "residual under 0.1 deg");
    expect(near(pose.x, taught.x, 1e-9) && near(pose.y, taught.y, 1e-9)
        && near(pose.z, taught.z, 1e-9), "XYZ is held, only ABC is commanded");
}

} // namespace

int main()
{
    testHardware();
    testEuler();
    testDistortionAndLift();
    testPlaneFit();
    testOrientation();
    testHandEye();
    testStegerAndReconstruct();
    testLevelingLoop();
    if (gFails) {
        std::cout << gFails << " failed\n";
        return 1;
    }
    std::cout << "all passed\n";
    return 0;
}
