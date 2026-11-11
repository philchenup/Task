#ifndef PLANE_LEVEL_H
#define PLANE_LEVEL_H

// Automatic plane leveling for a tube-sheet welding robot.
// Implements CN112894209A (cross-laser sensor) on the KUKA cell in this
// repository: sensor calibration, Steger stripe centers, hand-eye AX = XB,
// and the least-squares rigid match the patent calls ICP.
//
// Hardware this model is built for (eye-in-hand, arc off while measuring):
//
//   Camera   Global-shutter GigE, Sony IMX273 class, 1440 x 1080, 3.45 um.
//            Basler acA1440-73gm or Hikrobot MV-CS016-10GM. Hardware trigger.
//   Lens     12 mm C-mount, 2/3 in, e.g. Computar V1228-MPY. Nominal
//            fx = fy = 12 / 0.00345 = 3478.26 px. At 250 mm the field is
//            103.5 x 77.6 mm, so a 0.1 deg tilt is an 0.09 mm rise across a
//            50 mm arm (about 1.2 px) and subpixel centers can see it.
//   Filter   450 nm bandpass, 20 nm FWHM, in the C-mount, in front of the sensor.
//   Laser    450 nm cross, Powell, 40 mW class 3R, 30 deg fan, line width
//            0.5 mm at 250 mm (about 7 px). Steger needs sigma >= width/sqrt(3);
//            the patent prints sigma < width/sqrt(3). The code uses the
//            boundary, sigma = width/sqrt(3), about 4.0 px.
//            Mounted ~70 mm off the camera axis so neither light plane
//            contains the camera origin. Planes at 90 deg, rigid to the camera.
//   Board    Glass or ceramic chessboard, flat to 0.02 mm, 11 x 8 inner
//            corners, 6 mm squares (pattern 60 x 42 mm). Matte, so the blue
//            line is visible on the light squares.
//   Robot    KR 16 R1610-2. The correction is an absolute XYZABC on the
//            existing Cartesian stream (command 4). XYZ stays, ABC changes.
//            Stop when the rotation that still remains is under 0.1 deg.
//
// Euler convention. The specification text says phi, theta, psi are the
// rotations about X, Y, Z, and extracts
//   phi   = atan2(r32, r33)     about X
//   theta = atan2(-r31, hypot(r32, r33))   about Y
//   psi   = atan2(r21, r11)     about Z
// which is R = Rz(psi) Ry(theta) Rx(phi). KUKA A, B, C is the same product
// Rz(A) Ry(B) Rx(C), so A = psi, B = theta, C = phi, in degrees.
// The printed product R = Rz(phi) Ry(theta) Rx(psi) swaps the names of phi
// and psi relative to that extraction. This code follows the extraction and
// the sentence that names the axes, because that is the matrix KUKA applies.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tubeplate {

struct Vec3 {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct Mat3 {
    double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

// Rigid transform of a column vector: p_out = R p_in + t.
struct Pose {
    Mat3 R;
    Vec3 t;
};

// Pinhole intrinsics plus Brown-Conrady distortion (k1, k2, k3, p1, p2).
// Pixel (u, v), normalized ideal (x, y) = ((X/Z), (Y/Z)):
//   xh = x (1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x y + p2 (r^2 + 2 x^2)
//   yh = y (1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y^2) + 2 p2 x y
//   u  = fx xh + skew yh + cx
//   v  = fy yh + cy
struct Camera {
    double fx = 1;
    double fy = 1;
    double cx = 0;
    double cy = 0;
    double skew = 0;
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
    int width = 0;
    int height = 0;
};

// a X + b Y + c Z + d = 0, with (a, b, c) a unit normal.
struct LightPlane {
    double a = 0, b = 0, c = 1, d = 0;
};

struct Hardware {
    int width = 1440;
    int height = 1080;
    double pixelSizeMm = 0.00345;
    double focalLengthMm = 12.0;
    double standoffMm = 250.0;
    double laserNm = 450.0;
    double filterBandwidthNm = 20.0;
    double laserPowerMw = 40.0;
    double fanAngleDeg = 30.0;
    double lineWidthMm = 0.50;
    double baselineMm = 70.0;
    int boardInnerCols = 11;
    int boardInnerRows = 8;
    double squareMm = 6.0;
    double angleToleranceDeg = 0.1;
    int maxIterations = 8;
    double exposureUs = 500.0;
};

struct StripeCenter {
    double u = 0;
    double v = 0;
    double nx = 0;
    double ny = 0;
};

struct RobotPose {
    double x = 0, y = 0, z = 0; // mm, tool origin in the active base
    double a = 0, b = 0, c = 0; // deg, KUKA A B C
};

struct LevelCommand {
    bool ok = false;
    RobotPose target;
    double phiDeg = 0;    // about X, of the base-frame correction
    double thetaDeg = 0;  // about Y
    double psiDeg = 0;    // about Z
    double tiltDeg = 0;   // axis-angle of that correction
    bool withinTolerance = false;
};

Hardware selectedHardware();
Camera nominalCamera(const Hardware& hw);
// Steger smoothing sigma, in pixels, for the selected line width and standoff.
double stripeSigmaPx(const Hardware& hw);

Vec3 add(Vec3 a, Vec3 b);
Vec3 sub(Vec3 a, Vec3 b);
Vec3 scale(Vec3 a, double s);
double dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
double norm(Vec3 a);
Vec3 normalized(Vec3 a);

Mat3 matMul(const Mat3& a, const Mat3& b);
Vec3 matVec(const Mat3& a, Vec3 v);
Mat3 transpose(const Mat3& a);
double det(const Mat3& a);
Mat3 inverse(const Mat3& a);

Pose poseMul(const Pose& a, const Pose& b);
Pose poseInverse(const Pose& a);
Vec3 transformPoint(const Pose& a, Vec3 p);

// R = Rz(psi) Ry(theta) Rx(phi). Angles in radians.
Mat3 eulerMatrix(double phiX, double thetaY, double psiZ);
void eulerAngles(const Mat3& r, double& phiX, double& thetaY, double& psiZ);
double rotationAngleDeg(const Mat3& r);

Mat3 kukaMatrix(double aDeg, double bDeg, double cDeg);
void kukaAngles(const Mat3& r, double& aDeg, double& bDeg, double& cDeg);
Pose robotPoseToFrame(const RobotPose& p);
RobotPose frameToRobotPose(const Pose& f);

void distortNormalized(const Camera& cam, double x, double y, double& xd, double& yd);
// Inverse of distortNormalized. Returns false if the Newton iteration diverges.
bool undistortNormalized(const Camera& cam, double xd, double yd, double& x, double& y);
// Ideal normalized coordinates of a (possibly distorted) pixel.
bool pixelToNormalized(const Camera& cam, double u, double v, double& x, double& y);
void project(const Camera& cam, Vec3 pc, double& u, double& v);

// Patent step 1, light-plane point. (u, v) is a stripe center on the
// chessboard. Hc takes board coordinates into the camera, Zw = 0 on the
// board. Solves
//   [Xw Yw s]^T = [r1 r2 -ray]^{-1} (-T)
// and returns the camera point s * ray. ray = M^{-1} [u v 1]^T after
// the pixel has been undistorted.
bool liftStripeOnBoard(const Camera& cam, const Pose& boardInCamera,
    double u, double v, Vec3& cameraPoint);

bool fitLightPlane(const std::vector<Vec3>& points, LightPlane& plane);

// Ray through an undistorted pixel, intersected with a calibrated light plane.
bool triangulate(const Camera& cam, const LightPlane& plane,
    double u, double v, Vec3& cameraPoint);

// Patent step 3. image is row-major, width * height, values in the same
// units as minIntensity (0..1 or 0..255). sigma is the Gaussian standard
// deviation in pixels; use stripeSigmaPx for the selected laser.
// A center is kept when the subpixel step lands inside the pixel and the
// second derivative along the normal is below -ridgeThreshold (bright ridge).
void extractStripeCenters(const float* image, int width, int height,
    double sigma, double ridgeThreshold, float minIntensity,
    std::vector<StripeCenter>& centers);

// Split centers into the two arms of the cross and intersect each arm with
// its light plane. planeOfArm[k] is 0 or 1.
struct Reconstruction {
    bool ok = false;
    std::vector<Vec3> cameraPoints;
    std::vector<int> planeOfArm;
};

Reconstruction reconstructCross(const float* image, int width, int height,
    const Camera& camera, const LightPlane planes[2],
    double sigma, double nominalZMm,
    double ridgeThreshold, float minIntensity);

// Patent step 2.
// Hc[i] is the chessboard in the camera at pose i (calibrateCamera extrinsic).
// Hg[i] is the tool in the base at the same pose (KUKA XYZABC).
// On success T_tool_sensor maps a camera point into the tool frame:
//   p_tool = T_tool_sensor * p_camera
//   p_base = T_base_tool * p_tool
// The claim builds A = Hc_j Hc_i^{-1}, B = Hg_j^{-1} Hg_i and writes A X = X B.
// With those two definitions the sensor-to-tool matrix is the X that satisfies
// B X = X A. solveAxEqualsXb is the printed equation; sensorToTool returns
// the matrix the point-cloud sentence uses.
bool solveAxEqualsXb(const std::vector<Pose>& a, const std::vector<Pose>& b, Pose& x);
bool sensorToTool(const std::vector<Pose>& boardInCamera,
    const std::vector<Pose>& toolInBase, Pose& tToolSensor);

// Patent step 4. Paired clouds, same length. Finds R, t minimizing
// sum ||R p_i + t - q_i||^2 by centering and the SVD of sum p' q'^T.
// The specification's expanded sum mixes centered and raw points; the
// stationary point of the problem it states just above that sum is this one,
// and it gives the T = q_bar - R p_bar written in the next sentence.
bool absoluteOrientation(const std::vector<Vec3>& reference,
    const std::vector<Vec3>& current, Mat3& rotation, Vec3& translation);

// Turn two unstructured crosses into index-aligned samples.
bool pairCrosses(const std::vector<Vec3>& reference,
    const std::vector<Vec3>& current,
    std::vector<Vec3>& pairedReference,
    std::vector<Vec3>& pairedCurrent);

// One leveling iteration.
// Both crosses are in the tool frame, each transformed with the robot pose
// at the moment it was captured (see basePointsToTool). Comparing them in
// the base leaves a residual after the plate has been followed, because the
// cross itself moved; in the tool frame the residual goes to zero when the
// torch is square to the plate.
// R maps the reference cross onto the current cross. The tool has to rotate
// with that change, which is the right multiply
//   R_base_tool_new = R_base_tool * R
// XYZ is unchanged. Command 4 takes the absolute target pose.
// phi, theta, psi are the Euler angles of R (tool frame). withinTolerance
// is true when the axis-angle of R is below toleranceDeg.
LevelCommand levelOnce(const std::vector<Vec3>& referenceTool,
    const std::vector<Vec3>& currentTool,
    const RobotPose& current,
    double toleranceDeg);

std::vector<Vec3> basePointsToTool(const std::vector<Vec3>& base,
    const RobotPose& toolInBase);

Vec3 cameraPointToBase(Vec3 cameraPoint, const Pose& toolFromSensor,
    const RobotPose& toolInBase);

} // namespace tubeplate

#endif
