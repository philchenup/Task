#include "plane_level.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tubeplate {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp01(double v)
{
    if (v < -1.0) return -1.0;
    if (v > 1.0) return 1.0;
    return v;
}

Mat3 rx(double a)
{
    const double c = std::cos(a);
    const double s = std::sin(a);
    Mat3 r;
    r.m[0] = 1; r.m[1] = 0; r.m[2] = 0;
    r.m[3] = 0; r.m[4] = c; r.m[5] = -s;
    r.m[6] = 0; r.m[7] = s; r.m[8] = c;
    return r;
}

Mat3 ry(double a)
{
    const double c = std::cos(a);
    const double s = std::sin(a);
    Mat3 r;
    r.m[0] = c; r.m[1] = 0; r.m[2] = s;
    r.m[3] = 0; r.m[4] = 1; r.m[5] = 0;
    r.m[6] = -s; r.m[7] = 0; r.m[8] = c;
    return r;
}

Mat3 rz(double a)
{
    const double c = std::cos(a);
    const double s = std::sin(a);
    Mat3 r;
    r.m[0] = c; r.m[1] = -s; r.m[2] = 0;
    r.m[3] = s; r.m[4] = c; r.m[5] = 0;
    r.m[6] = 0; r.m[7] = 0; r.m[8] = 1;
    return r;
}

// Symmetric 3x3 eigenproblem. A is overwritten by its diagonal.
// Columns of V are the eigenvectors.
void jacobiEigen(double a[9], double v[9])
{
    for (int i = 0; i < 9; ++i) v[i] = 0;
    v[0] = v[4] = v[8] = 1;
    for (int iter = 0; iter < 24; ++iter) {
        int p = 0;
        int q = 1;
        double best = std::fabs(a[1]);
        if (std::fabs(a[2]) > best) { best = std::fabs(a[2]); p = 0; q = 2; }
        if (std::fabs(a[5]) > best) { best = std::fabs(a[5]); p = 1; q = 2; }
        if (best < 1e-15) break;

        const double app = a[p * 3 + p];
        const double aqq = a[q * 3 + q];
        const double apq = a[p * 3 + q];
        const double tau = (aqq - app) / (2.0 * apq);
        const double sign = tau >= 0 ? 1.0 : -1.0;
        const double t = sign / (std::fabs(tau) + std::sqrt(1.0 + tau * tau));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double s = t * c;

        a[p * 3 + p] = app - t * apq;
        a[q * 3 + q] = aqq + t * apq;
        a[p * 3 + q] = a[q * 3 + p] = 0;

        const int r = 3 - p - q;
        const double arp = a[r * 3 + p];
        const double arq = a[r * 3 + q];
        a[r * 3 + p] = a[p * 3 + r] = c * arp - s * arq;
        a[r * 3 + q] = a[q * 3 + r] = s * arp + c * arq;

        for (int row = 0; row < 3; ++row) {
            const double vp = v[row * 3 + p];
            const double vq = v[row * 3 + q];
            v[row * 3 + p] = c * vp - s * vq;
            v[row * 3 + q] = s * vp + c * vq;
        }
    }
}

Vec3 column(const double v[9], int c)
{
    return Vec3{v[c], v[3 + c], v[6 + c]};
}

void setColumn(Mat3& m, int c, Vec3 col)
{
    m.m[c] = col.x;
    m.m[3 + c] = col.y;
    m.m[6 + c] = col.z;
}

Vec3 column(const Mat3& m, int c)
{
    return Vec3{m.m[c], m.m[3 + c], m.m[6 + c]};
}

bool svd(const Mat3& h, Mat3& u, double s[3], Mat3& v)
{
    double g[9] = {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += h.m[k * 3 + r] * h.m[k * 3 + c];
            g[r * 3 + c] = sum;
        }
    }
    double ev[9];
    jacobiEigen(g, ev);
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int i, int j) {
        return g[i * 3 + i] > g[j * 3 + j];
    });

    for (int k = 0; k < 3; ++k) {
        const double eval = g[order[k] * 3 + order[k]];
        s[k] = eval > 0 ? std::sqrt(eval) : 0;
        setColumn(v, k, column(ev, order[k]));
    }

    // A cross is planar, so the third singular value is numerical noise.
    // Rebuild that axis from the cross product instead of dividing by it.
    const double floor = std::max(1e-9, 1e-8 * s[0]);
    int ranked = 0;
    for (int k = 0; k < 3; ++k) {
        if (s[k] > floor) {
            setColumn(u, k, normalized(matVec(h, column(v, k))));
            ++ranked;
        } else {
            s[k] = 0;
            setColumn(u, k, Vec3{});
        }
    }
    if (ranked == 0) return false;
    if (ranked == 1) {
        const Vec3 a = column(u, 0);
        const Vec3 tmp = std::fabs(a.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        const Vec3 b = normalized(cross(a, tmp));
        setColumn(u, 1, b);
        setColumn(u, 2, cross(a, b));
    } else {
        const Vec3 a = column(u, 0);
        Vec3 b = column(u, 1);
        b = normalized(sub(b, scale(a, dot(a, b))));
        setColumn(u, 1, b);
        setColumn(u, 2, cross(a, b));
    }
    return true;
}

bool solve3(double a[3][4], double x[3])
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col])) pivot = row;
        }
        if (std::fabs(a[pivot][col]) < 1e-14) return false;
        if (pivot != col) {
            for (int k = col; k < 4; ++k) std::swap(a[col][k], a[pivot][k]);
        }
        const double div = a[col][col];
        for (int k = col; k < 4; ++k) a[col][k] /= div;
        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double f = a[row][col];
            for (int k = col; k < 4; ++k) a[row][k] -= f * a[col][k];
        }
    }
    x[0] = a[0][3];
    x[1] = a[1][3];
    x[2] = a[2][3];
    return true;
}

Vec3 rodrigues(const Mat3& r)
{
    const double c = clamp01((r.m[0] + r.m[4] + r.m[8] - 1.0) * 0.5);
    const double angle = std::acos(c);
    if (angle < 1e-12) return Vec3{};
    const double s = std::sin(angle);
    // (R - R^T) / (2 sin) is the unit axis. Scale by the angle.
    const double scaleAxis = angle / (2.0 * s);
    return Vec3{
        (r.m[7] - r.m[5]) * scaleAxis,
        (r.m[2] - r.m[6]) * scaleAxis,
        (r.m[3] - r.m[1]) * scaleAxis
    };
}

void convAxis(const float* in, float* out, int width, int height,
    const double* kernel, int radius, bool horizontal)
{
    const int n = width * height;
    for (int i = 0; i < n; ++i) out[i] = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double sum = 0;
            for (int k = -radius; k <= radius; ++k) {
                int xx = x;
                int yy = y;
                // Convolution, not correlation: odd kernels (first derivatives)
                // must sample I(x - k) or the subpixel step walks away from the ridge.
                if (horizontal) {
                    xx = x - k;
                    if (xx < 0) xx = 0;
                    if (xx >= width) xx = width - 1;
                } else {
                    yy = y - k;
                    if (yy < 0) yy = 0;
                    if (yy >= height) yy = height - 1;
                }
                sum += kernel[k + radius] * in[yy * width + xx];
            }
            out[y * width + x] = static_cast<float>(sum);
        }
    }
}

void gaussianKernels(double sigma, std::vector<double>& g,
    std::vector<double>& gx, std::vector<double>& gxx, int& radius)
{
    radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
    const int n = 2 * radius + 1;
    g.resize(n);
    gx.resize(n);
    gxx.resize(n);
    const double s2 = sigma * sigma;
    const double s4 = s2 * s2;
    double sum = 0;
    for (int i = -radius; i <= radius; ++i) {
        const double e = std::exp(-0.5 * i * i / s2);
        g[i + radius] = e;
        sum += e;
    }
    for (int i = -radius; i <= radius; ++i) {
        const double e = g[i + radius] / sum;
        g[i + radius] = e;
        gx[i + radius] = -i / s2 * e;
        gxx[i + radius] = (i * i / s4 - 1.0 / s2) * e;
    }
}

struct Arm {
    std::vector<Vec3> points;
    Vec3 origin;
    Vec3 direction;
};

bool splitArms(const std::vector<Vec3>& points, Arm arms[2])
{
    if (points.size() < 8) return false;
    Vec3 c{};
    for (Vec3 p : points) c = add(c, p);
    c = scale(c, 1.0 / static_cast<double>(points.size()));

    double cov[9] = {};
    for (Vec3 p : points) {
        const Vec3 d = sub(p, c);
        cov[0] += d.x * d.x; cov[1] += d.x * d.y; cov[2] += d.x * d.z;
        cov[3] += d.y * d.x; cov[4] += d.y * d.y; cov[5] += d.y * d.z;
        cov[6] += d.z * d.x; cov[7] += d.z * d.y; cov[8] += d.z * d.z;
    }
    double ev[9];
    jacobiEigen(cov, ev);
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int i, int j) {
        return cov[i * 3 + i] > cov[j * 3 + j];
    });
    // Smallest eigenvector is the plane normal. The other two span the cross.
    const Vec3 e1 = normalized(column(ev, order[0]));
    const Vec3 e2 = normalized(column(ev, order[1]));

    std::vector<double> ang(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec3 d = sub(points[i], c);
        double a = std::atan2(dot(d, e2), dot(d, e1));
        if (a < 0) a += kPi; // undirected line
        ang[i] = a;
    }
    double mean[2] = {ang[0], ang[0] + kPi * 0.5};
    if (mean[1] >= kPi) mean[1] -= kPi;
    std::vector<int> label(points.size(), 0);
    for (int iter = 0; iter < 16; ++iter) {
        double sum[2] = {};
        double wsum[2] = {};
        for (size_t i = 0; i < points.size(); ++i) {
            auto dist = [](double a, double b) {
                double d = std::fabs(a - b);
                if (d > kPi * 0.5) d = kPi - d;
                return d;
            };
            label[i] = dist(ang[i], mean[0]) <= dist(ang[i], mean[1]) ? 0 : 1;
            // Circular mean on an undirected angle, via the double angle.
            const double th = 2.0 * ang[i];
            sum[label[i]] += std::sin(th);
            wsum[label[i]] += std::cos(th);
        }
        for (int k = 0; k < 2; ++k) {
            if (std::fabs(sum[k]) + std::fabs(wsum[k]) < 1e-12) return false;
            mean[k] = 0.5 * std::atan2(sum[k], wsum[k]);
            if (mean[k] < 0) mean[k] += kPi;
        }
    }

    for (int k = 0; k < 2; ++k) {
        arms[k].points.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            if (label[i] == k) arms[k].points.push_back(points[i]);
        }
        if (arms[k].points.size() < 4) return false;
        Vec3 o{};
        for (Vec3 p : arms[k].points) o = add(o, p);
        o = scale(o, 1.0 / static_cast<double>(arms[k].points.size()));
        double ac[9] = {};
        for (Vec3 p : arms[k].points) {
            const Vec3 d = sub(p, o);
            ac[0] += d.x * d.x; ac[1] += d.x * d.y; ac[2] += d.x * d.z;
            ac[3] += d.y * d.x; ac[4] += d.y * d.y; ac[5] += d.y * d.z;
            ac[6] += d.z * d.x; ac[7] += d.z * d.y; ac[8] += d.z * d.z;
        }
        double aev[9];
        jacobiEigen(ac, aev);
        int which = 0;
        if (ac[4] > ac[0]) which = 1;
        if (ac[8] > ac[which * 3 + which]) which = 2;
        Vec3 dir = normalized(column(aev, which));
            // Largest component positive, so a symmetric arm does not flip on noise.
        if ((std::fabs(dir.x) >= std::fabs(dir.y) && std::fabs(dir.x) >= std::fabs(dir.z) && dir.x < 0)
            || (std::fabs(dir.y) > std::fabs(dir.x) && std::fabs(dir.y) >= std::fabs(dir.z) && dir.y < 0)
            || (std::fabs(dir.z) > std::fabs(dir.x) && std::fabs(dir.z) > std::fabs(dir.y) && dir.z < 0))
            dir = scale(dir, -1);
        arms[k].origin = o;
        arms[k].direction = dir;
    }
    return true;
}

void sampleArm(const Arm& arm, double halfLen, int count, std::vector<Vec3>& out)
{
    if (count < 2) return;
    for (int i = 0; i < count; ++i) {
        const double s = -halfLen + (2.0 * halfLen) * i / (count - 1);
        out.push_back(add(arm.origin, scale(arm.direction, s)));
    }
}

double halfLength(const Arm& arm)
{
    double m = 0;
    for (Vec3 p : arm.points) {
        const double s = std::fabs(dot(sub(p, arm.origin), arm.direction));
        if (s > m) m = s;
    }
    return m;
}

struct ImageArm {
    std::vector<StripeCenter> pts;
    double tx = 1;
    double ty = 0;
};

bool clusterImageArms(const std::vector<StripeCenter>& centers, ImageArm arms[2])
{
    if (centers.size() < 8) return false;
    std::vector<double> ang(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        // Tangent is perpendicular to the Hessian normal. Undirected.
        double a = std::atan2(-centers[i].nx, centers[i].ny);
        if (a < 0) a += kPi;
        ang[i] = a;
    }
    double mean[2] = {ang[0], ang[0] + 0.5 * kPi};
    if (mean[1] >= kPi) mean[1] -= kPi;
    std::vector<int> label(centers.size(), 0);
    for (int iter = 0; iter < 16; ++iter) {
        double ss[2] = {};
        double cc[2] = {};
        for (size_t i = 0; i < centers.size(); ++i) {
            auto dist = [](double a, double b) {
                double d = std::fabs(a - b);
                if (d > 0.5 * kPi) d = kPi - d;
                return d;
            };
            label[i] = dist(ang[i], mean[0]) <= dist(ang[i], mean[1]) ? 0 : 1;
            ss[label[i]] += std::sin(2.0 * ang[i]);
            cc[label[i]] += std::cos(2.0 * ang[i]);
        }
        for (int k = 0; k < 2; ++k) {
            if (std::fabs(ss[k]) + std::fabs(cc[k]) < 1e-12) return false;
            mean[k] = 0.5 * std::atan2(ss[k], cc[k]);
            if (mean[k] < 0) mean[k] += kPi;
        }
    }
    for (int k = 0; k < 2; ++k) {
        arms[k].pts.clear();
        for (size_t i = 0; i < centers.size(); ++i) {
            if (label[i] == k) arms[k].pts.push_back(centers[i]);
        }
        if (arms[k].pts.size() < 4) return false;
        arms[k].tx = std::cos(mean[k]);
        arms[k].ty = std::sin(mean[k]);
    }
    return true;
}

bool planeImageTangent(const Camera& cam, const LightPlane& plane, double z,
    double& tx, double& ty)
{
    const double rhs = -plane.c * z - plane.d;
    double x = 0;
    double y = 0;
    if (std::fabs(plane.a) >= std::fabs(plane.b)) {
        if (std::fabs(plane.a) < 1e-12) return false;
        x = rhs / plane.a;
    } else {
        y = rhs / plane.b;
    }
    // (-b, a, 0) lies in the light plane and in Z = const.
    const Vec3 p1{x, y, z};
    const Vec3 p2{x - plane.b, y + plane.a, z};
    double u1, v1, u2, v2;
    project(cam, p1, u1, v1);
    project(cam, p2, u2, v2);
    const double du = u2 - u1;
    const double dv = v2 - v1;
    const double n = std::hypot(du, dv);
    if (n < 1e-9) return false;
    tx = du / n;
    ty = dv / n;
    return true;
}

} // namespace

Hardware selectedHardware()
{
    return Hardware{};
}

Camera nominalCamera(const Hardware& hw)
{
    Camera cam;
    cam.fx = hw.focalLengthMm / hw.pixelSizeMm;
    cam.fy = cam.fx;
    cam.cx = (hw.width - 1) * 0.5;
    cam.cy = (hw.height - 1) * 0.5;
    cam.skew = 0;
    cam.width = hw.width;
    cam.height = hw.height;
    return cam;
}

double stripeSigmaPx(const Hardware& hw)
{
    const double mmPerPx = hw.standoffMm * hw.pixelSizeMm / hw.focalLengthMm;
    const double widthPx = hw.lineWidthMm / mmPerPx;
    return widthPx / std::sqrt(3.0);
}

Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 normalized(Vec3 a)
{
    const double n = norm(a);
    if (n < 1e-15) return Vec3{};
    return scale(a, 1.0 / n);
}

Mat3 matMul(const Mat3& a, const Mat3& b)
{
    Mat3 c;
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
            c.m[r * 3 + col] =
                a.m[r * 3 + 0] * b.m[col] +
                a.m[r * 3 + 1] * b.m[3 + col] +
                a.m[r * 3 + 2] * b.m[6 + col];
        }
    }
    return c;
}

Vec3 matVec(const Mat3& a, Vec3 v)
{
    return {
        a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z,
        a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z,
        a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z
    };
}

Mat3 transpose(const Mat3& a)
{
    Mat3 b;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            b.m[r * 3 + c] = a.m[c * 3 + r];
    return b;
}

double det(const Mat3& a)
{
    return a.m[0] * (a.m[4] * a.m[8] - a.m[5] * a.m[7])
        - a.m[1] * (a.m[3] * a.m[8] - a.m[5] * a.m[6])
        + a.m[2] * (a.m[3] * a.m[7] - a.m[4] * a.m[6]);
}

Mat3 inverse(const Mat3& a)
{
    const double d = det(a);
    Mat3 b;
    if (std::fabs(d) < 1e-15) return b;
    const double inv = 1.0 / d;
    b.m[0] = (a.m[4] * a.m[8] - a.m[5] * a.m[7]) * inv;
    b.m[1] = (a.m[2] * a.m[7] - a.m[1] * a.m[8]) * inv;
    b.m[2] = (a.m[1] * a.m[5] - a.m[2] * a.m[4]) * inv;
    b.m[3] = (a.m[5] * a.m[6] - a.m[3] * a.m[8]) * inv;
    b.m[4] = (a.m[0] * a.m[8] - a.m[2] * a.m[6]) * inv;
    b.m[5] = (a.m[2] * a.m[3] - a.m[0] * a.m[5]) * inv;
    b.m[6] = (a.m[3] * a.m[7] - a.m[4] * a.m[6]) * inv;
    b.m[7] = (a.m[1] * a.m[6] - a.m[0] * a.m[7]) * inv;
    b.m[8] = (a.m[0] * a.m[4] - a.m[1] * a.m[3]) * inv;
    return b;
}

Pose poseMul(const Pose& a, const Pose& b)
{
    Pose c;
    c.R = matMul(a.R, b.R);
    c.t = add(matVec(a.R, b.t), a.t);
    return c;
}

Pose poseInverse(const Pose& a)
{
    Pose b;
    b.R = transpose(a.R);
    b.t = matVec(b.R, scale(a.t, -1));
    return b;
}

Vec3 transformPoint(const Pose& a, Vec3 p)
{
    return add(matVec(a.R, p), a.t);
}

Mat3 eulerMatrix(double phiX, double thetaY, double psiZ)
{
    return matMul(matMul(rz(psiZ), ry(thetaY)), rx(phiX));
}

void eulerAngles(const Mat3& r, double& phiX, double& thetaY, double& psiZ)
{
    const double cy = std::hypot(r.m[7], r.m[8]);
    thetaY = std::atan2(-r.m[6], cy);
    if (cy > 1e-8) {
        phiX = std::atan2(r.m[7], r.m[8]);
        psiZ = std::atan2(r.m[3], r.m[0]);
    } else {
        phiX = 0;
        psiZ = std::atan2(-r.m[1], r.m[4]);
    }
}

double rotationAngleDeg(const Mat3& r)
{
    const double c = clamp01((r.m[0] + r.m[4] + r.m[8] - 1.0) * 0.5);
    return std::acos(c) * 180.0 / kPi;
}

Mat3 kukaMatrix(double aDeg, double bDeg, double cDeg)
{
    const double d = kPi / 180.0;
    return eulerMatrix(cDeg * d, bDeg * d, aDeg * d);
}

void kukaAngles(const Mat3& r, double& aDeg, double& bDeg, double& cDeg)
{
    double phi, theta, psi;
    eulerAngles(r, phi, theta, psi);
    const double d = 180.0 / kPi;
    cDeg = phi * d;
    bDeg = theta * d;
    aDeg = psi * d;
}

Pose robotPoseToFrame(const RobotPose& p)
{
    Pose f;
    f.R = kukaMatrix(p.a, p.b, p.c);
    f.t = {p.x, p.y, p.z};
    return f;
}

RobotPose frameToRobotPose(const Pose& f)
{
    RobotPose p;
    p.x = f.t.x;
    p.y = f.t.y;
    p.z = f.t.z;
    kukaAngles(f.R, p.a, p.b, p.c);
    return p;
}

void distortNormalized(const Camera& cam, double x, double y, double& xd, double& yd)
{
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
    xd = x * radial + 2.0 * cam.p1 * x * y + cam.p2 * (r2 + 2.0 * x * x);
    yd = y * radial + cam.p1 * (r2 + 2.0 * y * y) + 2.0 * cam.p2 * x * y;
}

bool undistortNormalized(const Camera& cam, double xd, double yd, double& x, double& y)
{
    x = xd;
    y = yd;
    if (cam.k1 == 0 && cam.k2 == 0 && cam.k3 == 0 && cam.p1 == 0 && cam.p2 == 0)
        return true;
    for (int iter = 0; iter < 15; ++iter) {
        double xh, yh;
        distortNormalized(cam, x, y, xh, yh);
        const double ex = xh - xd;
        const double ey = yh - yd;
        if (ex * ex + ey * ey < 1e-24) return true;
        const double eps = 1e-6;
        double xhx, yhx, xhy, yhy;
        distortNormalized(cam, x + eps, y, xhx, yhx);
        distortNormalized(cam, x, y + eps, xhy, yhy);
        const double j00 = (xhx - xh) / eps;
        const double j10 = (yhx - yh) / eps;
        const double j01 = (xhy - xh) / eps;
        const double j11 = (yhy - yh) / eps;
        const double detj = j00 * j11 - j01 * j10;
        if (std::fabs(detj) < 1e-14) return false;
        x -= (j11 * ex - j01 * ey) / detj;
        y -= (-j10 * ex + j00 * ey) / detj;
    }
    double xh, yh;
    distortNormalized(cam, x, y, xh, yh);
    return (xh - xd) * (xh - xd) + (yh - yd) * (yh - yd) < 1e-16;
}

bool pixelToNormalized(const Camera& cam, double u, double v, double& x, double& y)
{
    if (cam.fx == 0 || cam.fy == 0) return false;
    const double yd = (v - cam.cy) / cam.fy;
    const double xd = (u - cam.cx - cam.skew * yd) / cam.fx;
    return undistortNormalized(cam, xd, yd, x, y);
}

void project(const Camera& cam, Vec3 pc, double& u, double& v)
{
    const double z = pc.z == 0 ? 1e-15 : pc.z;
    double xd, yd;
    distortNormalized(cam, pc.x / z, pc.y / z, xd, yd);
    u = cam.fx * xd + cam.skew * yd + cam.cx;
    v = cam.fy * yd + cam.cy;
}

bool liftStripeOnBoard(const Camera& cam, const Pose& boardInCamera,
    double u, double v, Vec3& cameraPoint)
{
    double x, y;
    if (!pixelToNormalized(cam, u, v, x, y)) return false;
    const Vec3 r1 = column(boardInCamera.R, 0);
    const Vec3 r2 = column(boardInCamera.R, 1);
    const Vec3 t = boardInCamera.t;
    double aug[3][4] = {
        {r1.x, r2.x, -x, -t.x},
        {r1.y, r2.y, -y, -t.y},
        {r1.z, r2.z, -1.0, -t.z}
    };
    double sol[3];
    if (!solve3(aug, sol)) return false;
    const double s = sol[2];
    if (s <= 0) return false;
    cameraPoint = Vec3{s * x, s * y, s};
    return true;
}

bool fitLightPlane(const std::vector<Vec3>& points, LightPlane& plane)
{
    if (points.size() < 3) return false;
    Vec3 c{};
    for (Vec3 p : points) c = add(c, p);
    c = scale(c, 1.0 / static_cast<double>(points.size()));
    double cov[9] = {};
    for (Vec3 p : points) {
        const Vec3 d = sub(p, c);
        cov[0] += d.x * d.x; cov[1] += d.x * d.y; cov[2] += d.x * d.z;
        cov[3] += d.y * d.x; cov[4] += d.y * d.y; cov[5] += d.y * d.z;
        cov[6] += d.z * d.x; cov[7] += d.z * d.y; cov[8] += d.z * d.z;
    }
    double ev[9];
    jacobiEigen(cov, ev);
    int which = 0;
    if (cov[4] < cov[which * 3 + which]) which = 1;
    if (cov[8] < cov[which * 3 + which]) which = 2;
    Vec3 n = normalized(column(ev, which));
    if (n.z < 0) n = scale(n, -1);
    plane.a = n.x;
    plane.b = n.y;
    plane.c = n.z;
    plane.d = -dot(n, c);
    return norm(n) > 0.5;
}

bool triangulate(const Camera& cam, const LightPlane& plane,
    double u, double v, Vec3& cameraPoint)
{
    double x, y;
    if (!pixelToNormalized(cam, u, v, x, y)) return false;
    const double denom = plane.a * x + plane.b * y + plane.c;
    if (std::fabs(denom) < 1e-12) return false;
    const double s = -plane.d / denom;
    if (s <= 0) return false;
    cameraPoint = {s * x, s * y, s};
    return true;
}

void extractStripeCenters(const float* image, int width, int height,
    double sigma, double ridgeThreshold, float minIntensity,
    std::vector<StripeCenter>& centers)
{
    centers.clear();
    if (!image || width < 3 || height < 3 || sigma < 0.4) return;
    std::vector<double> g, gx, gxx;
    int radius = 0;
    gaussianKernels(sigma, g, gx, gxx, radius);

    const int n = width * height;
    std::vector<float> tmp(n), rx(n), ry(n), rxx(n), rxy(n), ryy(n);
    std::vector<float> smooth(n);

    convAxis(image, tmp.data(), width, height, gx.data(), radius, true);
    convAxis(tmp.data(), rx.data(), width, height, g.data(), radius, false);
    convAxis(image, tmp.data(), width, height, g.data(), radius, true);
    convAxis(tmp.data(), ry.data(), width, height, gx.data(), radius, false);
    convAxis(tmp.data(), smooth.data(), width, height, g.data(), radius, false);

    convAxis(image, tmp.data(), width, height, gxx.data(), radius, true);
    convAxis(tmp.data(), rxx.data(), width, height, g.data(), radius, false);
    convAxis(image, tmp.data(), width, height, g.data(), radius, true);
    convAxis(tmp.data(), ryy.data(), width, height, gxx.data(), radius, false);
    convAxis(image, tmp.data(), width, height, gx.data(), radius, true);
    convAxis(tmp.data(), rxy.data(), width, height, gx.data(), radius, false);

    centers.reserve(static_cast<size_t>(std::max(width, height)) * 4);
    for (int y = radius; y < height - radius; ++y) {
        for (int x = radius; x < width - radius; ++x) {
            const int i = y * width + x;
            if (smooth[i] < minIntensity) continue;
            const double a = rxx[i];
            const double b = rxy[i];
            const double c = ryy[i];
            const double tr = a + c;
            const double disc2 = (a - c) * (a - c) + 4.0 * b * b;
            const double disc = std::sqrt(std::max(0.0, disc2));
            const double lMax = 0.5 * (tr + disc);
            const double lMin = 0.5 * (tr - disc);
            const double lambda = std::fabs(lMax) >= std::fabs(lMin) ? lMax : lMin;
            // Bright ridge: curvature along the normal is negative.
            if (lambda >= -ridgeThreshold) continue;

            double nx, ny;
            if (std::fabs(b) > 1e-12) {
                nx = b;
                ny = lambda - a;
            } else if (std::fabs(a) >= std::fabs(c)) {
                nx = 1;
                ny = 0;
            } else {
                nx = 0;
                ny = 1;
            }
            const double nn = std::hypot(nx, ny);
            if (nn < 1e-15) continue;
            nx /= nn;
            ny /= nn;

            const double denom =
                nx * nx * a + 2.0 * nx * ny * b + ny * ny * c;
            if (std::fabs(denom) < 1e-12) continue;
            const double t = -(nx * rx[i] + ny * ry[i]) / denom;
            const double du = t * nx;
            const double dv = t * ny;
            if (du < -0.5 || du > 0.5 || dv < -0.5 || dv > 0.5) continue;
            centers.push_back(StripeCenter{
                x + du, y + dv, nx, ny
            });
        }
    }
}

Reconstruction reconstructCross(const float* image, int width, int height,
    const Camera& camera, const LightPlane planes[2],
    double sigma, double nominalZMm,
    double ridgeThreshold, float minIntensity)
{
    Reconstruction out;
    std::vector<StripeCenter> centers;
    extractStripeCenters(image, width, height, sigma, ridgeThreshold, minIntensity, centers);
    ImageArm arms[2];
    if (!clusterImageArms(centers, arms)) return out;

    double tx[2], ty[2];
    if (!planeImageTangent(camera, planes[0], nominalZMm, tx[0], ty[0])) return out;
    if (!planeImageTangent(camera, planes[1], nominalZMm, tx[1], ty[1])) return out;

    int assign[2] = {-1, -1};
    bool used[2] = {false, false};
    for (int k = 0; k < 2; ++k) {
        const double d0 = std::fabs(arms[k].tx * tx[0] + arms[k].ty * ty[0]);
        const double d1 = std::fabs(arms[k].tx * tx[1] + arms[k].ty * ty[1]);
        const int id = d0 >= d1 ? 0 : 1;
        if (used[id]) return out;
        used[id] = true;
        assign[k] = id;
    }

    for (int k = 0; k < 2; ++k) {
        for (const StripeCenter& s : arms[k].pts) {
            Vec3 p;
            if (!triangulate(camera, planes[assign[k]], s.u, s.v, p)) continue;
            out.cameraPoints.push_back(p);
            out.planeOfArm.push_back(assign[k]);
        }
    }
    out.ok = out.cameraPoints.size() >= 8;
    return out;
}

bool absoluteOrientation(const std::vector<Vec3>& reference,
    const std::vector<Vec3>& current, Mat3& rotation, Vec3& translation)
{
    const size_t n = reference.size();
    if (n < 3 || current.size() != n) return false;
    Vec3 pc{}, qc{};
    for (size_t i = 0; i < n; ++i) {
        pc = add(pc, reference[i]);
        qc = add(qc, current[i]);
    }
    pc = scale(pc, 1.0 / static_cast<double>(n));
    qc = scale(qc, 1.0 / static_cast<double>(n));

    Mat3 h{};
    h.m[0] = h.m[4] = h.m[8] = 0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3 p = sub(reference[i], pc);
        const Vec3 q = sub(current[i], qc);
        h.m[0] += p.x * q.x; h.m[1] += p.x * q.y; h.m[2] += p.x * q.z;
        h.m[3] += p.y * q.x; h.m[4] += p.y * q.y; h.m[5] += p.y * q.z;
        h.m[6] += p.z * q.x; h.m[7] += p.z * q.y; h.m[8] += p.z * q.z;
    }
    // H = sum p q^T = U S V^T, R = V U^T maps p onto q.
    Mat3 u, v;
    double s[3];
    if (!svd(h, u, s, v)) return false;
    rotation = matMul(v, transpose(u));
    if (det(rotation) < 0) {
        setColumn(v, 2, scale(column(v, 2), -1));
        rotation = matMul(v, transpose(u));
    }
    translation = sub(qc, matVec(rotation, pc));
    return true;
}

bool pairCrosses(const std::vector<Vec3>& reference,
    const std::vector<Vec3>& current,
    std::vector<Vec3>& pairedReference,
    std::vector<Vec3>& pairedCurrent)
{
    Arm ref[2], cur[2];
    if (!splitArms(reference, ref) || !splitArms(current, cur)) return false;
    const double scoreKeep =
        std::fabs(dot(ref[0].direction, cur[0].direction)) +
        std::fabs(dot(ref[1].direction, cur[1].direction));
    const double scoreSwap =
        std::fabs(dot(ref[0].direction, cur[1].direction)) +
        std::fabs(dot(ref[1].direction, cur[0].direction));
    if (scoreSwap > scoreKeep) std::swap(cur[0], cur[1]);
    for (int k = 0; k < 2; ++k) {
        if (dot(ref[k].direction, cur[k].direction) < 0)
            cur[k].direction = scale(cur[k].direction, -1);
    }

    pairedReference.clear();
    pairedCurrent.clear();
    constexpr int kCount = 21;
    for (int k = 0; k < 2; ++k) {
        double len = std::min(halfLength(ref[k]), halfLength(cur[k]));
        len *= 0.85;
        if (len < 1.0) return false;
        sampleArm(ref[k], len, kCount, pairedReference);
        sampleArm(cur[k], len, kCount, pairedCurrent);
    }
    return pairedReference.size() == pairedCurrent.size();
}

bool solveAxEqualsXb(const std::vector<Pose>& a, const std::vector<Pose>& b, Pose& x)
{
    if (a.size() != b.size() || a.size() < 2) return false;
    std::vector<Vec3> beta;
    std::vector<Vec3> alpha;
    beta.reserve(a.size());
    alpha.reserve(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        const Vec3 av = rodrigues(a[i].R);
        const Vec3 bv = rodrigues(b[i].R);
        if (norm(av) < 1e-4 || norm(bv) < 1e-4) continue;
        alpha.push_back(av);
        beta.push_back(bv);
    }
    if (alpha.size() < 2) return false;
    // Ra rx = rx Rb implies log(Ra) = rx log(Rb).
    Mat3 rx;
    Vec3 dummy;
    if (!absoluteOrientation(beta, alpha, rx, dummy)) return false;

    double ata[9] = {};
    double atb[3] = {};
    for (size_t i = 0; i < a.size(); ++i) {
        const Mat3& ra = a[i].R;
        Mat3 m = ra;
        m.m[0] -= 1; m.m[4] -= 1; m.m[8] -= 1;
        const Vec3 rhs = sub(matVec(rx, b[i].t), a[i].t);
        const double rhsv[3] = {rhs.x, rhs.y, rhs.z};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                double sum = 0;
                for (int k = 0; k < 3; ++k) sum += m.m[k * 3 + r] * m.m[k * 3 + c];
                ata[r * 3 + c] += sum;
            }
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += m.m[k * 3 + r] * rhsv[k];
            atb[r] += sum;
        }
    }
    double aug[3][4] = {
        {ata[0], ata[1], ata[2], atb[0]},
        {ata[3], ata[4], ata[5], atb[1]},
        {ata[6], ata[7], ata[8], atb[2]}
    };
    double t[3];
    if (!solve3(aug, t)) return false;
    x.R = rx;
    x.t = {t[0], t[1], t[2]};
    return true;
}

bool sensorToTool(const std::vector<Pose>& boardInCamera,
    const std::vector<Pose>& toolInBase, Pose& tToolSensor)
{
    const size_t n = boardInCamera.size();
    if (n < 3 || toolInBase.size() != n) return false;
    std::vector<Pose> a;
    std::vector<Pose> b;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            // Claim: A = Hc_j * Hc_i^{-1}, B = Hg_j^{-1} * Hg_i.
            a.push_back(poseMul(boardInCamera[j], poseInverse(boardInCamera[i])));
            b.push_back(poseMul(poseInverse(toolInBase[j]), toolInBase[i]));
        }
    }
    // B X = X A, with X the sensor-to-tool matrix. See the header note.
    return solveAxEqualsXb(b, a, tToolSensor);
}

std::vector<Vec3> basePointsToTool(const std::vector<Vec3>& base,
    const RobotPose& toolInBase)
{
    const Pose tool = poseInverse(robotPoseToFrame(toolInBase));
    std::vector<Vec3> out;
    out.reserve(base.size());
    for (Vec3 p : base) out.push_back(transformPoint(tool, p));
    return out;
}

LevelCommand levelOnce(const std::vector<Vec3>& referenceTool,
    const std::vector<Vec3>& currentTool,
    const RobotPose& current,
    double toleranceDeg)
{
    LevelCommand cmd;
    cmd.target = current;
    std::vector<Vec3> pref, pcur;
    if (!pairCrosses(referenceTool, currentTool, pref, pcur)) return cmd;
    Mat3 rotation;
    Vec3 translation;
    if (!absoluteOrientation(pref, pcur, rotation, translation)) return cmd;
    (void)translation;

    const Pose now = robotPoseToFrame(current);
    Pose next = now;
    // R is the tool-frame rotation that takes the taught cross to the one
    // seen now. Right-multiplying rotates the tool with the plate.
    next.R = matMul(now.R, rotation);
    cmd.target = frameToRobotPose(next);
    cmd.target.x = current.x;
    cmd.target.y = current.y;
    cmd.target.z = current.z;

    double phi, theta, psi;
    eulerAngles(rotation, phi, theta, psi);
    const double d = 180.0 / kPi;
    cmd.phiDeg = phi * d;
    cmd.thetaDeg = theta * d;
    cmd.psiDeg = psi * d;
    cmd.tiltDeg = rotationAngleDeg(rotation);
    cmd.withinTolerance = cmd.tiltDeg < toleranceDeg;
    cmd.ok = true;
    return cmd;
}

Vec3 cameraPointToBase(Vec3 cameraPoint, const Pose& toolFromSensor,
    const RobotPose& toolInBase)
{
    const Vec3 inTool = transformPoint(toolFromSensor, cameraPoint);
    return transformPoint(robotPoseToFrame(toolInBase), inTool);
}

} // namespace tubeplate
