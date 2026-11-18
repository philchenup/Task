#include "plane_level/plane_level.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace plane_level {
namespace {

void gaussianDerivativeKernels(double sigma, cv::Mat& g0, cv::Mat& g1, cv::Mat& g2) {
    const int radius = std::max(1, static_cast<int>(std::ceil(4.0 * sigma)));
    const int n = 2 * radius + 1;
    g0.create(1, n, CV_64F);
    g1.create(1, n, CV_64F);
    g2.create(1, n, CV_64F);
    const double s2 = sigma * sigma;
    const double s4 = s2 * s2;
    double sum = 0;
    for (int i = -radius; i <= radius; ++i) {
        const double g = std::exp(-0.5 * static_cast<double>(i * i) / s2);
        g0.at<double>(0, i + radius) = g;
        sum += g;
    }
    for (int i = -radius; i <= radius; ++i) {
        const double g = g0.at<double>(0, i + radius) / sum;
        const double x = static_cast<double>(i);
        g0.at<double>(0, i + radius) = g;
        // sepFilter2D correlates. An odd derivative must be stored with the
        // opposite sign so correlation equals convolution with dG/dx.
        // The second derivative is even, so its kernel is stored as-is.
        g1.at<double>(0, i + radius) = x / s2 * g;
        g2.at<double>(0, i + radius) = (x * x / s4 - 1.0 / s2) * g;
    }
}

cv::Mat asGray64(const cv::Mat& gray) {
    cv::Mat out;
    if (gray.channels() == 3) cv::cvtColor(gray, out, cv::COLOR_BGR2GRAY);
    else gray.copyTo(out);
    out.convertTo(out, CV_64F);
    return out;
}

}

std::vector<StripePoint> extractStripeCenters(const cv::Mat& gray, const StegerParams& params) {
    if (gray.empty()) throw std::invalid_argument("empty stripe image");
    if (params.stripeWidthPx <= 0) throw std::invalid_argument("stripe width must be positive");
    const double sigmaLimit = params.stripeWidthPx / std::sqrt(3.0);
    double sigma = params.sigma;
    if (sigma <= 0) sigma = params.stripeWidthPx / (2.0 * std::sqrt(3.0));
    if (sigma >= sigmaLimit) {
        throw std::invalid_argument("Steger sigma must stay below omega / sqrt(3)");
    }
    const cv::Mat image = asGray64(gray);
    cv::Mat g0, g1, g2;
    gaussianDerivativeKernels(sigma, g0, g1, g2);
    cv::Mat rx, ry, rxx, ryy, rxy;
    cv::sepFilter2D(image, rx, CV_64F, g1, g0, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
    cv::sepFilter2D(image, ry, CV_64F, g0, g1, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
    cv::sepFilter2D(image, rxx, CV_64F, g2, g0, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
    cv::sepFilter2D(image, ryy, CV_64F, g0, g2, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
    cv::sepFilter2D(image, rxy, CV_64F, g1, g1, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);

    std::vector<StripePoint> centers;
    for (int y = 1; y < image.rows - 1; ++y) {
        for (int x = 1; x < image.cols - 1; ++x) {
            if (image.at<double>(y, x) < params.minIntensity) continue;
            const double a = rxx.at<double>(y, x);
            const double b = rxy.at<double>(y, x);
            const double c = ryy.at<double>(y, x);
            const double half = 0.5 * (a + c);
            const double disc = std::sqrt(std::max(0.0, half * half - (a * c - b * b)));
            const double lMax = half + disc;
            const double lMin = half - disc;
            const double lambda = std::abs(lMax) >= std::abs(lMin) ? lMax : lMin;
            // Bright ridge: curvature along the normal is negative.
            if (!(lambda < -params.strength)) continue;
            // (H - lambda I) n = 0. The cross-derivative form
            // n = (b, lambda - rxx) vanishes when the ridge is vertical:
            // rxy = 0 and lambda = rxx. Fall back to the axis that owns lambda.
            double nx = b;
            double ny = lambda - a;
            if (std::hypot(nx, ny) < 1e-12) {
                if (std::abs(a - lambda) <= std::abs(c - lambda)) {
                    nx = 1;
                    ny = 0;
                } else {
                    nx = 0;
                    ny = 1;
                }
            } else {
                const double nn = std::hypot(nx, ny);
                nx /= nn;
                ny /= nn;
            }
            const double grad = nx * rx.at<double>(y, x) + ny * ry.at<double>(y, x);
            // t = -(n · grad) / (n^T H n). For a unit eigenvector, n^T H n = lambda.
            const double t = -grad / lambda;
            if (std::abs(t * nx) > 0.5 || std::abs(t * ny) > 0.5) continue;
            StripePoint p;
            p.x = static_cast<double>(x) + t * nx;
            p.y = static_cast<double>(y) + t * ny;
            p.nx = nx;
            p.ny = ny;
            p.response = lambda;
            centers.push_back(p);
        }
    }
    return centers;
}

CrossSplit splitCross(const std::vector<StripePoint>& points) {
    CrossSplit split;
    if (points.size() < 4) return split;
    auto tangent = [](const StripePoint& p) {
        return cv::Vec2d(-p.ny, p.nx);
    };
    size_t seed0 = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].response < points[seed0].response) seed0 = i;
    }
    const cv::Vec2d t0 = tangent(points[seed0]);
    size_t seed1 = 0;
    double best = 1;
    for (size_t i = 0; i < points.size(); ++i) {
        const double aligned = std::abs(t0.dot(tangent(points[i])));
        if (aligned < best) {
            best = aligned;
            seed1 = i;
        }
    }
    cv::Vec2d m0 = t0;
    cv::Vec2d m1 = tangent(points[seed1]);
    std::vector<int> label(points.size(), 0);
    for (int iter = 0; iter < 4; ++iter) {
        cv::Vec2d s0(0, 0), s1(0, 0);
        for (size_t i = 0; i < points.size(); ++i) {
            cv::Vec2d t = tangent(points[i]);
            if (t.dot(m0) < 0) t = -t;
            const double a0 = std::abs(m0.dot(t));
            const double a1 = std::abs(m1.dot(t));
            if (a0 >= a1) {
                label[i] = 0;
                s0 += t;
            } else {
                label[i] = 1;
                if (t.dot(m1) < 0) t = -t;
                s1 += t;
            }
        }
        if (cv::norm(s0) > 1e-9) m0 = s0 * (1.0 / cv::norm(s0));
        if (cv::norm(s1) > 1e-9) m1 = s1 * (1.0 / cv::norm(s1));
    }
    for (size_t i = 0; i < points.size(); ++i) {
        if (label[i] == 0) split.arm0.push_back(points[i]);
        else split.arm1.push_back(points[i]);
    }
    return split;
}

}  // namespace plane_level
