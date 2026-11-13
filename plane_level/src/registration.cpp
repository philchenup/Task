#include "plane_level/plane_level.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace plane_level {
namespace {

cv::Vec3d meanOf(const std::vector<cv::Vec3d>& points) {
    cv::Vec3d c(0, 0, 0);
    for (const cv::Vec3d& p : points) c += p;
    return c * (1.0 / static_cast<double>(points.size()));
}

double rmsOf(const std::vector<cv::Vec3d>& source, const std::vector<cv::Vec3d>& target, const Rigid3d& xform) {
    double acc = 0;
    for (size_t i = 0; i < source.size(); ++i) {
        const cv::Vec3d e = target[i] - xform.apply(source[i]);
        acc += e.dot(e);
    }
    return std::sqrt(acc / static_cast<double>(source.size()));
}

std::vector<cv::Vec3d> reversedCopy(std::vector<cv::Vec3d> points) {
    std::reverse(points.begin(), points.end());
    return points;
}

}

AlignResult absoluteOrientation(const std::vector<cv::Vec3d>& source, const std::vector<cv::Vec3d>& target) {
    if (source.size() != target.size() || source.size() < 3) {
        throw std::invalid_argument("absolute orientation needs 3 or more paired points");
    }
    const cv::Vec3d ps = meanOf(source);
    const cv::Vec3d pt = meanOf(target);
    cv::Matx33d H = cv::Matx33d::zeros();
    for (size_t i = 0; i < source.size(); ++i) {
        H += (source[i] - ps) * (target[i] - pt).t();
    }
    cv::SVD svd(cv::Mat(H), cv::SVD::FULL_UV);
    cv::Matx33d U, Vt;
    svd.u.copyTo(U);
    svd.vt.copyTo(Vt);
    cv::Matx33d V = Vt.t();
    cv::Matx33d R = V * U.t();
    if (cv::determinant(cv::Mat(R)) < 0) {
        V(0, 2) = -V(0, 2);
        V(1, 2) = -V(1, 2);
        V(2, 2) = -V(2, 2);
        R = V * U.t();
    }
    AlignResult result;
    result.sourceToTarget = Rigid3d::fromRotationTranslation(R, pt - R * ps);
    result.rms = rmsOf(source, target, result.sourceToTarget);
    return result;
}

std::vector<cv::Vec3d> resampleByArcLength(const std::vector<cv::Vec3d>& points, int samples) {
    if (samples < 2) throw std::invalid_argument("resample needs at least 2 samples");
    if (points.size() < 2) return points;
    const cv::Vec3d c = meanOf(points);
    cv::Matx33d cov = cv::Matx33d::zeros();
    for (const cv::Vec3d& p : points) {
        const cv::Vec3d e = p - c;
        cov += e * e.t();
    }
    cv::SVD svd(cv::Mat(cov), cv::SVD::FULL_UV);
    cv::Vec3d axis(svd.u.at<double>(0, 0), svd.u.at<double>(1, 0), svd.u.at<double>(2, 0));
    std::vector<std::pair<double, cv::Vec3d>> ordered;
    ordered.reserve(points.size());
    for (const cv::Vec3d& p : points) ordered.emplace_back(axis.dot(p - c), p);
    std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<double> s(ordered.size(), 0);
    for (size_t i = 1; i < ordered.size(); ++i) {
        s[i] = s[i - 1] + cv::norm(ordered[i].second - ordered[i - 1].second);
    }
    const double length = s.back();
    std::vector<cv::Vec3d> out;
    out.reserve(static_cast<size_t>(samples));
    if (length < 1e-12) {
        out.assign(static_cast<size_t>(samples), ordered.front().second);
        return out;
    }
    for (int k = 0; k < samples; ++k) {
        const double target = length * static_cast<double>(k) / static_cast<double>(samples - 1);
        size_t hi = 1;
        while (hi + 1 < s.size() && s[hi] < target) ++hi;
        const double span = s[hi] - s[hi - 1];
        const double u = span < 1e-12 ? 0 : (target - s[hi - 1]) / span;
        out.push_back(ordered[hi - 1].second * (1.0 - u) + ordered[hi].second * u);
    }
    return out;
}

AlignResult alignCrosses(
    const std::vector<cv::Vec3d>& current0,
    const std::vector<cv::Vec3d>& current1,
    const std::vector<cv::Vec3d>& reference0,
    const std::vector<cv::Vec3d>& reference1,
    int samplesPerArm) {
    const std::vector<cv::Vec3d> c0 = resampleByArcLength(current0, samplesPerArm);
    const std::vector<cv::Vec3d> c1 = resampleByArcLength(current1, samplesPerArm);
    const std::vector<cv::Vec3d> r0 = resampleByArcLength(reference0, samplesPerArm);
    const std::vector<cv::Vec3d> r1 = resampleByArcLength(reference1, samplesPerArm);
    const std::vector<cv::Vec3d> c0f = reversedCopy(c0);
    const std::vector<cv::Vec3d> c1f = reversedCopy(c1);

    auto pack = [](const std::vector<cv::Vec3d>& a, const std::vector<cv::Vec3d>& b) {
        std::vector<cv::Vec3d> out = a;
        out.insert(out.end(), b.begin(), b.end());
        return out;
    };
    const std::vector<cv::Vec3d> refSame = pack(r0, r1);
    const std::vector<cv::Vec3d> refSwap = pack(r1, r0);
    const std::vector<cv::Vec3d> curs[4] = {pack(c0, c1), pack(c0f, c1), pack(c0, c1f), pack(c0f, c1f)};

    AlignResult best;
    best.rms = 1e300;
    for (int swap = 0; swap < 2; ++swap) {
        const std::vector<cv::Vec3d>& ref = swap ? refSwap : refSame;
        for (const std::vector<cv::Vec3d>& cur : curs) {
            const AlignResult candidate = absoluteOrientation(cur, ref);
            if (candidate.rms < best.rms) best = candidate;
        }
    }
    return best;
}

}  // namespace plane_level
