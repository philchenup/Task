#include "lobb_weld/demo_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lobb {
namespace {

constexpr double kPi = 3.14159265358979323846;

pcl::PointXYZ xyz(float x, float y, float z) {
  pcl::PointXYZ p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

}  // namespace

DemoCloud makeSineFilletCloud() {
  // 底板 z = 0，一块沿正弦交线立起的曲面板。交线是一条开曲线焊缝。
  constexpr float kSpacing = 1.25f;
  constexpr float kNoise = 0.15f;
  constexpr float kAmp = 12.f;
  constexpr float kGamma = 68.f * static_cast<float>(kPi) / 180.f;

  std::mt19937 rng(1);
  std::normal_distribution<float> noise(0.f, kNoise);

  DemoCloud demo;
  demo.name = "sine-fillet";
  demo.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  demo.cloud->points.reserve(20000);
  demo.max_mean_error = 1.0;
  demo.max_p90_error = 1.8;
  demo.min_coverage = 0.80;
  demo.coverage_distance = 2.5;

  auto seam_y = [](float x) { return kAmp * std::sin(static_cast<float>(kPi) * x / 40.f); };

  for (float x = -78.f; x <= 78.f + 1e-4f; x += kSpacing) {
    for (float y = -52.f; y <= 52.f + 1e-4f; y += kSpacing) {
      demo.cloud->points.push_back(xyz(x + noise(rng), y + noise(rng), noise(rng)));
    }
  }

  const float s_step = kSpacing * 0.85f;
  for (float t = -58.f; t <= 58.f + 1e-4f; t += s_step) {
    const float dydx = kAmp * static_cast<float>(kPi) / 40.f * std::cos(static_cast<float>(kPi) * t / 40.f);
    Eigen::Vector2f tangent(1.f, dydx);
    tangent.normalize();
    // 切向顺时针转 90°，得到交线一侧的水平法向。
    const Eigen::Vector2f n_in(tangent.y(), -tangent.x());
    for (float s = s_step; s <= 26.f; s += s_step) {
      const float x = t + s * std::cos(kGamma) * n_in.x();
      const float y = seam_y(t) + s * std::cos(kGamma) * n_in.y();
      const float z = s * std::sin(kGamma);
      demo.cloud->points.push_back(xyz(x + noise(rng), y + noise(rng), z + noise(rng)));
    }
  }

  demo.cloud->width = static_cast<uint32_t>(demo.cloud->points.size());
  demo.cloud->height = 1;
  demo.cloud->is_dense = false;

  for (float t = -58.f; t <= 58.f; t += 0.4f) {
    demo.ground_truth.emplace_back(t, seam_y(t), 0.0);
  }
  return demo;
}

DemoCloud makePipePlateCloud() {
  // 竖管插在底板上，焊缝是整圈圆。用来检查闭合曲面焊缝。
  constexpr float kSpacing = 1.3f;
  constexpr float kNoise = 0.12f;
  constexpr float kRadius = 30.f;

  std::mt19937 rng(2);
  std::normal_distribution<float> noise(0.f, kNoise);

  DemoCloud demo;
  demo.name = "pipe-plate";
  demo.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  demo.cloud->points.reserve(20000);
  demo.max_mean_error = 1.15;
  demo.max_p90_error = 2.0;
  demo.min_coverage = 0.85;
  demo.coverage_distance = 2.5;

  for (float x = -68.f; x <= 68.f + 1e-4f; x += kSpacing) {
    for (float y = -68.f; y <= 68.f + 1e-4f; y += kSpacing) {
      demo.cloud->points.push_back(xyz(x + noise(rng), y + noise(rng), noise(rng)));
    }
  }

  const float dtheta = kSpacing / kRadius;
  for (float z = kSpacing * 0.5f; z <= 34.f; z += kSpacing * 0.85f) {
    for (float th = 0.f; th < 2.f * static_cast<float>(kPi) - 1e-4f; th += dtheta) {
      const float x = kRadius * std::cos(th);
      const float y = kRadius * std::sin(th);
      demo.cloud->points.push_back(xyz(x + noise(rng), y + noise(rng), z + noise(rng)));
    }
  }

  demo.cloud->width = static_cast<uint32_t>(demo.cloud->points.size());
  demo.cloud->height = 1;
  demo.cloud->is_dense = false;

  for (int i = 0; i < 360; ++i) {
    const double th = kPi * i / 180.0;
    demo.ground_truth.emplace_back(kRadius * std::cos(th), kRadius * std::sin(th), 0.0);
  }
  return demo;
}

CurveScore scoreCurve(const std::vector<Eigen::Vector3d>& seam,
                      const std::vector<Eigen::Vector3d>& ground_truth,
                      double coverage_distance) {
  CurveScore score;
  if (seam.empty() || ground_truth.empty()) {
    return score;
  }

  std::vector<double> errors;
  errors.reserve(seam.size());
  for (const auto& p : seam) {
    double best = 1e9;
    for (const auto& g : ground_truth) {
      best = std::min(best, (p - g).norm());
    }
    errors.push_back(best);
  }
  std::sort(errors.begin(), errors.end());
  double sum = 0.0;
  for (double e : errors) {
    sum += e;
  }
  score.mean = sum / static_cast<double>(errors.size());
  score.p90 = errors[static_cast<size_t>(0.9 * (errors.size() - 1))];

  int covered = 0;
  for (const auto& g : ground_truth) {
    double best = 1e9;
    for (const auto& p : seam) {
      best = std::min(best, (p - g).norm());
    }
    if (best <= coverage_distance) {
      ++covered;
    }
  }
  score.coverage = static_cast<double>(covered) / static_cast<double>(ground_truth.size());
  return score;
}

}  // namespace lobb
