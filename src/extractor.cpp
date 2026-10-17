#include "lobb_weld/extractor.hpp"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lobb {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Features {
  std::vector<float> surface_variation;
  std::vector<float> linearity;
  std::vector<float> planarity;
  std::vector<float> boundary;
  std::vector<float> corner;
  std::vector<float> shift;
  std::vector<float> resultant;
  std::vector<float> normal_variation;
};

struct Dihedral {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
  Eigen::Vector3d approach = Eigen::Vector3d::UnitZ();
};

float percentile(std::vector<float> values, float q) {
  if (values.empty()) {
    return 0.f;
  }
  q = std::clamp(q, 0.f, 1.f);
  const size_t k = static_cast<size_t>(q * static_cast<float>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
  return values[k];
}

float medianOf(std::vector<float> values) { return percentile(std::move(values), 0.5f); }

// 稳健归一化到 [0, 1] 后，用 Otsu 阈值做 Sigmoid。对应论文中的非线性激活。
std::vector<float> activate(const std::vector<float>& values, float gain) {
  const size_t n = values.size();
  std::vector<float> out(n, 0.f);
  if (n == 0) {
    return out;
  }
  const float p5 = percentile(values, 0.05f);
  const float p95 = percentile(values, 0.95f);
  const float span = std::max(p95 - p5, 1e-6f);

  constexpr int kBins = 48;
  std::vector<int> hist(kBins, 0);
  std::vector<float> unit(n, 0.f);
  for (size_t i = 0; i < n; ++i) {
    const float u = std::clamp((values[i] - p5) / span, 0.f, 1.f);
    unit[i] = u;
    int bin = static_cast<int>(u * (kBins - 1));
    bin = std::clamp(bin, 0, kBins - 1);
    ++hist[bin];
  }

  std::vector<double> prob(kBins, 0.0);
  for (int i = 0; i < kBins; ++i) {
    prob[i] = static_cast<double>(hist[i]) / static_cast<double>(n);
  }
  double best_sigma = -1.0;
  int best = kBins / 2;
  double omega = 0.0;
  double mu = 0.0;
  double mu_t = 0.0;
  for (int i = 0; i < kBins; ++i) {
    mu_t += prob[i] * ((static_cast<double>(i) + 0.5) / kBins);
  }
  for (int i = 0; i < kBins - 1; ++i) {
    omega += prob[i];
    mu += prob[i] * ((static_cast<double>(i) + 0.5) / kBins);
    const double denom = omega * (1.0 - omega);
    if (denom < 1e-12) {
      continue;
    }
    const double sigma = (mu_t * omega - mu) * (mu_t * omega - mu) / denom;
    if (sigma > best_sigma) {
      best_sigma = sigma;
      best = i;
    }
  }
  const float tau = (static_cast<float>(best) + 0.5f) / static_cast<float>(kBins);
  for (size_t i = 0; i < n; ++i) {
    out[i] = 1.f / (1.f + std::exp(-gain * (unit[i] - tau)));
  }
  return out;
}

// 以 primary 维的低/高分位初始化的 2 类 K-means。分层聚类的每一层都用它。
std::vector<int> kmeans2(const std::vector<float>& data, int n, int d, int primary) {
  std::vector<int> labels(n, 0);
  if (n < 4 || d < 1) {
    return labels;
  }
  std::vector<float> column(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    column[static_cast<size_t>(i)] = data[static_cast<size_t>(i * d + primary)];
  }
  const float low = percentile(column, 0.25f);
  const float high = percentile(column, 0.75f);

  std::vector<float> cents(static_cast<size_t>(2 * d), 0.f);
  std::vector<int> seed_count(2, 0);
  for (int i = 0; i < n; ++i) {
    const int group = column[static_cast<size_t>(i)] >= high ? 1 : (column[static_cast<size_t>(i)] <= low ? 0 : -1);
    if (group < 0) {
      continue;
    }
    ++seed_count[group];
    for (int j = 0; j < d; ++j) {
      cents[static_cast<size_t>(group * d + j)] += data[static_cast<size_t>(i * d + j)];
    }
  }
  for (int g = 0; g < 2; ++g) {
    if (seed_count[g] == 0) {
      const float target = g == 0 ? low : high;
      int pick = 0;
      float best = std::numeric_limits<float>::max();
      for (int i = 0; i < n; ++i) {
        const float dist = std::abs(column[static_cast<size_t>(i)] - target);
        if (dist < best) {
          best = dist;
          pick = i;
        }
      }
      for (int j = 0; j < d; ++j) {
        cents[static_cast<size_t>(g * d + j)] = data[static_cast<size_t>(pick * d + j)];
      }
    } else {
      for (int j = 0; j < d; ++j) {
        cents[static_cast<size_t>(g * d + j)] /= static_cast<float>(seed_count[g]);
      }
    }
  }

  for (int iter = 0; iter < 30; ++iter) {
    bool changed = false;
    std::vector<float> sum(static_cast<size_t>(2 * d), 0.f);
    std::vector<int> count(2, 0);
    for (int i = 0; i < n; ++i) {
      float best = std::numeric_limits<float>::max();
      int best_label = 0;
      for (int g = 0; g < 2; ++g) {
        float dist = 0.f;
        for (int j = 0; j < d; ++j) {
          const float diff = data[static_cast<size_t>(i * d + j)] - cents[static_cast<size_t>(g * d + j)];
          dist += diff * diff;
        }
        if (dist < best) {
          best = dist;
          best_label = g;
        }
      }
      if (labels[static_cast<size_t>(i)] != best_label) {
        labels[static_cast<size_t>(i)] = best_label;
        changed = true;
      }
      ++count[best_label];
      for (int j = 0; j < d; ++j) {
        sum[static_cast<size_t>(best_label * d + j)] += data[static_cast<size_t>(i * d + j)];
      }
    }
    for (int g = 0; g < 2; ++g) {
      if (count[g] == 0) {
        continue;
      }
      for (int j = 0; j < d; ++j) {
        cents[static_cast<size_t>(g * d + j)] = sum[static_cast<size_t>(g * d + j)] / static_cast<float>(count[g]);
      }
    }
    if (!changed && iter > 0) {
      break;
    }
  }
  return labels;
}

float columnMean(const std::vector<float>& column, const std::vector<int>& labels, int label) {
  double sum = 0.0;
  int count = 0;
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == label) {
      sum += column[i];
      ++count;
    }
  }
  return count == 0 ? 0.f : static_cast<float>(sum / count);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                               const ExtractParams& params) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr current(new pcl::PointCloud<pcl::PointXYZ>);
  current->reserve(input->size());
  for (const auto& p : input->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      current->push_back(p);
    }
  }
  current->width = static_cast<uint32_t>(current->size());
  current->height = 1;
  current->is_dense = true;
  if (current->size() < 50) {
    throw std::runtime_error("有效点数太少，无法提取焊缝。");
  }

  if (params.voxel > 1e-6f) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(current);
    voxel.setLeafSize(params.voxel, params.voxel, params.voxel);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*filtered);
    current = filtered;
  }

  if (params.statistical_filter && current->size() > static_cast<size_t>(params.sor_mean_k + 5)) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(current);
    sor.setMeanK(params.sor_mean_k);
    sor.setStddevMulThresh(params.sor_stddev);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered);
    current = filtered;
  }

  current->width = static_cast<uint32_t>(current->size());
  current->height = 1;
  if (current->size() < 50) {
    throw std::runtime_error("滤波后点数太少。请减小体素，或关闭统计滤波。");
  }
  return current;
}

float estimateMedianNn(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  const int n = static_cast<int>(cloud->size());
  const int step = std::max(1, n / 2000);
  std::vector<float> distances;
  distances.reserve(static_cast<size_t>(n / step + 1));
  std::vector<int> indices(2);
  std::vector<float> squared(2);
  for (int i = 0; i < n; i += step) {
    if (tree.nearestKSearch(cloud->points[static_cast<size_t>(i)], 2, indices, squared) >= 2) {
      distances.push_back(std::sqrt(squared[1]));
    }
  }
  if (distances.empty()) {
    throw std::runtime_error("无法估计点间距。");
  }
  return medianOf(std::move(distances));
}

void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    pcl::KdTreeFLANN<pcl::PointXYZ>& tree, float radius, int min_neighbors,
                    std::vector<Eigen::Vector3f>& normals) {
  const int n = static_cast<int>(cloud->size());
  normals.assign(static_cast<size_t>(n), Eigen::Vector3f::Zero());
  std::vector<int> indices;
  std::vector<float> squared;
  indices.reserve(64);
  squared.reserve(64);

  for (int i = 0; i < n; ++i) {
    indices.clear();
    squared.clear();
    if (tree.radiusSearch(cloud->points[static_cast<size_t>(i)], radius, indices, squared) < min_neighbors) {
      continue;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int count = 0;
    for (int id : indices) {
      if (id == i) {
        continue;
      }
      const auto& p = cloud->points[static_cast<size_t>(id)];
      mean += Eigen::Vector3d(p.x, p.y, p.z);
      ++count;
    }
    if (count < min_neighbors) {
      continue;
    }
    mean /= static_cast<double>(count);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int id : indices) {
      if (id == i) {
        continue;
      }
      const auto& p = cloud->points[static_cast<size_t>(id)];
      const Eigen::Vector3d d(p.x, p.y, p.z);
      const Eigen::Vector3d r = d - mean;
      cov += r * r.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0.0) {
      normal = -normal;
    }
    normals[static_cast<size_t>(i)] = normal.cast<float>();
  }
}

Features computeLobb(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     pcl::KdTreeFLANN<pcl::PointXYZ>& tree, const std::vector<Eigen::Vector3f>& normals,
                     float radius, int min_neighbors) {
  const int n = static_cast<int>(cloud->size());
  Features features;
  features.surface_variation.assign(n, 0.f);
  features.linearity.assign(n, 0.f);
  features.planarity.assign(n, 0.f);
  features.boundary.assign(n, 0.f);
  features.corner.assign(n, 0.f);
  features.shift.assign(n, 0.f);
  features.resultant.assign(n, 0.f);
  features.normal_variation.assign(n, 0.f);

  std::vector<int> indices;
  std::vector<float> squared;
  indices.reserve(128);
  squared.reserve(128);

  for (int i = 0; i < n; ++i) {
    indices.clear();
    squared.clear();
    tree.radiusSearch(cloud->points[static_cast<size_t>(i)], radius, indices, squared);
    const auto& pi = cloud->points[static_cast<size_t>(i)];
    const Eigen::Vector3d origin(pi.x, pi.y, pi.z);

    std::vector<Eigen::Vector3d> neighbors;
    neighbors.reserve(indices.size());
    for (int id : indices) {
      if (id == i) {
        continue;
      }
      const auto& p = cloud->points[static_cast<size_t>(id)];
      neighbors.emplace_back(p.x, p.y, p.z);
    }
    if (static_cast<int>(neighbors.size()) < min_neighbors) {
      continue;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) {
      mean += p;
    }
    mean /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
      const Eigen::Vector3d r = p - mean;
      cov += r * r.transpose();
    }
    cov /= static_cast<double>(neighbors.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const double l1 = std::max(0.0, solver.eigenvalues()(2));
    const double l2 = std::max(0.0, solver.eigenvalues()(1));
    const double l3 = std::max(0.0, solver.eigenvalues()(0));
    const double sum_l = l1 + l2 + l3 + 1e-12;
    features.surface_variation[static_cast<size_t>(i)] = static_cast<float>(l3 / sum_l);
    features.linearity[static_cast<size_t>(i)] = static_cast<float>((l1 - l2) / (l1 + 1e-12));
    features.planarity[static_cast<size_t>(i)] = static_cast<float>((l2 - l3) / (l1 + 1e-12));

    // 局部有向包围盒：主轴由邻域协方差给出，查询点在盒内的位置区分内部、边界和角点。
    Eigen::Matrix3d axes;
    axes.col(0) = solver.eigenvectors().col(2);
    axes.col(1) = solver.eigenvectors().col(1);
    axes.col(2) = solver.eigenvectors().col(0);
    Eigen::Vector3d qmin = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d qmax = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    Eigen::Vector3d resultant = Eigen::Vector3d::Zero();
    int direction_count = 0;
    for (const auto& p : neighbors) {
      const Eigen::Vector3d delta = p - origin;
      const Eigen::Vector3d local = axes.transpose() * delta;
      qmin = qmin.cwiseMin(local);
      qmax = qmax.cwiseMax(local);
      const double norm = delta.norm();
      if (norm > 1e-9) {
        resultant += delta / norm;
        ++direction_count;
      }
    }
    const Eigen::Vector3d extent = (qmax - qmin).cwiseMax(1e-9);
    const Eigen::Vector3d unit = ((-qmin).cwiseQuotient(extent)).cwiseMax(0.0).cwiseMin(1.0);
    Eigen::Vector3d extremity;
    for (int axis = 0; axis < 3; ++axis) {
      extremity(axis) = 1.0 - 2.0 * std::min(unit(axis), 1.0 - unit(axis));
    }
    features.boundary[static_cast<size_t>(i)] =
        static_cast<float>(std::max(extremity(0), extremity(1)));
    features.corner[static_cast<size_t>(i)] = static_cast<float>(extremity(0) * extremity(1));
    features.shift[static_cast<size_t>(i)] = static_cast<float>((mean - origin).norm() / radius);
    if (direction_count > 0) {
      features.resultant[static_cast<size_t>(i)] =
          static_cast<float>((resultant / static_cast<double>(direction_count)).norm());
    }

    // 盒内法向分散度：光滑弯曲面很小，两侧曲面相交的折痕明显更大。
    const Eigen::Vector3f base = normals[static_cast<size_t>(i)];
    if (base.squaredNorm() > 0.25f) {
      Eigen::Vector3d acc = Eigen::Vector3d::Zero();
      int valid = 0;
      for (int id : indices) {
        if (id == i) {
          continue;
        }
        Eigen::Vector3d n = normals[static_cast<size_t>(id)].cast<double>();
        if (n.squaredNorm() < 0.25) {
          continue;
        }
        if (n.dot(base.cast<double>()) < 0.0) {
          n = -n;
        }
        acc += n;
        ++valid;
      }
      if (valid >= min_neighbors / 2) {
        const Eigen::Vector3d mean_n = acc.normalized();
        double deviation = 0.0;
        int used = 0;
        for (int id : indices) {
          if (id == i) {
            continue;
          }
          Eigen::Vector3d n = normals[static_cast<size_t>(id)].cast<double>();
          if (n.squaredNorm() < 0.25) {
            continue;
          }
          if (n.dot(mean_n) < 0.0) {
            n = -n;
          }
          deviation += 1.0 - n.dot(mean_n);
          ++used;
        }
        if (used > 0) {
          features.normal_variation[static_cast<size_t>(i)] = static_cast<float>(deviation / used);
        }
      }
    }
  }
  return features;
}

std::optional<Dihedral> fitDihedral(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    pcl::KdTreeFLANN<pcl::PointXYZ>& tree, const Eigen::Vector3d& query,
                                    float radius, int min_neighbors, float parallel_dot,
                                    float min_fraction, float max_plane_rms) {
  pcl::PointXYZ probe;
  probe.x = static_cast<float>(query.x());
  probe.y = static_cast<float>(query.y());
  probe.z = static_cast<float>(query.z());
  std::vector<int> indices;
  std::vector<float> squared;
  if (tree.radiusSearch(probe, radius, indices, squared) < min_neighbors) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(indices.size());
  for (size_t k = 0; k < indices.size(); ++k) {
    if (squared[k] < 1e-12f) {
      continue;
    }
    const auto& p = cloud->points[static_cast<size_t>(indices[k])];
    points.emplace_back(p.x, p.y, p.z);
  }
  if (static_cast<int>(points.size()) < min_neighbors) {
    return std::nullopt;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : points) {
    const Eigen::Vector3d r = p - mean;
    cov += r * r.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  const Eigen::Vector3d axis_u = solver.eigenvectors().col(1);
  const Eigen::Vector3d axis_v = solver.eigenvectors().col(0);

  // 沿主方向看横截面。折痕的横截面是两支，K-means 会把它们分开；光滑曲面则仍共面。
  std::vector<Eigen::Vector2d> section;
  section.reserve(points.size());
  for (const auto& p : points) {
    const Eigen::Vector3d r = p - mean;
    section.emplace_back(r.dot(axis_u), r.dot(axis_v));
  }

  Eigen::Vector2d section_mean = Eigen::Vector2d::Zero();
  for (const auto& s : section) {
    section_mean += s;
  }
  section_mean /= static_cast<double>(section.size());
  size_t far_a = 0;
  double far_dist = -1.0;
  for (size_t i = 0; i < section.size(); ++i) {
    const double dist = (section[i] - section_mean).squaredNorm();
    if (dist > far_dist) {
      far_dist = dist;
      far_a = i;
    }
  }
  size_t far_b = 0;
  far_dist = -1.0;
  for (size_t i = 0; i < section.size(); ++i) {
    const double dist = (section[i] - section[far_a]).squaredNorm();
    if (dist > far_dist) {
      far_dist = dist;
      far_b = i;
    }
  }
  Eigen::Vector2d c0 = section[far_a];
  Eigen::Vector2d c1 = section[far_b];
  std::vector<int> group(section.size(), 0);
  for (int iter = 0; iter < 16; ++iter) {
    Eigen::Vector2d sum0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum1 = Eigen::Vector2d::Zero();
    int n0 = 0;
    int n1 = 0;
    for (size_t i = 0; i < section.size(); ++i) {
      const int g = (section[i] - c1).squaredNorm() < (section[i] - c0).squaredNorm() ? 1 : 0;
      group[i] = g;
      if (g == 0) {
        sum0 += section[i];
        ++n0;
      } else {
        sum1 += section[i];
        ++n1;
      }
    }
    if (n0 == 0 || n1 == 0) {
      return std::nullopt;
    }
    c0 = sum0 / static_cast<double>(n0);
    c1 = sum1 / static_cast<double>(n1);
  }

  int n0 = 0;
  int n1 = 0;
  for (int g : group) {
    if (g == 0) {
      ++n0;
    } else {
      ++n1;
    }
  }
  const double fraction = static_cast<double>(std::min(n0, n1)) / static_cast<double>(group.size());
  if (fraction < min_fraction) {
    return std::nullopt;
  }

  auto fit_plane = [&](int which, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& rms) {
    center.setZero();
    int count = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      if (group[i] == which) {
        center += points[i];
        ++count;
      }
    }
    center /= static_cast<double>(count);
    Eigen::Matrix3d plane_cov = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points.size(); ++i) {
      if (group[i] != which) {
        continue;
      }
      const Eigen::Vector3d r = points[i] - center;
      plane_cov += r * r.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane_solver(plane_cov);
    normal = plane_solver.eigenvectors().col(0);
    rms = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      if (group[i] != which) {
        continue;
      }
      const double residual = normal.dot(points[i] - center);
      rms += residual * residual;
    }
    rms = std::sqrt(rms / static_cast<double>(count));
  };

  Eigen::Vector3d c_a, c_b, n_a, n_b;
  double rms_a = 0.0;
  double rms_b = 0.0;
  fit_plane(0, c_a, n_a, rms_a);
  fit_plane(1, c_b, n_b, rms_b);
  if (std::abs(n_a.dot(n_b)) > parallel_dot) {
    return std::nullopt;
  }
  if (rms_a > max_plane_rms || rms_b > max_plane_rms) {
    return std::nullopt;
  }

  Eigen::Vector3d direction = n_a.cross(n_b);
  const double direction_norm = direction.norm();
  if (direction_norm < 1e-8) {
    return std::nullopt;
  }
  direction /= direction_norm;

  Eigen::Matrix3d system;
  system.row(0) = n_a.transpose();
  system.row(1) = n_b.transpose();
  system.row(2) = direction.transpose();
  const Eigen::Vector3d rhs(n_a.dot(c_a), n_b.dot(c_b), direction.dot(query));
  const Eigen::Vector3d origin = system.colPivHouseholderQr().solve(rhs);
  if (!origin.allFinite() || (origin - query).norm() > radius) {
    return std::nullopt;
  }

  // 从邻域质心指向交线：焊枪在张开的一侧，朝焊缝送进。
  Eigen::Vector3d approach = origin - mean;
  if (approach.norm() < 1e-6) {
    if (n_a.dot(n_b) < 0.0) {
      n_b = -n_b;
    }
    approach = n_a + n_b;
  }
  if (approach.norm() < 1e-6) {
    approach = Eigen::Vector3d::UnitZ();
  }
  approach.normalize();

  Dihedral dihedral;
  dihedral.origin = origin;
  dihedral.direction = direction;
  dihedral.approach = approach;
  return dihedral;
}

struct SnapPoint {
  Eigen::Vector3d position;
  Eigen::Vector3d approach;
};

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const VoxelKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    size_t h = static_cast<size_t>(key.x) * 73856093u;
    h ^= static_cast<size_t>(key.y) * 19349663u;
    h ^= static_cast<size_t>(key.z) * 83492791u;
    return h;
  }
};

std::vector<SnapPoint> voxelAverage(const std::vector<SnapPoint>& points, float leaf) {
  std::unordered_map<VoxelKey, std::pair<SnapPoint, int>, VoxelKeyHash> bins;
  bins.reserve(points.size());
  for (const auto& point : points) {
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.position.x() / leaf));
    key.y = static_cast<int>(std::floor(point.position.y() / leaf));
    key.z = static_cast<int>(std::floor(point.position.z() / leaf));
    auto& slot = bins[key];
    slot.first.position += point.position;
    slot.first.approach += point.approach;
    ++slot.second;
  }
  std::vector<SnapPoint> averaged;
  averaged.reserve(bins.size());
  for (const auto& item : bins) {
    SnapPoint point = item.second.first;
    const double count = static_cast<double>(item.second.second);
    point.position /= count;
    if (point.approach.norm() < 1e-6) {
      point.approach = Eigen::Vector3d::UnitZ();
    } else {
      point.approach.normalize();
    }
    averaged.push_back(point);
  }
  std::sort(averaged.begin(), averaged.end(), [](const SnapPoint& a, const SnapPoint& b) {
    if (a.position.x() != b.position.x()) {
      return a.position.x() < b.position.x();
    }
    if (a.position.y() != b.position.y()) {
      return a.position.y() < b.position.y();
    }
    return a.position.z() < b.position.z();
  });
  return averaged;
}

std::vector<int> componentOf(const std::vector<SnapPoint>& points, float tolerance) {
  const int n = static_cast<int>(points.size());
  std::vector<int> component(n, -1);
  if (n == 0) {
    return component;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(static_cast<size_t>(n));
  for (const auto& point : points) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(point.position.x());
    p.y = static_cast<float>(point.position.y());
    p.z = static_cast<float>(point.position.z());
    cloud->push_back(p);
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);

  int current = 0;
  std::vector<int> stack;
  std::vector<int> indices;
  std::vector<float> squared;
  for (int i = 0; i < n; ++i) {
    if (component[i] >= 0) {
      continue;
    }
    stack.push_back(i);
    component[i] = current;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      indices.clear();
      squared.clear();
      tree.radiusSearch(cloud->points[static_cast<size_t>(u)], tolerance, indices, squared);
      for (int v : indices) {
        if (component[v] < 0) {
          component[v] = current;
          stack.push_back(v);
        }
      }
    }
    ++current;
  }
  return component;
}

std::vector<int> diameterPath(const std::vector<SnapPoint>& points, const std::vector<int>& members) {
  const int n = static_cast<int>(members.size());
  if (n <= 1) {
    return members;
  }

  std::vector<double> best_edge(n, std::numeric_limits<double>::max());
  std::vector<int> parent(n, -1);
  std::vector<char> used(n, 0);
  best_edge[0] = 0.0;
  for (int iter = 0; iter < n; ++iter) {
    int u = -1;
    double best = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
      if (!used[i] && best_edge[i] < best) {
        best = best_edge[i];
        u = i;
      }
    }
    if (u < 0) {
      break;
    }
    used[u] = 1;
    for (int v = 0; v < n; ++v) {
      if (used[v] || v == u) {
        continue;
      }
      const double dist = (points[members[u]].position - points[members[v]].position).norm();
      if (dist < best_edge[v]) {
        best_edge[v] = dist;
        parent[v] = u;
      }
    }
  }

  std::vector<std::vector<std::pair<int, double>>> graph(n);
  for (int v = 1; v < n; ++v) {
    if (parent[v] < 0) {
      continue;
    }
    const double dist = (points[members[v]].position - points[members[parent[v]]].position).norm();
    graph[v].push_back({parent[v], dist});
    graph[parent[v]].push_back({v, dist});
  }

  auto farthest = [&](int source) {
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> previous(n, -1);
    std::vector<char> seen(n, 0);
    dist[source] = 0.0;
    for (int iter = 0; iter < n; ++iter) {
      int u = -1;
      double best = std::numeric_limits<double>::infinity();
      for (int i = 0; i < n; ++i) {
        if (!seen[i] && dist[i] < best) {
          best = dist[i];
          u = i;
        }
      }
      if (u < 0) {
        break;
      }
      seen[u] = 1;
      for (const auto& edge : graph[u]) {
        const double next = dist[u] + edge.second;
        if (next < dist[edge.first]) {
          dist[edge.first] = next;
          previous[edge.first] = u;
        }
      }
    }
    int far = source;
    for (int i = 0; i < n; ++i) {
      if (dist[i] > dist[far]) {
        far = i;
      }
    }
    return std::make_pair(far, previous);
  };

  const int end_a = farthest(0).first;
  const auto second = farthest(end_a);
  std::vector<int> path;
  for (int node = second.first; node >= 0; node = second.second[node]) {
    path.push_back(members[node]);
    if (node == end_a) {
      break;
    }
  }
  return path;
}

void smoothPath(std::vector<SnapPoint>& path, bool closed, int window, int iterations) {
  if (path.size() < 5 || window < 1) {
    return;
  }
  const int n = static_cast<int>(path.size());
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<SnapPoint> next = path;
    for (int i = 0; i < n; ++i) {
      if (!closed && (i == 0 || i == n - 1)) {
        continue;
      }
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      Eigen::Vector3d approach = Eigen::Vector3d::Zero();
      int count = 0;
      for (int k = -window; k <= window; ++k) {
        int j = i + k;
        if (closed) {
          j = (j % n + n) % n;
        } else if (j < 0 || j >= n) {
          continue;
        }
        position += path[static_cast<size_t>(j)].position;
        approach += path[static_cast<size_t>(j)].approach;
        ++count;
      }
      next[static_cast<size_t>(i)].position = position / static_cast<double>(count);
      if (approach.norm() > 1e-6) {
        next[static_cast<size_t>(i)].approach = approach.normalized();
      }
    }
    path.swap(next);
  }
}

double pathLength(const std::vector<SnapPoint>& path, bool closed) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i].position - path[i - 1].position).norm();
  }
  if (closed && path.size() > 2) {
    length += (path.front().position - path.back().position).norm();
  }
  return length;
}

std::vector<SnapPoint> resample(const std::vector<SnapPoint>& path, bool closed, double step) {
  std::vector<SnapPoint> samples;
  if (path.size() < 2 || step <= 1e-6) {
    return samples;
  }
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> approaches;
  std::vector<double> cumulative;
  positions.reserve(path.size() + 1);
  approaches.reserve(path.size() + 1);
  cumulative.reserve(path.size() + 1);
  cumulative.push_back(0.0);
  for (const auto& point : path) {
    positions.push_back(point.position);
    approaches.push_back(point.approach);
  }
  if (closed) {
    positions.push_back(path.front().position);
    approaches.push_back(path.front().approach);
  }
  for (size_t i = 1; i < positions.size(); ++i) {
    cumulative.push_back(cumulative.back() + (positions[i] - positions[i - 1]).norm());
  }
  const double total = cumulative.back();
  if (total < step) {
    return samples;
  }
  const int count = std::max(2, static_cast<int>(std::floor(total / step)));
  const double used_step = total / static_cast<double>(count);
  for (int i = 0; i < count; ++i) {
    const double s = used_step * static_cast<double>(i);
    auto it = std::upper_bound(cumulative.begin(), cumulative.end(), s);
    size_t hi = static_cast<size_t>(std::distance(cumulative.begin(), it));
    if (hi == 0) {
      hi = 1;
    }
    if (hi >= cumulative.size()) {
      hi = cumulative.size() - 1;
    }
    const size_t lo = hi - 1;
    const double span = std::max(1e-9, cumulative[hi] - cumulative[lo]);
    const double t = std::clamp((s - cumulative[lo]) / span, 0.0, 1.0);
    SnapPoint sample;
    sample.position = (1.0 - t) * positions[lo] + t * positions[hi];
    sample.approach = (1.0 - t) * approaches[lo] + t * approaches[hi];
    if (sample.approach.norm() < 1e-6) {
      sample.approach = Eigen::Vector3d::UnitZ();
    } else {
      sample.approach.normalize();
    }
    samples.push_back(sample);
  }
  return samples;
}

void orientPath(std::vector<SnapPoint>& path, bool closed) {
  if (path.size() < 2) {
    return;
  }
  if (!closed) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& point : path) {
      mean += point.position;
    }
    mean /= static_cast<double>(path.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& point : path) {
      const Eigen::Vector3d r = point.position - mean;
      cov += r * r.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const Eigen::Vector3d axis = solver.eigenvectors().col(2);
    if (path.front().position.dot(axis) > path.back().position.dot(axis)) {
      std::reverse(path.begin(), path.end());
    }
    return;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : path) {
    mean += point.position;
  }
  mean /= static_cast<double>(path.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& point : path) {
    const Eigen::Vector3d r = point.position - mean;
    cov += r * r.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  const Eigen::Vector3d normal = solver.eigenvectors().col(0);
  const Eigen::Vector3d x_axis = solver.eigenvectors().col(2);
  const Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();
  double area = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    const Eigen::Vector3d a = path[i].position - mean;
    const Eigen::Vector3d b = path[(i + 1) % path.size()].position - mean;
    const double ax = a.dot(x_axis);
    const double ay = a.dot(y_axis);
    const double bx = b.dot(x_axis);
    const double by = b.dot(y_axis);
    area += ax * by - bx * ay;
  }
  // 从法向正侧看过去保持逆时针。法向尽量朝上，便于圆管这类平放焊缝。
  if (normal.z() < 0.0) {
    area = -area;
  }
  if (area < 0.0) {
    std::reverse(path.begin(), path.end());
  }
}

std::vector<SeamPose> makePoses(const std::vector<SnapPoint>& path, bool closed) {
  std::vector<SeamPose> poses;
  const int n = static_cast<int>(path.size());
  if (n < 2) {
    return poses;
  }
  poses.reserve(static_cast<size_t>(n));
  Eigen::Vector3d previous_approach = path.front().approach;
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d tangent;
    if (n == 2) {
      tangent = path[1].position - path[0].position;
    } else if (!closed && i == 0) {
      tangent = path[1].position - path[0].position;
    } else if (!closed && i == n - 1) {
      tangent = path[n - 1].position - path[n - 2].position;
    } else {
      const int prev = closed ? (i - 1 + n) % n : i - 1;
      const int next = closed ? (i + 1) % n : i + 1;
      tangent = path[static_cast<size_t>(next)].position - path[static_cast<size_t>(prev)].position;
    }
    Eigen::Vector3d approach = path[static_cast<size_t>(i)].approach;
    if (approach.dot(previous_approach) < 0.0) {
      approach = -approach;
    }
    previous_approach = approach;
    if (tangent.norm() < 1e-9) {
      tangent = Eigen::Vector3d::UnitX();
    }
    tangent.normalize();
    tangent = tangent - approach * tangent.dot(approach);
    if (tangent.norm() < 1e-6) {
      const Eigen::Vector3d helper =
          std::abs(approach.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
      tangent = (helper - approach * helper.dot(approach)).normalized();
    } else {
      tangent.normalize();
    }
    Eigen::Vector3d binormal = approach.cross(tangent);
    if (binormal.norm() < 1e-6) {
      binormal = Eigen::Vector3d::UnitY();
    } else {
      binormal.normalize();
    }
    SeamPose pose;
    pose.position = path[static_cast<size_t>(i)].position;
    pose.tangent = tangent;
    pose.approach = approach;
    pose.binormal = binormal;
    poses.push_back(pose);
  }
  return poses;
}

std::vector<SeamCurve> curvesFromSnaps(const std::vector<SnapPoint>& snaps, float leaf, float min_length) {
  std::vector<SeamCurve> seams;
  if (snaps.size() < 8) {
    return seams;
  }
  const std::vector<SnapPoint> thinned = voxelAverage(snaps, leaf);
  const float tolerance = leaf * 2.8f;
  const std::vector<int> component = componentOf(thinned, tolerance);
  if (component.empty()) {
    return seams;
  }
  const int groups = *std::max_element(component.begin(), component.end()) + 1;
  for (int group = 0; group < groups; ++group) {
    std::vector<int> members;
    for (size_t i = 0; i < component.size(); ++i) {
      if (component[i] == group) {
        members.push_back(static_cast<int>(i));
      }
    }
    if (members.size() < 8) {
      continue;
    }
    std::vector<int> order = diameterPath(thinned, members);
    if (order.size() < 8) {
      continue;
    }
    std::vector<SnapPoint> path;
    path.reserve(order.size());
    for (int index : order) {
      path.push_back(thinned[static_cast<size_t>(index)]);
    }
    const double end_gap = (path.front().position - path.back().position).norm();
    const bool closed = end_gap < leaf * 2.6;
    const double length = pathLength(path, closed);
    if (length < min_length) {
      continue;
    }
    smoothPath(path, closed, 2, 3);
    orientPath(path, closed);
    const double step = std::max(static_cast<double>(leaf), length / 400.0);
    std::vector<SnapPoint> samples = resample(path, closed, step);
    if (samples.size() < 8) {
      samples = path;
    }
    orientPath(samples, closed);
    SeamCurve curve;
    curve.closed = closed;
    curve.poses = makePoses(samples, closed);
    curve.length = pathLength(samples, closed);
    if (curve.poses.size() >= 8 && curve.length >= min_length) {
      seams.push_back(std::move(curve));
    }
  }
  std::sort(seams.begin(), seams.end(),
            [](const SeamCurve& a, const SeamCurve& b) { return a.length > b.length; });
  return seams;
}

}  // namespace

const char* labelName(PointLabel label) {
  switch (label) {
    case PointLabel::Plane:
      return "plane";
    case PointLabel::Boundary:
      return "boundary";
    case PointLabel::Crease:
      return "crease";
    case PointLabel::Corner:
      return "corner";
  }
  return "unknown";
}

ExtractResult extractCurvedSeam(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const ExtractParams& params) {
  if (!input || input->empty()) {
    throw std::runtime_error("点云为空。");
  }

  ExtractResult result;
  result.cloud = preprocess(input, params);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(result.cloud);

  result.median_nn = estimateMedianNn(result.cloud, tree);
  result.feature_radius =
      params.feature_radius > 1e-6f ? params.feature_radius : params.radius_nn_factor * result.median_nn;
  result.normal_radius = std::max(2.6f * result.median_nn, 0.38f * result.feature_radius);
  if (result.normal_radius >= result.feature_radius) {
    result.normal_radius = 0.45f * result.feature_radius;
  }

  std::cout << "预处理后点数: " << result.cloud->size() << "\n"
            << "中位点间距: " << result.median_nn << "\n"
            << "LOBB 半径: " << result.feature_radius << "，法向半径: " << result.normal_radius << "\n";

  std::vector<Eigen::Vector3f> normals;
  computeNormals(result.cloud, tree, result.normal_radius, std::max(6, params.min_neighbors / 2), normals);
  const Features features =
      computeLobb(result.cloud, tree, normals, result.feature_radius, params.min_neighbors);

  const auto act_boundary = activate(features.boundary, params.activation_gain);
  const auto act_linearity = activate(features.linearity, params.activation_gain);
  const auto act_shift = activate(features.shift, params.activation_gain);
  const auto act_normal = activate(features.normal_variation, params.activation_gain);
  const auto act_variation = activate(features.surface_variation, params.activation_gain);
  const auto act_resultant = activate(features.resultant, params.activation_gain);

  const int n = static_cast<int>(result.cloud->size());
  result.labels.assign(static_cast<size_t>(n), PointLabel::Plane);

  // 第一层：边界 / 非边界。边界点的有向包围盒把查询点推到盒的一侧。
  std::vector<float> level1(static_cast<size_t>(n * 3));
  for (int i = 0; i < n; ++i) {
    level1[static_cast<size_t>(i * 3 + 0)] = act_boundary[static_cast<size_t>(i)];
    level1[static_cast<size_t>(i * 3 + 1)] = act_linearity[static_cast<size_t>(i)];
    level1[static_cast<size_t>(i * 3 + 2)] = act_shift[static_cast<size_t>(i)];
  }
  const std::vector<int> split1 = kmeans2(level1, n, 3, 0);
  const float mean_b0 = columnMean(act_boundary, split1, 0);
  const float mean_b1 = columnMean(act_boundary, split1, 1);
  const int boundary_label = mean_b1 > mean_b0 ? 1 : 0;
  const float boundary_gap = std::abs(mean_b1 - mean_b0);
  const float boundary_mid = 0.5f * (mean_b0 + mean_b1);
  if (boundary_gap > 0.12f) {
    for (int i = 0; i < n; ++i) {
      if (split1[static_cast<size_t>(i)] == boundary_label &&
          act_boundary[static_cast<size_t>(i)] >= boundary_mid) {
        result.labels[static_cast<size_t>(i)] = PointLabel::Boundary;
      }
    }
  }

  // 第二层：在非边界点里分平面 / 折痕。折痕的法向分散度和包围盒厚度更高。
  std::vector<int> rest;
  rest.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (result.labels[static_cast<size_t>(i)] == PointLabel::Plane) {
      rest.push_back(i);
    }
  }
  if (rest.size() >= 8) {
    std::vector<float> level2(rest.size() * 3);
    std::vector<float> normal_column(rest.size());
    for (size_t k = 0; k < rest.size(); ++k) {
      const int i = rest[k];
      level2[k * 3 + 0] = act_normal[static_cast<size_t>(i)];
      level2[k * 3 + 1] = act_variation[static_cast<size_t>(i)];
      level2[k * 3 + 2] = act_resultant[static_cast<size_t>(i)];
      normal_column[k] = act_normal[static_cast<size_t>(i)];
    }
    const std::vector<int> split2 = kmeans2(level2, static_cast<int>(rest.size()), 3, 0);
    const float mean_n0 = columnMean(normal_column, split2, 0);
    const float mean_n1 = columnMean(normal_column, split2, 1);
    const int crease_label = mean_n1 > mean_n0 ? 1 : 0;
    const float crease_gap = std::abs(mean_n1 - mean_n0);
    const float crease_mid = 0.5f * (mean_n0 + mean_n1);
    if (crease_gap > 0.08f) {
      for (size_t k = 0; k < rest.size(); ++k) {
        if (split2[k] == crease_label && normal_column[k] >= crease_mid) {
          result.labels[static_cast<size_t>(rest[k])] = PointLabel::Crease;
        }
      }
    }
  }

  // 第三层：边界点里，包围盒两个切向都贴近盒面的是角点。
  for (int i = 0; i < n; ++i) {
    if (result.labels[static_cast<size_t>(i)] == PointLabel::Boundary &&
        features.corner[static_cast<size_t>(i)] > 0.22f && features.boundary[static_cast<size_t>(i)] > 0.40f) {
      result.labels[static_cast<size_t>(i)] = PointLabel::Corner;
    }
  }

  const float max_plane_rms = std::max(1.f, 4.f * result.median_nn);
  std::vector<SnapPoint> snaps;
  snaps.reserve(result.cloud->size() / 10);
  int crease_seen = 0;
  for (int i = 0; i < n; ++i) {
    if (result.labels[static_cast<size_t>(i)] != PointLabel::Crease) {
      continue;
    }
    ++crease_seen;
    const auto& p = result.cloud->points[static_cast<size_t>(i)];
    const Eigen::Vector3d query(p.x, p.y, p.z);
    const std::optional<Dihedral> dihedral =
        fitDihedral(result.cloud, tree, query, result.feature_radius, params.min_neighbors,
                    params.parallel_normal_dot, params.min_cluster_fraction, max_plane_rms);
    if (!dihedral) {
      // 聚类把它分到折痕，但邻域并不是两张相交曲面，改回平面，避免把弯曲板面当成焊缝。
      result.labels[static_cast<size_t>(i)] = PointLabel::Plane;
      continue;
    }
    std::vector<int> nearest(1);
    std::vector<float> nearest_d2(1);
    pcl::PointXYZ probe;
    probe.x = static_cast<float>(dihedral->origin.x());
    probe.y = static_cast<float>(dihedral->origin.y());
    probe.z = static_cast<float>(dihedral->origin.z());
    if (tree.nearestKSearch(probe, 1, nearest, nearest_d2) < 1 ||
        std::sqrt(nearest_d2[0]) > 3.f * result.median_nn) {
      result.labels[static_cast<size_t>(i)] = PointLabel::Plane;
      continue;
    }
    SnapPoint snap;
    snap.position = dihedral->origin;
    snap.approach = dihedral->approach;
    snaps.push_back(snap);
  }

  const float leaf = std::max(1.4f * result.median_nn, 0.35f * result.feature_radius);
  result.seams = curvesFromSnaps(snaps, leaf, 18.f * result.median_nn);

  for (const auto& seam : result.seams) {
    if (seam.closed || seam.poses.size() < 2) {
      continue;
    }
    for (int end = 0; end < 2; ++end) {
      const Eigen::Vector3d& position = end == 0 ? seam.poses.front().position : seam.poses.back().position;
      pcl::PointXYZ probe;
      probe.x = static_cast<float>(position.x());
      probe.y = static_cast<float>(position.y());
      probe.z = static_cast<float>(position.z());
      std::vector<int> nearest(1);
      std::vector<float> nearest_d2(1);
      if (tree.nearestKSearch(probe, 1, nearest, nearest_d2) >= 1) {
        result.labels[static_cast<size_t>(nearest[0])] = PointLabel::Corner;
      }
    }
  }

  for (PointLabel label : result.labels) {
    switch (label) {
      case PointLabel::Plane:
        ++result.plane_count;
        break;
      case PointLabel::Boundary:
        ++result.boundary_count;
        break;
      case PointLabel::Crease:
        ++result.crease_count;
        break;
      case PointLabel::Corner:
        ++result.corner_count;
        break;
    }
  }

  std::cout << "分层聚类: 平面 " << result.plane_count << "，边界 " << result.boundary_count << "，折痕 "
            << result.crease_count << "，角点 " << result.corner_count << "\n"
            << "折痕候选 " << crease_seen << "，两平面求交成功 " << snaps.size() << "，焊缝 "
            << result.seams.size() << " 条\n";
  for (size_t i = 0; i < result.seams.size(); ++i) {
    std::cout << "  焊缝 " << i << ": 长度 " << result.seams[i].length << "，点数 " << result.seams[i].poses.size()
              << (result.seams[i].closed ? "，闭合" : "，开放") << "\n";
  }
  return result;
}

}  // namespace lobb
