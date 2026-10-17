// 平板上的曲线工件：用 PCL 曲率提取弯曲焊缝，并连成焊根轨迹。
//
// 单位：毫米。
//
// 比「全场景曲率阈值」更稳的做法（本程序已按此实现）：
//   1. RANSAC 拟合底板。焊缝在接触带里，不在工件立边或顶缘。
//   2. 在完整点云上估计主曲率。交界点的邻域同时包含底板和壁面，曲率会抬高；
//      光滑曲壁的曲率大约是 1/R，不会单独把焊缝标出来。
//   3. 只在接触带内保留高曲率点。若曲率没有形成连续脊线（光滑壳体会这样），
//      退回整条接触带，再做局部 PCA 收成中心线。两平面求交只能得到直线，
//      不用于这条弯曲焊缝。
//
// 用法:
//   weld_curvature workpiece.pcd
//   weld_curvature workpiece.ply --band 16 --no-vis --out weld_trajectory_mm.txt

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace {

struct Params {
  float voxel = 1.0f;
  float plane_thresh = 1.5f;
  float gap = 2.0f;
  float band = 12.0f;
  float normal_radius = 8.0f;
  float curvature_radius = 10.0f;
  float quantile = 0.72f;
  float cluster_eps = 6.0f;
  int min_cluster = 25;
  float thin_radius = 10.0f;
  float sample_step = 2.0f;
  float min_length = 30.0f;
  bool visualize = true;
  std::string out = "weld_trajectory_mm.txt";
};

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

struct Plane {
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  float d = 0.0f;
  std::vector<int> inliers;
};

struct Curve {
  std::vector<Eigen::Vector3f> points;
};

float heightOf(const pcl::PointXYZ& point, const Plane& plane) {
  return plane.normal.dot(Eigen::Vector3f(point.x, point.y, point.z)) + plane.d;
}

Eigen::Vector3f toEigen(const pcl::PointXYZ& point) {
  return Eigen::Vector3f(point.x, point.y, point.z);
}

pcl::PointXYZ toPcl(const Eigen::Vector3f& point) {
  return pcl::PointXYZ(point.x(), point.y(), point.z());
}

Eigen::Vector3f projectToPlane(const Eigen::Vector3f& point, const Plane& plane) {
  return point - (plane.normal.dot(point) + plane.d) * plane.normal;
}

CloudPtr loadCloud(const std::string& path) {
  CloudPtr cloud(new Cloud);
  const auto dot = path.find_last_of('.');
  const std::string ext = dot == std::string::npos ? "" : path.substr(dot);
  int status = -1;
  if (ext == ".ply" || ext == ".PLY") {
    status = pcl::io::loadPLYFile(path, *cloud);
  } else {
    status = pcl::io::loadPCDFile(path, *cloud);
    if (status < 0) {
      status = pcl::io::loadPLYFile(path, *cloud);
    }
  }
  if (status < 0 || cloud->empty()) {
    throw std::runtime_error("无法读取点云: " + path);
  }
  std::vector<int> finite;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, finite);
  return cloud;
}

CloudPtr preprocess(const CloudPtr& cloud, float voxel) {
  CloudPtr down(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(cloud);
  grid.setLeafSize(voxel, voxel, voxel);
  grid.filter(*down);

  CloudPtr filtered(new Cloud);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(down);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*filtered);
  if (filtered->size() < 100) {
    throw std::runtime_error("降采样后点数过少。确认点云单位是毫米，或减小 --voxel。");
  }
  return filtered;
}

Plane fitBasePlane(const CloudPtr& cloud, float thresh) {
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(2000);
  seg.setDistanceThreshold(thresh);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coeff);
  if (inliers->indices.size() < 50 || coeff->values.size() < 4) {
    throw std::runtime_error("底板平面拟合失败。");
  }

  Eigen::Vector3f normal(coeff->values[0], coeff->values[1], coeff->values[2]);
  const float norm = normal.norm();
  normal /= norm;
  float d = coeff->values[3] / norm;

  int above = 0;
  int below = 0;
  for (const auto& point : cloud->points) {
    const float height = normal.dot(toEigen(point)) + d;
    if (height > thresh * 2.0f) {
      ++above;
    } else if (height < -thresh * 2.0f) {
      ++below;
    }
  }
  if (below > above) {
    normal = -normal;
    d = -d;
  }

  Plane plane;
  plane.normal = normal;
  plane.d = d;
  plane.inliers = inliers->indices;
  return plane;
}

std::vector<float> principalCurvature(const CloudPtr& cloud, float normal_radius, float curvature_radius) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setInputCloud(cloud);
  normal_est.setSearchMethod(tree);
  normal_est.setRadiusSearch(normal_radius);
  normal_est.compute(*normals);

  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curves(new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> curve_est;
  curve_est.setInputCloud(cloud);
  curve_est.setInputNormals(normals);
  curve_est.setSearchMethod(tree);
  curve_est.setRadiusSearch(curvature_radius);
  curve_est.compute(*curves);

  std::vector<float> curvature(cloud->size(), 0.0f);
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float pc1 = curves->points[i].pc1;
    const float pc2 = curves->points[i].pc2;
    if (std::isfinite(pc1) && std::isfinite(pc2)) {
      curvature[i] = std::max(std::fabs(pc1), std::fabs(pc2));
    }
  }
  return curvature;
}

float quantile(std::vector<float> values, float q) {
  if (values.empty()) {
    return 0.0f;
  }
  q = std::clamp(q, 0.0f, 1.0f);
  const std::size_t index = static_cast<std::size_t>(q * static_cast<float>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

float cloudSpan(const CloudPtr& cloud) {
  if (cloud->empty()) {
    return 0.0f;
  }
  Eigen::Vector3f lo = toEigen(cloud->points.front());
  Eigen::Vector3f hi = lo;
  for (const auto& point : cloud->points) {
    const Eigen::Vector3f p = toEigen(point);
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  return (hi - lo).norm();
}

CloudPtr gather(const CloudPtr& cloud, const std::vector<int>& indices) {
  CloudPtr out(new Cloud);
  out->reserve(indices.size());
  for (int index : indices) {
    out->push_back(cloud->points[static_cast<std::size_t>(index)]);
  }
  return out;
}

std::vector<CloudPtr> euclideanClusters(const CloudPtr& cloud, float eps, int min_points, float keep_ratio) {
  std::vector<CloudPtr> clusters;
  if (static_cast<int>(cloud->size()) < min_points) {
    return clusters;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> groups;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
  extractor.setClusterTolerance(eps);
  extractor.setMinClusterSize(min_points);
  extractor.setMaxClusterSize(static_cast<int>(cloud->size()));
  extractor.setSearchMethod(tree);
  extractor.setInputCloud(cloud);
  extractor.extract(groups);
  if (groups.empty()) {
    return clusters;
  }
  std::sort(groups.begin(), groups.end(), [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
    return a.indices.size() > b.indices.size();
  });
  const int cutoff = std::max(min_points, static_cast<int>(groups.front().indices.size() * keep_ratio));
  for (const auto& group : groups) {
    if (static_cast<int>(group.indices.size()) < cutoff) {
      break;
    }
    clusters.push_back(gather(cloud, group.indices));
  }
  return clusters;
}

std::vector<int> bandIndices(const CloudPtr& cloud, const Plane& plane, float gap, float band) {
  std::vector<int> indices;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float height = heightOf(cloud->points[i], plane);
    if (height > gap && height < band) {
      indices.push_back(static_cast<int>(i));
    }
  }
  return indices;
}

CloudPtr selectSeamPoints(const CloudPtr& cloud,
                          const Plane& plane,
                          const std::vector<float>& curvature,
                          const Params& params) {
  const std::vector<int> band = bandIndices(cloud, plane, params.gap, params.band);
  if (band.size() < 30) {
    throw std::runtime_error("接触带点数过少。增大 --band，或减小 --gap。");
  }
  std::vector<float> band_curvature;
  band_curvature.reserve(band.size());
  for (int index : band) {
    band_curvature.push_back(curvature[static_cast<std::size_t>(index)]);
  }
  const float p50 = quantile(band_curvature, 0.50f);
  const float p90 = quantile(band_curvature, 0.90f);
  const bool ridge = p90 > std::max(p50 * 1.8f, p50 + 1.0e-4f);
  const CloudPtr band_cloud = gather(cloud, band);

  std::vector<int> selected = band;
  if (ridge) {
    const float threshold = quantile(band_curvature, params.quantile);
    selected.clear();
    for (int index : band) {
      if (curvature[static_cast<std::size_t>(index)] >= threshold) {
        selected.push_back(index);
      }
    }
    const CloudPtr selected_cloud = gather(cloud, selected);
    if (selected.size() < 30 || cloudSpan(selected_cloud) < 0.70f * cloudSpan(band_cloud)) {
      std::cout << "高曲率点没有形成连续焊缝，改用整条接触带。\n";
      selected = band;
    } else {
      std::cout << "接触带曲率 p50=" << p50 << " p90=" << p90 << " ，保留高曲率点 " << selected.size()
                << " / " << band.size() << "\n";
    }
  } else {
    std::cout << "接触带曲率接近均匀 (p50=" << p50 << ", p90=" << p90
              << ")，焊缝取底板接触带而不是曲率阈值。\n";
  }
  std::cout << "接触带 " << band.size() << " 点，用于提轨迹 " << selected.size() << " 点\n";
  return gather(cloud, selected);
}

std::vector<Eigen::Vector3f> neighborPoints(const pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                                            const CloudPtr& cloud,
                                            const Eigen::Vector3f& query,
                                            float radius) {
  std::vector<int> indices;
  std::vector<float> distances;
  tree.radiusSearch(toPcl(query), radius, indices, distances);
  std::vector<Eigen::Vector3f> points;
  points.reserve(indices.size());
  for (int index : indices) {
    points.push_back(toEigen(cloud->points[static_cast<std::size_t>(index)]));
  }
  return points;
}

Eigen::Vector3f principalDirection(const std::vector<Eigen::Vector3f>& points, Eigen::Vector3f& mean) {
  mean = Eigen::Vector3f::Zero();
  for (const auto& point : points) {
    mean += point;
  }
  mean /= static_cast<float>(points.size());
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const auto& point : points) {
    const Eigen::Vector3f delta = point - mean;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<float>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  return solver.eigenvectors().col(2).normalized();
}

struct Thinned {
  CloudPtr cloud;
  std::vector<Eigen::Vector3f> directions;
};

Thinned thinLine(const CloudPtr& cloud, float radius) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  Thinned thinned;
  thinned.cloud.reset(new Cloud);
  thinned.cloud->reserve(cloud->size());
  thinned.directions.reserve(cloud->size());
  for (const auto& point : cloud->points) {
    const Eigen::Vector3f origin = toEigen(point);
    const std::vector<Eigen::Vector3f> neighbors = neighborPoints(tree, cloud, origin, radius);
    if (neighbors.size() < 3) {
      thinned.cloud->push_back(point);
      thinned.directions.push_back(Eigen::Vector3f::UnitX());
      continue;
    }
    Eigen::Vector3f mean;
    const Eigen::Vector3f direction = principalDirection(neighbors, mean);
    const Eigen::Vector3f moved = mean + direction.dot(origin - mean) * direction;
    thinned.cloud->push_back(toPcl(moved));
    thinned.directions.push_back(direction);
  }
  return thinned;
}

CloudPtr voxelAverage(const CloudPtr& cloud, float voxel) {
  CloudPtr out(new Cloud);
  if (cloud->empty() || voxel <= 0.0f) {
    return cloud;
  }
  struct Key {
    int x, y, z;
    bool operator<(const Key& other) const {
      if (x != other.x) return x < other.x;
      if (y != other.y) return y < other.y;
      return z < other.z;
    }
  };
  std::vector<Key> keys(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& point = cloud->points[i];
    keys[i] = Key{static_cast<int>(std::floor(point.x / voxel)),
                  static_cast<int>(std::floor(point.y / voxel)),
                  static_cast<int>(std::floor(point.z / voxel))};
  }
  std::vector<int> order(cloud->size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) { return keys[static_cast<std::size_t>(a)] < keys[static_cast<std::size_t>(b)]; });
  for (std::size_t begin = 0; begin < order.size();) {
    std::size_t end = begin + 1;
    while (end < order.size() &&
           !(keys[static_cast<std::size_t>(order[begin])] < keys[static_cast<std::size_t>(order[end])]) &&
           !(keys[static_cast<std::size_t>(order[end])] < keys[static_cast<std::size_t>(order[begin])])) {
      ++end;
    }
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (std::size_t i = begin; i < end; ++i) {
      mean += toEigen(cloud->points[static_cast<std::size_t>(order[i])]);
    }
    mean /= static_cast<float>(end - begin);
    out->push_back(toPcl(mean));
    begin = end;
  }
  return out;
}

std::vector<int> walkCurve(const CloudPtr& cloud, const std::vector<Eigen::Vector3f>& directions, int start, float step) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);
  std::vector<char> visited(cloud->size(), 0);
  visited[static_cast<std::size_t>(start)] = 1;
  std::vector<int> ordered{start};

  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& point : cloud->points) {
    centroid += toEigen(point);
  }
  centroid /= static_cast<float>(cloud->size());

  Eigen::Vector3f heading = directions[static_cast<std::size_t>(start)];
  if (heading.dot(centroid - toEigen(cloud->points[static_cast<std::size_t>(start)])) < 0.0f) {
    heading = -heading;
  }
  const float search = step * 0.85f;
  int index = start;
  while (ordered.size() < cloud->size()) {
    Eigen::Vector3f proposed = directions[static_cast<std::size_t>(index)];
    if (heading.dot(proposed) < 0.0f) {
      proposed = -proposed;
    }
    if (heading.dot(proposed) > 0.45f) {
      heading = proposed.normalized();
    }
    const Eigen::Vector3f origin = toEigen(cloud->points[static_cast<std::size_t>(index)]);
    const Eigen::Vector3f predicted = origin + heading * step;

    auto candidatesIn = [&](float radius) {
      std::vector<int> indices;
      std::vector<float> distances;
      tree.radiusSearch(toPcl(predicted), radius, indices, distances);
      std::vector<int> open;
      for (int candidate : indices) {
        if (!visited[static_cast<std::size_t>(candidate)]) {
          open.push_back(candidate);
        }
      }
      return open;
    };
    std::vector<int> candidates = candidatesIn(search);
    if (candidates.empty()) {
      candidates = candidatesIn(search * 2.0f);
    }
    if (candidates.empty()) {
      break;
    }

    int chosen = -1;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int candidate : candidates) {
      const Eigen::Vector3f delta = toEigen(cloud->points[static_cast<std::size_t>(candidate)]) - origin;
      const float length = delta.norm();
      if (length < 1.0e-6f) {
        continue;
      }
      const float cosine = delta.dot(heading) / length;
      if (cosine <= 0.25f) {
        continue;
      }
      const float predicted_distance = (toEigen(cloud->points[static_cast<std::size_t>(candidate)]) - predicted).norm();
      const float score = cosine - 0.2f * predicted_distance / step;
      if (score > best_score) {
        best_score = score;
        chosen = candidate;
      }
    }
    if (chosen < 0) {
      break;
    }
    visited[static_cast<std::size_t>(chosen)] = 1;
    ordered.push_back(chosen);
    index = chosen;
  }
  return ordered;
}

std::vector<Eigen::Vector3f> sortOpenCurve(const CloudPtr& cloud, const std::vector<Eigen::Vector3f>& directions, float step) {
  if (cloud->size() < 4) {
    std::vector<Eigen::Vector3f> raw;
    for (const auto& point : cloud->points) raw.push_back(toEigen(point));
    return raw;
  }
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const auto& point : cloud->points) mean += toEigen(point);
  mean /= static_cast<float>(cloud->size());
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  for (const auto& point : cloud->points) {
    const Eigen::Vector3f delta = toEigen(point) - mean;
    covariance += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  const Eigen::Vector3f axis = solver.eigenvectors().col(2);
  int low = 0;
  int high = 0;
  float low_value = std::numeric_limits<float>::infinity();
  float high_value = -std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const float projection = (toEigen(cloud->points[i]) - mean).dot(axis);
    if (projection < low_value) {
      low_value = projection;
      low = static_cast<int>(i);
    }
    if (projection > high_value) {
      high_value = projection;
      high = static_cast<int>(i);
    }
  }

  auto asPoints = [&](const std::vector<int>& indices) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(indices.size());
    for (int index : indices) points.push_back(toEigen(cloud->points[static_cast<std::size_t>(index)]));
    return points;
  };
  auto lengthOf = [](const std::vector<Eigen::Vector3f>& points) {
    float length = 0.0f;
    for (std::size_t i = 1; i < points.size(); ++i) length += (points[i] - points[i - 1]).norm();
    return length;
  };

  std::vector<Eigen::Vector3f> best;
  float best_length = -1.0f;
  for (int start : {low, high}) {
    const std::vector<Eigen::Vector3f> ordered = asPoints(walkCurve(cloud, directions, start, step));
    const float length = lengthOf(ordered);
    if (length > best_length) {
      best = ordered;
      best_length = length;
    }
  }
  return best;
}

std::vector<Eigen::Vector3f> cleanPolyline(const std::vector<Eigen::Vector3f>& points, float min_step) {
  std::vector<Eigen::Vector3f> kept;
  if (points.empty()) return kept;
  kept.push_back(points.front());
  for (std::size_t i = 1; i < points.size(); ++i) {
    if ((points[i] - kept.back()).norm() < min_step) continue;
    if (kept.size() >= 2 && (kept.back() - kept[kept.size() - 2]).dot(points[i] - kept.back()) <= 0.0f) continue;
    kept.push_back(points[i]);
  }
  return kept;
}

std::vector<Eigen::Vector3f> resample(const std::vector<Eigen::Vector3f>& points, float step) {
  if (points.size() < 2) return points;
  std::vector<float> station{0.0f};
  for (std::size_t i = 1; i < points.size(); ++i) {
    station.push_back(station.back() + (points[i] - points[i - 1]).norm());
  }
  const float total = station.back();
  if (total < step) return points;
  std::vector<Eigen::Vector3f> out;
  for (float target = 0.0f; target < total; target += step) {
    const auto upper = std::upper_bound(station.begin(), station.end(), target);
    std::size_t index = static_cast<std::size_t>(std::distance(station.begin(), upper));
    if (index == 0) index = 1;
    if (index >= station.size()) index = station.size() - 1;
    const float span = station[index] - station[index - 1];
    const float t = span > 1.0e-6f ? (target - station[index - 1]) / span : 0.0f;
    out.push_back(points[index - 1] * (1.0f - t) + points[index] * t);
  }
  if (out.empty() || (out.back() - points.back()).norm() > step * 0.3f) {
    out.push_back(points.back());
  }
  return out;
}

std::vector<Eigen::Vector3f> traceCluster(const CloudPtr& cluster, const Plane& plane, const Params& params) {
  CloudPtr projected(new Cloud);
  projected->reserve(cluster->size());
  for (const auto& point : cluster->points) {
    projected->push_back(toPcl(projectToPlane(toEigen(point), plane)));
  }
  if (projected->size() < 15) return {};

  const Thinned thinned = thinLine(projected, params.thin_radius);
  const float voxel = std::max(params.sample_step, params.thin_radius * 0.35f);
  CloudPtr centers = voxelAverage(thinned.cloud, voxel);
  if (centers->size() < 8) return {};

  const Thinned retangent = thinLine(centers, std::max(params.thin_radius, params.sample_step * 4.0f));
  const float walk_step = std::max(params.sample_step * 2.0f, params.thin_radius * 0.45f);
  std::vector<Eigen::Vector3f> ordered = sortOpenCurve(centers, retangent.directions, walk_step);
  ordered = cleanPolyline(ordered, params.sample_step * 0.8f);
  if (ordered.size() < 4) return {};
  return resample(ordered, params.sample_step);
}

float polylineLength(const std::vector<Eigen::Vector3f>& points) {
  float length = 0.0f;
  for (std::size_t i = 1; i < points.size(); ++i) length += (points[i] - points[i - 1]).norm();
  return length;
}

void saveTrajectory(const std::string& path, const std::vector<Curve>& curves) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("无法写入 " + path);
  stream << "# seam_id x_mm y_mm z_mm\n";
  stream.setf(std::ios::fixed);
  stream.precision(6);
  for (std::size_t seam = 0; seam < curves.size(); ++seam) {
    for (const auto& point : curves[seam].points) {
      stream << seam << " " << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
  }
  std::cout << "轨迹已写入 " << path << "\n";
}

void visualize(const CloudPtr& cloud,
               const Plane& plane,
               const CloudPtr& seam,
               const std::vector<Curve>& curves) {
  pcl::visualization::PCLVisualizer viewer("弯曲焊缝  灰=场景  红=曲率/接触点  绿=焊根");
  viewer.setBackgroundColor(0.04, 0.05, 0.10);

  CloudPtr plate(new Cloud);
  CloudPtr workpiece(new Cloud);
  std::vector<char> on_plane(cloud->size(), 0);
  for (int index : plane.inliers) on_plane[static_cast<std::size_t>(index)] = 1;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    (on_plane[i] ? plate : workpiece)->push_back(cloud->points[i]);
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plate_color(plate, 70, 80, 100);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> part_color(workpiece, 170, 174, 180);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> seam_color(seam, 240, 110, 30);
  viewer.addPointCloud(plate, plate_color, "plate");
  viewer.addPointCloud(workpiece, part_color, "part");
  viewer.addPointCloud(seam, seam_color, "seam");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plate");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "part");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "seam");

  int line_id = 0;
  for (const auto& curve : curves) {
    for (std::size_t i = 1; i < curve.points.size(); ++i) {
      const Eigen::Vector3f a = curve.points[i - 1] + plane.normal * 3.0f;
      const Eigen::Vector3f b = curve.points[i] + plane.normal * 3.0f;
      viewer.addLine(toPcl(a), toPcl(b), 0.15, 0.95, 0.25, "line" + std::to_string(line_id++));
    }
  }
  viewer.addCoordinateSystem(30.0);
  std::cout << "关闭可视化窗口后程序结束。\n";
  while (!viewer.wasStopped()) {
    viewer.spinOnce(16);
  }
}

Params parseArgs(int argc, char** argv, std::string& cloud_path) {
  Params params;
  if (argc < 2) {
    throw std::runtime_error(
        "用法: weld_curvature <cloud.pcd|ply> [--voxel 1] [--plane 1.5] [--gap 2] [--band 12]\n"
        "       [--quantile 0.72] [--cluster 6] [--no-vis] [--out weld_trajectory_mm.txt]");
  }
  cloud_path = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string key = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("缺少参数值: ") + name);
      return std::string(argv[++i]);
    };
    if (key == "--voxel") params.voxel = std::stof(need("--voxel"));
    else if (key == "--plane") params.plane_thresh = std::stof(need("--plane"));
    else if (key == "--gap") params.gap = std::stof(need("--gap"));
    else if (key == "--band") params.band = std::stof(need("--band"));
    else if (key == "--quantile") params.quantile = std::stof(need("--quantile"));
    else if (key == "--cluster") params.cluster_eps = std::stof(need("--cluster"));
    else if (key == "--normal-radius") params.normal_radius = std::stof(need("--normal-radius"));
    else if (key == "--curvature-radius") params.curvature_radius = std::stof(need("--curvature-radius"));
    else if (key == "--thin-radius") params.thin_radius = std::stof(need("--thin-radius"));
    else if (key == "--step") params.sample_step = std::stof(need("--step"));
    else if (key == "--min-length") params.min_length = std::stof(need("--min-length"));
    else if (key == "--out") params.out = need("--out");
    else if (key == "--no-vis") params.visualize = false;
    else throw std::runtime_error("未知参数: " + key);
  }
  return params;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string cloud_path;
    const Params params = parseArgs(argc, argv, cloud_path);
    const CloudPtr raw = loadCloud(cloud_path);
    std::cout << "读取 " << raw->size() << " 点\n";
    const CloudPtr cloud = preprocess(raw, params.voxel);
    std::cout << "降采样后 " << cloud->size() << " 点，体素 " << params.voxel << " mm\n";

    const Plane plane = fitBasePlane(cloud, params.plane_thresh);
    const float inlier_ratio = static_cast<float>(plane.inliers.size()) / static_cast<float>(cloud->size());
    std::cout << "底板内点 " << plane.inliers.size() << " (" << static_cast<int>(inlier_ratio * 100.0f)
              << "%)，法向 " << plane.normal.transpose() << "\n";

    const std::vector<float> curvature =
        principalCurvature(cloud, params.normal_radius, params.curvature_radius);
    const CloudPtr seam = selectSeamPoints(cloud, plane, curvature, params);

    std::vector<CloudPtr> clusters = euclideanClusters(seam, params.cluster_eps, params.min_cluster, 0.30f);
    if (clusters.empty()) clusters.push_back(seam);
    std::cout << "焊缝簇 " << clusters.size() << " 条\n";

    std::vector<Curve> curves;
    for (const auto& cluster : clusters) {
      Curve curve;
      curve.points = traceCluster(cluster, plane, params);
      const float length = polylineLength(curve.points);
      if (curve.points.size() < 4 || length < params.min_length) continue;
      curves.push_back(std::move(curve));
      std::cout << "  轨迹 " << curves.size() << ": " << curves.back().points.size() << " 点, 长度 " << length
                << " mm\n";
    }
    if (curves.empty()) {
      throw std::runtime_error("没有形成足够长的焊缝轨迹。尝试增大 --band 或 --cluster。");
    }
    saveTrajectory(params.out, curves);
    if (params.visualize) visualize(cloud, plane, seam, curves);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
}
