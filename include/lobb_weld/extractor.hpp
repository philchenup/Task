#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace lobb {

// 点的类别，对应论文里分层聚类的输出。
// 平面点是背景；折痕是曲面交线（焊缝）；边界是工件外轮廓；角点是轮廓拐角或焊缝端点。
enum class PointLabel { Plane = 0, Boundary = 1, Crease = 2, Corner = 3 };

struct ExtractParams {
  // 体素边长，与点云同单位。<=0 表示不降采样。
  float voxel = 0.f;
  bool statistical_filter = true;
  int sor_mean_k = 20;
  float sor_stddev = 2.5f;

  // LOBB 邻域半径。<=0 时按中位最近邻距离 × radius_nn_factor 自动取值。
  // 半径需要大到能同时看到焊缝两侧的曲面，大约若干倍点间距。
  float feature_radius = 0.f;
  float radius_nn_factor = 6.5f;

  int min_neighbors = 12;
  // Sigmoid 增益。越大，激活后两类分得越开。
  float activation_gain = 12.f;
  // 两侧法向点积大于该值时认为不是折痕（曲面太平或只是光滑弯曲）。
  float parallel_normal_dot = 0.92f;
  float min_cluster_fraction = 0.12f;
};

struct SeamPose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::UnitX();    // 焊接行进方向
  Eigen::Vector3d approach = Eigen::Vector3d::UnitZ();   // 焊枪指向焊缝的方向
  Eigen::Vector3d binormal = Eigen::Vector3d::UnitY();
};

struct SeamCurve {
  std::vector<SeamPose> poses;
  bool closed = false;
  double length = 0.0;
};

struct ExtractResult {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<PointLabel> labels;
  std::vector<SeamCurve> seams;
  float median_nn = 0.f;
  float feature_radius = 0.f;
  float normal_radius = 0.f;
  int plane_count = 0;
  int boundary_count = 0;
  int crease_count = 0;
  int corner_count = 0;
};

// 按 Zhang 等，IEEE TASE 2025（DOI 10.1109/TASE.2023.3345868）的流程提取曲面焊缝：
//   1. 为每个点构建局部有向包围盒（LOBB）特征；
//   2. 对特征做非线性激活，拉开平面 / 折痕 / 边界；
//   3. 分层 K-means 得到折痕点、边界点和角点；
//   4. 在折痕带上拟合两侧切平面，交线即曲面焊缝，再排序、重采样并给出焊枪姿态。
ExtractResult extractCurvedSeam(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                const ExtractParams& params);

const char* labelName(PointLabel label);

}  // namespace lobb
