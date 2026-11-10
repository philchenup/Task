#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <vector>

namespace lobb {

struct DemoCloud {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Eigen::Vector3d> ground_truth;
  const char* name = "";
  // 评测阈值，单位与点云一致（演示数据为毫米）。
  double max_mean_error = 1.0;
  double max_p90_error = 1.8;
  double min_coverage = 0.8;
  double coverage_distance = 2.5;
};

DemoCloud makeSineFilletCloud();
DemoCloud makePipePlateCloud();

struct CurveScore {
  double mean = 0.0;
  double p90 = 0.0;
  double coverage = 0.0;
};

// seam 点到真值曲线的距离，以及真值曲线被焊缝覆盖的比例。
CurveScore scoreCurve(const std::vector<Eigen::Vector3d>& seam,
                      const std::vector<Eigen::Vector3d>& ground_truth,
                      double coverage_distance);

}  // namespace lobb
