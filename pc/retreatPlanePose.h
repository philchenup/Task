#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 输入为相机坐标系下的点云，单位毫米。点类型与 cloud.h 中的 PointXYZRGBN 相同，
// ct::Cloud 可以直接传入。按点云中的全部有限点计算。
// 少于 30 个点，或没有法向与相机 +Z 夹角不超过 10° 的平面时返回 false，且不修改 pose。
// 成功时对内点做最小二乘精细拟合，输出平面质心沿该法向反向平移 300mm 的 Eigen::Affine3d。
// 位姿 Z 轴与相机 +Z 同向，X/Y 朝向相机坐标轴。
bool computeRetreatPlanePose(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, Eigen::Affine3d& pose);
