/*
 * 眼在手上：用基座系中已知的一个尖点，以及相机在多个末端姿态下看到的该点，求手眼矩阵。
 *
 * 已知
 *   p_B                         三角锥尖点在基座系中的坐标（TCP 触碰得到）
 *   T_BE,i                      第 i 次拍摄时末端在基座系中的位姿（Affine3d，作用在点上：p_B = T_BE p_E）
 *   p_C,i                       同一尖点在相机系中的坐标
 * 未知
 *   T_EC = (R, t)               把相机系中的点变到末端系：p_E = R p_C + t
 *
 * 由刚体连接：
 *   p_B = R_BE (R p_C + t) + t_BE
 *   p_E := R_BE^T (p_B - t_BE) = R p_C + t
 * 多个姿态给出点对 (p_C,i, p_E,i)，用 Kabsch 求唯一刚体变换。
 * 相机系中的尖点样本不共线时解唯一；共线时绕该直线的转动不可观测。
 */

#ifndef HAND_EYE_APEX_HPP
#define HAND_EYE_APEX_HPP

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

struct HandEyeEstimate
{
  Eigen::Matrix4d T_ec = Eigen::Matrix4d::Identity(); // 相机系 -> 末端系
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double rms_m = 0;          // 点对配准的均方根残差
  double second_to_first = 0; // 相机点云协方差的 σ2/σ1，过小表示接近共线
  bool degenerate = false;
};

inline Eigen::Vector3d apexInEndEffector(const Eigen::Vector3d &apex_base, const Eigen::Affine3d &T_be)
{
  return T_be.inverse() * apex_base;
}

inline HandEyeEstimate solveEyeInHandFromApex(const Eigen::Vector3d &apex_base,
                                               const std::vector<Eigen::Affine3d> &T_be,
                                               const std::vector<Eigen::Vector3d> &apex_camera)
{
  const std::size_t n = T_be.size();
  if (n != apex_camera.size()) {
    throw std::invalid_argument("poses and camera points must have the same length");
  }
  if (n < 3) {
    throw std::invalid_argument("at least 3 poses are required");
  }

  std::vector<Eigen::Vector3d> p_e(n);
  Eigen::Vector3d mean_c = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_e = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    p_e[i] = apexInEndEffector(apex_base, T_be[i]);
    mean_c += apex_camera[i];
    mean_e += p_e[i];
  }
  mean_c /= static_cast<double>(n);
  mean_e /= static_cast<double>(n);

  // H = Σ (p_E - mean_E) (p_C - mean_C)^T ，使 R 满足 p_E ≈ R p_C + t
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d cov_c = Eigen::Matrix3d::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d qc = apex_camera[i] - mean_c;
    const Eigen::Vector3d qe = p_e[i] - mean_e;
    H += qe * qc.transpose();
    cov_c += qc * qc.transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd_h(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d U = svd_h.matrixU();
  const Eigen::Matrix3d V = svd_h.matrixV();
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  // det(U V^T) < 0 时是反射，把最小奇异方向翻到 SO(3)
  if ((U * V.transpose()).determinant() < 0.0) {
    D(2, 2) = -1.0;
  }
  const Eigen::Matrix3d R = U * D * V.transpose();
  const Eigen::Vector3d t = mean_e - R * mean_c;

  double sum_sq = 0;
  for (std::size_t i = 0; i < n; ++i) {
    sum_sq += (R * apex_camera[i] + t - p_e[i]).squaredNorm();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd_c(cov_c, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d sv = svd_c.singularValues();
  const double ratio = (sv(0) > 0.0) ? sv(1) / sv(0) : 0.0;

  HandEyeEstimate out;
  out.R = R;
  out.t = t;
  out.T_ec.setIdentity();
  out.T_ec.block<3, 3>(0, 0) = R;
  out.T_ec.block<3, 1>(0, 3) = t;
  out.rms_m = std::sqrt(sum_sq / static_cast<double>(n));
  out.second_to_first = ratio;
  // 第二奇异值远小于第一奇异值：样本接近共线，绕该轴的旋转没有被观测到
  out.degenerate = ratio < 1e-2;
  return out;
}

#endif
