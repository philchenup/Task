#include "hand_eye_apex.hpp"

#include <cstdlib>
#include <iostream>
#include <random>

namespace
{
Eigen::Matrix3d randomRotation(std::mt19937 &rng)
{
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::Matrix3d A;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      A(r, c) = gauss(rng);
    }
  }
  Eigen::HouseholderQR<Eigen::Matrix3d> qr(A);
  Eigen::Matrix3d Q = qr.householderQ();
  if (Q.determinant() < 0.0) {
    Q.col(2) *= -1.0;
  }
  return Q;
}

Eigen::Affine3d poseFromIsometry(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}
} // namespace

int main()
{
  std::mt19937 rng(7);
  std::normal_distribution<double> trans(0.0, 0.4);
  std::normal_distribution<double> noise(0.0, 0.0005); // 0.5 mm

  const Eigen::Matrix3d R_true = randomRotation(rng);
  const Eigen::Vector3d t_true(0.04, -0.02, 0.08);
  const Eigen::Vector3d apex_base(0.55, 0.10, 0.02);

  std::vector<Eigen::Affine3d> poses;
  std::vector<Eigen::Vector3d> apex_camera;
  poses.reserve(12);
  for (int i = 0; i < 12; ++i) {
    Eigen::Affine3d pose = poseFromIsometry(randomRotation(rng), Eigen::Vector3d(trans(rng), trans(rng), 0.4 + trans(rng)));
    // 末端必须在尖点附近，且尖点位于相机前方
    const Eigen::Vector3d p_e = pose.inverse() * apex_base;
    const Eigen::Vector3d p_c = R_true.transpose() * (p_e - t_true);
    if (p_c.z() < 0.05) {
      --i;
      continue;
    }
    poses.push_back(pose);
    apex_camera.push_back(p_c);
  }

  const HandEyeEstimate exact = solveEyeInHandFromApex(apex_base, poses, apex_camera);
  const double rot_err = Eigen::AngleAxisd(exact.R.transpose() * R_true).angle();
  const double trans_err = (exact.t - t_true).norm();
  std::cout << "noise-free rotation error (deg) " << rot_err * 180.0 / M_PI << "\n";
  std::cout << "noise-free translation error (mm) " << trans_err * 1000.0 << "\n";
  std::cout << "noise-free rms (mm) " << exact.rms_m * 1000.0 << "  sigma2/sigma1 " << exact.second_to_first
            << "  degenerate " << exact.degenerate << "\n";

  int failures = 0;
  if (rot_err > 1e-8 || trans_err > 1e-8 || exact.degenerate) {
    std::cerr << "exact recovery failed\n";
    ++failures;
  }

  std::vector<Eigen::Vector3d> noisy = apex_camera;
  for (Eigen::Vector3d &p : noisy) {
    p += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
  }
  const HandEyeEstimate noisy_est = solveEyeInHandFromApex(apex_base, poses, noisy);
  const double rot_n = Eigen::AngleAxisd(noisy_est.R.transpose() * R_true).angle();
  const double trans_n = (noisy_est.t - t_true).norm();
  std::cout << "0.5 mm noise rotation error (deg) " << rot_n * 180.0 / M_PI << "\n";
  std::cout << "0.5 mm noise translation error (mm) " << trans_n * 1000.0 << "\n";
  std::cout << "0.5 mm noise rms (mm) " << noisy_est.rms_m * 1000.0 << "\n";
  if (rot_n > 2.0 * M_PI / 180.0 || trans_n > 0.005) {
    std::cerr << "noisy recovery outside expected bound\n";
    ++failures;
  }

  // 共线：相机只沿一条直线平移去看尖点，绕该轴的旋转不可观测
  std::vector<Eigen::Affine3d> line_poses;
  std::vector<Eigen::Vector3d> line_cam;
  const Eigen::Matrix3d R_fixed = randomRotation(rng);
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector3d cam_point(0.0, 0.0, 0.3 + 0.02 * i);
    const Eigen::Vector3d p_e = R_true * cam_point + t_true;
    line_poses.push_back(poseFromIsometry(R_fixed, apex_base - R_fixed * p_e));
    line_cam.push_back(cam_point);
  }
  const HandEyeEstimate line = solveEyeInHandFromApex(apex_base, line_poses, line_cam);
  std::cout << "collinear sigma2/sigma1 " << line.second_to_first << " degenerate " << line.degenerate << "\n";
  if (!line.degenerate) {
    std::cerr << "collinear configuration was not flagged\n";
    ++failures;
  }

  if (failures != 0) {
    return EXIT_FAILURE;
  }
  std::cout << "ALL TESTS PASSED\n";
  return EXIT_SUCCESS;
}
