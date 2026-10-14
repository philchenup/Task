/*
 * UrKinematics 自检：
 *  1. 零位正运动学与 UR 官方 DH 数据对比
 *  2. 解析雅可比 eJe / fJe 与数值微分雅可比对比
 *  3. 验证 eJe 对基座坐标系旋转不敏感
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpMath.h>

#include "ur_kinematics.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
// 数值几何雅可比：对每个关节做有限差分，速度旋量用 vpExponentialMap::inverse 提取
void numericalJacobian(const UrKinematics &kin, const vpColVector &q, vpMatrix &eJe_num, vpMatrix &fJe_num)
{
  const double h = 1e-6;
  eJe_num.resize(6, 6);
  fJe_num.resize(6, 6);
  vpHomogeneousMatrix fMe = kin.get_fMe(q);
  for (unsigned int i = 0; i < 6; ++i) {
    vpColVector qp = q, qm = q;
    qp[i] += h;
    qm[i] -= h;
    vpHomogeneousMatrix fMe_p = kin.get_fMe(qp);
    vpHomogeneousMatrix fMe_m = kin.get_fMe(qm);

    // 末端系中的速度旋量：e(-)Me(+) = exp(v * 2h)
    vpHomogeneousMatrix dMe = fMe_m.inverse() * fMe_p;
    vpColVector v_e = vpExponentialMap::inverse(dMe, 2 * h);
    // 基座系中：线速度为原点位移的差分，角速度旋转到基座系
    vpColVector v_f(6);
    for (unsigned int r = 0; r < 3; ++r) {
      v_f[r] = (fMe_p[r][3] - fMe_m[r][3]) / (2 * h);
    }
    vpColVector w_e(3);
    for (unsigned int r = 0; r < 3; ++r) w_e[r] = v_e[r + 3];
    vpColVector w_f = vpMatrix(fMe.getRotationMatrix()) * w_e;
    for (unsigned int r = 0; r < 3; ++r) v_f[r + 3] = w_f[r];

    for (unsigned int r = 0; r < 6; ++r) {
      eJe_num[r][i] = v_e[r];
      fJe_num[r][i] = v_f[r];
    }
  }
}
} // namespace

int main()
{
  int failures = 0;

  // ---- 1. UR5e 零位 ----
  {
    UrKinematics kin(UrKinematics::UR5e);
    const UrDhParameters &dh = kin.getDhParameters();
    vpColVector q(6, 0.0);
    vpHomogeneousMatrix fMe = kin.get_fMe(q);
    // 零位时 UR 的 TCP 位置为 (a2+a3, -(d4+d6), d1-d5)
    double ex = dh.a2 + dh.a3, ey = -(dh.d4 + dh.d6), ez = dh.d1 - dh.d5;
    double err = std::sqrt(vpMath::sqr(fMe[0][3] - ex) + vpMath::sqr(fMe[1][3] - ey) + vpMath::sqr(fMe[2][3] - ez));
    std::cout << "[FK zero] p = (" << fMe[0][3] << ", " << fMe[1][3] << ", " << fMe[2][3] << ")  expected (" << ex
      << ", " << ey << ", " << ez << ")  err=" << err << std::endl;
    if (err > 1e-9) failures++;
  }

  // ---- 2. 雅可比：随机位形下与数值雅可比对比 ----
  {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-M_PI, M_PI);
    const UrKinematics::Model models[] = { UrKinematics::UR3e, UrKinematics::UR5, UrKinematics::UR5e, UrKinematics::UR10e };
    for (UrKinematics::Model m : models) {
      UrKinematics kin(m);
      // 同时测试带 TCP 偏置的情况
      kin.set_flangeMtcp(vpHomogeneousMatrix(0.01, -0.02, 0.15, 0.1, -0.2, 0.3));
      double max_err_e = 0, max_err_f = 0;
      for (int trial = 0; trial < 50; ++trial) {
        vpColVector q(6);
        for (unsigned int i = 0; i < 6; ++i) q[i] = U(rng);
        vpMatrix eJe, fJe, eJe_num, fJe_num;
        kin.get_eJe(q, eJe);
        kin.get_fJe(q, fJe);
        numericalJacobian(kin, q, eJe_num, fJe_num);
        max_err_e = std::max(max_err_e, (eJe - eJe_num).getMaxValue());
        max_err_e = std::max(max_err_e, -(eJe - eJe_num).getMinValue());
        max_err_f = std::max(max_err_f, (fJe - fJe_num).getMaxValue());
        max_err_f = std::max(max_err_f, -(fJe - fJe_num).getMinValue());
      }
      std::cout << "[Jacobian model " << m << "] max|eJe-num| = " << max_err_e << "  max|fJe-num| = " << max_err_f
        << std::endl;
      if (max_err_e > 1e-5 || max_err_f > 1e-5) failures++;
    }
  }

  // ---- 3. eJe 与基座坐标系无关：把基座绕 z 旋转 180° 不应改变 eJe ----
  {
    UrKinematics kin(UrKinematics::UR5e);
    vpColVector q(6);
    q[0] = 0.3; q[1] = -1.9; q[2] = 1.7; q[3] = -1.4; q[4] = -1.6; q[5] = 0.5;
    vpMatrix eJe, fJe;
    kin.get_eJe(q, eJe);
    kin.get_fJe(q, fJe);
    vpHomogeneousMatrix fpMf(0, 0, 0, 0, 0, M_PI);
    vpVelocityTwistMatrix fpVf(fpMf);
    vpMatrix fpJe = fpVf * fJe;
    vpHomogeneousMatrix fpMe = fpMf * kin.get_fMe(q);
    vpVelocityTwistMatrix eVfp(vpTranslationVector(0, 0, 0), fpMe.getRotationMatrix().inverse());
    vpMatrix eJe_from_fp = eVfp * fpJe;
    double err = std::max((eJe - eJe_from_fp).getMaxValue(), -(eJe - eJe_from_fp).getMinValue());
    std::cout << "[eJe base invariance] err = " << err << std::endl;
    if (err > 1e-12) failures++;
  }

  std::cout << (failures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << std::endl;
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
