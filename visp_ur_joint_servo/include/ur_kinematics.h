/*
 * UR 系列机械臂运动学模型（标准 DH 参数）
 *
 * ViSP 自带的 vpRobotUniversalRobots 没有实现 get_eJe()/get_fJe()，
 * 因此 vpServo 无法直接以 EYEINHAND_L_cVe_eJe 模式输出关节速度。
 * 本文件补上这一环：给定关节角 q，计算
 *   - fMe : 末端(法兰/TCP)在基座坐标系下的位姿（正运动学）
 *   - fJe : 基座坐标系下表达的几何雅可比
 *   - eJe : 末端坐标系下表达的几何雅可比  <-- vpServo::set_eJe() 需要的量
 *
 * DH 参数来源：Universal Robots 官方文档
 * "Parameters for calculations of kinematics and dynamics"
 * 坐标系约定与 UR 控制器 (RTDE getActualTCPPose, Base 特征坐标系) 一致。
 */

#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

struct UrDhParameters
{
  double d1, a2, a3, d4, d5, d6;
};

class UrKinematics
{
public:
  enum Model { UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e, UR20, UR30 };

  explicit UrKinematics(Model model = UR5e) : m_tcp_M_set(false) { setModel(model); }

  explicit UrKinematics(const std::string &model_name) : m_tcp_M_set(false) { setModel(parseModel(model_name)); }

  static Model parseModel(const std::string &name)
  {
    std::string s;
    for (char c : name) {
      s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (s == "ur3") return UR3;
    if (s == "ur3e") return UR3e;
    if (s == "ur5") return UR5;
    if (s == "ur5e") return UR5e;
    if (s == "ur10") return UR10;
    if (s == "ur10e") return UR10e;
    if (s == "ur16e") return UR16e;
    if (s == "ur20") return UR20;
    if (s == "ur30") return UR30;
    throw std::invalid_argument("Unknown UR model: " + name);
  }

  void setModel(Model model)
  {
    switch (model) {
    case UR3:   m_dh = { 0.1519,  -0.24365, -0.21325, 0.11235,  0.08535, 0.0819  }; break;
    case UR3e:  m_dh = { 0.15185, -0.24355, -0.2132,  0.13105,  0.08535, 0.0921  }; break;
    case UR5:   m_dh = { 0.089159, -0.425,  -0.39225, 0.10915,  0.09465, 0.0823  }; break;
    case UR5e:  m_dh = { 0.1625,  -0.425,   -0.3922,  0.1333,   0.0997,  0.0996  }; break;
    case UR10:  m_dh = { 0.1273,  -0.612,   -0.5723,  0.163941, 0.1157,  0.0922  }; break;
    case UR10e: m_dh = { 0.1807,  -0.6127,  -0.57155, 0.17415,  0.11985, 0.11655 }; break;
    case UR16e: m_dh = { 0.1807,  -0.4784,  -0.36,    0.17415,  0.11985, 0.11655 }; break;
    case UR20:  m_dh = { 0.2363,  -0.8620,  -0.7287,  0.2010,   0.1593,  0.1543  }; break;
    case UR30:  m_dh = { 0.2363,  -0.6370,  -0.5037,  0.2010,   0.1593,  0.1543  }; break;
    }
  }

  void setDhParameters(const UrDhParameters &dh) { m_dh = dh; }
  const UrDhParameters &getDhParameters() const { return m_dh; }

  /*!
   * 可选：设置控制器中配置的 TCP 偏置（flange -> tcp）。
   * 若手眼标定 eMc 是相对于控制器 TCP 做的（ViSP 的 vpRobotUniversalRobots::getPosition(END_EFFECTOR_FRAME)
   * 返回的就是 TCP 位姿），则这里应传入 ur_rtde 的 getTCPOffset()，
   * 这样 "e" 坐标系才与标定时一致。不设置时 e = 法兰坐标系 (tool0)。
   */
  void set_flangeMtcp(const vpHomogeneousMatrix &flange_M_tcp)
  {
    m_flange_M_tcp = flange_M_tcp;
    m_tcp_M_set = true;
  }

  void clear_flangeMtcp() { m_tcp_M_set = false; }

  /*! 正运动学：返回末端坐标系 e 在基座坐标系 f 下的位姿 fMe */
  vpHomogeneousMatrix get_fMe(const vpColVector &q) const
  {
    std::vector<vpHomogeneousMatrix> T;
    computeLinkTransforms(q, T);
    vpHomogeneousMatrix fMe = T[6];
    if (m_tcp_M_set) {
      fMe = fMe * m_flange_M_tcp;
    }
    return fMe;
  }

  /*!
   * 基座坐标系下的几何雅可比 fJe (6x6)：
   *   [v_e ; w_e]_f = fJe * q_dot
   * v_e 为 e 坐标系原点的线速度，均在 f 系中表达。
   */
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const
  {
    std::vector<vpHomogeneousMatrix> T;
    computeLinkTransforms(q, T);

    vpHomogeneousMatrix fMe = T[6];
    if (m_tcp_M_set) {
      fMe = fMe * m_flange_M_tcp;
    }
    vpColVector p_e(3);
    for (unsigned int r = 0; r < 3; ++r) {
      p_e[r] = fMe[r][3];
    }

    fJe.resize(6, 6);
    for (unsigned int i = 0; i < 6; ++i) {
      // 关节 i 的轴线 z_{i-1} 与原点 p_{i-1}（T[i] = fM_{i}, T[0] = I）
      vpColVector z(3), p(3);
      for (unsigned int r = 0; r < 3; ++r) {
        z[r] = T[i][r][2];
        p[r] = T[i][r][3];
      }
      vpColVector jv = vpColVector::crossProd(z, p_e - p);
      for (unsigned int r = 0; r < 3; ++r) {
        fJe[r][i] = jv[r];
        fJe[r + 3][i] = z[r];
      }
    }
  }

  /*!
   * 末端坐标系下的几何雅可比 eJe (6x6)：
   *   [v_e ; w_e]_e = eJe * q_dot
   * 这正是 ViSP vpServo::set_eJe() 所需的矩阵（EYEINHAND_L_cVe_eJe 模式）。
   * 注意：eJe 对基座坐标系的选取不敏感（基座旋转 180° 等不会影响 eJe）。
   */
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const
  {
    vpMatrix fJe;
    get_fJe(q, fJe);
    vpRotationMatrix eRf = get_fMe(q).getRotationMatrix().inverse();
    // 只旋转、不平移（eVf 的 "旋转块对角" 形式），因为 fJe 已经是 e 原点的速度
    vpVelocityTwistMatrix eVf(vpTranslationVector(0, 0, 0), eRf);
    eJe = eVf * fJe;
  }

  /*! 雅可比最小奇异值（可操作度指标），用于奇异位形保护 */
  double minSingularValue(const vpColVector &q) const
  {
    vpMatrix eJe, Jp;
    vpColVector sv;
    get_eJe(q, eJe);
    eJe.pseudoInverse(Jp, sv, 1e-12);
    double smin = sv[0];
    for (unsigned int i = 1; i < sv.size(); ++i) {
      if (sv[i] < smin) smin = sv[i];
    }
    return smin;
  }

  /*! 将 UR 控制器的 TCP 位姿 (x,y,z,rx,ry,rz: 轴角) 转为 vpHomogeneousMatrix */
  static vpHomogeneousMatrix urPoseToHomogeneousMatrix(const std::vector<double> &pose)
  {
    if (pose.size() != 6) {
      throw std::invalid_argument("UR pose must have 6 elements");
    }
    vpTranslationVector t(pose[0], pose[1], pose[2]);
    vpThetaUVector tu(pose[3], pose[4], pose[5]);
    return vpHomogeneousMatrix(t, tu);
  }

private:
  /*!
   * 标准 DH: T_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
   * 输出 T[0..6]，T[i] = fM_i（T[0] = I, T[6] = 法兰）
   */
  void computeLinkTransforms(const vpColVector &q, std::vector<vpHomogeneousMatrix> &T) const
  {
    if (q.size() != 6) {
      throw std::invalid_argument("UR joint vector must have 6 elements");
    }
    const double a[6] = { 0.0, m_dh.a2, m_dh.a3, 0.0, 0.0, 0.0 };
    const double d[6] = { m_dh.d1, 0.0, 0.0, m_dh.d4, m_dh.d5, m_dh.d6 };
    const double alpha[6] = { M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0 };

    T.assign(7, vpHomogeneousMatrix());
    for (unsigned int i = 0; i < 6; ++i) {
      const double ct = std::cos(q[i]), st = std::sin(q[i]);
      const double ca = std::cos(alpha[i]), sa = std::sin(alpha[i]);
      vpHomogeneousMatrix A;
      A[0][0] = ct;  A[0][1] = -st * ca; A[0][2] = st * sa;  A[0][3] = a[i] * ct;
      A[1][0] = st;  A[1][1] = ct * ca;  A[1][2] = -ct * sa; A[1][3] = a[i] * st;
      A[2][0] = 0.0; A[2][1] = sa;       A[2][2] = ca;       A[2][3] = d[i];
      A[3][0] = 0.0; A[3][1] = 0.0;      A[3][2] = 0.0;      A[3][3] = 1.0;
      T[i + 1] = T[i] * A;
    }
  }

  UrDhParameters m_dh;
  vpHomogeneousMatrix m_flange_M_tcp;
  bool m_tcp_M_set;
};

#endif // UR_KINEMATICS_H
