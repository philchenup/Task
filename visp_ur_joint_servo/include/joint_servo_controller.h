/*
 * 基于 ViSP vpServo 的 PBVS(位置视觉伺服) -> UR 关节空间实时控制器。
 *
 * 控制律（ViSP vpServo::EYEINHAND_L_cVe_eJe）：
 *     q_dot = -lambda * ( L_e * cVe * eJe )^+ * e
 * 其中
 *   e   = [ cd_t_c ; theta_u(cd_R_c) ]   —— 由 vpFeatureTranslation(cdMc) + vpFeatureThetaU(cdRc) 组成
 *   L_e = 对应交互矩阵（ViSP 内部计算）
 *   cVe = 相机/末端速度旋量变换（由手眼标定 eMc 得到）
 *   eJe = UR 末端雅可比（UrKinematics 提供）
 *
 * 每个控制周期：
 *   1. 读取关节角 q
 *   2. 取最新 cMo，并用机器人自运动补偿到当前时刻
 *   3. cdMc = cdMo * cMo^-1，更新特征
 *   4. set_cVe / set_eJe，computeControlLaw() 得到 q_dot
 *   5. 速度/加速度/关节限位/奇异保护
 *   6. 积分 q_cmd += q_dot*dt，用 servoJ 发送（或直接 speedJ 发送 q_dot）
 *   7. 记录轨迹 (t, q_cmd, q_dot, e)
 */

#ifndef JOINT_SERVO_CONTROLLER_H
#define JOINT_SERVO_CONTROLLER_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>

#include "object_pose_source.h"
#include "ur_joint_interface.h"
#include "ur_kinematics.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

struct JointServoConfig
{
  enum OutputMode { SERVOJ, SPEEDJ };

  OutputMode output_mode = SERVOJ;
  double control_rate_hz = 500.0;          // e-Series: 500, CB3: 125

  // 增益
  double lambda = 0.5;                     // 常数增益
  bool use_adaptive_gain = false;          // vpAdaptiveGain(gain_at_zero, gain_at_inf, slope_at_zero)
  double gain_at_zero = 1.5;
  double gain_at_inf = 0.4;
  double slope_at_zero = 30.0;

  // 关节空间限制
  double max_joint_velocity = vpMath::rad(60.0);      // rad/s
  double max_joint_acceleration = vpMath::rad(300.0); // rad/s^2
  double joint_limit_margin = vpMath::rad(2.0);       // 距软限位多少开始停止
  double q_min = -2.0 * M_PI;
  double q_max = 2.0 * M_PI;

  // 奇异保护：eJe 最小奇异值低于该阈值 -> 停止
  double min_singular_value = 0.01;

  // servoj 参数
  double servoj_lookahead = 0.1;   // [0.03, 0.2]
  double servoj_gain = 300.0;      // [100, 2000]
  double speedj_acceleration = 2.0;

  // 视觉看门狗：超过该时长没有新的 cMo 则停止
  double vision_timeout_s = 0.5;

  // 收敛判据（仅用于提示/可选停止）
  double convergence_threshold_t = 0.001;             // m
  double convergence_threshold_tu = vpMath::rad(0.5); // rad
  bool stop_on_convergence = false;

  // 当积分得到的 q_cmd 与实际 q 偏差过大（跟踪不上/被外力阻挡）时重新同步
  double resync_threshold = vpMath::rad(5.0);

  bool compensate_robot_motion = true;
  bool verbose = false;
  std::string log_file;   // CSV 轨迹文件，空则不记录
};

struct JointServoStatus
{
  unsigned long iteration = 0;
  double t = 0;
  vpColVector q;        // 实际关节角
  vpColVector q_cmd;    // 发送的目标关节角
  vpColVector q_dot;    // 发送的关节速度
  vpColVector error;    // 视觉特征误差 (6)
  double error_t = 0;   // 平移误差范数 (m)
  double error_tu = 0;  // 旋转误差范数 (rad)
  bool converged = false;
  bool servoing = false;
  std::string stop_reason;
};

class JointServoController
{
public:
  typedef std::function<void(const JointServoStatus &)> StatusCallback;

  JointServoController(std::shared_ptr<UrJointInterface> robot, const UrKinematics &kin,
                       const vpHomogeneousMatrix &eMc, const vpHomogeneousMatrix &cdMo,
                       ObjectPoseSource &pose_source, const JointServoConfig &cfg)
    : m_robot(robot), m_kin(kin), m_eMc(eMc), m_cMe(eMc.inverse()), m_cVe(m_cMe), m_cdMo(cdMo),
    m_pose_source(pose_source), m_cfg(cfg), m_t(vpFeatureTranslation::cdMc), m_tu(vpFeatureThetaU::cdRc),
    m_td(vpFeatureTranslation::cdMc), m_tud(vpFeatureThetaU::cdRc), m_request_stop(false)
  {
    m_task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    m_task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    m_task.addFeature(m_t, m_td);
    m_task.addFeature(m_tu, m_tud);
    if (m_cfg.use_adaptive_gain) {
      m_task.setLambda(vpAdaptiveGain(m_cfg.gain_at_zero, m_cfg.gain_at_inf, m_cfg.slope_at_zero));
    }
    else {
      m_task.setLambda(m_cfg.lambda);
    }
  }

  void set_cdMo(const vpHomogeneousMatrix &cdMo) { m_cdMo = cdMo; }
  void setStatusCallback(StatusCallback cb) { m_status_cb = cb; }
  void requestStop() { m_request_stop = true; }
  vpServo &task() { return m_task; }

  /*!
   * 阻塞运行控制环，直到 requestStop()、收敛(可选)或安全保护触发。
   * \return 停止原因
   */
  std::string run()
  {
    const double dt = 1.0 / m_cfg.control_rate_hz;
    JointServoStatus st;
    st.q_dot = vpColVector(6, 0.0);
    st.error = vpColVector(6, 0.0);

    vpColVector q_cmd = m_robot->getActualQ();
    vpColVector qd_prev(6, 0.0);
    const auto t_begin = std::chrono::steady_clock::now();

    std::ofstream log;
    if (!m_cfg.log_file.empty()) {
      log.open(m_cfg.log_file);
      log << "t,q1,q2,q3,q4,q5,q6,qcmd1,qcmd2,qcmd3,qcmd4,qcmd5,qcmd6,qd1,qd2,qd3,qd4,qd5,qd6,"
        "e_tx,e_ty,e_tz,e_tux,e_tuy,e_tuz,err_t,err_tu\n";
    }

    std::string stop_reason;
    bool moving = false;

    while (!m_request_stop) {
      auto t_cycle = m_robot->initPeriod();
      st.iteration++;
      st.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_begin).count();

      vpColVector q = m_robot->getActualQ();
      st.q = q;

      // ---- 1. 取得物体位姿（含看门狗与自运动补偿）----
      ObjectPoseSample sample = m_pose_source.getLatest();
      double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - sample.stamp).count();
      bool pose_ok = sample.valid && age <= m_cfg.vision_timeout_s;

      vpColVector qd(6, 0.0);
      bool send = false;

      if (pose_ok) {
        vpHomogeneousMatrix cMo = sample.cMo;
        if (m_cfg.compensate_robot_motion && sample.seq != 0) {
          vpHomogeneousMatrix fMc_now = m_kin.get_fMe(q) * m_eMc;
          cMo = fMc_now.inverse() * sample.fMc * sample.cMo;
        }

        // ---- 2. 更新 PBVS 特征 ----
        vpHomogeneousMatrix cdMc = m_cdMo * cMo.inverse();
        m_t.buildFrom(cdMc);
        m_tu.buildFrom(cdMc);

        // ---- 3. 关节空间控制律 ----
        vpMatrix eJe;
        m_kin.get_eJe(q, eJe);
        m_task.set_cVe(m_cVe);
        m_task.set_eJe(eJe);
        qd = m_task.computeControlLaw();
        st.error = m_task.getError();

        vpTranslationVector cd_t_c = cdMc.getTranslationVector();
        vpThetaUVector cd_tu_c = cdMc.getThetaUVector();
        st.error_t = std::sqrt(cd_t_c.sumSquare());
        st.error_tu = std::sqrt(cd_tu_c.sumSquare());
        st.converged = (st.error_t < m_cfg.convergence_threshold_t) && (st.error_tu < m_cfg.convergence_threshold_tu);

        // ---- 4. 安全保护 ----
        if (m_task.getTaskRank() < 6) {
          stop_reason = "task jacobian rank < 6 (singular configuration)";
          break;
        }
        double smin = minSingularValue(eJe);
        if (smin < m_cfg.min_singular_value) {
          std::ostringstream oss;
          oss << "eJe min singular value " << smin << " < " << m_cfg.min_singular_value << " (near singularity)";
          stop_reason = oss.str();
          break;
        }
        if (!std::isfinite(qd.sumSquare())) {
          stop_reason = "non-finite joint velocity";
          break;
        }

        saturateVelocity(qd, m_cfg.max_joint_velocity);
        limitAcceleration(qd, qd_prev, m_cfg.max_joint_acceleration * dt);

        if (m_cfg.stop_on_convergence && st.converged) {
          stop_reason = "converged";
          break;
        }
        send = true;
      }
      else {
        // 没有有效视觉信息：平滑减速到 0
        qd = 0.0;
        limitAcceleration(qd, qd_prev, m_cfg.max_joint_acceleration * dt);
        send = moving; // 只在尚未停稳时继续发送
        if (m_cfg.verbose && sample.valid && st.iteration % 100 == 0) {
          std::cout << "[warn] object pose too old (" << age * 1000 << " ms), decelerating" << std::endl;
        }
      }

      // ---- 5. 积分得到关节轨迹点并发送 ----
      if ((q_cmd - q).infinityNorm() > m_cfg.resync_threshold) {
        if (m_cfg.verbose) {
          std::cout << "[warn] q_cmd drifted from actual q, resynchronizing" << std::endl;
        }
        q_cmd = q;
      }

      vpColVector q_next = q_cmd + qd * dt;
      if (!withinJointLimits(q_next)) {
        stop_reason = "joint limit reached";
        break;
      }

      if (send) {
        bool ok = true;
        if (m_cfg.output_mode == JointServoConfig::SERVOJ) {
          ok = m_robot->servoJ(q_next, dt, m_cfg.servoj_lookahead, m_cfg.servoj_gain);
        }
        else {
          ok = m_robot->speedJ(qd, m_cfg.speedj_acceleration, dt);
        }
        if (!ok) {
          stop_reason = "robot interface rejected command";
          break;
        }
        q_cmd = q_next;
        moving = qd.infinityNorm() > 1e-6;
      }
      else {
        q_cmd = q; // 未发送指令时保持与实际同步
      }
      qd_prev = qd;

      st.q_cmd = q_cmd;
      st.q_dot = qd;
      st.servoing = send;

      if (log.is_open()) {
        log << st.t;
        for (unsigned i = 0; i < 6; ++i) log << "," << q[i];
        for (unsigned i = 0; i < 6; ++i) log << "," << q_cmd[i];
        for (unsigned i = 0; i < 6; ++i) log << "," << qd[i];
        for (unsigned i = 0; i < 6; ++i) log << "," << st.error[i];
        log << "," << st.error_t << "," << st.error_tu << "\n";
      }
      if (m_status_cb) {
        m_status_cb(st);
      }
      if (m_cfg.verbose && st.iteration % static_cast<unsigned long>(m_cfg.control_rate_hz) == 0) {
        std::cout << "t=" << st.t << " err_t=" << st.error_t << " m err_tu=" << vpMath::deg(st.error_tu)
          << " deg qd(deg/s)=" << (qd * (180.0 / M_PI)).t() << std::endl;
      }

      m_robot->waitPeriod(t_cycle, dt);
    }

    if (stop_reason.empty()) {
      stop_reason = "stop requested";
    }
    m_robot->stop(m_cfg.max_joint_acceleration);
    st.stop_reason = stop_reason;
    st.servoing = false;
    if (m_status_cb) {
      m_status_cb(st);
    }
    return stop_reason;
  }

private:
  static void saturateVelocity(vpColVector &qd, double qd_max)
  {
    double scale = 1.0;
    for (unsigned int i = 0; i < qd.size(); ++i) {
      double r = std::fabs(qd[i]) / qd_max;
      if (r > scale) scale = r;
    }
    if (scale > 1.0) {
      qd /= scale;
    }
  }

  static void limitAcceleration(vpColVector &qd, const vpColVector &qd_prev, double dqd_max)
  {
    vpColVector dqd = qd - qd_prev;
    double scale = 1.0;
    for (unsigned int i = 0; i < dqd.size(); ++i) {
      double r = std::fabs(dqd[i]) / dqd_max;
      if (r > scale) scale = r;
    }
    if (scale > 1.0) {
      qd = qd_prev + dqd / scale;
    }
  }

  bool withinJointLimits(const vpColVector &q) const
  {
    for (unsigned int i = 0; i < q.size(); ++i) {
      if (q[i] < m_cfg.q_min + m_cfg.joint_limit_margin || q[i] > m_cfg.q_max - m_cfg.joint_limit_margin) {
        return false;
      }
    }
    return true;
  }

  static double minSingularValue(const vpMatrix &J)
  {
    vpMatrix Jp;
    vpColVector sv;
    J.pseudoInverse(Jp, sv, 1e-12);
    double smin = sv[0];
    for (unsigned int i = 1; i < sv.size(); ++i) {
      if (sv[i] < smin) smin = sv[i];
    }
    return smin;
  }

  std::shared_ptr<UrJointInterface> m_robot;
  UrKinematics m_kin;
  vpHomogeneousMatrix m_eMc, m_cMe;
  vpVelocityTwistMatrix m_cVe;
  vpHomogeneousMatrix m_cdMo;
  ObjectPoseSource &m_pose_source;
  JointServoConfig m_cfg;

  vpServo m_task;
  vpFeatureTranslation m_t;
  vpFeatureThetaU m_tu;
  vpFeatureTranslation m_td;
  vpFeatureThetaU m_tud;

  StatusCallback m_status_cb;
  std::atomic<bool> m_request_stop;
};

#endif // JOINT_SERVO_CONTROLLER_H
