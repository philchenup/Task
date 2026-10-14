/*
 * UR 关节级实时控制接口抽象。
 *
 * - UrRtdeJointInterface : 通过 ur_rtde 真机控制（servoJ / speedJ），需要编译时找到 ur_rtde。
 * - UrSimulatedJointInterface : 纯数值积分仿真，无需机器人，用于离线验证控制律与轨迹。
 *
 * 控制器 (JointServoController) 只依赖 UrJointInterface，
 * 因此也可以很容易地换成 ROS2 / 自定义 TCP 协议等其它关节轨迹输出通道。
 */

#ifndef UR_JOINT_INTERFACE_H
#define UR_JOINT_INTERFACE_H

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

class UrJointInterface
{
public:
  virtual ~UrJointInterface() {}

  /*! 当前关节角 (rad) */
  virtual vpColVector getActualQ() = 0;

  /*! 当前关节速度 (rad/s) */
  virtual vpColVector getActualQd() = 0;

  /*! 控制器配置的 TCP 偏置 (x,y,z,rx,ry,rz)，无则返回 6 个 0 */
  virtual std::vector<double> getTCPOffset() = 0;

  /*! 控制器报告的 TCP 位姿 (x,y,z,rx,ry,rz)，用于启动时校验 DH 模型 */
  virtual std::vector<double> getActualTCPPose() = 0;

  /*!
   * 关节位置实时跟踪 (UR servoj)。
   * \param q_target   目标关节角
   * \param dt         本周期时长 (s)
   * \param lookahead  servoj lookahead_time [0.03, 0.2] s
   * \param gain       servoj gain [100, 2000]
   */
  virtual bool servoJ(const vpColVector &q_target, double dt, double lookahead, double gain) = 0;

  /*! 关节速度控制 (UR speedj) */
  virtual bool speedJ(const vpColVector &qd, double acceleration, double dt) = 0;

  /*! 停止 servoj / speedj 模式 */
  virtual void stop(double deceleration) = 0;

  /*! 周期开始时间戳 */
  virtual std::chrono::steady_clock::time_point initPeriod() { return std::chrono::steady_clock::now(); }

  /*! 阻塞到周期结束 */
  virtual void waitPeriod(const std::chrono::steady_clock::time_point &t_start, double dt)
  {
    std::this_thread::sleep_until(t_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                std::chrono::duration<double>(dt)));
  }

  virtual bool isConnected() { return true; }
  virtual std::string name() const = 0;
};

/* ------------------------------------------------------------------------- */
/*                 仿真实现：q_{k+1} = q_k + qd * dt（一阶积分）                 */
/* ------------------------------------------------------------------------- */
class UrSimulatedJointInterface : public UrJointInterface
{
public:
  explicit UrSimulatedJointInterface(const vpColVector &q0) : m_q(q0), m_qd(6, 0.0), m_q_prev(q0), m_last_dt(0) {}

  vpColVector getActualQ() override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_q;
  }
  vpColVector getActualQd() override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_qd;
  }
  std::vector<double> getTCPOffset() override { return std::vector<double>(6, 0.0); }
  std::vector<double> getActualTCPPose() override { return std::vector<double>(6, 0.0); }

  bool servoJ(const vpColVector &q_target, double dt, double, double) override
  {
    // 理想跟踪：直接到达目标（真实 servoj 内部有 lookahead/gain 构成的低通）
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_prev = m_q;
    m_q = q_target;
    m_last_dt = dt;
    m_qd = (dt > 0) ? (m_q - m_q_prev) * (1.0 / dt) : vpColVector(6, 0.0);
    return true;
  }

  bool speedJ(const vpColVector &qd, double, double dt) override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_q_prev = m_q;
    m_qd = qd;
    m_q = m_q + qd * dt;
    m_last_dt = dt;
    return true;
  }

  void stop(double) override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_qd = 0.0;
  }
  std::string name() const override { return "simulated-UR"; }

private:
  std::mutex m_mutex;
  vpColVector m_q, m_qd, m_q_prev;
  double m_last_dt;
};

/* ------------------------------------------------------------------------- */
/*                         真机实现：ur_rtde                                   */
/* ------------------------------------------------------------------------- */
#if defined(HAVE_UR_RTDE)
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

class UrRtdeJointInterface : public UrJointInterface
{
public:
  /*!
   * \param ip         机器人 IP
   * \param frequency  RTDE 频率，e-Series 最高 500 Hz，CB3 最高 125 Hz；-1 表示使用控制器默认
   */
  UrRtdeJointInterface(const std::string &ip, double frequency = -1.0)
    : m_control(std::make_shared<ur_rtde::RTDEControlInterface>(ip, frequency)),
    m_receive(std::make_shared<ur_rtde::RTDEReceiveInterface>(ip, frequency))
  { }

  ~UrRtdeJointInterface() override
  {
    try {
      m_control->servoStop();
      m_control->speedStop();
      m_control->stopScript();
    }
    catch (...) {
    }
  }

  vpColVector getActualQ() override { return vpColVector(m_receive->getActualQ()); }
  vpColVector getActualQd() override { return vpColVector(m_receive->getActualQd()); }
  std::vector<double> getTCPOffset() override { return m_control->getTCPOffset(); }
  std::vector<double> getActualTCPPose() override { return m_receive->getActualTCPPose(); }

  bool servoJ(const vpColVector &q_target, double dt, double lookahead, double gain) override
  {
    // speed/acceleration 参数在当前 URScript 版本中被忽略，但必须传入
    return m_control->servoJ(q_target.toStdVector(), 0.0, 0.0, dt, lookahead, gain);
  }

  bool speedJ(const vpColVector &qd, double acceleration, double dt) override
  {
    return m_control->speedJ(qd.toStdVector(), acceleration, dt);
  }

  void stop(double deceleration) override
  {
    m_control->servoStop(deceleration);
    m_control->speedStop(deceleration);
  }

  std::chrono::steady_clock::time_point initPeriod() override { return m_control->initPeriod(); }

  void waitPeriod(const std::chrono::steady_clock::time_point &t_start, double) override
  {
    m_control->waitPeriod(t_start);
  }

  bool isConnected() override { return m_control->isConnected() && m_receive->isConnected(); }
  std::string name() const override { return "ur_rtde"; }

  std::shared_ptr<ur_rtde::RTDEControlInterface> control() { return m_control; }
  std::shared_ptr<ur_rtde::RTDEReceiveInterface> receive() { return m_receive; }

private:
  std::shared_ptr<ur_rtde::RTDEControlInterface> m_control;
  std::shared_ptr<ur_rtde::RTDEReceiveInterface> m_receive;
};
#endif // HAVE_UR_RTDE

#endif // UR_JOINT_INTERFACE_H
