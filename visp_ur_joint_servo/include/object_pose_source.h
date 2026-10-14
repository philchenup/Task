/*
 * 视觉跟踪结果（物体在相机坐标系下的位姿 cMo）与控制线程之间的线程安全缓冲区。
 *
 * 用法：你已有的跟踪器（vpMbGenericTracker / AprilTag / 深度学习位姿估计等）
 * 每得到一帧位姿，就调用
 *     pose_source.update(cMo, fMc_at_image_time);
 * 控制线程以更高频率（UR e-Series 500 Hz，CB3 125 Hz）调用 getLatest() 读取。
 *
 * 为了让 500 Hz 的控制环在两帧图像之间也能获得合理的 cMo，这里同时记录
 * 图像时刻相机在基座系下的位姿 fMc_img。控制线程用当前关节角重算 fMc_now，
 * 并用 cMo_now = fMc_now^-1 * fMc_img * cMo_img 做"里程计补偿"。
 * 这对静止物体是精确的；对运动物体相当于零阶保持 + 机器人自运动补偿。
 */

#ifndef OBJECT_POSE_SOURCE_H
#define OBJECT_POSE_SOURCE_H

#include <chrono>
#include <mutex>

#include <visp3/core/vpHomogeneousMatrix.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

struct ObjectPoseSample
{
  vpHomogeneousMatrix cMo;        // 物体在相机系下的位姿（跟踪器输出）
  vpHomogeneousMatrix fMc;        // 取像时刻相机在基座系下的位姿（用于运动补偿）
  std::chrono::steady_clock::time_point stamp;
  unsigned long seq = 0;
  bool valid = false;
};

class ObjectPoseSource
{
public:
  void update(const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &fMc_at_image_time,
              std::chrono::steady_clock::time_point stamp = std::chrono::steady_clock::now())
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sample.cMo = cMo;
    m_sample.fMc = fMc_at_image_time;
    m_sample.stamp = stamp;
    m_sample.seq++;
    m_sample.valid = true;
  }

  /*! 跟踪丢失时调用，控制线程将停止发送速度 */
  void invalidate()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sample.valid = false;
  }

  ObjectPoseSample getLatest() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sample;
  }

private:
  mutable std::mutex m_mutex;
  ObjectPoseSample m_sample;
};

#endif // OBJECT_POSE_SOURCE_H
