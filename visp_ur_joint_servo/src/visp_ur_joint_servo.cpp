/*
 * ViSP PBVS -> UR 关节空间实时伺服
 *
 * 输入：视觉跟踪得到的物体位姿 cMo（物体在相机系下）
 * 输出：实时关节轨迹 q_cmd(t)（servoj）或关节速度 q_dot(t)（speedj），直接发给 UR 控制器
 *
 * 线程结构：
 *   - 视觉线程：你的跟踪器 -> ObjectPoseSource::update(cMo, fMc)      （相机帧率，~30 Hz）
 *   - 控制线程：JointServoController::run()                            （500 Hz e-Series / 125 Hz CB3）
 *
 * 用法示例：
 *   # 离线仿真（无机器人、无相机）
 *   ./visp_ur_joint_servo --sim --model ur5e --log traj.csv --verbose
 *
 *   # 真机（需 ur_rtde；示例跟踪器为 RealSense + AprilTag，需 ViSP 编译时带 librealsense2/apriltag）
 *   ./visp_ur_joint_servo --ip 192.168.0.100 --model ur5e --eMc eMc.yaml --mode servoj --rate 500
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpTime.h>

#include "joint_servo_controller.h"
#include "object_pose_source.h"
#include "ur_joint_interface.h"
#include "ur_kinematics.h"

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_DISPLAY)
#define HAVE_EXAMPLE_TRACKER
#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
std::atomic<bool> g_quit(false);
JointServoController *g_controller = nullptr;

void onSignal(int)
{
  g_quit = true;
  if (g_controller) {
    g_controller->requestStop();
  }
}

struct Options
{
  std::string ip = "192.168.0.100";
  std::string model = "ur5e";
  std::string eMc_file;
  std::string log_file;
  std::string mode = "servoj";
  double rate = 500.0;
  double lambda = 0.5;
  bool adaptive_gain = false;
  bool sim = false;
  bool sim_moving_object = false;
  bool verbose = false;
  bool stop_on_convergence = false;
  double max_qdot_deg = 60.0;
  double max_qddot_deg = 300.0;
  double vision_timeout = 0.5;
  double sim_duration = 10.0;
  // 期望的物体在相机系下的位姿 cdMo（默认：物体在相机前方 0.3 m，物体 z 轴指向相机）
  vpPoseVector cdMo_pose = vpPoseVector(0, 0, 0.30, M_PI, 0, 0);
  double tag_size = 0.12;
};

void printHelp(const char *prog, const Options &o)
{
  std::cout
    << prog << " [options]\n"
    << "  --ip <addr>                 UR IP (default " << o.ip << ")\n"
    << "  --model <ur3|ur3e|ur5|ur5e|ur10|ur10e|ur16e|ur20|ur30>  (default " << o.model << ")\n"
    << "  --eMc <file.yaml>           手眼标定结果 (vpPoseVector YAML: 相机相对末端)\n"
    << "  --cdMo tx ty tz tux tuy tuz 期望的物体在相机系下位姿 [m, rad]\n"
    << "  --mode <servoj|speedj>      关节轨迹(位置)或关节速度输出 (default " << o.mode << ")\n"
    << "  --rate <hz>                 控制频率 (e-Series 500, CB3 125) (default " << o.rate << ")\n"
    << "  --lambda <v>                常数增益 (default " << o.lambda << ")\n"
    << "  --adaptive-gain             使用 vpAdaptiveGain\n"
    << "  --max-qdot <deg/s>          关节速度上限 (default " << o.max_qdot_deg << ")\n"
    << "  --max-qddot <deg/s^2>       关节加速度上限 (default " << o.max_qddot_deg << ")\n"
    << "  --vision-timeout <s>        无新位姿超时停止 (default " << o.vision_timeout << ")\n"
    << "  --stop-on-convergence       收敛后停止，不再持续跟踪\n"
    << "  --log <file.csv>            记录关节轨迹\n"
    << "  --tag-size <m>              示例 AprilTag 跟踪器的 tag 尺寸 (default " << o.tag_size << ")\n"
    << "  --sim                       无机器人仿真\n"
    << "  --sim-moving                仿真中物体缓慢运动\n"
    << "  --sim-duration <s>          仿真时长 (default " << o.sim_duration << ")\n"
    << "  --verbose\n";
}

bool parseArgs(int argc, char **argv, Options &o)
{
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](double &v) { if (i + 1 < argc) { v = std::atof(argv[++i]); return true; } return false; };
    auto nexts = [&](std::string &v) { if (i + 1 < argc) { v = argv[++i]; return true; } return false; };
    if (a == "--ip") nexts(o.ip);
    else if (a == "--model") nexts(o.model);
    else if (a == "--eMc") nexts(o.eMc_file);
    else if (a == "--log") nexts(o.log_file);
    else if (a == "--mode") nexts(o.mode);
    else if (a == "--rate") next(o.rate);
    else if (a == "--lambda") next(o.lambda);
    else if (a == "--max-qdot") next(o.max_qdot_deg);
    else if (a == "--max-qddot") next(o.max_qddot_deg);
    else if (a == "--vision-timeout") next(o.vision_timeout);
    else if (a == "--sim-duration") next(o.sim_duration);
    else if (a == "--tag-size") next(o.tag_size);
    else if (a == "--adaptive-gain") o.adaptive_gain = true;
    else if (a == "--stop-on-convergence") o.stop_on_convergence = true;
    else if (a == "--sim") o.sim = true;
    else if (a == "--sim-moving") o.sim_moving_object = true;
    else if (a == "--verbose") o.verbose = true;
    else if (a == "--cdMo") {
      if (i + 6 >= argc) return false;
      for (unsigned k = 0; k < 6; ++k) o.cdMo_pose[k] = std::atof(argv[++i]);
    }
    else if (a == "--help" || a == "-h") { printHelp(argv[0], o); return false; }
    else { std::cerr << "Unknown option " << a << std::endl; printHelp(argv[0], o); return false; }
  }
  return true;
}

/* ------------------------------------------------------------------------- */
/*            仿真视觉线程：根据机器人当前位姿合成 cMo（30 Hz + 噪声）           */
/* ------------------------------------------------------------------------- */
void simulatedVisionThread(std::shared_ptr<UrJointInterface> robot, UrKinematics kin, vpHomogeneousMatrix eMc,
                           vpHomogeneousMatrix fMo0, bool moving, ObjectPoseSource *src)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> n_t(0.0, 0.0005), n_r(0.0, vpMath::rad(0.1));
  const auto t0 = std::chrono::steady_clock::now();
  while (!g_quit) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    vpHomogeneousMatrix fMo = fMo0;
    if (moving) {
      vpHomogeneousMatrix oMo_t(0.03 * std::sin(0.5 * t), 0.03 * std::cos(0.5 * t), 0.0, 0, 0, vpMath::rad(10) * std::sin(0.3 * t));
      fMo = fMo0 * oMo_t;
    }
    vpColVector q = robot->getActualQ();
    vpHomogeneousMatrix fMc = kin.get_fMe(q) * eMc;
    vpHomogeneousMatrix noise(n_t(rng), n_t(rng), n_t(rng), n_r(rng), n_r(rng), n_r(rng));
    vpHomogeneousMatrix cMo = fMc.inverse() * fMo * noise;
    src->update(cMo, fMc);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
}

/* ------------------------------------------------------------------------- */
/*        真机视觉线程：把你自己的跟踪器接到这里（下面是 AprilTag 示例）          */
/* ------------------------------------------------------------------------- */
void userTrackerThread(std::shared_ptr<UrJointInterface> robot, UrKinematics kin, vpHomogeneousMatrix eMc,
                       double tag_size, ObjectPoseSource *src)
{
#if defined(HAVE_EXAMPLE_TRACKER)
  try {
    vpRealSense2 rs;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    rs.open(config);
    vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    vpImage<unsigned char> I(480, 640);
#if defined(VISP_HAVE_X11)
    vpDisplayX display(I, 10, 10, "Tracker");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I, 10, 10, "Tracker");
#else
    vpDisplayOpenCV display(I, 10, 10, "Tracker");
#endif

    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
    detector.setAprilTagQuadDecimate(2);
    detector.setDisplayTag(true);

    while (!g_quit) {
      rs.acquire(I);
      // 取像时刻的 fMc：用当前关节角（若需更高精度可用图像时间戳做插值）
      vpColVector q = robot->getActualQ();
      vpHomogeneousMatrix fMc = kin.get_fMe(q) * eMc;

      vpDisplay::display(I);
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tag_size, cam, cMo_vec);
      if (cMo_vec.size() == 1) {
        src->update(cMo_vec[0], fMc);
        vpDisplay::displayFrame(I, cMo_vec[0], cam, tag_size / 2, vpColor::none, 3);
      }
      else {
        src->invalidate();
      }
      vpDisplay::displayText(I, 20, 20, "Click to quit", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        g_quit = true;
        if (g_controller) g_controller->requestStop();
      }
    }
  }
  catch (const vpException &e) {
    std::cerr << "Tracker error: " << e.getMessage() << std::endl;
    g_quit = true;
    if (g_controller) g_controller->requestStop();
  }
#else
  // ================= 在这里接入你自己的跟踪器 =================
  // 伪代码：
  //   while (!g_quit) {
  //     grab(I);
  //     vpColVector q = robot->getActualQ();                 // 尽量取靠近曝光时刻的 q
  //     vpHomogeneousMatrix fMc = kin.get_fMe(q) * eMc;
  //     if (tracker.track(I, cMo)) src->update(cMo, fMc);
  //     else                        src->invalidate();
  //   }
  (void)robot; (void)kin; (void)eMc; (void)tag_size; (void)src;
  std::cerr << "ViSP was built without RealSense2/AprilTag: no example tracker available.\n"
    "Plug your own tracker into userTrackerThread()." << std::endl;
  g_quit = true;
  if (g_controller) g_controller->requestStop();
#endif
}

/* ------------------------------------------------------------------------- */
/*     启动自检：DH 正运动学 vs 控制器报告的 TCP 位姿（验证模型/坐标系一致）       */
/* ------------------------------------------------------------------------- */
void checkKinematicsAgainstController(UrKinematics &kin, UrJointInterface &robot)
{
  std::vector<double> tcp_offset = robot.getTCPOffset();
  kin.set_flangeMtcp(UrKinematics::urPoseToHomogeneousMatrix(tcp_offset));

  vpColVector q = robot.getActualQ();
  vpHomogeneousMatrix fMe_dh = kin.get_fMe(q);
  vpHomogeneousMatrix fMe_ctrl = UrKinematics::urPoseToHomogeneousMatrix(robot.getActualTCPPose());
  vpHomogeneousMatrix dM = fMe_dh.inverse() * fMe_ctrl;
  double dt = std::sqrt(dM.getTranslationVector().sumSquare());
  double dr = vpMath::deg(std::sqrt(dM.getThetaUVector().sumSquare()));
  std::cout << "Kinematics self-check: |t_err| = " << dt * 1000 << " mm, |r_err| = " << dr << " deg" << std::endl;
  if (dt > 0.005 || dr > 1.0) {
    std::cout << "WARNING: DH model does not match controller pose. Check --model (robot type) "
      "and TCP offset. (A 180 deg error about base z only affects this check, not eJe.)" << std::endl;
  }
}
} // namespace

int main(int argc, char **argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    return EXIT_FAILURE;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    // ---- 手眼标定 eMc ----
    vpPoseVector ePc(0, 0, 0.05, 0, 0, 0); // 默认：相机在法兰前方 5 cm，光轴与法兰 z 轴同向
    if (!opt.eMc_file.empty()) {
      ePc.loadYAML(opt.eMc_file, ePc);
    }
    else {
      std::cout << "Warning: --eMc not given, using default eMc" << std::endl;
    }
    vpHomogeneousMatrix eMc(ePc);
    vpHomogeneousMatrix cdMo(opt.cdMo_pose);
    std::cout << "eMc:\n" << eMc << "\ncdMo:\n" << cdMo << std::endl;

    UrKinematics kin(opt.model);

    // ---- 机器人接口 ----
    std::shared_ptr<UrJointInterface> robot;
    if (opt.sim) {
      vpColVector q0(6);
      q0[0] = vpMath::rad(0); q0[1] = vpMath::rad(-100); q0[2] = vpMath::rad(100);
      q0[3] = vpMath::rad(-90); q0[4] = vpMath::rad(-90); q0[5] = 0;
      robot = std::make_shared<UrSimulatedJointInterface>(q0);
    }
    else {
#if defined(HAVE_UR_RTDE)
      std::cout << "Connecting to UR at " << opt.ip << " ..." << std::endl;
      robot = std::make_shared<UrRtdeJointInterface>(opt.ip, opt.rate);
      checkKinematicsAgainstController(kin, *robot);
      std::cout << "WARNING: the robot will move. Keep the emergency stop at hand. Press Enter to continue..." << std::endl;
      std::cin.ignore();
#else
      std::cerr << "Built without ur_rtde: only --sim is available." << std::endl;
      return EXIT_FAILURE;
#endif
    }

    // ---- 控制器配置 ----
    JointServoConfig cfg;
    cfg.output_mode = (opt.mode == "speedj") ? JointServoConfig::SPEEDJ : JointServoConfig::SERVOJ;
    cfg.control_rate_hz = opt.rate;
    cfg.lambda = opt.lambda;
    cfg.use_adaptive_gain = opt.adaptive_gain;
    cfg.max_joint_velocity = vpMath::rad(opt.max_qdot_deg);
    cfg.max_joint_acceleration = vpMath::rad(opt.max_qddot_deg);
    cfg.vision_timeout_s = opt.vision_timeout;
    cfg.stop_on_convergence = opt.stop_on_convergence;
    cfg.log_file = opt.log_file;
    cfg.verbose = opt.verbose;

    ObjectPoseSource pose_source;
    JointServoController controller(robot, kin, eMc, cdMo, pose_source, cfg);
    g_controller = &controller;

    // ---- 视觉线程 ----
    std::thread vision;
    if (opt.sim) {
      // 初始让物体偏离期望位姿，观察收敛
      vpHomogeneousMatrix cMo_init(0.08, -0.05, 0.45, M_PI + vpMath::rad(15), vpMath::rad(-10), vpMath::rad(20));
      vpHomogeneousMatrix fMo0 = kin.get_fMe(robot->getActualQ()) * eMc * cMo_init;
      vision = std::thread(simulatedVisionThread, robot, kin, eMc, fMo0, opt.sim_moving_object, &pose_source);
    }
    else {
      vision = std::thread(userTrackerThread, robot, kin, eMc, opt.tag_size, &pose_source);
    }

    std::thread sim_timer;
    if (opt.sim) {
      sim_timer = std::thread([&]() {
        auto t_end = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(opt.sim_duration));
        while (!g_quit && std::chrono::steady_clock::now() < t_end) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        g_quit = true;
        controller.requestStop();
      });
    }

    // ---- 控制环（阻塞）----
    std::cout << "Servo loop running at " << opt.rate << " Hz, mode=" << opt.mode << ", robot=" << robot->name() << std::endl;
    std::string reason = controller.run();
    std::cout << "Servo loop stopped: " << reason << std::endl;

    g_quit = true;
    if (vision.joinable()) vision.join();
    if (sim_timer.joinable()) sim_timer.join();
    g_controller = nullptr;
  }
  catch (const vpException &e) {
    std::cerr << "ViSP exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
