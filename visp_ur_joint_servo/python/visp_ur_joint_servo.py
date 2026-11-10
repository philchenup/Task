#!/usr/bin/env python3
"""
ViSP PBVS -> UR 关节空间实时伺服（Python 版本）

依赖：
  pip install ur_rtde numpy
  ViSP Python 绑定（ViSP >= 3.6.1，源码编译时开启 BUILD_PYTHON_BINDINGS，或 conda-forge 的 visp）

控制律（ViSP vpServo::EYEINHAND_L_cVe_eJe）：
    q_dot = -lambda * (L_e * cVe * eJe)^+ * e
e = [cd_t_c ; theta_u(cd_R_c)]，eJe 由本文件中的 UR DH 模型给出。

用法：
  python3 visp_ur_joint_servo.py --sim --model ur5e --log traj.csv
  python3 visp_ur_joint_servo.py --ip 192.168.0.100 --model ur5e --eMc eMc.yaml

接入你自己的跟踪器：把 cMo 送进 ObjectPoseSource.update(cMo, fMc)（见 user_tracker_thread）。
"""

import argparse
import math
import sys
import threading
import time

import numpy as np

from visp.core import HomogeneousMatrix, Matrix, PoseVector, ThetaUVector, TranslationVector, VelocityTwistMatrix
from visp.visual_features import FeatureThetaU, FeatureTranslation
from visp.vs import AdaptiveGain, Servo


# --------------------------------------------------------------------------- #
#                            UR 运动学（标准 DH）                               #
# --------------------------------------------------------------------------- #
UR_DH = {
    #           d1        a2        a3        d4        d5       d6
    "ur3":   (0.1519,  -0.24365, -0.21325, 0.11235,  0.08535, 0.0819),
    "ur3e":  (0.15185, -0.24355, -0.2132,  0.13105,  0.08535, 0.0921),
    "ur5":   (0.089159, -0.425,  -0.39225, 0.10915,  0.09465, 0.0823),
    "ur5e":  (0.1625,  -0.425,   -0.3922,  0.1333,   0.0997,  0.0996),
    "ur10":  (0.1273,  -0.612,   -0.5723,  0.163941, 0.1157,  0.0922),
    "ur10e": (0.1807,  -0.6127,  -0.57155, 0.17415,  0.11985, 0.11655),
    "ur16e": (0.1807,  -0.4784,  -0.36,    0.17415,  0.11985, 0.11655),
    "ur20":  (0.2363,  -0.8620,  -0.7287,  0.2010,   0.1593,  0.1543),
    "ur30":  (0.2363,  -0.6370,  -0.5037,  0.2010,   0.1593,  0.1543),
}


def ur_pose_to_matrix(pose):
    """UR 控制器 (x,y,z,rx,ry,rz) 轴角 -> 4x4"""
    t = TranslationVector(pose[0], pose[1], pose[2])
    tu = ThetaUVector(pose[3], pose[4], pose[5])
    return np.array(HomogeneousMatrix(t, tu).numpy())


class UrKinematics:
    def __init__(self, model="ur5e", flange_M_tcp=None):
        d1, a2, a3, d4, d5, d6 = UR_DH[model.lower()]
        self.a = [0.0, a2, a3, 0.0, 0.0, 0.0]
        self.d = [d1, 0.0, 0.0, d4, d5, d6]
        self.alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
        self.flange_M_tcp = np.eye(4) if flange_M_tcp is None else np.asarray(flange_M_tcp)

    def _link_transforms(self, q):
        T = [np.eye(4)]
        for i in range(6):
            ct, st = math.cos(q[i]), math.sin(q[i])
            ca, sa = math.cos(self.alpha[i]), math.sin(self.alpha[i])
            A = np.array([
                [ct, -st * ca, st * sa, self.a[i] * ct],
                [st, ct * ca, -ct * sa, self.a[i] * st],
                [0.0, sa, ca, self.d[i]],
                [0.0, 0.0, 0.0, 1.0],
            ])
            T.append(T[-1] @ A)
        return T

    def fMe(self, q):
        return self._link_transforms(q)[6] @ self.flange_M_tcp

    def fJe(self, q):
        T = self._link_transforms(q)
        fMe = T[6] @ self.flange_M_tcp
        p_e = fMe[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            z = T[i][:3, 2]
            p = T[i][:3, 3]
            J[:3, i] = np.cross(z, p_e - p)
            J[3:, i] = z
        return J

    def eJe(self, q):
        """末端坐标系中表达的几何雅可比 —— vpServo.set_eJe() 需要的量"""
        fMe = self.fMe(q)
        eRf = fMe[:3, :3].T
        R6 = np.zeros((6, 6))
        R6[:3, :3] = eRf
        R6[3:, 3:] = eRf
        return R6 @ self.fJe(q)


# --------------------------------------------------------------------------- #
#                       跟踪器 -> 控制器 的线程安全缓冲                          #
# --------------------------------------------------------------------------- #
class ObjectPoseSource:
    def __init__(self):
        self._lock = threading.Lock()
        self.cMo = None     # 4x4 numpy
        self.fMc = None     # 取像时刻相机在基座系下位姿（运动补偿用）
        self.stamp = 0.0
        self.valid = False

    def update(self, cMo, fMc, stamp=None):
        with self._lock:
            self.cMo = np.array(cMo, dtype=float)
            self.fMc = np.array(fMc, dtype=float)
            self.stamp = time.monotonic() if stamp is None else stamp
            self.valid = True

    def invalidate(self):
        with self._lock:
            self.valid = False

    def latest(self):
        with self._lock:
            return self.valid, self.cMo, self.fMc, self.stamp


# --------------------------------------------------------------------------- #
#                              机器人接口                                       #
# --------------------------------------------------------------------------- #
class SimulatedUr:
    def __init__(self, q0):
        self._lock = threading.Lock()
        self.q = np.array(q0, dtype=float)
        self.qd = np.zeros(6)

    def getActualQ(self):
        with self._lock:
            return self.q.copy()

    def getTCPOffset(self):
        return [0.0] * 6

    def getActualTCPPose(self):
        return [0.0] * 6

    def servoJ(self, q_target, dt, lookahead, gain):
        with self._lock:
            self.qd = (np.asarray(q_target) - self.q) / dt
            self.q = np.asarray(q_target, dtype=float).copy()
        return True

    def speedJ(self, qd, acc, dt):
        with self._lock:
            self.qd = np.asarray(qd, dtype=float)
            self.q = self.q + self.qd * dt
        return True

    def stop(self, dec):
        pass

    def initPeriod(self):
        return time.monotonic()

    def waitPeriod(self, t_start, dt):
        remaining = t_start + dt - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)


class RtdeUr:
    def __init__(self, ip, frequency):
        import rtde_control
        import rtde_receive
        self.ctrl = rtde_control.RTDEControlInterface(ip, frequency)
        self.recv = rtde_receive.RTDEReceiveInterface(ip, frequency)

    def getActualQ(self):
        return np.array(self.recv.getActualQ())

    def getTCPOffset(self):
        return self.ctrl.getTCPOffset()

    def getActualTCPPose(self):
        return self.recv.getActualTCPPose()

    def servoJ(self, q_target, dt, lookahead, gain):
        return self.ctrl.servoJ(list(map(float, q_target)), 0.0, 0.0, dt, lookahead, gain)

    def speedJ(self, qd, acc, dt):
        return self.ctrl.speedJ(list(map(float, qd)), acc, dt)

    def stop(self, dec):
        self.ctrl.servoStop(dec)
        self.ctrl.speedStop(dec)

    def initPeriod(self):
        return self.ctrl.initPeriod()

    def waitPeriod(self, t_start, dt):
        self.ctrl.waitPeriod(t_start)


# --------------------------------------------------------------------------- #
#                      PBVS -> 关节速度 -> 关节轨迹 控制器                       #
# --------------------------------------------------------------------------- #
class JointServoController:
    def __init__(self, robot, kin, eMc, cdMo, pose_source, args):
        self.robot = robot
        self.kin = kin
        self.eMc = np.asarray(eMc)
        self.cdMo = np.asarray(cdMo)
        self.src = pose_source
        self.args = args
        self.stop_requested = threading.Event()

        self.cVe = VelocityTwistMatrix(HomogeneousMatrix(np.linalg.inv(self.eMc)))

        self.t = FeatureTranslation(FeatureTranslation.cdMc)
        self.tu = FeatureThetaU(FeatureThetaU.cdRc)
        self.td = FeatureTranslation(FeatureTranslation.cdMc)
        self.tud = FeatureThetaU(FeatureThetaU.cdRc)

        self.task = Servo()
        self.task.setServo(Servo.EYEINHAND_L_cVe_eJe)
        self.task.setInteractionMatrixType(Servo.CURRENT, Servo.PSEUDO_INVERSE)
        self.task.addFeature(self.t, self.td)
        self.task.addFeature(self.tu, self.tud)
        if args.adaptive_gain:
            self.task.setLambda(AdaptiveGain(1.5, 0.4, 30.0))
        else:
            self.task.setLambda(args.lambda_)

    @staticmethod
    def _scale_to_limit(v, vmax):
        s = np.max(np.abs(v) / vmax)
        return v / s if s > 1.0 else v

    def run(self):
        a = self.args
        dt = 1.0 / a.rate
        qd_max = math.radians(a.max_qdot)
        dqd_max = math.radians(a.max_qddot) * dt
        q_cmd = self.robot.getActualQ()
        qd_prev = np.zeros(6)
        moving = False
        t0 = time.monotonic()
        it = 0
        log = None
        if a.log:
            log = open(a.log, "w")
            log.write("t," + ",".join(f"q{i}" for i in range(1, 7)) + "," + ",".join(f"qcmd{i}" for i in range(1, 7))
                      + "," + ",".join(f"qd{i}" for i in range(1, 7)) + ",err_t,err_tu\n")

        reason = "stop requested"
        while not self.stop_requested.is_set():
            t_cycle = self.robot.initPeriod()
            it += 1
            now = time.monotonic()
            q = self.robot.getActualQ()

            valid, cMo_img, fMc_img, stamp = self.src.latest()
            pose_ok = valid and (now - stamp) <= a.vision_timeout
            qd = np.zeros(6)
            send = False
            err_t = err_tu = 0.0

            if pose_ok:
                cMo = cMo_img
                if not a.no_motion_compensation:
                    fMc_now = self.kin.fMe(q) @ self.eMc
                    cMo = np.linalg.inv(fMc_now) @ fMc_img @ cMo_img

                cdMc = HomogeneousMatrix(self.cdMo @ np.linalg.inv(cMo))
                self.t.buildFrom(cdMc)
                self.tu.buildFrom(cdMc)

                eJe = self.kin.eJe(q)
                self.task.set_cVe(self.cVe)
                self.task.set_eJe(Matrix(eJe))
                qd = np.array(self.task.computeControlLaw().numpy()).reshape(-1)

                err_t = float(np.linalg.norm(np.array(cdMc.getTranslationVector().numpy()).reshape(-1)))
                err_tu = float(np.linalg.norm(np.array(cdMc.getThetaUVector().numpy()).reshape(-1)))
                converged = err_t < a.conv_t and err_tu < math.radians(a.conv_tu)

                if self.task.getTaskRank() < 6:
                    reason = "task jacobian rank < 6"
                    break
                smin = np.linalg.svd(eJe, compute_uv=False).min()
                if smin < a.min_sv:
                    reason = f"near singularity (sigma_min={smin:.4f})"
                    break
                if not np.all(np.isfinite(qd)):
                    reason = "non-finite joint velocity"
                    break

                qd = self._scale_to_limit(qd, qd_max)
                qd = qd_prev + self._scale_to_limit(qd - qd_prev, dqd_max)
                if a.stop_on_convergence and converged:
                    reason = "converged"
                    break
                send = True
            else:
                qd = qd_prev + self._scale_to_limit(-qd_prev, dqd_max)  # 平滑减速
                send = moving

            if np.max(np.abs(q_cmd - q)) > math.radians(5.0):
                q_cmd = q.copy()
            q_next = q_cmd + qd * dt
            if np.any(np.abs(q_next) > 2 * math.pi - math.radians(2.0)):
                reason = "joint limit reached"
                break

            if send:
                if a.mode == "servoj":
                    ok = self.robot.servoJ(q_next, dt, a.lookahead, a.gain)
                else:
                    ok = self.robot.speedJ(qd, a.speedj_acc, dt)
                if not ok:
                    reason = "robot rejected command"
                    break
                q_cmd = q_next
                moving = np.max(np.abs(qd)) > 1e-6
            else:
                q_cmd = q.copy()
            qd_prev = qd

            if log:
                log.write(f"{now - t0:.6f}," + ",".join(f"{x:.6f}" for x in q) + "," + ",".join(f"{x:.6f}" for x in q_cmd)
                          + "," + ",".join(f"{x:.6f}" for x in qd) + f",{err_t:.6f},{err_tu:.6f}\n")
            if a.verbose and it % int(a.rate) == 0:
                print(f"t={now - t0:6.2f}s err_t={err_t * 1000:7.2f} mm err_tu={math.degrees(err_tu):6.2f} deg "
                      f"qd(deg/s)={np.round(np.degrees(qd), 2)}")

            self.robot.waitPeriod(t_cycle, dt)

        self.robot.stop(math.radians(a.max_qddot))
        if log:
            log.close()
        return reason


# --------------------------------------------------------------------------- #
#                                 视觉线程                                      #
# --------------------------------------------------------------------------- #
def simulated_vision_thread(robot, kin, eMc, fMo0, moving, src, stop_evt):
    rng = np.random.default_rng(0)
    t0 = time.monotonic()
    while not stop_evt.is_set():
        t = time.monotonic() - t0
        fMo = fMo0
        if moving:
            oMo_t = np.array(HomogeneousMatrix(0.03 * math.sin(0.5 * t), 0.03 * math.cos(0.5 * t), 0.0,
                                               0.0, 0.0, math.radians(10) * math.sin(0.3 * t)).numpy())
            fMo = fMo0 @ oMo_t
        q = robot.getActualQ()
        fMc = kin.fMe(q) @ eMc
        n = rng.normal(0, 1, 6) * np.array([5e-4] * 3 + [math.radians(0.1)] * 3)
        noise = np.array(HomogeneousMatrix(*n).numpy())
        src.update(np.linalg.inv(fMc) @ fMo @ noise, fMc)
        time.sleep(1.0 / 30)


def user_tracker_thread(robot, kin, eMc, src, stop_evt):
    """
    在这里接入你自己的跟踪器，例如：
        while not stop_evt.is_set():
            I = grab()
            q = robot.getActualQ()                     # 尽量取靠近曝光时刻的 q
            fMc = kin.fMe(q) @ eMc
            ok, cMo = tracker.track(I)                 # cMo: 4x4 numpy 或 HomogeneousMatrix
            if ok: src.update(np.array(cMo), fMc)
            else:  src.invalidate()
    """
    print("user_tracker_thread: 请接入你的跟踪器 (cMo -> src.update)", file=sys.stderr)
    stop_evt.set()


def load_pose_yaml(filename):
    """读取 ViSP vpPoseVector::saveYAML 生成的文件（手眼标定 eMc.yaml）"""
    ePc = PoseVector()
    try:
        if PoseVector.loadYAML(filename, ePc):
            return ePc
    except AttributeError:
        pass
    values = []
    with open(filename) as f:
        in_data = False
        for line in f:
            if line.strip().startswith("data:"):
                in_data = True
                continue
            if in_data and "[" in line:
                values.append(float(line.strip().strip("-").strip().strip("[]")))
    if len(values) != 6:
        raise RuntimeError(f"Cannot parse pose vector from {filename}")
    return PoseVector(*values)


# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", default="192.168.0.100")
    p.add_argument("--model", default="ur5e", choices=sorted(UR_DH.keys()))
    p.add_argument("--eMc", default="", help="手眼标定 YAML (vpPoseVector)")
    p.add_argument("--cdMo", nargs=6, type=float, default=[0, 0, 0.30, math.pi, 0, 0],
                   help="期望的物体在相机系下位姿 tx ty tz tux tuy tuz")
    p.add_argument("--mode", default="servoj", choices=["servoj", "speedj"])
    p.add_argument("--rate", type=float, default=500.0)
    p.add_argument("--lambda", dest="lambda_", type=float, default=0.5)
    p.add_argument("--adaptive-gain", action="store_true")
    p.add_argument("--max-qdot", type=float, default=60.0, help="deg/s")
    p.add_argument("--max-qddot", type=float, default=300.0, help="deg/s^2")
    p.add_argument("--min-sv", type=float, default=0.01)
    p.add_argument("--lookahead", type=float, default=0.1)
    p.add_argument("--gain", type=float, default=300.0)
    p.add_argument("--speedj-acc", type=float, default=2.0)
    p.add_argument("--vision-timeout", type=float, default=0.5)
    p.add_argument("--conv-t", type=float, default=0.001, help="m")
    p.add_argument("--conv-tu", type=float, default=0.5, help="deg")
    p.add_argument("--stop-on-convergence", action="store_true")
    p.add_argument("--no-motion-compensation", action="store_true")
    p.add_argument("--log", default="")
    p.add_argument("--sim", action="store_true")
    p.add_argument("--sim-moving", action="store_true")
    p.add_argument("--sim-duration", type=float, default=10.0)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    if args.eMc:
        ePc = load_pose_yaml(args.eMc)
    else:
        print("Warning: --eMc not given, using default eMc")
        ePc = PoseVector(0, 0, 0.05, 0, 0, 0)
    eMc = np.array(HomogeneousMatrix(ePc).numpy())
    cdMo = np.array(HomogeneousMatrix(*args.cdMo).numpy())

    kin = UrKinematics(args.model)
    stop_evt = threading.Event()

    if args.sim:
        q0 = np.radians([0, -100, 100, -90, -90, 0])
        robot = SimulatedUr(q0)
    else:
        robot = RtdeUr(args.ip, args.rate)
        kin.flange_M_tcp = ur_pose_to_matrix(robot.getTCPOffset())
        q = robot.getActualQ()
        dM = np.linalg.inv(kin.fMe(q)) @ ur_pose_to_matrix(robot.getActualTCPPose())
        dr = math.degrees(np.linalg.norm(np.array(HomogeneousMatrix(dM).getThetaUVector().numpy()).reshape(-1)))
        print(f"Kinematics self-check: |t_err|={np.linalg.norm(dM[:3, 3]) * 1000:.2f} mm |r_err|={dr:.2f} deg")
        input("WARNING: the robot will move. Press Enter to continue...")

    src = ObjectPoseSource()
    ctrl = JointServoController(robot, kin, eMc, cdMo, src, args)

    if args.sim:
        cMo_init = np.array(HomogeneousMatrix(0.08, -0.05, 0.45, math.pi + math.radians(15),
                                              math.radians(-10), math.radians(20)).numpy())
        fMo0 = kin.fMe(robot.getActualQ()) @ eMc @ cMo_init
        vision = threading.Thread(target=simulated_vision_thread,
                                  args=(robot, kin, eMc, fMo0, args.sim_moving, src, stop_evt), daemon=True)
        timer = threading.Timer(args.sim_duration, lambda: (stop_evt.set(), ctrl.stop_requested.set()))
        timer.start()
    else:
        vision = threading.Thread(target=user_tracker_thread, args=(robot, kin, eMc, src, stop_evt), daemon=True)
    vision.start()

    try:
        reason = ctrl.run()
    except KeyboardInterrupt:
        ctrl.stop_requested.set()
        robot.stop(math.radians(args.max_qddot))
        reason = "KeyboardInterrupt"
    stop_evt.set()
    print("Servo loop stopped:", reason)


if __name__ == "__main__":
    main()
