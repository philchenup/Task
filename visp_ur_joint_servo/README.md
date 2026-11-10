# ViSP 视觉伺服 → UR 机械臂关节空间实时控制

本目录给出一套基于 [ViSP (lagadic/visp)](https://github.com/lagadic/visp) 的
**位置视觉伺服 (PBVS) → 关节速度 → 关节轨迹** 实时控制实现：

输入是你已经得到的 *物体在相机坐标系下的位姿* `cMo`，输出是以 500 Hz (e-Series) / 125 Hz (CB3)
实时发给 UR 控制器的 **关节轨迹点 `q_cmd(t)`（servoj）** 或 **关节速度 `q_dot(t)`（speedj）**。

```
                 30 Hz                                   500 Hz
 你的跟踪器 ──cMo──▶ ObjectPoseSource ──▶ JointServoController ──q_cmd/q_dot──▶ UR (ur_rtde servoJ/speedJ)
                                            │   vpServo(EYEINHAND_L_cVe_eJe)
                                            │   eJe ← UrKinematics(DH)
                                            └── q ← RTDE getActualQ()
```

---

## 1. ViSP 与本任务相关的分析

### 1.1 模块划分

| 模块 | 与本任务的关系 |
|---|---|
| `visp_core` | `vpHomogeneousMatrix`、`vpVelocityTwistMatrix`、`vpMatrix::pseudoInverse` 等数学工具 |
| `visp_visual_features` | `vpFeatureTranslation(cdMc)` + `vpFeatureThetaU(cdRc)` 组成 PBVS 的 6 维特征 |
| `visp_vs` | `vpServo`：根据特征误差、交互矩阵、`cVe`、`eJe` 计算速度指令；`vpAdaptiveGain` |
| `visp_robot` | `vpRobotUniversalRobots`：基于 `ur_rtde` 的 UR 接口（见 1.3 的局限） |
| `visp_tracker / detection / vision` | 你已有的跟踪器属于这一层，输出 `cMo` 即可，本方案不关心它如何得到 |

### 1.2 `vpServo` 的控制模式

`vpServo::setServo()` 提供以下模式（`modules/vs/include/visp3/vs/vpServo.h`）：

| 模式 | 输出 | 需要用户提供 |
|---|---|---|
| `EYEINHAND_CAMERA` | 相机速度 `v_c = -λ L⁺ e` | 无 |
| `EYEINHAND_L_cVe_eJe` | **关节速度 `q̇ = -λ (L cVe eJe)⁺ e`** | `set_cVe()`、`set_eJe()`（每个周期） |
| `EYETOHAND_*` | 关节速度（眼在外） | `cVf / fVe / eJe / fJe` |

要直接输出关节量，必须使用 `EYEINHAND_L_cVe_eJe`，其中：

- `cVe`：末端 → 相机的速度旋量变换，由手眼标定结果 `eMc` 得到（`task.set_cVe(eMc.inverse())`）
- `eJe`：机器人**末端坐标系下**的几何雅可比，随关节角变化，每个周期都要重新 `set_eJe()`
  （`computeControlLaw()` 之后 ViSP 会把 `init_eJe` 复位，见 `vpServo.cpp`）

### 1.3 `vpRobotUniversalRobots` 的能力与局限

`modules/robot/src/real-robot/universal-robots/vpRobotUniversalRobots.cpp`：

- 底层就是 `ur_rtde` 的 `RTDEControlInterface / RTDEReceiveInterface`
- `setVelocity(JOINT_STATE, q̇)` → `speedJ(q̇, a=0.5, dt=1ms)`
- `setVelocity(CAMERA_FRAME, v_c)` → 先用 `fVe * eVc` 转到基座系，再 `speedL(...)`，**逆运动学由 UR 控制器完成**
- `get_eJe()` / `get_fJe()` **未实现**（空函数，标注 "Not implemented yet"）
- 没有 `servoJ` 封装

因此 ViSP 官方 UR 教程 (`example/servo-universal-robots/servoUniversalRobotsPBVS.cpp`,
`tutorial-universal-robot-pbvs`) 走的是 **路线 A：`EYEINHAND_CAMERA` → `v_c` → `speedL`**，
不会输出关节轨迹。

### 1.4 两条技术路线

| | 路线 A（ViSP 官方示例） | 路线 B（本目录实现） |
|---|---|---|
| vpServo 模式 | `EYEINHAND_CAMERA` | `EYEINHAND_L_cVe_eJe` |
| 输出 | 相机笛卡尔速度 `v_c` | 关节速度 `q̇` → 积分为关节轨迹 `q_cmd` |
| 发送方式 | `speedL` | `servoJ`（位置）或 `speedJ`（速度） |
| 雅可比 | 由 UR 控制器隐式处理 | 需自带 UR 运动学模型（`UrKinematics`） |
| 可控性 | 无法处理奇异/关节限位/零空间 | 可做奇异检测、关节限速限加速、二次任务 (`vpServo::secondaryTask`) |
| 轨迹可见性 | 无 | 每个周期得到 `q_cmd`、`q̇`，可记录/转发给其它控制栈 |

路线 B 正是你要的"实时输出关节轨迹用于关节控制"。缺的那块（`eJe`）用 UR 官方 DH 参数补上。

---

## 2. 控制律与实现要点

### 2.1 PBVS 特征与控制律

```
e      = [ cd_t_c ;  θu(cd_R_c) ]            cdMc = cdMo · cMo⁻¹
L_e    = [ cd_R_c  0 ; 0  L_ω ]              (ViSP 内部，CURRENT 交互矩阵)
q̇      = -λ · ( L_e · cVe · eJe )⁺ · e        (vpServo::EYEINHAND_L_cVe_eJe)
q_cmd ← q_cmd + q̇·Δt                           Δt = 1/500 s
servoJ(q_cmd, 0, 0, Δt, lookahead=0.1, gain=300)
```

`cdMo` 是你希望"物体在相机系中呈现"的目标位姿，例如 `(0, 0, 0.3, π, 0, 0)`：物体在光轴前 0.3 m 且正对相机。
物体运动时，`cMo` 持续更新，控制律就持续把相机"拉"到相对物体的期望位姿上——即实时跟踪。

### 2.2 多速率与机器人自运动补偿

视觉 30 Hz，控制 500 Hz。两帧之间 `cMo` 不变但机器人在动，直接用旧 `cMo` 会引入滞后/抖动。
跟踪器发布时同时记录取像时刻的 `fMc_img`（由当时关节角正解得到），控制线程用

```
cMo_now = fMc_now⁻¹ · fMc_img · cMo_img
```

将物体位姿"搬"到当前相机系（静止物体精确；运动物体相当于零阶保持 + 自运动补偿）。

### 2.3 `eJe` 与坐标系约定

- `UrKinematics` 用 UR 官方标准 DH 参数（UR3/3e/5/5e/10/10e/16e/20/30）计算 `fMe`、`fJe`、`eJe`
- `eJe` 对基座坐标系的选取不敏感（基座绕 z 转 180° 不改变 `eJe`，`test_ur_kinematics` 中有验证），
  所以 UR "Base" 与 ROS `base_link` 之争不影响控制
- **`e` 坐标系必须与手眼标定 `eMc` 时用的一致**。ViSP 的 `vpRobotUniversalRobots::getPosition(END_EFFECTOR_FRAME)`
  返回的是控制器 TCP 位姿，所以若标定时 TCP 偏置非零，需要 `kin.set_flangeMtcp(getTCPOffset())`
  （真机模式下程序启动时自动做，并把 DH 正解与 `getActualTCPPose()` 对比自检）

### 2.4 安全保护（控制线程内）

- 关节速度饱和（整体缩放保持方向）、关节加速度限制
- 关节软限位 ±2π − margin
- 奇异检测：`vpServo::getTaskRank() < 6` 或 `σ_min(eJe) < 0.01` 即停止
- 视觉看门狗：超过 `vision_timeout`（默认 0.5 s）没有新位姿 → 平滑减速至 0
- `q_cmd` 与实际 `q` 偏差 > 5° 时重新同步（防止 servoj 跟不上时累积）
- 退出/异常 → `servoStop()/speedStop()`

---

## 3. 代码结构

```
visp_ur_joint_servo/
├── CMakeLists.txt
├── include/
│   ├── ur_kinematics.h           UR DH 正解 + fJe/eJe（vpServo::set_eJe 所需）
│   ├── object_pose_source.h      跟踪器 → 控制线程 的线程安全 cMo 缓冲（带 fMc 时间戳）
│   ├── ur_joint_interface.h      关节控制接口抽象：ur_rtde 真机 / 纯仿真
│   └── joint_servo_controller.h  vpServo(EYEINHAND_L_cVe_eJe) → q̇ → q_cmd → servoJ/speedJ
├── src/
│   ├── visp_ur_joint_servo.cpp   主程序：参数、线程、示例跟踪器(RealSense+AprilTag)/仿真视觉
│   └── test_ur_kinematics.cpp    DH 零位、数值雅可比、基座不变性 测试
├── python/
│   └── visp_ur_joint_servo.py    同一方案的 Python 版本（ViSP Python 绑定 + ur_rtde）
└── config/
    └── eMc_example.yaml          手眼标定文件格式示例
```

## 4. 编译与运行

依赖：ViSP ≥ 3.6（`libvisp-dev` 或源码），[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)（真机），CMake ≥ 3.10。

```bash
cd visp_ur_joint_servo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release     # 未找到 ur_rtde 时自动退化为仅仿真
make -j
ctest                                     # 运动学测试 + 3 s 仿真收敛测试
```

离线仿真（无机器人、无相机，验证控制律与轨迹）：

```bash
./visp_ur_joint_servo --sim --model ur5e --log traj.csv --verbose
./visp_ur_joint_servo --sim --sim-moving --lambda 2 --verbose        # 物体运动
./visp_ur_joint_servo --sim --adaptive-gain --mode speedj --stop-on-convergence
```

真机：

```bash
./visp_ur_joint_servo --ip 192.168.0.100 --model ur5e --eMc eMc.yaml \
    --mode servoj --rate 500 --lambda 0.5 --max-qdot 40 --log traj.csv
```

Python 版本：

```bash
pip install ur_rtde numpy      # ViSP Python 绑定需 ViSP ≥ 3.6.1 源码编译或 conda-forge
python3 python/visp_ur_joint_servo.py --sim --verbose
python3 python/visp_ur_joint_servo.py --ip 192.168.0.100 --model ur5e --eMc eMc.yaml
```

轨迹 CSV 列：`t, q1..q6(实际), qcmd1..6(发送), qd1..6, e_tx..e_tuz, err_t, err_tu`。

## 5. 接入你自己的跟踪器

只需要在视觉线程里把每一帧的 `cMo` 送进 `ObjectPoseSource`：

```cpp
// src/visp_ur_joint_servo.cpp : userTrackerThread()
while (!g_quit) {
  grabber.acquire(I);
  vpColVector q = robot->getActualQ();                // 尽量靠近曝光时刻
  vpHomogeneousMatrix fMc = kin.get_fMe(q) * eMc;     // 取像时刻相机位姿（运动补偿用）

  if (tracker.track(I, cMo))  pose_source.update(cMo, fMc);   // vpMbGenericTracker / 深度学习 / AprilTag ...
  else                        pose_source.invalidate();       // 跟丢 → 控制线程自动减速停止
}
```

仓库里自带的示例跟踪器是 RealSense + AprilTag（与 ViSP 官方 UR 教程一致），ViSP 编译时带
`librealsense2` 和 `apriltag` 即可直接运行。

## 6. 参数调节建议

| 参数 | 说明 |
|---|---|
| `--lambda` | 误差指数收敛的时间常数 1/λ。0.5 → 2 s；跟踪快速目标可用 1~3，配合 `--adaptive-gain` 避免起步过冲 |
| `--rate` | e-Series 500，CB3 125。必须与 RTDE 频率一致，`waitPeriod()` 由 ur_rtde 保证周期 |
| `servoj_lookahead / gain` | 0.1 s / 300 为平滑默认；要更跟手可 0.03~0.05 s / 500~1000，但会放大视觉噪声 |
| `--max-qdot / --max-qddot` | 关节限速限加速，首次上机建议 20 deg/s |
| `--mode speedj` | 跳过积分直接发关节速度，和 ViSP `setVelocity(JOINT_STATE)` 等价，但轨迹不再显式可见 |
| `--cdMo` | 期望的物体在相机系位姿；对称物体注意 θu 的 π 翻转歧义（见官方示例中的 `oMo` 处理） |

## 7. 验证结果（仿真）

`test_ur_kinematics`：

```
[FK zero] p = (-0.8172, -0.2329, 0.0628)  expected (-0.8172, -0.2329, 0.0628)  err=0
[Jacobian model UR3e/UR5/UR5e/UR10e] max|eJe-num| ≈ 3e-10
[eJe base invariance] err = 1.1e-16
ALL TESTS PASSED
```

`visp_ur_joint_servo --sim --verbose`（λ=0.5，初始偏差 0.14 m / 13°）：误差按 e^{-0.5t} 指数收敛，
6 s 后 0.019 m / 1.8°；`--adaptive-gain --stop-on-convergence` 约 8 s 内到达 1 mm / 0.5° 并停止；
`--sim-moving --lambda 2` 跟踪运动物体时稳态误差约 7 mm。

## 8. 上机注意

- 先用 `--sim` 和低增益、低限速确认方向正确；上机时手握急停
- 控制 PC 建议 RT 内核或至少独占核心，500 Hz 周期抖动会直接体现在 servoj 上
- 视觉延迟越大，等效相位滞后越大，λ 需相应降低；可在 `ObjectPoseSource::update` 中传入精确的曝光时间戳
- 若手眼标定来自 ViSP `tutorial-hand-eye-calibration`，`eMc.yaml` 格式与 `config/eMc_example.yaml` 相同
