# PROFINET 上位机 ↔ KUKA 焊缝通信（精简版）

128 字节过程映像：`$IN/$OUT[3025..4048]`，cifX 字节偏移 **378**（当映像字节 0 对应 `$IN[1]`）。焊机手动量：`$IN[2001]` 区，偏移 **250**，40 字节。

## 命令 `RS_CMD`

| 值 | 含义 | 说明 |
|---|------|------|
| 1 | 关节运动 | `RJ_A1..RJ_E3` + 轴/外部轴速度加速度 [%] |
| 2 | 焊缝配置 | 两点 `WS_P1_*` / `WS_P2_*` + 摆动 `WS_SPEED/AMP/FREQ/WEAVE` + 焊机 Job/Wire/Volt/Dyn |
| 3 | 执行焊接 | 需先下发 cmd 2，且 `RS_ENABLE=1` |
| 4 | 清除任务缓冲 | |

每次命令在同一帧内翻转 `RS_TOGGLE`；机器人沿边沿锁存，并在 `TC_ACK_TOGGLE` 回显。

## 推荐上位机流程

1. **点动到位**：`slotSendJoint`（`enable=true`，带 `JointCommand` 速度/加速度）。
2. **下发焊缝**：`slotSendWeldSetup`（两点 + `WeldProcess` + `WeldTaskParams`）。
3. **起弧焊接**：`slotSendTaskCommand`（`cmd=KukaCmdWeldExecute`）。
4. **焊机维护**：`slotSendWeldData`（`weld_mode`、`weld_arc_start`、`gas_ready`、`extern_control_enable`、`weld_job_number`、校正值等必要位）。

## 机器人上行

- **RobotGetData**：轴/外部轴速度加速度、E1–E3、A1–A6 关节角。
- **WeldGetData**：电弧稳定、电源就绪、焊接中、电压/电流/送丝速度。
- **KukaStatus**：驱动/使能/模式/程序状态、`pcLinkOk`、心跳、E4–E6。
- **TaskAck**：任务状态、错误码、进度、`arcOn`。

## 目录

- `pc/Profinet.{h,cpp}` — 上位机 cifX 打包与解析
- `krc/R1/System/$config.dat` — IO 信号与缓冲变量
- `krc/R1/Program/pc2kuka.src` — 命令解释与焊缝 LIN+摆动
- `krc/R1/System/sps.sub` — 周期反馈与 EWM 输出仲裁