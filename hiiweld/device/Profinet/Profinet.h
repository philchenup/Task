/**
 * @file Profinet.h
 * @brief KUKA Profinet/cifX 通信：128 字节过程映像、焊接与关节轨迹命令编解码。
 *
 * KUKA $IN/$OUT[3025..4048]：128 字节 / 1024 位，字节对齐自 3025；
 * cifX 字节偏移 (3025-1)/8 = 378，映像缓冲 bit0 对应 $IN[3025]。
 * PC 在 KukaHeader::toggle 上翻转，KUKA 沿边沿锁存并在 TaskAck::toggle 回显。
 *
 * RS_CMD（简化）：
 *  1 关节目标  2 焊接设置  3 执行锁存焊任务  4 清焊缓冲
 *  5 追加重轨迹点  6 运行缓冲轨迹（RS_CONT 交融）  7 清关节轨迹缓冲
 *
 * 焊机手动 I/O：WeldSendData @ $IN[2001]，cifX 偏移 250，40 字节。
 */
#ifndef PROFINETCOMM_H
#define PROFINETCOMM_H

#include <QObject>
#include <QString>
#include <QMetaType>
#include <QMutex>

#include "cifXUser.h"
#include "cifxErrors.h"

#include <cstdint>
#include <cstddef>

class QTimer;

constexpr uint32_t kKukaImageBytes = 128;   /**< KUKA 命令/状态映像长度（字节）。 */
constexpr uint32_t kKukaSignalBase = 3025;  /**< KUKA 信号区起始 IO 编号。 */
constexpr uint32_t kKukaCommandOffset = (kKukaSignalBase - 1) / 8; /**< 命令区 cifX 偏移 378。 */
constexpr uint32_t kKukaStatusOffset = kKukaCommandOffset;         /**< 状态区偏移（与命令同区读回）。 */
constexpr uint32_t kKukaWeldCmdOffset = 250; /**< 焊机命令块 cifX 偏移。 */
constexpr uint32_t kKukaWeldCmdBytes = 40;   /**< 焊机命令块长度。 */

/** @brief 发往 KUKA 的 RS_CMD 命令字。 */
enum KukaCmd : uint8_t {
    KukaCmdNone = 0,
    KukaCmdJoint = 1,
    KukaCmdWeldSetup = 2,
    KukaCmdWeldExecute = 3,
    KukaCmdWeldClear = 4,
    KukaCmdJointTrajAppend = 5,
    KukaCmdJointTrajRun = 6,
    KukaCmdJointTrajClear = 7
};

/** @brief 摆动工艺类型。 */
enum KukaWeave : uint8_t {
    KukaWeaveOff = 0,
    KukaWeaveTriangle = 1,
    KukaWeaveSine = 2
};

/** @brief KUKA 任务状态机。 */
enum KukaTaskState : uint8_t {
    KukaTaskIdle = 0,
    KukaTaskReady = 1,
    KukaTaskActive = 2,
    KukaTaskDone = 3,
    KukaTaskError = 4
};

/** @brief KUKA 任务错误码。 */
enum KukaTaskError : uint8_t {
    KukaErrNone = 0,
    KukaErrPointCount = 1,
    KukaErrPage = 2,
    KukaErrFrame = 3,
    KukaErrArc = 4,
    KukaErrAbort = 5,
    KukaErrCommand = 6,
    KukaErrCorner = 7,
    KukaErrEnable = 8,
    KukaErrWeave = 9,
    KukaErrTrajEmpty = 10,
    KukaErrTrajFull = 11
};

/** @brief 自映像解析的机器人关节/TCP 反馈。 */
struct RobotGetData
{
    float axis_vel;
    float axis_acc;
    float ext_axis_vel;
    float ext_axis_acc;

    float E1;
    float E2;
    float E3;
    float SJ1;
    float SJ2;
    float SJ3;
    float SJ4;
    float SJ5;
    float SJ6;

    float X; /**< TCP X（mm，当前 BASE/TOOL）。 */
    float Y;
    float Z;
    float A; /**< 姿态 A（度）。 */
    float B;
    float C;
};
Q_DECLARE_METATYPE(RobotGetData)

/** @brief 焊机侧监控量（弧稳、给定、过程标志）。 */
struct WeldGetData
{
    bool arc_stability;
    bool welding_ps_ready;
    bool welding_processing;
    uint16_t monitor_voltage;
    uint16_t monitor_current;
    uint16_t monitor_wire_speed;
};
Q_DECLARE_METATYPE(WeldGetData)

/** @brief KUKA 运行状态（模式、程序、安全、心跳等）。 */
struct KukaStatus
{
    uint8_t actTool = 0;
    uint8_t actBase = 0;
    uint8_t overridePct = 0;
    uint8_t modeOp = 0;    /**< 1 T1, 2 T2, 3 AUT, 4 EX */
    uint8_t proState = 0;  /**< 1 free, 2 reset, 3 active, 4 stop, 5 end */
    bool drivesOn = false;
    bool moveEnable = false;
    bool stopMess = false;
    bool robotReady = false;
    bool pcLinkOk = false;
    uint16_t appErr = 0;
    uint16_t heartbeat = 0;
    float E4 = 0.f;
    float E5 = 0.f;
    float E6 = 0.f;
};
Q_DECLARE_METATYPE(KukaStatus)

/** @brief 任务应答（命令回显、忙、进度、轨迹点数）。 */
struct TaskAck
{
    uint8_t cmd = 0;
    uint8_t taskState = 0;
    uint8_t errCode = 0;
    uint16_t taskId = 0;
    bool toggle = false;
    bool busy = false;
    bool arcOn = false;
    uint16_t frameNo = 0;
    uint16_t progress = 0; /**< 0..10000 */
    uint8_t trajPointCount = 0;
};
Q_DECLARE_METATYPE(TaskAck)

/** @brief 一帧输入映像的完整反馈。 */
struct KukaFeedback
{
    RobotGetData robot;
    WeldGetData weld;
    KukaStatus status;
    TaskAck task;
};

/** @brief 下发命令头（含 alive/toggle 等握手位）。 */
struct KukaHeader
{
    uint8_t cmd = KukaCmdNone;
    uint8_t toolId = 1;
    uint8_t baseId = 0;
    bool enable = false;
    bool alive = false;   /**< 由 ProfinetComm 周期翻转。 */
    bool reset = false;
    bool goHome = false;
    bool abort = false;
    bool toggle = false;  /**< 发送时由 ProfinetComm 翻转。 */
    bool cont = false;    /**< 命令 6：轨迹点 PTP 交融。 */
    uint16_t taskId = 0;
    uint16_t frameNo = 0; /**< 追加轨迹：1 表示新路径起点（清缓冲）。 */
};

/** @brief 关节空间运动命令（A1–A6、外部轴、速度加速度百分比）。 */
struct JointCommand
{
    float a1 = 0.f, a2 = 0.f, a3 = 0.f, a4 = 0.f, a5 = 0.f, a6 = 0.f;
    float e1 = 0.f, e2 = 0.f, e3 = 0.f;
    float axisSpeed = 30.f;
    float axisAcc = 50.f;
    float extSpeed = 100.f;
    float extAcc = 100.f;
};

/** @brief 焊缝点位姿（XYZABC，毫米/度）。 */
struct WeldPoint
{
    float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
};

/** @brief 焊接过程参数（速度、摆动、Job/送丝/电压等）。 */
struct WeldProcess
{
    float speedMmS = 0.f;
    float ampMm = 0.f;
    float freqHz = 0.f;
    uint8_t weave = KukaWeaveOff;
    uint8_t job = 0;
    uint16_t wire = 0;
    uint16_t volt = 0;
    uint16_t dyn = 0;
};

/** @brief 焊接任务辅助参数（起弧/收弧/空移、轴速度等）。 */
struct WeldTaskParams
{
    float craterMm = 0.f;
    float approachMm = 50.f;
    float airSpeedMmS = 500.f;
    float axisSpeedPct = 30.f;
    float axisAccPct = 50.f;
    float extSpeedPct = 100.f;
    float extAccPct = 100.f;
    uint8_t samples = 0; /**< 0 表示默认采样数（三角 4 / 正弦 8）。 */
};

/** @brief 焊机手动命令（$IN[2001] 区，对应 PLC_EWM_*）。 */
struct WeldSendData
{
    bool weld_mode = false;
    bool weld_arc_start = false;
    bool robot_ready = false;
    bool gas_ready = false;
    bool extern_control_enable = false;
    bool weld_errack = false;
    uint8_t weld_job_number = 0;
    uint16_t weld_wire_speed_corr = 0;
    uint16_t weld_arc_volt_corr = 0;
    uint16_t weld_dynamin_corr = 0;
};
Q_DECLARE_METATYPE(WeldSendData)

/** @brief cifX 读写区域与周期配置。 */
struct ProfinetIoLayout
{
    uint32_t area = 0;
    uint32_t commandOffset = kKukaCommandOffset;
    uint32_t weldCmdOffset = kKukaWeldCmdOffset;
    uint32_t statusOffset = kKukaStatusOffset;
    uint32_t timeoutMs = 10;
    int cycleMs = 50;
};

/**
 * @brief Profinet/cifX 通信对象：初始化板卡、周期 IO、组帧与解析 KUKA/焊机数据。
 */
class ProfinetComm : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief 构造并注册 Qt 元类型。
     * @param parent Qt 父对象。
     */
    explicit ProfinetComm(QObject* parent = nullptr);

    /** @brief 停止周期 IO 并关闭 cifX 通道。 */
    ~ProfinetComm() override;

    /**
     * @brief 设置过程映像区域与偏移。
     * @param layout IO 布局参数。
     */
    void setIoLayout(const ProfinetIoLayout& layout);

    /** @brief 释放 Profinet 板卡（同 releaseProfinetCard 槽逻辑）。 */
    void releaseProfinetCard();

    /**
     * @brief 仅编码命令头到位图 send。
     * @param send 至少 128 字节输出缓冲。
     * @param head 命令头。
     */
    void MakeSendHeader(uint8_t* send, const KukaHeader& head);

    /**
     * @brief 编码关节运动命令（头 + A1–A6/E + 速度加速度）。
     */
    void MakeSendJoint(uint8_t* send, size_t len, const KukaHeader& head, const JointCommand& cmd);

    /**
     * @brief 编码焊接设置（起终点、工艺、任务参数）。
     */
    void MakeSendWeldSetup(uint8_t* send, size_t len, const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2,
        const WeldProcess& proc, const WeldTaskParams& params);

    /**
     * @brief 编码焊机手动 I/O 块（40 字节）。
     */
    void MakeSendWeldData(uint8_t* send, size_t len, const WeldSendData& in);

    /** @brief 从映像解析机器人反馈。 */
    void ParseRecvRobotData(const uint8_t* recv, size_t len, RobotGetData& in);

    /** @brief 从映像子区解析焊机反馈。 */
    void ParseRecvWeldData(const uint8_t* recv, size_t len, WeldGetData& in);

    /** @brief 解析整帧 128 字节为 KukaFeedback。 */
    void ParseRecvImage(const uint8_t* recv, size_t len, KukaFeedback& out);

    /**
     * @brief cifX 写过程数据（带错误上报）。
     * @return 成功 true。
     */
    bool ProfinetWrite(uint32_t areaNumber, uint32_t offset, uint32_t len,
        const uint8_t* buffer, uint32_t timeoutMs);

    /**
     * @brief cifX 读过程数据（带错误上报）。
     */
    bool ProfinetRead(uint32_t areaNumber, uint32_t offset, uint32_t len,
        uint8_t* buffer, uint32_t timeoutMs);

    /** @brief 线程安全读取缓存的机器人数据。 */
    RobotGetData latestRobotData() const;
    /** @brief 线程安全读取焊机数据。 */
    WeldGetData latestWeldData() const;
    /** @brief 线程安全读取 KUKA 状态。 */
    KukaStatus latestStatus() const;
    /** @brief 线程安全读取任务应答。 */
    TaskAck latestTaskAck() const;

public slots:
    /** @brief 打开 cifX 驱动与通道，置 READY/总线 ON 并启动 IO 周期。 */
    void initProfinet();

    /**
     * @brief 启动定时 onIoCycle。
     * @param intervalMs 周期毫秒，0 表示用 layout.cycleMs。
     */
    void startIoCycle(int intervalMs = 0);

    /** @brief 停止 IO 定时器。 */
    void stopIoCycle();

    /** @brief 下发单点关节命令（RS_CMD=1）。 */
    void slotSendJoint(const KukaHeader& head, const JointCommand& cmd);
    /** @brief 追加重轨迹点（RS_CMD=5）。 */
    void slotAppendJointTrajectory(const KukaHeader& head, const JointCommand& cmd);
    /** @brief 运行缓冲轨迹（RS_CMD=6）。 */
    void slotRunJointTrajectory(const KukaHeader& head);
    /** @brief 清空轨迹缓冲（RS_CMD=7）。 */
    void slotClearJointTrajectory(const KukaHeader& head);
    /** @brief 焊接设置（RS_CMD=2）。 */
    void slotSendWeldSetup(const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2,
        const WeldProcess& proc, const WeldTaskParams& params);
    /** @brief 执行或清除焊任务（RS_CMD=3/4）。 */
    void slotSendTaskCommand(const KukaHeader& head);
    /** @brief 写焊机手动命令块。 */
    void slotSendWeldData(const WeldSendData& data);

signals:
    void sigInfoMsg(const QString& msg);
    void sigErrMsg(const QString& msg);
    void sigWeldData(const WeldGetData& data);
    void sigRobotData(const RobotGetData& data);
    void sigKukaStatus(const KukaStatus& data);
    void sigTaskAck(const TaskAck& data);
    void sigConnected(const bool& m_connected);

private slots:
    /** @brief 定时：翻转 alive、写输出、读输入并发射信号。 */
    void onIoCycle();

private:
    void closeChannel();
    void writeCachedOutputs();
    void readInputs();
    KukaHeader prepare(KukaHeader head);
    void commitImage(const uint8_t* image);
    bool cifxTransfer(bool write, uint32_t area, uint32_t offset, uint32_t len,
        uint8_t* buffer, uint32_t timeoutMs, bool latchError);

    bool profinetInited = false;
    QString szBoardname = QStringLiteral("cifX0");
    CIFXHANDLE m_hDriver = NULL;
    uint32_t m_ulChannel = 0;
    CIFXHANDLE m_hChannel = NULL;

    ProfinetIoLayout m_layout;
    uint8_t m_image[kKukaImageBytes] = {};
    uint8_t m_weldImage[kKukaWeldCmdBytes] = {};
    bool m_imageValid = false;
    bool m_weldValid = false;
    bool m_alive = false;
    bool m_lastToggle = false;

    RobotGetData m_robotIn{};
    WeldGetData m_weldIn{};
    KukaStatus m_status{};
    TaskAck m_task{};

    QTimer* m_ioTimer = nullptr;
    mutable QMutex m_cacheMutex;
    QMutex m_busMutex;
    int32_t m_lastIoError = 0;
};

#endif