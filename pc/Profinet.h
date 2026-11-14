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

// KUKA block is labeled $IN/$OUT[3024..4048]. $IN[3024] is one pad bit so that
// the 128-byte process image is byte-aligned at $IN[3025] (bit 0 of byte offset
// 378). ProfinetComm packs bits relative to $IN[3025] (local bit 0 = RS_CMD).
// The PC toggles KukaHeader::toggle on each command; KUKA latches on the edge
// and echoes it in TaskAck::toggle.
//
// RS_CMD (simplified)
//   1  joint target       A1..A6, E1..E3, axis/ext speed and acceleration [%]
//   2  weld setup         two seam endpoints (XYZABC), one weave/process block,
//                         approach / crater / air speed and PTP dynamics
//   3  execute latched weld task
//   4  clear weld buffer
//   5  append one joint waypoint to the trajectory buffer (RJ_*)
//   6  run all buffered joint waypoints (RS_CONT = blend between points)
//   7  clear joint trajectory buffer
//
// Welder manual I/O: WeldSendData at $IN[2001], cifX offset 250, 40 bytes.

constexpr uint32_t kKukaImageBytes = 128;
constexpr uint32_t kKukaSignalBase = 3024;   // first signal in $config.dat block
constexpr uint32_t kKukaImageBase = 3025;    // first byte-aligned data bit (RS_CMD)
constexpr uint32_t kKukaCommandOffset = (kKukaImageBase - 1) / 8; // 378
constexpr uint32_t kKukaStatusOffset = kKukaCommandOffset;
constexpr uint32_t kKukaWeldCmdOffset = 250;
constexpr uint32_t kKukaWeldCmdBytes = 40;

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

enum KukaWeave : uint8_t {
    KukaWeaveOff = 0,
    KukaWeaveTriangle = 1,
    KukaWeaveSine = 2
};

enum KukaTaskState : uint8_t {
    KukaTaskIdle = 0,
    KukaTaskReady = 1,
    KukaTaskActive = 2,
    KukaTaskDone = 3,
    KukaTaskError = 4
};

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
};
Q_DECLARE_METATYPE(RobotGetData)

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

struct KukaStatus
{
    uint8_t actTool = 0;
    uint8_t actBase = 0;
    uint8_t overridePct = 0;
    uint8_t modeOp = 0;    // 1 T1, 2 T2, 3 AUT, 4 EX
    uint8_t proState = 0;  // 1 free, 2 reset, 3 active, 4 stop, 5 end
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
    uint16_t progress = 0; // 0..10000
    uint8_t trajPointCount = 0; // buffered waypoints after append / 0 after run
};
Q_DECLARE_METATYPE(TaskAck)

struct KukaFeedback
{
    RobotGetData robot;
    WeldGetData weld;
    KukaStatus status;
    TaskAck task;
};

struct KukaHeader
{
    uint8_t cmd = KukaCmdNone;
    uint8_t toolId = 1;
    uint8_t baseId = 0;
    bool enable = false;
    bool alive = false;   // owned by ProfinetComm I/O cycle
    bool reset = false;
    bool goHome = false;
    bool abort = false;
    bool toggle = false;  // owned by ProfinetComm on send
    bool cont = false;    // PTP approximation between trajectory waypoints (cmd 6)
    uint16_t taskId = 0;
    uint16_t frameNo = 0; // append: 1 = first point of a new path (clears buffer)
};

struct JointCommand
{
    float a1 = 0.f, a2 = 0.f, a3 = 0.f, a4 = 0.f, a5 = 0.f, a6 = 0.f;
    float e1 = 0.f, e2 = 0.f, e3 = 0.f;
    float axisSpeed = 30.f; // percent 1..100
    float axisAcc = 50.f;
    float extSpeed = 100.f;
    float extAcc = 100.f;
};

struct WeldPoint
{
    float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
};

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

struct WeldTaskParams
{
    float craterMm = 0.f;
    float approachMm = 50.f;
    float airSpeedMmS = 500.f;
    float axisSpeedPct = 30.f;
    float axisAccPct = 50.f;
    float extSpeedPct = 100.f;
    float extAccPct = 100.f;
    uint8_t samples = 0; // 0 -> 4 triangle / 8 sine
};

// Minimal manual welder commands on PLC_EWM_* ($IN[2001] block).
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

struct ProfinetIoLayout
{
    uint32_t area = 0;
    uint32_t commandOffset = kKukaCommandOffset;
    uint32_t weldCmdOffset = kKukaWeldCmdOffset;
    uint32_t statusOffset = kKukaStatusOffset;
    uint32_t timeoutMs = 10;
    int cycleMs = 50;
};

class ProfinetComm : public QObject
{
    Q_OBJECT
public:
    explicit ProfinetComm(QObject* parent = nullptr);
    ~ProfinetComm() override;

    void setIoLayout(const ProfinetIoLayout& layout);
    void releaseProfinetCard();

    void MakeSendHeader(uint8_t* send, const KukaHeader& head);
    void MakeSendJoint(uint8_t* send, size_t len, const KukaHeader& head, const JointCommand& cmd);
    void MakeSendWeldSetup(uint8_t* send, size_t len, const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2,
        const WeldProcess& proc, const WeldTaskParams& params);
    void MakeSendWeldData(uint8_t* send, size_t len, const WeldSendData& in);

    void ParseRecvRobotData(const uint8_t* recv, size_t len, RobotGetData& in);
    void ParseRecvWeldData(const uint8_t* recv, size_t len, WeldGetData& in);
    void ParseRecvImage(const uint8_t* recv, size_t len, KukaFeedback& out);

    bool ProfinetWrite(uint32_t areaNumber, uint32_t offset, uint32_t len,
        const uint8_t* buffer, uint32_t timeoutMs);
    bool ProfinetRead(uint32_t areaNumber, uint32_t offset, uint32_t len,
        uint8_t* buffer, uint32_t timeoutMs);

    RobotGetData latestRobotData() const;
    WeldGetData latestWeldData() const;
    KukaStatus latestStatus() const;
    TaskAck latestTaskAck() const;

public slots:
    void initProfinet();
    void startIoCycle(int intervalMs = 0);
    void stopIoCycle();

    void slotSendJoint(const KukaHeader& head, const JointCommand& cmd);
    void slotAppendJointTrajectory(const KukaHeader& head, const JointCommand& cmd);
    void slotRunJointTrajectory(const KukaHeader& head);
    void slotClearJointTrajectory(const KukaHeader& head);
    void slotSendWeldSetup(const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2,
        const WeldProcess& proc, const WeldTaskParams& params);
    void slotSendTaskCommand(const KukaHeader& head);
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