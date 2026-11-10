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

// KUKA inputs are $IN[3025]..$IN[4232], 151 bytes. Each seam pose includes
// E1..E3, which moved the welder block past 4047. Outputs stay
// $OUT[3025]..$OUT[4047] in a 128 byte image. Byte 0 is signal 3025.
// When byte 0 of the cifX process image is $IN[1] / $OUT[1], the byte
// offset is (3025 - 1) / 8 = 378.
//
// Signals do not overlap. The PC writes one motion command per transfer
// and flips KukaHeader::toggle in that same transfer. KUKA latches motion
// on the edge and echoes it in TaskAck. Wait for TaskAck::toggle to match
// before sending the next sample when every sample has to be executed.
// TaskAck::frameNo echoes KukaHeader::pointIndex.
//
// RS_CMD
//   1 realtime joint target, RJ_A1..A6 and RJ_E1..E3
//   2 joint trajectory, one planned waypoint per toggle, same RJ_* bytes
//   3 one weld seam: start, end, two tool-Y tilts, weave
//   4 execute the latched seam
//   5 clear
//
// There is no Cartesian stream. Command 2 is the host's joint plan.
// pointIndex is 1..pointTotal. cont blends every waypoint except the last
// (pointIndex >= pointTotal). pointTotal 0 leaves the stream open.
// The host fills e1..e3 on joint commands and on both seam endpoints.
// S, T and E4..E6 of a seam still come from the actual pose.
//
// Command 3 layout, bit 0 = $IN[3025]:
//   start XYZABC E1 E2 E3 at bit 384, end at bit 672, 9 floats each
//   start tilt, end tilt, speed, amplitude, frequency at bit 960
//   weave is header byte 8 (bit 64): 0 off, 1 triangle, 2 sine
// Send command 3, then command 4, with the same task id.
//
// Welder commands are WS_* at byte 140 of this image ($IN[4145], 11 bytes).
// They are not selected by RS_CMD. Every motion transfer keeps the last
// WeldSendData slice, and slotSendWeldData updates that slice alone.

constexpr uint32_t kKukaCommandBytes = 151;
constexpr uint32_t kKukaStatusBytes = 128;
constexpr uint32_t kKukaImageBytes = kKukaCommandBytes;
constexpr uint32_t kKukaCommandOffset = 378;
constexpr uint32_t kKukaStatusOffset = 378;
constexpr uint32_t kKukaWeldSliceOffset = 140;
constexpr uint32_t kKukaWeldSliceBytes = 11;

enum KukaCmd : uint8_t {
    KukaCmdNone = 0,
    KukaCmdJoint = 1,
    KukaCmdJointTraj = 2,
    KukaCmdWeldSeam = 3,
    KukaCmdWeldExecute = 4,
    KukaCmdWeldClear = 5
};

enum KukaWeave : uint8_t {
    KukaWeaveOff = 0,
    KukaWeaveTriangle = 1,
    KukaWeaveSine = 2
};

enum KukaTaskState : uint8_t {
    KukaTaskIdle = 0,
    KukaTaskReceiving = 1,
    KukaTaskReady = 2,
    KukaTaskWelding = 3,
    KukaTaskDone = 4,
    KukaTaskError = 5
};

// 1 bad seam, 2 seam missing or task id mismatch, 3 tool or base,
// 4 arc timeout, 5 aborted, 6 unknown command, 7 unused,
// 8 enable off, 9 weave type
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
    KukaErrWeave = 9
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

    float X;
    float Y;
    float Z;
    float A;
    float B;
    float C;
};
Q_DECLARE_METATYPE(RobotGetData)

struct WeldGetData
{
    bool    arc_stability;
    bool    welding_ps_ready;
    bool    weld_wire_sticking;
    bool    weld_error;
    bool    welding_processing;
    bool    weld_malfunction;
    uint8_t ErrCode;
    uint16_t monitor_gi_real_voltage;
    uint16_t monitor_gi_real_current;
    uint16_t monitor_gi_real_wire_speed;
};
Q_DECLARE_METATYPE(WeldGetData)

struct KukaStatus
{
    uint8_t statusS = 0;
    uint8_t statusT = 0;
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
    float tcpVel = 0.f;    // mm/s
    float e4 = 0.f;
    float e5 = 0.f;
    float e6 = 0.f;
};
Q_DECLARE_METATYPE(KukaStatus)

struct TaskAck
{
    uint8_t cmd = 0;
    uint8_t pageMask = 0;  // bit0 seam latched
    uint8_t taskState = 0;
    uint8_t errCode = 0;
    uint16_t taskId = 0;
    uint8_t segActive = 0;
    uint8_t pointCount = 0;
    bool toggle = false;
    bool busy = false;
    bool arcOn = false;
    uint16_t frameNo = 0;  // echoes RS_POINT_INDEX
    uint16_t progress = 0; // 0..10000
};
Q_DECLARE_METATYPE(TaskAck)

struct KukaFeedback
{
    RobotGetData robot;
    WeldGetData weld;
    KukaStatus status;
    TaskAck task;
};

// toggle and alive are owned by ProfinetComm. Values set here are replaced
// on every send. alive flips once per I/O cycle so the KUKA watchdog sees
// the PC program, not each individual command.
struct KukaHeader
{
    uint8_t cmd = KukaCmdNone;
    uint8_t toolId = 1;
    uint8_t baseId = 0;
    bool enable = false;
    bool alive = false;
    bool reset = false;
    bool goHome = false;
    bool abort = false;
    bool toggle = false;
    bool cont = false;      // approximate the stream instead of stopping
    uint16_t taskId = 0;
    uint8_t pointIndex = 0; // trajectory waypoint, 1..pointTotal
    uint8_t pointTotal = 0; // 0 = the plan has no marked end
    uint8_t weave = KukaWeaveOff;
};

struct JointCommand
{
    float a1 = 0.f, a2 = 0.f, a3 = 0.f, a4 = 0.f, a5 = 0.f, a6 = 0.f;
    float e1 = 0.f, e2 = 0.f, e3 = 0.f;
    float axisSpeed = 30.f; // percent, 1..100, also scales E1..E3
    float axisAcc = 50.f;
};

// XYZABC plus E1..E3 in the selected base and tool. S, T and E4..E6 are
// latched from the actual pose when the seam is stored.
struct WeldPoint
{
    float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
    float e1 = 0.f, e2 = 0.f, e3 = 0.f;
};

// One seam. Tilts are degrees about tool Y. 0 skips that orientation move.
// speed / amp / freq are mm/s, mm and Hz. Job, wire, voltage and dynamics
// are not in this struct: they are latched from the live WeldSendData.
struct WeldSeam
{
    WeldPoint start;
    WeldPoint end;
    float startTiltDeg = 0.f;
    float endTiltDeg = 0.f;
    float speedMmS = 10.f;
    float ampMm = 0.f;
    float freqHz = 0.f;
    uint8_t weave = KukaWeaveOff;
};

// Welder command packed at byte 140 of the 3025 image ($IN[4145]).
// The corrections are raw integers, not IEEE754. The program output on
// the controller is 4 bits, so only the low 4 bits of weld_pro_number
// reach EWM_WELD_PROGRAM.
struct WeldSendData
{
    bool weld_mode;
    bool weld_arc_start;
    bool robot_ready;
    bool gas_ready;

    bool gun_clear;
    bool test_inching;
    bool weld_pos_search;
    bool weld_wire_back;

    bool weld_machine_switch;
    bool weld_errack;
    bool extern_control_enable;
    bool weld_simulation;

    bool weld_spot_mode;
    bool weld_send_wire_mode;
    bool weld_supper_mode;
    bool weld_lift_arc;

    uint8_t weld_operator_mode;
    uint8_t weld_pro_number;
    uint8_t weld_job_number;

    uint16_t weld_wire_speed_corr;
    uint16_t weld_arc_volt_corr;
    uint16_t weld_dynamin_corr;
};
Q_DECLARE_METATYPE(WeldSendData)

struct ProfinetIoLayout
{
    uint32_t area = 0;
    uint32_t commandOffset = kKukaCommandOffset;
    uint32_t weldCmdOffset = 0; // unused; WS_* is inside the 3025 image
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
    // Writes the seam and copies seam.weave into the header byte.
    void MakeSendWeldSeam(uint8_t* send, size_t len, const KukaHeader& head, const WeldSeam& seam);
    // Writes the 11 byte WS_* slice. Bit 0 of send is WS_WELD_MODE.
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
    // Command 2. One joint waypoint of the planned trajectory.
    // head.pointIndex / pointTotal mark the last point, which is not blended.
    void slotSendJointTrajectory(const KukaHeader& head, const JointCommand& cmd);
    // Command 3. Start, end, both deflection angles and the weave.
    void slotSendWeldSeam(const KukaHeader& head, const WeldSeam& seam);
    // cmd 5 clears. Any other value executes the latched seam.
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
    uint8_t m_weldSlice[kKukaWeldSliceBytes] = {};
    bool m_imageValid = false;
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
