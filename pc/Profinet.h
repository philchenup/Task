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

// KUKA $IN/$OUT[3025..4048] is one 128 byte image in each direction.
// Byte 0 of that image is signal 3025. When byte 0 of the cifX process
// image is $IN[1] / $OUT[1], the byte offset is (3025 - 1) / 8 = 378.
//
// The PC writes one command per transfer and flips KukaHeader::toggle in
// that same transfer. KUKA latches on the edge and echoes it in TaskAck.
// Wait for TaskAck::toggle to match before sending the next sample when
// every sample has to be executed.
//
// RS_CMD
//   1 realtime joint target
//   2 joint trajectory, one waypoint per toggle, same bytes as command 1
//   3 four weld endpoints and four corner distances
//   5 one seam recipe shared by every seam, plus the task parameters
//   6 execute
//   7 clear
//
// Command 3 fills the 112 byte payload: point i is XYZABC at bit
// 128 + i * 192, and corner i is one REAL millimetre distance at bit
// 896 + i * 32. E1..E3 and S/T are not transmitted. The controller copies
// them from the actual pose, so the positioner stays put.
// KukaHeader::closed == false: seams = pointCount - 1, so 4 points are 3
// seams and the distances on the two ends are ignored.
// KukaHeader::closed == true: seams = pointCount. Four points are then 4
// head-to-tail seams, and the last seam runs from point 4 back to point 1.
// A closed chain needs at least 3 points.
//
// Command 5: one 20 byte recipe at bit 128. The controller copies it onto
// every seam. Bits 288..767 are spare. The task parameters follow at bit
// 768. Send command 3, then command 5, then command 6, with one task id.
//
// Welder commands are not in this image. WeldSendData is written to the
// PLC_EWM_* block at $IN[2001], byte offset 250, 40 bytes through $IN[2320].

constexpr uint32_t kKukaImageBytes = 128;
constexpr uint32_t kKukaCommandOffset = 378;
constexpr uint32_t kKukaStatusOffset = 378;
constexpr uint32_t kKukaWeldCmdOffset = 250;
constexpr uint32_t kKukaWeldCmdBytes = 40;

enum KukaCmd : uint8_t {
    KukaCmdNone = 0,
    KukaCmdJoint = 1,
    KukaCmdJointTraj = 2,
    KukaCmdWeldPoints = 3,
    KukaCmdWeldProcess = 5,
    KukaCmdWeldExecute = 6,
    KukaCmdWeldClear = 7
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

// 1 point count, 2 page missing, 3 tool or base, 4 arc timeout,
// 5 aborted, 6 unknown command, 7 corner does not fit, 8 enable off,
// 9 weave type
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
    uint8_t pageMask = 0;  // bit0 weld points, bit1 process
    uint8_t taskState = 0;
    uint8_t errCode = 0;
    uint16_t taskId = 0;
    uint8_t segActive = 0;
    uint8_t pointCount = 0;
    bool toggle = false;
    bool busy = false;
    bool arcOn = false;
    uint16_t frameNo = 0;
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
    bool cont = false;     // approximate the stream instead of stopping
    bool arcTrack = false; // reserved, latched with the task
    uint16_t taskId = 0;
    uint8_t pointCount = 0; // 1..4, or 3..4 when closed
    bool closed = false;    // bit 56, RS_CLOSED. 4 points then make 4 seams
    uint16_t frameNo = 0;
};

struct JointCommand
{
    float a1 = 0.f, a2 = 0.f, a3 = 0.f, a4 = 0.f, a5 = 0.f, a6 = 0.f;
    float e1 = 0.f, e2 = 0.f, e3 = 0.f;
    float axisSpeed = 30.f; // percent
    float axisAcc = 50.f;
    float extSpeed = 100.f;
    float extAcc = 100.f;
};

// XYZABC only. Four of these plus four corner distances fill the image.
// The controller latches E1..E3, S and T from the actual pose.
struct WeldPoint
{
    float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
};

struct WeldSegment
{
    float speedMmS = 0.f;
    float ampMm = 0.f;
    float freqHz = 0.f;
    uint8_t weave = KukaWeaveOff;
    uint8_t job = 0;
    uint16_t wire = 0; // raw welder integers, not IEEE754
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
    uint8_t samples = 0; // 0 -> 4 for a triangle, 8 for a sine
};

// Manual welder commands. Packed onto PLC_EWM_* at $IN[2001], not into the
// 3025 image. weld_pro_number keeps its low 4 bits, which is the width of
// PLC_EWM_WELD_PROGRAM. The corrections are raw 16 bit values.
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
    // cornerNMm is the symmetric corner distance at point N, in mm.
    // An open chain ignores the distances on the first and last used points.
    void MakeSendWeldPoints(uint8_t* send, size_t len, const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2, const WeldPoint& p3, const WeldPoint& p4,
        float corner1Mm, float corner2Mm, float corner3Mm, float corner4Mm);
    // One recipe. The controller applies it to every seam in the chain.
    void MakeSendWeldProcess(uint8_t* send, size_t len, const KukaHeader& head,
        const WeldSegment& seg, const WeldTaskParams& params);
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
    // Command 2. One joint waypoint, packed with the same layout as command 1.
    void slotSendJointTrajectory(const KukaHeader& head, const JointCommand& cmd);
    // Command 3. All four endpoints and the four corner distances at once.
    // head.pointCount is 1..4. head.closed selects 4 seams instead of 3.
    void slotSendWeldPoints(const KukaHeader& head,
        const WeldPoint& p1, const WeldPoint& p2, const WeldPoint& p3, const WeldPoint& p4,
        float corner1Mm, float corner2Mm, float corner3Mm, float corner4Mm);
    void slotSendWeldProcess(const KukaHeader& head,
        const WeldSegment& seg, const WeldTaskParams& params);
    // cmd 7 clears. Any other value executes.
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
