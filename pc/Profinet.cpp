#include "Profinet.h"

#include <QByteArray>
#include <QMetaObject>
#include <QMutexLocker>
#include <QThread>
#include <QTimer>

#include <cstdint>
#include <cstring>

namespace {
    constexpr size_t kBitAlive = 25;

    void setBit(uint8_t* buf, size_t bitIndex, bool value)
    {
        if (value)
            buf[bitIndex / 8] |= static_cast<uint8_t>(1u << (bitIndex % 8));
        else
            buf[bitIndex / 8] &= static_cast<uint8_t>(~(1u << (bitIndex % 8)));
    }
    void setBits(uint8_t* buf, size_t startBit, size_t bitCount, uint32_t value)
    {
        for (size_t i = 0; i < bitCount; ++i)
            setBit(buf, startBit + i, ((value >> i) & 0x01u) != 0);
    }
    bool getBit(const uint8_t* buf, size_t bitIndex)
    {
        return ((buf[bitIndex / 8] >> (bitIndex % 8)) & 0x01u) != 0;
    }
    uint32_t getBits(const uint8_t* buf, size_t startBit, size_t bitCount)
    {
        uint32_t value = 0;
        for (size_t i = 0; i < bitCount; ++i)
        {
            if (getBit(buf, startBit + i))
                value |= (1u << i);
        }
        return value;
    }
    uint32_t floatToU32(float value)
    {
        uint32_t raw = 0;
        std::memcpy(&raw, &value, sizeof(raw));
        return raw;
    }
    float u32ToFloat(uint32_t raw)
    {
        float value = 0.f;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }

    void writeXyzAbc(uint8_t* send, size_t bit, const WeldPoint& p)
    {
        setBits(send, bit + 0, 32, floatToU32(p.x));
        setBits(send, bit + 32, 32, floatToU32(p.y));
        setBits(send, bit + 64, 32, floatToU32(p.z));
        setBits(send, bit + 96, 32, floatToU32(p.a));
        setBits(send, bit + 128, 32, floatToU32(p.b));
        setBits(send, bit + 160, 32, floatToU32(p.c));
    }
    void writeSegment(uint8_t* send, size_t bit, const WeldSegment& s)
    {
        setBits(send, bit + 0, 32, floatToU32(s.speedMmS));
        setBits(send, bit + 32, 32, floatToU32(s.ampMm));
        setBits(send, bit + 64, 32, floatToU32(s.freqHz));
        setBits(send, bit + 96, 8, s.weave);
        setBits(send, bit + 104, 8, s.job);
        setBits(send, bit + 112, 16, s.wire);
        setBits(send, bit + 128, 16, s.volt);
        setBits(send, bit + 144, 16, s.dyn);
    }
}

ProfinetComm::ProfinetComm(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<RobotGetData>("RobotGetData");
    qRegisterMetaType<WeldGetData>("WeldGetData");
    qRegisterMetaType<KukaStatus>("KukaStatus");
    qRegisterMetaType<TaskAck>("TaskAck");
    qRegisterMetaType<WeldSendData>("WeldSendData");
}

ProfinetComm::~ProfinetComm()
{
    if (m_ioTimer)
        m_ioTimer->stop();
    closeChannel();
}

void ProfinetComm::setIoLayout(const ProfinetIoLayout& layout)
{
    QMutexLocker lock(&m_cacheMutex);
    m_layout = layout;
}

void ProfinetComm::initProfinet()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this]() { initProfinet(); }, Qt::QueuedConnection);
        return;
    }

    bool already = false;
    QByteArray baBoard;
    uint32_t channel = 0;
    {
        QMutexLocker lock(&m_busMutex);
        already = profinetInited;
        baBoard = szBoardname.toLocal8Bit();
        channel = m_ulChannel;
    }
    if (already) {
        emit sigInfoMsg("ProfinetCard already initialized!");
        return;
    }

    int32_t lRet = xDriverOpen(&m_hDriver);
    if (CIFX_NO_ERROR != lRet) {
        emit sigErrMsg("ProfinetCard xDriverOpen failed!");
        m_hDriver = nullptr;
        return;
    }

    lRet = xChannelOpen(m_hDriver, baBoard.data(), channel, &m_hChannel);
    if (CIFX_NO_ERROR != lRet) {
        emit sigErrMsg("ProfinetCard xChannelOpen failed!");
        xDriverClose(m_hDriver);
        m_hDriver = nullptr;
        m_hChannel = nullptr;
        return;
    }

    CHANNEL_INFORMATION tInfo = { 0 };
    if (CIFX_NO_ERROR == xChannelInfo(m_hChannel, sizeof(tInfo), &tInfo))
    {
        QString msg = QString("[cifX] Handware: %1  Version: %2.%3.%4 build %5")
            .arg(QString::fromLatin1(reinterpret_cast<const char*>(tInfo.abFWName)))
            .arg(tInfo.usFWMajor)
            .arg(tInfo.usFWMinor)
            .arg(tInfo.usFWBuild)
            .arg(tInfo.usFWRevision);
        emit sigInfoMsg(msg);
    }

    uint32_t ulState = 0;
    lRet = xChannelHostState(m_hChannel, CIFX_HOST_STATE_READY, &ulState, 5000);
    if (CIFX_NO_ERROR != lRet) {
        emit sigErrMsg("ProfinetCard xChannelHostState(READY) failed!");
        closeChannel();
        return;
    }

    lRet = xChannelBusState(m_hChannel, CIFX_BUS_STATE_ON, &ulState, 0);
    if (CIFX_NO_ERROR != lRet) {
        emit sigErrMsg("ProfinetCard xChannelBusState(ON) failed!");
        closeChannel();
        return;
    }

    int cycleMs = 50;
    {
        QMutexLocker lock(&m_busMutex);
        profinetInited = true;
        emit sigConnected(profinetInited);
    }
    {
        QMutexLocker lock(&m_cacheMutex);
        cycleMs = m_layout.cycleMs;
    }
    startIoCycle(cycleMs);
    emit sigInfoMsg("ProfinetCard initialized done!");
}

void ProfinetComm::releaseProfinetCard()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this]() { releaseProfinetCard(); }, Qt::QueuedConnection);
        return;
    }
    stopIoCycle();
    closeChannel();
}

void ProfinetComm::closeChannel()
{
    QString channelLog;
    QString driverLog;
    {
        QMutexLocker lock(&m_busMutex);
        uint32_t ulState = 0;
        if (m_hChannel) {
            xChannelBusState(m_hChannel, CIFX_BUS_STATE_OFF, &ulState, 0);
            xChannelHostState(m_hChannel, CIFX_HOST_STATE_NOT_READY, &ulState, 5000);
            xChannelClose(m_hChannel);
            m_hChannel = nullptr;
            channelLog = QStringLiteral("ProfinetCard cifX channel closed!");
        }
        if (m_hDriver) {
            xDriverClose(m_hDriver);
            m_hDriver = nullptr;
            driverLog = QStringLiteral("ProfinetCard cifX driver closed!");
        }
        profinetInited = false;
        emit sigConnected(profinetInited);
    }
    if (!channelLog.isEmpty())
        emit sigInfoMsg(channelLog);
    if (!driverLog.isEmpty())
        emit sigInfoMsg(driverLog);
}

void ProfinetComm::startIoCycle(int intervalMs)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, intervalMs]() { startIoCycle(intervalMs); }, Qt::QueuedConnection);
        return;
    }

    if (intervalMs <= 0) {
        QMutexLocker lock(&m_cacheMutex);
        intervalMs = m_layout.cycleMs;
    }
    if (intervalMs < 1)
        intervalMs = 1;

    if (!m_ioTimer) {
        m_ioTimer = new QTimer(this);
        m_ioTimer->setTimerType(Qt::PreciseTimer);
        connect(m_ioTimer, &QTimer::timeout, this, &ProfinetComm::onIoCycle);
    }
    m_ioTimer->start(intervalMs);
}

void ProfinetComm::stopIoCycle()
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this]() { stopIoCycle(); }, Qt::QueuedConnection);
        return;
    }
    if (m_ioTimer)
        m_ioTimer->stop();
}

void ProfinetComm::onIoCycle()
{
    {
        QMutexLocker lock(&m_cacheMutex);
        if (m_imageValid) {
            m_alive = !m_alive;
            setBit(m_image, kBitAlive, m_alive);
        }
    }
    writeCachedOutputs();
    readInputs();
}

KukaHeader ProfinetComm::prepare(KukaHeader head)
{
    head.toggle = !m_lastToggle;
    m_lastToggle = head.toggle;
    head.alive = m_alive;
    return head;
}

void ProfinetComm::commitImage(const uint8_t* image)
{
    {
        QMutexLocker lock(&m_cacheMutex);
        std::memcpy(m_image, image, kKukaImageBytes);
        m_imageValid = true;
    }
    writeCachedOutputs();
}

void ProfinetComm::slotSendJoint(const KukaHeader& head, const JointCommand& cmd)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, head, cmd]() { slotSendJoint(head, cmd); }, Qt::QueuedConnection);
        return;
    }
    KukaHeader h = prepare(head);
    h.cmd = KukaCmdJoint;
    uint8_t buf[kKukaImageBytes] = {};
    MakeSendJoint(buf, sizeof(buf), h, cmd);
    commitImage(buf);
}

void ProfinetComm::slotSendJointTrajectory(const KukaHeader& head, const JointCommand& cmd)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, head, cmd]() {
            slotSendJointTrajectory(head, cmd);
        }, Qt::QueuedConnection);
        return;
    }
    KukaHeader h = prepare(head);
    h.cmd = KukaCmdJointTraj;
    uint8_t buf[kKukaImageBytes] = {};
    MakeSendJoint(buf, sizeof(buf), h, cmd);
    commitImage(buf);
}

void ProfinetComm::slotSendWeldPoints(const KukaHeader& head,
    const WeldPoint& p1, const WeldPoint& p2, const WeldPoint& p3, const WeldPoint& p4,
    float corner1Mm, float corner2Mm, float corner3Mm, float corner4Mm)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, head, p1, p2, p3, p4,
            corner1Mm, corner2Mm, corner3Mm, corner4Mm]() {
            slotSendWeldPoints(head, p1, p2, p3, p4, corner1Mm, corner2Mm, corner3Mm, corner4Mm);
        }, Qt::QueuedConnection);
        return;
    }
    KukaHeader h = prepare(head);
    h.cmd = KukaCmdWeldPoints;
    uint8_t buf[kKukaImageBytes] = {};
    MakeSendWeldPoints(buf, sizeof(buf), h, p1, p2, p3, p4,
        corner1Mm, corner2Mm, corner3Mm, corner4Mm);
    commitImage(buf);
}

void ProfinetComm::slotSendWeldProcess(const KukaHeader& head,
    const WeldSegment& seg, const WeldTaskParams& params)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, head, seg, params]() {
            slotSendWeldProcess(head, seg, params);
        }, Qt::QueuedConnection);
        return;
    }
    KukaHeader h = prepare(head);
    h.cmd = KukaCmdWeldProcess;
    uint8_t buf[kKukaImageBytes] = {};
    MakeSendWeldProcess(buf, sizeof(buf), h, seg, params);
    commitImage(buf);
}

void ProfinetComm::slotSendTaskCommand(const KukaHeader& head)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, head]() { slotSendTaskCommand(head); }, Qt::QueuedConnection);
        return;
    }
    KukaHeader h = prepare(head);
    if (h.cmd != KukaCmdWeldClear)
        h.cmd = KukaCmdWeldExecute;
    uint8_t buf[kKukaImageBytes] = {};
    MakeSendHeader(buf, h);
    commitImage(buf);
}

void ProfinetComm::slotSendWeldData(const WeldSendData& data)
{
    if (QThread::currentThread() != thread()) {
        QMetaObject::invokeMethod(this, [this, data]() { slotSendWeldData(data); }, Qt::QueuedConnection);
        return;
    }
    uint8_t buf[kKukaWeldCmdBytes] = {};
    MakeSendWeldData(buf, sizeof(buf), data);
    {
        QMutexLocker lock(&m_cacheMutex);
        std::memcpy(m_weldImage, buf, kKukaWeldCmdBytes);
        m_weldValid = true;
    }
    writeCachedOutputs();
}

void ProfinetComm::writeCachedOutputs()
{
    uint8_t image[kKukaImageBytes];
    uint8_t weld[kKukaWeldCmdBytes];
    bool hasImage = false;
    bool hasWeld = false;
    uint32_t area = 0;
    uint32_t commandOff = 0;
    uint32_t weldOff = 0;
    uint32_t timeoutMs = 10;
    {
        QMutexLocker lock(&m_cacheMutex);
        hasImage = m_imageValid;
        hasWeld = m_weldValid;
        if (hasImage)
            std::memcpy(image, m_image, kKukaImageBytes);
        if (hasWeld)
            std::memcpy(weld, m_weldImage, kKukaWeldCmdBytes);
        area = m_layout.area;
        commandOff = m_layout.commandOffset;
        weldOff = m_layout.weldCmdOffset;
        timeoutMs = m_layout.timeoutMs;
    }

    if (hasImage)
        cifxTransfer(true, area, commandOff, kKukaImageBytes, image, timeoutMs, true);
    if (hasWeld)
        cifxTransfer(true, area, weldOff, kKukaWeldCmdBytes, weld, timeoutMs, true);
}

void ProfinetComm::readInputs()
{
    uint32_t area = 0;
    uint32_t statusOff = 0;
    uint32_t timeoutMs = 10;
    {
        QMutexLocker lock(&m_cacheMutex);
        area = m_layout.area;
        statusOff = m_layout.statusOffset;
        timeoutMs = m_layout.timeoutMs;
    }

    uint8_t buf[kKukaImageBytes] = {};
    if (!cifxTransfer(false, area, statusOff, kKukaImageBytes, buf, timeoutMs, true))
        return;

    KukaFeedback feedback{};
    ParseRecvImage(buf, sizeof(buf), feedback);
    {
        QMutexLocker lock(&m_cacheMutex);
        m_robotIn = feedback.robot;
        m_weldIn = feedback.weld;
        m_status = feedback.status;
        m_task = feedback.task;
    }
    emit sigRobotData(feedback.robot);
    emit sigWeldData(feedback.weld);
    emit sigKukaStatus(feedback.status);
    emit sigTaskAck(feedback.task);
}

RobotGetData ProfinetComm::latestRobotData() const
{
    QMutexLocker lock(&m_cacheMutex);
    return m_robotIn;
}

WeldGetData ProfinetComm::latestWeldData() const
{
    QMutexLocker lock(&m_cacheMutex);
    return m_weldIn;
}

KukaStatus ProfinetComm::latestStatus() const
{
    QMutexLocker lock(&m_cacheMutex);
    return m_status;
}

TaskAck ProfinetComm::latestTaskAck() const
{
    QMutexLocker lock(&m_cacheMutex);
    return m_task;
}

bool ProfinetComm::cifxTransfer(bool write,
    uint32_t area,
    uint32_t offset,
    uint32_t len,
    uint8_t* buffer,
    uint32_t timeoutMs,
    bool latchError)
{
    if (!buffer || len == 0) {
        if (!latchError)
            emit sigErrMsg(write ? QStringLiteral("ProfinetWrite invalid parameter!")
                : QStringLiteral("ProfinetRead invalid parameter!"));
        return false;
    }

    int32_t ret = CIFX_NO_ERROR;
    bool ready = false;
    {
        QMutexLocker lock(&m_busMutex);
        ready = profinetInited && m_hChannel;
        if (ready) {
            ret = write
                ? xChannelIOWrite(m_hChannel, area, offset, len, buffer, timeoutMs)
                : xChannelIORead(m_hChannel, area, offset, len, buffer, timeoutMs);
        }
    }
    if (!ready) {
        if (!latchError)
            emit sigErrMsg(QStringLiteral("ProfinetCard not initialized!"));
        return false;
    }
    if (ret != CIFX_NO_ERROR) {
        const bool report = !latchError || ret != m_lastIoError;
        if (latchError)
            m_lastIoError = ret;
        if (report) {
            emit sigErrMsg(QString("xChannelIO%1 failed, err=0x%2 area=%3 off=%4 len=%5")
                .arg(write ? QStringLiteral("Write") : QStringLiteral("Read"))
                .arg(static_cast<uint32_t>(ret), 8, 16, QChar('0'))
                .arg(area)
                .arg(offset)
                .arg(len));
        }
        return false;
    }
    if (latchError && m_lastIoError != CIFX_NO_ERROR) {
        m_lastIoError = CIFX_NO_ERROR;
        emit sigInfoMsg(write ? QStringLiteral("Profinet write recovered")
            : QStringLiteral("Profinet read recovered"));
    }
    return true;
}

void ProfinetComm::MakeSendHeader(uint8_t* send, const KukaHeader& head)
{
    if (!send)
        return;
    setBits(send, 0, 8, head.cmd);
    setBits(send, 8, 8, head.toolId);
    setBits(send, 16, 8, head.baseId);
    setBit(send, 24, head.enable);
    setBit(send, 25, head.alive);
    setBit(send, 26, head.reset);
    setBit(send, 27, head.goHome);
    setBit(send, 28, head.abort);
    setBit(send, 29, head.toggle);
    setBit(send, 30, head.cont);
    setBit(send, 31, head.arcTrack);
    setBits(send, 32, 16, head.taskId);
    setBits(send, 48, 8, head.pointCount);
    setBit(send, 56, head.closed);
    setBits(send, 64, 16, head.frameNo);
}

void ProfinetComm::MakeSendJoint(uint8_t* send, size_t len, const KukaHeader& head, const JointCommand& cmd)
{
    if (!send || len < kKukaImageBytes)
        return;
    std::memset(send, 0, len);
    MakeSendHeader(send, head);
    setBits(send, 128, 32, floatToU32(cmd.a1));
    setBits(send, 160, 32, floatToU32(cmd.a2));
    setBits(send, 192, 32, floatToU32(cmd.a3));
    setBits(send, 224, 32, floatToU32(cmd.a4));
    setBits(send, 256, 32, floatToU32(cmd.a5));
    setBits(send, 288, 32, floatToU32(cmd.a6));
    setBits(send, 320, 32, floatToU32(cmd.e1));
    setBits(send, 352, 32, floatToU32(cmd.e2));
    setBits(send, 384, 32, floatToU32(cmd.e3));
    setBits(send, 416, 32, floatToU32(cmd.axisSpeed));
    setBits(send, 448, 32, floatToU32(cmd.axisAcc));
    setBits(send, 480, 32, floatToU32(cmd.extSpeed));
    setBits(send, 512, 32, floatToU32(cmd.extAcc));
}

void ProfinetComm::MakeSendWeldPoints(uint8_t* send, size_t len, const KukaHeader& head,
    const WeldPoint& p1, const WeldPoint& p2, const WeldPoint& p3, const WeldPoint& p4,
    float corner1Mm, float corner2Mm, float corner3Mm, float corner4Mm)
{
    if (!send || len < kKukaImageBytes)
        return;
    std::memset(send, 0, len);
    MakeSendHeader(send, head);
    writeXyzAbc(send, 128, p1);
    writeXyzAbc(send, 320, p2);
    writeXyzAbc(send, 512, p3);
    writeXyzAbc(send, 704, p4);
    setBits(send, 896, 32, floatToU32(corner1Mm));
    setBits(send, 928, 32, floatToU32(corner2Mm));
    setBits(send, 960, 32, floatToU32(corner3Mm));
    setBits(send, 992, 32, floatToU32(corner4Mm));
}

void ProfinetComm::MakeSendWeldProcess(uint8_t* send, size_t len, const KukaHeader& head,
    const WeldSegment& seg, const WeldTaskParams& params)
{
    if (!send || len < kKukaImageBytes)
        return;
    std::memset(send, 0, len);
    MakeSendHeader(send, head);
    writeSegment(send, 128, seg);
    setBits(send, 768, 32, floatToU32(params.craterMm));
    setBits(send, 800, 32, floatToU32(params.approachMm));
    setBits(send, 832, 32, floatToU32(params.airSpeedMmS));
    setBits(send, 864, 32, floatToU32(params.axisSpeedPct));
    setBits(send, 896, 32, floatToU32(params.axisAccPct));
    setBits(send, 928, 32, floatToU32(params.extSpeedPct));
    setBits(send, 960, 32, floatToU32(params.extAccPct));
    setBits(send, 992, 8, params.samples);
}

void ProfinetComm::MakeSendWeldData(uint8_t* send, size_t len, const WeldSendData& in)
{
    // Bits are relative to $IN[2001]. The extension at $IN[2273] is bit 272.
    if (!send || len < kKukaWeldCmdBytes)
        return;
    std::memset(send, 0, len);
    setBit(send, 0, (in.weld_operator_mode & 0x01u) != 0);
    setBit(send, 1, (in.weld_operator_mode & 0x02u) != 0);
    setBit(send, 2, in.weld_mode);
    setBit(send, 3, in.weld_machine_switch);
    setBit(send, 4, in.weld_arc_start);
    setBit(send, 5, in.robot_ready);
    setBit(send, 6, in.gas_ready);
    setBit(send, 7, in.test_inching);
    setBit(send, 8, in.weld_pos_search);
    setBit(send, 9, in.weld_wire_back);
    setBit(send, 10, in.extern_control_enable);
    setBit(send, 11, in.weld_simulation);
    setBits(send, 12, 4, in.weld_pro_number & 0x0Fu);
    setBits(send, 16, 8, in.weld_job_number);
    setBits(send, 24, 16, in.weld_wire_speed_corr);
    setBit(send, 272, in.gun_clear);
    setBit(send, 273, in.weld_errack);
    setBit(send, 274, in.weld_spot_mode);
    setBit(send, 275, in.weld_send_wire_mode);
    setBit(send, 276, in.weld_supper_mode);
    setBit(send, 277, in.weld_lift_arc);
    setBits(send, 280, 8, in.weld_operator_mode);
    setBits(send, 288, 16, in.weld_arc_volt_corr);
    setBits(send, 304, 16, in.weld_dynamin_corr);
}

void ProfinetComm::ParseRecvWeldData(const uint8_t* recv, size_t len, WeldGetData& in)
{
    if (!recv || len < 8)
        return;
    in.arc_stability = getBit(recv, 0);
    in.welding_ps_ready = getBit(recv, 1);
    in.weld_wire_sticking = getBit(recv, 2);
    in.weld_error = getBit(recv, 3);
    in.welding_processing = getBit(recv, 4);
    in.weld_malfunction = getBit(recv, 5);
    in.ErrCode = static_cast<uint8_t>(getBits(recv, 6, 8));
    in.monitor_gi_real_voltage = static_cast<uint16_t>(getBits(recv, 14, 16));
    in.monitor_gi_real_current = static_cast<uint16_t>(getBits(recv, 30, 16));
    in.monitor_gi_real_wire_speed = static_cast<uint16_t>(getBits(recv, 46, 16));
}

void ProfinetComm::ParseRecvRobotData(const uint8_t* recv, size_t len, RobotGetData& in)
{
    if (!recv || len < 76)
        return;
    in.axis_vel = u32ToFloat(getBits(recv, 0, 32));
    in.axis_acc = u32ToFloat(getBits(recv, 32, 32));
    in.ext_axis_vel = u32ToFloat(getBits(recv, 64, 32));
    in.ext_axis_acc = u32ToFloat(getBits(recv, 96, 32));
    in.E1 = u32ToFloat(getBits(recv, 128, 32));
    in.E2 = u32ToFloat(getBits(recv, 160, 32));
    in.E3 = u32ToFloat(getBits(recv, 192, 32));
    in.SJ1 = u32ToFloat(getBits(recv, 224, 32));
    in.SJ2 = u32ToFloat(getBits(recv, 256, 32));
    in.SJ3 = u32ToFloat(getBits(recv, 288, 32));
    in.SJ4 = u32ToFloat(getBits(recv, 320, 32));
    in.SJ5 = u32ToFloat(getBits(recv, 352, 32));
    in.SJ6 = u32ToFloat(getBits(recv, 384, 32));
    in.X = u32ToFloat(getBits(recv, 416, 32));
    in.Y = u32ToFloat(getBits(recv, 448, 32));
    in.Z = u32ToFloat(getBits(recv, 480, 32));
    in.A = u32ToFloat(getBits(recv, 512, 32));
    in.B = u32ToFloat(getBits(recv, 544, 32));
    in.C = u32ToFloat(getBits(recv, 576, 32));
}

void ProfinetComm::ParseRecvImage(const uint8_t* recv, size_t len, KukaFeedback& out)
{
    if (!recv || len < kKukaImageBytes)
        return;
    ParseRecvRobotData(recv, len, out.robot);
    ParseRecvWeldData(recv + 76, len - 76, out.weld);

    out.status.statusS = static_cast<uint8_t>(getBits(recv, 672, 8));
    out.status.statusT = static_cast<uint8_t>(getBits(recv, 680, 8));
    out.status.actTool = static_cast<uint8_t>(getBits(recv, 688, 8));
    out.status.actBase = static_cast<uint8_t>(getBits(recv, 696, 8));
    out.status.overridePct = static_cast<uint8_t>(getBits(recv, 704, 8));
    out.status.modeOp = static_cast<uint8_t>(getBits(recv, 712, 8));
    out.status.proState = static_cast<uint8_t>(getBits(recv, 720, 8));
    out.status.drivesOn = getBit(recv, 728);
    out.status.moveEnable = getBit(recv, 729);
    out.status.stopMess = getBit(recv, 730);
    out.status.robotReady = getBit(recv, 731);
    out.status.pcLinkOk = getBit(recv, 732);
    out.status.appErr = static_cast<uint16_t>(getBits(recv, 736, 16));
    out.status.heartbeat = static_cast<uint16_t>(getBits(recv, 752, 16));
    out.status.tcpVel = u32ToFloat(getBits(recv, 768, 32));
    out.status.e4 = u32ToFloat(getBits(recv, 800, 32));
    out.status.e5 = u32ToFloat(getBits(recv, 832, 32));
    out.status.e6 = u32ToFloat(getBits(recv, 864, 32));

    out.task.cmd = static_cast<uint8_t>(getBits(recv, 896, 8));
    out.task.pageMask = static_cast<uint8_t>(getBits(recv, 904, 8));
    out.task.taskState = static_cast<uint8_t>(getBits(recv, 912, 8));
    out.task.errCode = static_cast<uint8_t>(getBits(recv, 920, 8));
    out.task.taskId = static_cast<uint16_t>(getBits(recv, 928, 16));
    out.task.segActive = static_cast<uint8_t>(getBits(recv, 944, 8));
    out.task.pointCount = static_cast<uint8_t>(getBits(recv, 952, 8));
    out.task.toggle = getBit(recv, 968);
    out.task.busy = getBit(recv, 969);
    out.task.arcOn = getBit(recv, 970);
    out.task.frameNo = static_cast<uint16_t>(getBits(recv, 976, 16));
    out.task.progress = static_cast<uint16_t>(getBits(recv, 992, 16));
}

bool ProfinetComm::ProfinetRead(uint32_t areaNumber,
    uint32_t offset,
    uint32_t len,
    uint8_t* buffer,
    uint32_t timeoutMs)
{
    return cifxTransfer(false, areaNumber, offset, len, buffer, timeoutMs, false);
}

bool ProfinetComm::ProfinetWrite(uint32_t areaNumber,
    uint32_t offset,
    uint32_t len,
    const uint8_t* buffer,
    uint32_t timeoutMs)
{
    return cifxTransfer(true, areaNumber, offset, len,
        const_cast<uint8_t*>(buffer), timeoutMs, false);
}
