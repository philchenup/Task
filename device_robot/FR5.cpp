#include "FR5.h"
#include "tool/MathUtils.h"

#include <QThread>
#include <QMetaObject>
#include <QDebug>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

void zeroPose(DescPose& pose)
{
    std::memset(&pose, 0, sizeof(pose));
}

void zeroJoint(JointPos& joint)
{
    std::memset(&joint, 0, sizeof(joint));
}

void zeroExaxis(ExaxisPos& epos)
{
    std::memset(&epos, 0, sizeof(epos));
}

} // namespace

FR5::FR5(QObject* parent)
    : IRobot(parent)
    , m_feedbackTimer(new QTimer(this))
    , m_drillTimer(new QTimer(this))
{
    m_feedbackTimer->setInterval(100);
    connect(m_feedbackTimer, &QTimer::timeout, this, &FR5::onFeedbackTimerTimeout);

    connect(this, &FR5::_startFeedbackTimer, this, &FR5::startFeedbackTimer, Qt::QueuedConnection);
    connect(this, &FR5::_stopFeedbackTimer, this, &FR5::stopFeedbackTimer, Qt::QueuedConnection);

    QObject::connect(m_drillTimer, &QTimer::timeout, this, &FR5::onDrillStep);
    QObject::connect(this, &FR5::sigMoveFinished, this, &FR5::moveNextPose);
}

FR5::~FR5()
{
    m_feedbackTimer->stop();
    m_drillTimer->stop();
    if (m_connected)
        m_robot.CloseRPC();
}

bool FR5::isConnected() const { return m_connected.load(); }

bool FR5::isEnabled() const { return m_enabled.load(); }

void FR5::connectRobot(const RobotPara& para)
{
    if (m_connected) {
        emit sigStatusMessage(QStringLiteral("法奥 FR5 已连接，无需重复连接"));
        return;
    }

    const std::string ip = para.IpAddress;
    if (ip.empty()) {
        emit sigError(QStringLiteral("法奥 FR5 IP 为空"));
        emit sigConnected(false);
        return;
    }

    QThread* t = QThread::create([=]() { connectTask(ip); });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void FR5::connectTask(const std::string& ip)
{
    const int ret = m_robot.RPC(ip.c_str());
    if (ret != 0) {
        QMetaObject::invokeMethod(this, [=]() {
            emit sigError(QStringLiteral("法奥 FR5 连接失败（IP: %1）").arg(QString::fromStdString(ip)));
            emit sigConnected(false);
            }, Qt::QueuedConnection);
        return;
    }

    m_robot.Mode(0);
    m_robot.SetSpeed(static_cast<int>(m_globalSpeed));

    QMetaObject::invokeMethod(this, [=]() {
        m_connected = true;
        emit _startFeedbackTimer();
        emit sigStatusMessage(QStringLiteral("法奥 FR5 连接成功"));
        emit sigConnected(true);
        }, Qt::QueuedConnection);
}

void FR5::disconnectRobot()
{
    if (!m_connected) {
        emit sigStatusMessage(QStringLiteral("法奥 FR5 未连接"));
        return;
    }

    m_drillTimer->stop();

    QThread* t = QThread::create([=]() {
        const int ret = m_robot.CloseRPC();
        QMetaObject::invokeMethod(this, [=]() {
            m_connected = false;
            m_enabled = false;
            {
                std::lock_guard<std::mutex> lk(m_snapMutex);
                m_snap.valid = false;
            }
            emit _stopFeedbackTimer();
            if (ret == 0)
                emit sigStatusMessage(QStringLiteral("法奥 FR5 断开成功"));
            else
                emit sigError(QStringLiteral("法奥 FR5 断开时出现错误，已强制断连"));
            emit sigConnected(false);
            emit sigEnabled(false);
            }, Qt::QueuedConnection);
        });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void FR5::enableRobot(bool enable)
{
    if (!m_connected) {
        emit sigError(QStringLiteral("法奥 FR5 未连接，无法使能"));
        return;
    }

    std::thread([=]() {
        const int ret = m_robot.RobotEnable(enable ? 1 : 0);
        if (enable && ret == 0)
            m_robot.Mode(0);

        QMetaObject::invokeMethod(this, [=]() {
            if (ret == 0) {
                m_enabled = enable;
                emit sigEnabled(enable);
                emit sigStatusMessage(enable ? QStringLiteral("法奥 FR5 已使能")
                                              : QStringLiteral("法奥 FR5 已下使能"));
            } else {
                emit sigError(enable ? QStringLiteral("法奥 FR5 使能失败")
                                      : QStringLiteral("法奥 FR5 下使能失败"));
            }
            }, Qt::QueuedConnection);
        }).detach();
}

void FR5::clearError()
{
    if (!m_connected) {
        emit sigError(QStringLiteral("法奥 FR5 未连接，无法清错"));
        return;
    }

    std::thread([=]() {
        int mainCode = 0;
        int subCode = 0;
        m_robot.GetRobotErrorCode(&mainCode, &subCode);
        const int ret = m_robot.ResetAllError();
        QMetaObject::invokeMethod(this, [=]() {
            if (ret == 0) {
                emit sigStatusMessage(QStringLiteral("法奥 FR5 错误已清除（%1 / %2）")
                    .arg(mainCode).arg(subCode));
            } else {
                emit sigError(QStringLiteral("法奥 FR5 清错失败（%1 / %2）").arg(mainCode).arg(subCode));
            }
            }, Qt::QueuedConnection);
        }).detach();
}

void FR5::setSpeedFactor(int percent)
{
    if (percent < 1)
        percent = 1;
    if (percent > 100)
        percent = 100;
    m_globalSpeed = static_cast<double>(percent);
    if (m_connected)
        applySpeed();
    emit sigStatusMessage(QStringLiteral("法奥 FR5 速度已设置为 %1%").arg(percent));
}

void FR5::applySpeed()
{
    if (!m_connected)
        return;
    m_robot.SetSpeed(static_cast<int>(m_globalSpeed));
}

void FR5::resetPlace()
{
    count_grasp = 0;
    emit sigStatusMessage(QStringLiteral("法奥 FR5 抓取计数已重置"));
}

void FR5::moveJoint(const std::vector<double>& joint, const QString& tag)
{
    if (!m_connected || joint.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    const std::vector<double> jointCopy(joint.begin(), joint.begin() + 6);
    std::thread([=]() {
        JointPos jPos = toJoint(jointCopy);
        DescPose desc;
        zeroPose(desc);
        if (m_robot.GetForwardKin(&jPos, &desc) != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("法奥 FR5 关节正解失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        ExaxisPos epos;
        zeroExaxis(epos);
        DescPose offset;
        zeroPose(offset);
        const float ovl = 100.f;
        const int ret = m_robot.MoveJ(&jPos, &desc, 0, 0, 100.f, 100.f, ovl, &epos, 0.f, 0, &offset);
        if (ret != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("法奥 FR5 关节运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }
        waitArriveJointAndNotify(jointCopy, tag);
        }).detach();
}

void FR5::movePose(const std::vector<double>& pose, const QString& tag)
{
    if (!m_connected || pose.size() < 6) {
        emit sigMoveFinished(false, tag);
        return;
    }

    const std::vector<double> poseCopy(pose.begin(), pose.begin() + 6);
    std::thread([=]() {
        DescPose cart;
        zeroPose(cart);
        cart.tran.x = poseCopy[0];
        cart.tran.y = poseCopy[1];
        cart.tran.z = poseCopy[2];
        cart.rpy.rx = poseCopy[3];
        cart.rpy.ry = poseCopy[4];
        cart.rpy.rz = poseCopy[5];

        const int ret = m_robot.MoveCart(&cart, 0, 0, 100.f, 100.f, 100.f, 0.f, -1);
        if (ret != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("法奥 FR5 笛卡尔运动指令发送失败 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        waitArriveAndNotify(poseCopy, tag);
        if (tag == "calib") {
            QMetaObject::invokeMethod(this, [this]() { emit triggerCam(); }, Qt::QueuedConnection);
        }
        }).detach();
}

void FR5::startGraspSequence(const std::vector<std::vector<double>>& path, const Eigen::Affine3f& tcp)
{
    if (!m_connected || !m_enabled || isStop) {
        emit sigError("Robot not ready to move!");
        return;
    }
    if (path.size() < 1) {
        emit sigError("Path size data not ready!");
        return;
    }
    if (!loadPlaceStatus) {
        emit sigError("Place config not ready now!");
        return;
    }

    m_tcp = tcp;
    m_path = densifyJoints(path);

    const std::vector<double> waitVec = toDouble(place_config.WaitJoint);
    moveJoint(waitVec, "start");
}

void FR5::loadPlaceConfig(const std::string& path)
{
    loadPlaceStatus = false;

    std::ifstream file(path);
    if (!file.is_open()) {
        emit sigError("load place config file failed!");
        return;
    }

    json placeConfig;
    try {
        file >> placeConfig;
        place_config = placeConfig.get<utils::PlaceConfig>();
        loadPlaceStatus = true;
    } catch (const std::exception& e) {
        file.close();
        emit sigError(QString("load place config file failed, error : %1").arg(e.what()));
        return;
    }
    file.close();
}

int FR5::runTrajectoryPoint(const std::vector<std::vector<double>>& path)
{
    if (!m_connected) {
        emit sigError(QStringLiteral("法奥 FR5 未连接，无法执行伺服轨迹"));
        return -1;
    }

    const std::vector<std::vector<double>> pathCopy = path;
    std::thread([=]() {
        int ret = m_robot.ServoMoveStart();
        if (ret != 0) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigError(QStringLiteral("servo mode enable failed!"));
                emit sigMoveFinished(false, QStringLiteral("pre_grasp"));
                }, Qt::QueuedConnection);
            return;
        }

        ExaxisPos axisPos;
        zeroExaxis(axisPos);
        auto next = std::chrono::steady_clock::now();
        for (const auto& qRad : pathCopy) {
            if (isStop)
                break;
            if (qRad.size() < 6)
                break;
            JointPos jpos;
            zeroJoint(jpos);
            // Grasp path matches the JAKA servo stream (radians). ServoJ takes degrees.
            for (int i = 0; i < 6; ++i)
                jpos.jPos[i] = qRad[i] * 180.0 / M_PI;
            m_robot.ServoJ(&jpos, &axisPos, 0.f, 0.f, 0.008f, 0.f, 0.f);
            next += std::chrono::milliseconds(8);
            std::this_thread::sleep_until(next);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        m_robot.ServoMoveEnd();

        const bool ok = !isStop;
        QMetaObject::invokeMethod(this, [=]() {
            emit sigMoveFinished(ok, QStringLiteral("tgt_grasp"));
            }, Qt::QueuedConnection);
        }).detach();
    return 0;
}

void FR5::startScrewSequence(
    const std::vector<std::pair<std::vector<double>, std::vector<double>>>& pose,
    const Eigen::Affine3f& tcp,
    const std::string rotType)
{
    if (!m_connected || isStop)
        return;
    m_tcp = tcp;
    m_rotType = rotType;
    m_screw_pose = pose;
    recvRet = "";
    currentIndex = 0;
    getCurrentJoint(capJoint);

    std::vector<double> currentPose;
    if (!getEndPose(currentPose)) {
        emit sigError("Get robot end pose error!");
        return;
    }

    for (auto& screw : m_screw_pose) {
        std::vector<double> approach;
        std::vector<double> target;
        if (!quat2eulerFr5(screw.first, approach) || !quat2eulerFr5(screw.second, target)) {
            emit sigError(QStringLiteral("法奥 FR5 四元数转欧拉角失败"));
            m_screw_pose.clear();
            return;
        }
        screw.first = approach;
        screw.second = target;
    }

    QTimer::singleShot(500, this, [this]() {
        if (m_screw_pose.empty())
            return;
        movePose(m_screw_pose[currentIndex].first, "grasp_pass");
    });
}

void FR5::onDrillStep()
{
    if (!m_connected || isStop)
        return;

    if (recvRet == "ok" || recvRet == "ng") {
        recvRet = "";
        m_drillTimer->stop();
        emit sigMoveFinished(true, "1_B");
        return;
    }
    if (downSpeed <= 0.0)
        downSpeed = 1.0;
    std::vector<double> offVec;
    calcOffMatrix(m_tcp, -downSpeed, m_rotType, offVec);
    movePose(offVec, "1_D");
}

void FR5::robotStop()
{
    isStop = true;
    m_drillTimer->stop();
    if (m_connected)
        m_robot.StopMotion();
}

void FR5::robotStart()
{
    isStop = false;
    m_grasp_pose.clear();
    m_screw_pose.clear();
    currentIndex = 0;

    QMutexLocker locker(&m_mutex_recv);
    recvRet = "";
}

void FR5::getScrewStatus(const std::string& status)
{
    QMutexLocker locker(&m_mutex_recv);
    if (status == "1")
        recvRet = "ok";
    else if (status == "2")
        recvRet = "ng";
    else
        recvRet = "";
}

void FR5::moveNextPose(bool ok, const QString& tag)
{
    if (!ok)
        return;
    if (isStop)
        return;

    if (tag == "start") {
        if (m_path.size() < 10)
            return;
        runTrajectoryPoint(m_path);
        m_path.clear();
        emit sigSimulation(place_config.wrlModelPath, m_tcp,
            (count_grasp / (place_config.array.layerX * place_config.array.layerY)) * place_config.array.obj_height);
    } else if (tag == "tgt_grasp") {
        QTimer::singleShot(100, this, [=]() { emit sigGraspMachine(true, "grasp_pass"); });
    } else if (tag == "grasp_pass") {
        const std::vector<double> passVec = toDouble(place_config.PassJoint);
        assert(passVec.size() == 6);
        QTimer::singleShot(200, this, [=]() { moveJoint(passVec, "pass"); });
    } else if (tag == "pass") {
        if (!place_config.useArray) {
            const std::vector<double> placeVec = toDouble(place_config.PlaceJoint);
            assert(placeVec.size() == 6);
            moveJoint(placeVec, "grasp_place");
        } else {
            const std::vector<double> placePassVec = toDouble(place_config.PlacePassJoint);
            assert(placePassVec.size() == 6);
            moveJoint(placePassVec, "grasp_pass_place");
        }
    } else if (tag == "grasp_pass_place") {
        std::vector<double> placeVec;
        getPlaceJoint(toDouble(place_config.PlaceJoint), place_config.array, count_grasp, place_config.XFirst, placeVec);
        moveJoint(placeVec, "grasp_place");
    } else if (tag == "grasp_place") {
        sigGraspMachine(false, "grasp_wait");
        count_grasp++;
    } else if (tag == "grasp_wait") {
        const std::vector<double> waitVec = toDouble(place_config.WaitJoint);
        QTimer::singleShot(500, this, [=]() { moveJoint(waitVec, "wait"); });
    }

    if (m_screw_pose.size() < 1)
        return;
    if (tag == "1_P") {
        movePose(m_screw_pose[currentIndex].second, "1_T");
    } else if (tag == "1_T") {
        emit sigScrewMachine();
        if (downSpeed <= 0)
            downSpeed = 1.0;
        m_drillTimer->start(static_cast<int>(1000 / downSpeed));
    } else if (tag == "1_B") {
        movePose(m_screw_pose[currentIndex].first, "Next");
    } else if (tag == "Next") {
        currentIndex++;
        if (currentIndex == static_cast<int>(m_screw_pose.size())) {
            currentIndex = 0;
            m_drillTimer->stop();
            QTimer::singleShot(500, this, [this]() { moveJoint(capJoint, "Home"); });
            return;
        }
        movePose(m_screw_pose[currentIndex].first, "1_P");
    }
}

void FR5::startFeedbackTimer() { m_feedbackTimer->start(); }

void FR5::stopFeedbackTimer() { m_feedbackTimer->stop(); }

bool FR5::getEndPose(std::vector<double>& pose)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid)
        return false;
    pose = m_snap.endVec;
    return true;
}

bool FR5::getCurrentJoint(std::vector<double>& j)
{
    std::lock_guard<std::mutex> lk(m_snapMutex);
    if (!m_snap.valid)
        return false;
    j = m_snap.joint;
    return true;
}

void FR5::onFeedbackTimerTimeout()
{
    if (!m_connected)
        return;

    updateSnapshot();

    std::vector<double> joint;
    std::vector<double> pose;
    {
        std::lock_guard<std::mutex> lk(m_snapMutex);
        if (!m_snap.valid)
            return;
        joint = m_snap.joint;
        pose = m_snap.endVec;
    }
    emit sigRobotStatus(joint, pose);
}

void FR5::updateSnapshot()
{
    std::vector<double> pose;
    std::vector<double> joint;
    if (!readCurrentPose(pose) || !readCurrentJoint(joint))
        return;

    FeedbackSnapshot snap;
    snap.valid = true;
    snap.endVec = pose;
    snap.joint = joint;

    std::lock_guard<std::mutex> lk(m_snapMutex);
    m_snap = std::move(snap);
}

bool FR5::readCurrentPose(std::vector<double>& pose)
{
    DescPose tcp;
    zeroPose(tcp);
    if (m_robot.GetActualTCPPose(1, &tcp) != 0)
        return false;

    pose.resize(6);
    pose[0] = tcp.tran.x;
    pose[1] = tcp.tran.y;
    pose[2] = tcp.tran.z;
    pose[3] = tcp.rpy.rx;
    pose[4] = tcp.rpy.ry;
    pose[5] = tcp.rpy.rz;
    return true;
}

bool FR5::readCurrentJoint(std::vector<double>& joint)
{
    JointPos jv;
    zeroJoint(jv);
    if (m_robot.GetActualJointPosDegree(1, &jv) != 0)
        return false;

    joint.resize(6);
    for (int i = 0; i < 6; ++i)
        joint[i] = jv.jPos[i];
    return true;
}

void FR5::sigUpdatePose()
{
    std::vector<double> curPose;
    if (readCurrentPose(curPose))
        emit sigPose(curPose);
}

void FR5::waitArriveAndNotify(const std::vector<double>& targetPose, const QString& tag, int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    auto near = [](double a, double b, double tol = 0.1) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected && !isStop) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("法奥 FR5 笛卡尔运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<double> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid)
                cur = m_snap.endVec;
        }
        if (cur.size() == 6) {
            const bool arrived = near(cur[0], targetPose[0]) && near(cur[1], targetPose[1])
                && near(cur[2], targetPose[2]) && near(cur[3], targetPose[3])
                && near(cur[4], targetPose[4]) && near(cur[5], targetPose[5]);
            if (arrived) {
                QMetaObject::invokeMethod(this, [=]() {
                    emit sigMoveFinished(true, tag);
                    }, Qt::QueuedConnection);
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    QMetaObject::invokeMethod(this, [=]() {
        emit sigMoveFinished(false, tag);
        }, Qt::QueuedConnection);
}

void FR5::waitArriveJointAndNotify(const std::vector<double>& targetJoint, const QString& tag, int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    auto near = [](double a, double b, double tol = 0.1) {
        return std::fabs(a - b) < tol;
    };

    while (m_connected && !isStop) {
        if (std::chrono::steady_clock::now() > deadline) {
            QMetaObject::invokeMethod(this, [=]() {
                emit sigMoveFinished(false, tag);
                emit sigError(QStringLiteral("法奥 FR5 关节运动超时 [%1]").arg(tag));
                }, Qt::QueuedConnection);
            return;
        }

        std::vector<double> cur;
        {
            std::lock_guard<std::mutex> lk(m_snapMutex);
            if (m_snap.valid)
                cur = m_snap.joint;
        }
        if (cur.size() == 6) {
            bool arrived = true;
            for (int i = 0; i < 6; ++i) {
                if (!near(cur[i], targetJoint[i])) {
                    arrived = false;
                    break;
                }
            }
            if (arrived) {
                QMetaObject::invokeMethod(this, [=]() {
                    emit sigMoveFinished(true, tag);
                    }, Qt::QueuedConnection);
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    QMetaObject::invokeMethod(this, [=]() {
        emit sigMoveFinished(false, tag);
        }, Qt::QueuedConnection);
}

void FR5::calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<double>& off_vec)
{
    MathUtils mu;
    std::vector<double> endVec;
    getEndPose(endVec);
    if (endVec.size() != 6)
        return;

    Eigen::Affine3d trans = Eigen::Affine3d::Identity();
    const Eigen::Vector3d eulerAngle(endVec[3], endVec[4], endVec[5]);
    const Eigen::Vector3d eulerRad = mu.deg2radVec(eulerAngle);
    trans.linear() = mu.eulerToMatrix(eulerRad, rotType);
    trans.translation() = Eigen::Vector3d(endVec[0], endVec[1], endVec[2]);

    Eigen::Affine3d off = Eigen::Affine3d::Identity();
    off.translation() = Eigen::Vector3d(0.0, 0.0, offInter);

    const Eigen::Affine3d offDis = trans * tcp.cast<double>() * off * tcp.inverse().cast<double>();
    off_vec.resize(6);
    off_vec[0] = offDis.translation().x();
    off_vec[1] = offDis.translation().y();
    off_vec[2] = offDis.translation().z();
    off_vec[3] = endVec[3];
    off_vec[4] = endVec[4];
    off_vec[5] = endVec[5];
}

bool FR5::quat2eulerFr5(const std::vector<double>& quat_in, std::vector<double>& euler_out)
{
    if (quat_in.size() < 7)
        return false;

    Eigen::Quaterniond quat(quat_in[3], quat_in[4], quat_in[5], quat_in[6]);
    if (quat.norm() < 1e-8)
        return false;
    quat.normalize();

    const Eigen::Vector3d zyx = quat.toRotationMatrix().eulerAngles(2, 1, 0);
    euler_out.resize(6);
    euler_out[0] = quat_in[0];
    euler_out[1] = quat_in[1];
    euler_out[2] = quat_in[2];
    euler_out[3] = zyx[2] * 180.0 / M_PI;
    euler_out[4] = zyx[1] * 180.0 / M_PI;
    euler_out[5] = zyx[0] * 180.0 / M_PI;
    return true;
}

std::vector<double> FR5::toDouble(const utils::Joint& j)
{
    return std::vector<double>{ j.j1, j.j2, j.j3, j.j4, j.j5, j.j6 };
}

std::vector<double> FR5::toVector(const JointPos& jv)
{
    std::vector<double> jointVec(6);
    for (int i = 0; i < 6; ++i)
        jointVec[i] = jv.jPos[i];
    return jointVec;
}

JointPos FR5::toJoint(const std::vector<double>& joint_vec)
{
    JointPos jv;
    zeroJoint(jv);
    const int n = std::min(6, static_cast<int>(joint_vec.size()));
    for (int i = 0; i < n; ++i)
        jv.jPos[i] = joint_vec[i];
    return jv;
}

std::vector<std::vector<double>> FR5::densifyJoints(const std::vector<std::vector<double>>& joints)
{
    const double dt = 0.008;
    const double maxVel = 40.0 * M_PI / 180.0;
    const double maxAcc = 3.0;
    const double hardVel = 2.0;
    const double minDisp = 1e-6;
    std::vector<std::vector<double>> out;
    if (joints.empty())
        return out;

    out.push_back(joints.front());
    auto maxAbsDiff = [](const std::vector<double>& a, const std::vector<double>& b) {
        double m = 0.0;
        const std::size_t n = std::min(a.size(), b.size());
        for (std::size_t d = 0; d < n; ++d)
            m = std::max(m, std::fabs(b[d] - a[d]));
        return m;
    };

    for (std::size_t i = 0; i + 1 < joints.size(); ++i) {
        const std::vector<double>& q0 = joints[i];
        const std::vector<double>& q1 = joints[i + 1];
        const double s = maxAbsDiff(q0, q1);
        if (s <= minDisp)
            continue;

        const double sCruise = maxVel * maxVel / maxAcc;
        double tMin = (s >= sCruise) ? (s / maxVel + maxVel / maxAcc) : (2.0 * std::sqrt(s / maxAcc));
        int n = std::max(1, static_cast<int>(std::ceil(tMin / dt)));
        n = std::max(n, static_cast<int>(std::ceil(s / (hardVel * dt))));

        double t = 0.0;
        double v = 0.0;
        const double a = maxAcc;
        double tAcc = 0.0;
        double tFlat = 0.0;
        for (int guard = 0; guard < 16; ++guard) {
            t = static_cast<double>(n) * dt;
            const double disc = a * a * t * t - 4.0 * a * s;
            if (disc < 0.0) {
                ++n;
                continue;
            }
            v = 0.5 * (a * t - std::sqrt(disc));
            tAcc = v / a;
            tFlat = t - 2.0 * tAcc;
            if (tFlat < 0.0) {
                tFlat = 0.0;
                tAcc = t / 2.0;
                v = a * tAcc;
            }
            if (v > maxVel * 1.000001) {
                ++n;
                continue;
            }
            break;
        }

        auto pos = [&](double time) {
            if (time <= 0.0)
                return 0.0;
            if (time >= t)
                return s;
            if (time < tAcc)
                return 0.5 * a * time * time;
            if (time < tAcc + tFlat)
                return 0.5 * a * tAcc * tAcc + v * (time - tAcc);
            const double tDec = time - tAcc - tFlat;
            return 0.5 * a * tAcc * tAcc + v * tFlat + v * tDec - 0.5 * a * tDec * tDec;
        };

        for (int k = 1; k <= n; ++k) {
            const double u = std::min(1.0, std::max(0.0, pos(k * dt) / s));
            std::vector<double> q(q0.size());
            for (std::size_t d = 0; d < q0.size(); ++d) {
                const double q1d = d < q1.size() ? q1[d] : q0[d];
                q[d] = q0[d] + u * (q1d - q0[d]);
            }
            out.push_back(q);
        }
    }
    return out;
}

int FR5::getPlaceJoint(const std::vector<double>& cur_place_joint, const utils::ArrayConfig& cfg,
    int n, bool x_first, std::vector<double>& new_place_joint)
{
    if (cur_place_joint.size() != 6 || cfg.layerX <= 0 || cfg.layerY <= 0)
        return -1;

    JointPos cur = toJoint(cur_place_joint);
    const int perLayer = cfg.layerX * cfg.layerY;
    const int iz = n / perLayer;
    const int idxInLayer = n % perLayer;
    int ix = 0;
    int iy = 0;
    if (x_first) {
        ix = idxInLayer % cfg.layerX;
        iy = idxInLayer / cfg.layerX;
    } else {
        iy = idxInLayer % cfg.layerY;
        ix = idxInLayer / cfg.layerY;
    }

    DescPose pose0;
    zeroPose(pose0);
    int ret = m_robot.GetForwardKin(&cur, &pose0);
    if (ret != 0)
        return ret;

    DescPose poseN = pose0;
    poseN.tran.x += static_cast<double>(ix) * cfg.obj_length;
    poseN.tran.y += static_cast<double>(iy) * cfg.obj_width;
    poseN.tran.z += static_cast<double>(iz) * cfg.obj_height;

    JointPos ref = cur;
    JointPos solved;
    zeroJoint(solved);
    ret = m_robot.GetInverseKinRef(0, &poseN, &ref, &solved);
    if (ret == 0)
        new_place_joint = toVector(solved);
    return ret;
}
