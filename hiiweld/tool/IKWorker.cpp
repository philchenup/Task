/**
 * @file IKWorker.cpp
 * @brief 后台线程逆解、回零与起始点路径规划。
 */
#include "IKWorker.h"
// #include "MainWindow.h"  // 归档：原工程主窗口

#include <QElapsedTimer>
#include <QMutexLocker>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/math/Unit.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/SimpleModel.h>

/** @brief 注册跨线程信号用的元类型。 */
IKWorker::IKWorker(QObject* parent) : QObject(parent)
{
    qRegisterMetaType<rl::math::Vector>("rl::math::Vector");
    qRegisterMetaType<std::vector<rl::math::Vector>>("std::vector<rl::math::Vector>");
    qRegisterMetaType<std::vector<rl::math::Vector3>>("std::vector<rl::math::Vector3>");
    qRegisterMetaType<IKSolveParams>("IKSolveParams");
    qRegisterMetaType<IKGoToStartParams>("IKGoToStartParams");
}

IKWorker::~IKWorker() = default;

/** @brief 在非运行态设置共享运动学模型。 */
void IKWorker::setKinematic(const std::shared_ptr<rl::mdl::Dynamic>& mdl)
{
    QMutexLocker locker(&m_kinMutex);
    if (m_running.load())
    {
        return;
    }
    m_kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(mdl);
}

/** @brief 请求中断当前规划/IK。 */
void IKWorker::requestStop()
{
    m_stopRequested.store(true);
}

/** @brief 构造正交工具坐标系。 */
rl::math::Transform IKWorker::pointToTransform(const DiscretePoint& pt)
{
    rl::math::Transform T = rl::math::Transform::Identity();

    T.translation() = rl::math::Vector3(pt.position.X(), pt.position.Y(), pt.position.Z());

    rl::math::Matrix33 R;
    R.col(0) = rl::math::Vector3(pt.xDir.X(), pt.xDir.Y(), pt.xDir.Z());
    R.col(1) = rl::math::Vector3(pt.yDir.X(), pt.yDir.Y(), pt.yDir.Z());
    R.col(2) = rl::math::Vector3(pt.zDir.X(), pt.zDir.Y(), pt.zDir.Z());

    R.col(0).normalize();
    R.col(2) = R.col(2).normalized();
    R.col(1) = R.col(2).cross(R.col(0)).normalized();
    R.col(2) = R.col(0).cross(R.col(1)).normalized();

    T.linear() = R;
    return T;
}

/** @brief 离散点列逆解，发射 progress/finished/failed。 */
void IKWorker::doSolve(const IKSolveParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    try
    {
        if (!kinematic)
            throw std::runtime_error("Kinematic not set. Call setKinematic() before doSolve().");

        const std::size_t dof = kinematic->getDof();
        const int totalPoints = static_cast<int>(params.trajectory.size());

        if (params.q_initial.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("Initial joint vector size mismatch with model DOF.");

        if (totalPoints == 0)
            throw std::runtime_error("Trajectory is empty.");

        if (params.railWindow <= 0.0)
            throw std::runtime_error("railWindow must be positive for interpolation.");

        emit started();

        const rl::math::Transform T_flange_to_tcp = params.T_flange_to_tcp;
        const rl::math::Transform T_tcp_to_flange = T_flange_to_tcp.inverse();
        const double railWindow = params.railWindow;

        rl::mdl::Joint* railJoint = (params.constrainRail && dof >= 1) ? kinematic->getJoint(0) : nullptr;
        rl::math::Vector railMin0, railMax0;
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }

        std::vector<rl::math::Vector> jointTrajectory;
        jointTrajectory.reserve(static_cast<std::size_t>(totalPoints) * 2);

        rl::math::Vector q_current = params.q_initial;
        kinematic->setPosition(q_current);
        kinematic->forwardPosition();

        int successCount = 0, failCount = 0, lastReportedPercent = -1;

        auto solveOne = [&](const rl::math::Transform& T_world_tcp) -> bool
            {
                if (m_stopRequested.load())
                    return false;

                const rl::math::Transform T_world_flange = T_world_tcp * T_tcp_to_flange;

                if (railJoint && railJoint->getDofPosition() == 1)
                {
                    const double y = std::min(
                        std::max(T_world_tcp.translation().y(), railMin0(0)),
                        railMax0(0));
                    constexpr double kRailEps = 1.0e-4;
                    double lo = std::max(railMin0(0), y - kRailEps);
                    double hi = std::min(railMax0(0), y + kRailEps);
                    if (lo > hi)
                    {
                        lo = y;
                        hi = y;
                    }

                    rl::math::Vector railMin(1), railMax(1);
                    railMin << lo;
                    railMax << hi;
                    railJoint->setMinimum(railMin);
                    railJoint->setMaximum(railMax);
                    q_current(0) = std::min(std::max(q_current(0), lo), hi);
                }

                kinematic->setPosition(q_current);
                kinematic->forwardPosition();

                rl::mdl::JacobianInverseKinematics ik(kinematic.get());
                ik.setDuration(std::chrono::milliseconds(std::max(params.timeoutMs, 1)));
                ik.setRandomRestarts(0);
                ik.setEpsilon(1.0e-4);
                ik.addGoal(T_world_flange, 0);

                const bool solved = ik.solve();
                const rl::math::Vector q = kinematic->getPosition();
                kinematic->setPosition(q);
                kinematic->forwardPosition();
                const double poseError =
                    kinematic->getOperationalPosition(0).toDelta(T_world_flange).norm();

                if (q.allFinite() && (solved || poseError < 1.0e-3))
                {
                    q_current = q;
                    jointTrajectory.push_back(q_current);
                    ++successCount;
                }
                else
                {
                    kinematic->setPosition(q_current);
                    kinematic->forwardPosition();
                    ++failCount;
                    jointTrajectory.push_back(q_current);
                }
                return true;
            };

        rl::math::Transform T_prev_tcp = rl::math::Transform::Identity();
        bool hasPrev = false;

        for (int k = 0; k < totalPoints; ++k)
        {
            if (m_stopRequested.load())
            {
                if (railJoint)
                {
                    railJoint->setMinimum(railMin0);
                    railJoint->setMaximum(railMax0);
                }
                m_running.store(false);
                emit aborted();
                return;
            }

            const rl::math::Transform T_curr_tcp = pointToTransform(params.trajectory[k]);

            if (hasPrev)
            {
                const rl::math::Vector3 p0 = T_prev_tcp.translation();
                const rl::math::Vector3 p1 = T_curr_tcp.translation();
                const double dist = (p1 - p0).norm();

                if (dist > railWindow)
                {
                    const int segments = static_cast<int>(std::ceil(dist / railWindow));
                    const Eigen::Quaterniond q0(T_prev_tcp.linear());
                    const Eigen::Quaterniond q1(T_curr_tcp.linear());

                    for (int i = 1; i < segments; ++i)
                    {
                        if (m_stopRequested.load())
                        {
                            if (railJoint)
                            {
                                railJoint->setMinimum(railMin0);
                                railJoint->setMaximum(railMax0);
                            }
                            m_running.store(false);
                            emit aborted();
                            return;
                        }

                        const double t = static_cast<double>(i) / static_cast<double>(segments);

                        rl::math::Transform T_mid = rl::math::Transform::Identity();
                        T_mid.translation() = (1.0 - t) * p0 + t * p1;
                        T_mid.linear() = q0.slerp(t, q1).normalized().toRotationMatrix();

                        if (!solveOne(T_mid))
                        {
                            if (railJoint)
                            {
                                railJoint->setMinimum(railMin0);
                                railJoint->setMaximum(railMax0);
                            }
                            m_running.store(false);
                            emit aborted();
                            return;
                        }
                    }
                }
            }

            if (!solveOne(T_curr_tcp))
            {
                if (railJoint)
                {
                    railJoint->setMinimum(railMin0);
                    railJoint->setMaximum(railMax0);
                }
                m_running.store(false);
                emit aborted();
                return;
            }

            T_prev_tcp = T_curr_tcp;
            hasPrev = true;

            int percent = static_cast<int>((k + 1) * 100LL / totalPoints);
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
        }

        if (railJoint)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }

        kinematic->setPosition(kinematic->getHomePosition());
        kinematic->forwardPosition();

        m_running.store(false);
        const int attempts = successCount + failCount;
        const double ratio = (attempts > 0) ? (static_cast<double>(successCount) / attempts) : 0.0;
        emit finished(jointTrajectory, ratio * 100, params.trajectory[0]);
    }
    catch (const std::exception& e)
    {
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker."));
    }
}

/** @brief 回零路径关节插补。 */
void IKWorker::doReturnHome(const IKReturnHomeParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    rl::math::Vector qFrozen;
    bool sceneFrozen = false;
    rl::mdl::Joint* railJoint = nullptr;
    rl::math::Vector railMin0, railMax0;

    try
    {
        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model || !kinematic)
            throw std::runtime_error("Kinematic not set. Call setKinematic() before doReturnHome().");

        const std::size_t dof = kinematic->getDof();
        if (params.q_home.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("q_home size mismatch with model DOF.");

        emit started();

        const rl::math::Vector q_current = kinematic->getPosition();
        if (q_current.size() != static_cast<Eigen::Index>(dof))
            throw std::runtime_error("Current joint vector size mismatch with model DOF.");
        qFrozen = q_current;
        sceneFrozen = true;

        const rl::math::Transform T_flange_to_tcp = params.T_flange_to_tcp;
        const rl::math::Transform T_tcp_to_flange = T_flange_to_tcp.inverse();
        const double jointStep = (params.jointStepRad > 0.0) ? params.jointStepRad : 0.02;
        const double railStep = (params.railStepLen > 0.0) ? params.railStepLen : 0.01;
        constexpr double kRetractMeter = 0.05;
        constexpr double kFlangeLiftMeter = 0.05;

        railJoint = kinematic->getJoint(0);
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }

        auto restoreRail = [&]() {
            if (railJoint)
            {
                railJoint->setMinimum(railMin0);
                railJoint->setMaximum(railMax0);
            }
            };

        int lastReportedPercent = -1;
        auto reportProgress = [&](int percent) {
            percent = std::max(0, std::min(100, percent));
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
            };

        auto toolXyz = [&](const rl::math::Vector& q) -> rl::math::Vector3 {
            kinematic->setPosition(q);
            kinematic->forwardPosition();
            const rl::math::Transform T_tcp =
                kinematic->getOperationalPosition(0) * T_flange_to_tcp;
            return T_tcp.translation();
            };

        auto appendLerp = [&](std::vector<rl::math::Vector>& path,
            const rl::math::Vector& aIn,
            const rl::math::Vector& bIn,
            double step)
            {
                const rl::math::Vector a = aIn;
                const rl::math::Vector b = bIn;
                const double maxDq = (b - a).cwiseAbs().maxCoeff();
                const int n = (maxDq <= 1e-12)
                    ? 0
                    : std::max(1, static_cast<int>(std::ceil(maxDq / step)));
                path.reserve(path.size() + static_cast<std::size_t>(n));
                for (int i = 1; i <= n; ++i)
                {
                    const double t = static_cast<double>(i) / static_cast<double>(n);
                    rl::math::Vector q = (1.0 - t) * a + t * b;
                    path.push_back(q);
                }
            };

        kinematic->setPosition(q_current);
        kinematic->forwardPosition();
        rl::math::Transform T_tcp =
            kinematic->getOperationalPosition(0) * T_flange_to_tcp;
        T_tcp.translation() -= T_tcp.linear().col(2) * kRetractMeter;

        double railNow = q_current(0);
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            railNow = std::min(std::max(q_current(0), railMin0(0)), railMax0(0));
            constexpr double eps = 1.0e-4;
            double lo = std::max(railMin0(0), railNow - eps);
            double hi = std::min(railMax0(0), railNow + eps);
            if (lo > hi)
            {
                lo = railNow;
                hi = railNow;
            }
            rl::math::Vector railMin(1), railMax(1);
            railMin << lo;
            railMax << hi;
            railJoint->setMinimum(railMin);
            railJoint->setMaximum(railMax);
        }

        rl::math::Vector qSeed = q_current;
        qSeed(0) = railNow;
        kinematic->setPosition(qSeed);
        kinematic->forwardPosition();

        rl::mdl::JacobianInverseKinematics ik(kinematic.get());
        ik.setDuration(std::chrono::milliseconds(std::max(params.timeoutMs, 1)));
        ik.setRandomRestarts(0);
        ik.setEpsilon(1.0e-4);
        ik.addGoal(T_tcp * T_tcp_to_flange, 0);
        const bool ikOk = ik.solve();

        rl::math::Vector qRetract = kinematic->getPosition();
        kinematic->setPosition(qRetract);
        kinematic->forwardPosition();
        const double poseError =
            kinematic->getOperationalPosition(0).toDelta(T_tcp * T_tcp_to_flange).norm();
        if (!qRetract.allFinite() || !(ikOk || poseError < 1.0e-3))
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: TCP retract 0.1 m along tool Z."));
            return;
        }
        qRetract(0) = railNow;
        reportProgress(15);

        kinematic->setPosition(qRetract);
        kinematic->forwardPosition();
        rl::math::Transform T_flange_lift = kinematic->getOperationalPosition(0);
        T_flange_lift.translation().z() -= kFlangeLiftMeter;

        kinematic->setPosition(qRetract);
        kinematic->forwardPosition();
        rl::mdl::JacobianInverseKinematics ikLift(kinematic.get());
        ikLift.setDuration(std::chrono::milliseconds(std::max(params.timeoutMs, 1)));
        ikLift.setRandomRestarts(0);
        ikLift.setEpsilon(1.0e-4);
        ikLift.addGoal(T_flange_lift, 0);
        const bool liftOk = ikLift.solve();

        rl::math::Vector qLift = kinematic->getPosition();
        kinematic->setPosition(qLift);
        kinematic->forwardPosition();
        const double liftError =
            kinematic->getOperationalPosition(0).toDelta(T_flange_lift).norm();
        if (!qLift.allFinite() || !(liftOk || liftError < 1.0e-3))
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: flange lift 0.2 m."));
            return;
        }
        qLift(0) = railNow;
        reportProgress(30);

        rl::math::Vector qArmHome = params.q_home;
        qArmHome(0) = railNow;

        std::vector<rl::math::Vector> path1;
        path1.push_back(q_current);
        if ((q_current - qRetract).cwiseAbs().maxCoeff() > 1e-9)
        {
            const rl::math::Vector qFrom = path1.back();
            appendLerp(path1, qFrom, qRetract, jointStep);
        }
        if ((path1.back() - qLift).cwiseAbs().maxCoeff() > 1e-9)
        {
            const rl::math::Vector qFrom = path1.back();
            appendLerp(path1, qFrom, qLift, jointStep);
        }

        if ((qLift - qArmHome).cwiseAbs().maxCoeff() > 1e-9)
        {
            QMutexLocker lock(&mw->mutex);

            if (!mw->start)
                mw->start = std::make_shared<rl::math::Vector>(qLift);
            else
                *mw->start = qLift;
            if (!mw->goal)
                mw->goal = std::make_shared<rl::math::Vector>(qArmHome);
            else
                *mw->goal = qArmHome;

            mw->planner->start = mw->start.get();
            mw->planner->goal = mw->goal.get();
            mw->planner->viewer = nullptr;

            if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(mw->planner.get()))
                rrt->reset();

            if (!mw->planner->verify())
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Invalid lift or home-arm configuration."));
                return;
            }

            if (!mw->planner->solve())
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Planner failed: lift pose to home joints."));
                return;
            }

            rl::plan::VectorList sparse = mw->planner->getPath();
            if (mw->optimizer)
            {
                mw->optimizer->setViewer(nullptr);
                mw->optimizer->process(sparse);
            }

            rl::math::Vector prev = path1.back();
            for (const rl::math::Vector& node : sparse)
            {
                rl::math::Vector qn = node;
                qn(0) = railNow;
                if ((qn - prev).cwiseAbs().maxCoeff() <= 1e-12)
                    continue;
                appendLerp(path1, prev, qn, jointStep);
                prev = path1.back();
            }
        }

        for (rl::math::Vector& q : path1)
            q(0) = railNow;
        if (path1.empty())
            path1.push_back(qArmHome);
        else if ((path1.back() - qArmHome).cwiseAbs().maxCoeff() > 1e-9)
        {
            const rl::math::Vector qFrom = path1.back();
            appendLerp(path1, qFrom, qArmHome, jointStep);
        }

        restoreRail();
        reportProgress(70);

        std::vector<rl::math::Vector> path2;
        rl::math::Vector qRail = path1.back();
        for (Eigen::Index i = 1; i < qRail.size(); ++i)
            qRail(i) = params.q_home(i);
        path1.back() = qRail;
        path2.push_back(qRail);

        const double y0 = qRail(0);
        const double yTarget = params.q_home(0);
        const double yDir = (yTarget >= y0) ? 1.0 : -1.0;
        double y = y0;
        while (std::abs(yTarget - y) > railStep)
        {
            y += yDir * railStep;
            qRail(0) = y;
            path2.push_back(qRail);
        }
        if (std::abs(yTarget - y) > 1e-9)
        {
            qRail(0) = yTarget;
            path2.push_back(qRail);
        }

        reportProgress(90);

        std::vector<rl::math::Vector> jointTrajectory = path1;
        std::size_t p2Begin = 0;
        if (!path2.empty() && !path1.empty()
            && (path2.front() - path1.back()).cwiseAbs().maxCoeff() < 1e-6)
        {
            p2Begin = 1;
        }
        jointTrajectory.insert(jointTrajectory.end(),
            path2.begin() + static_cast<std::ptrdiff_t>(p2Begin), path2.end());

        if ((jointTrajectory.back() - params.q_home).cwiseAbs().maxCoeff() > 1e-9)
        {
            const rl::math::Vector qFrom = jointTrajectory.back();
            appendLerp(jointTrajectory, qFrom, params.q_home, jointStep);
        }

        std::vector<rl::math::Vector3> toolPoints;
        toolPoints.reserve(jointTrajectory.size());
        for (const rl::math::Vector& q : jointTrajectory)
            toolPoints.push_back(toolXyz(q));

        kinematic->setPosition(q_current);
        kinematic->forwardPosition();

        reportProgress(100);
        m_running.store(false);
        emit finished_return(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        if (sceneFrozen && kinematic && qFrozen.size() > 0)
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
        }
        if (MainWindow* mw = MainWindow::instance())
        {
            QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
                Qt::QueuedConnection, Q_ARG(bool, true));
        }
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        if (sceneFrozen && kinematic && qFrozen.size() > 0)
        {
            kinematic->setPosition(qFrozen);
            kinematic->forwardPosition();
        }
        if (MainWindow* mw = MainWindow::instance())
        {
            QMetaObject::invokeMethod(mw, "setSceneFlushEnabled",
                Qt::QueuedConnection, Q_ARG(bool, true));
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (return home)."));
    }
}

/** @brief 移动到路径起点的关节规划。 */
void IKWorker::doGoToStart(const IKGoToStartParams& params)
{
    m_running.store(true);
    m_stopRequested.store(false);

    std::shared_ptr<rl::mdl::Kinematic> kinematic;
    {
        QMutexLocker locker(&m_kinMutex);
        kinematic = m_kinematic;
    }

    rl::mdl::Joint* railJoint = nullptr;
    rl::math::Vector railMin0, railMax0;

    try
    {
        MainWindow* mw = MainWindow::instance();
        if (!mw || !mw->planner || !mw->model || !kinematic) throw std::runtime_error("Kinematic not set. Call setKinematic() before doGoToStart().");

        const std::size_t dof = kinematic->getDof();
        if (params.q_home.size() != static_cast<Eigen::Index>(dof)) throw std::runtime_error("q_home size mismatch with model DOF.");

        emit started();

        const rl::math::Transform T_tcp_to_flange = params.T_flange_to_tcp.inverse();
        const rl::math::Transform T_world_tcp_start = pointToTransform(params.startPoint);
        const double yWeld = T_world_tcp_start.translation().y();

        railJoint = kinematic->getJoint(0);
        if (railJoint)
        {
            railMin0 = railJoint->getMinimum();
            railMax0 = railJoint->getMaximum();
        }

        auto restoreRail = [&]() {
            if (railJoint)
            {
                railJoint->setMinimum(railMin0);
                railJoint->setMaximum(railMax0);
            }
            };

        int lastReportedPercent = -1;
        auto reportProgress = [&](int percent) {
            percent = std::max(0, std::min(100, percent));
            if (percent != lastReportedPercent)
            {
                emit progress(percent);
                lastReportedPercent = percent;
            }
            };

        auto toolXyz = [&](const rl::math::Vector& q) -> rl::math::Vector3 {
            kinematic->setPosition(q);
            kinematic->forwardPosition();
            const rl::math::Transform T_tcp =
                kinematic->getOperationalPosition(0) * params.T_flange_to_tcp;
            return T_tcp.translation();
            };

        auto checkStop = [&]() -> bool {
            if (!m_stopRequested.load())
                return false;
            restoreRail();
            m_running.store(false);
            emit aborted();
            return true;
            };

        // ========== path1：只动地轨 Joint0，对齐焊接起点 Y，其它关节保持 Home ==========
        double kRailStepMm = params.railStepLen;
        double yTarget = yWeld;
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            yTarget = std::min(std::max(yWeld, railMin0(0)), railMax0(0));
        }

        std::vector<rl::math::Vector> path1;
        rl::math::Vector qRail = params.q_home;
        path1.push_back(qRail);

        const double y0 = params.q_home(0);
        const double yDir = (yTarget >= y0) ? 1.0 : -1.0;
        double y = y0;
        while (std::abs(yTarget - y) > kRailStepMm)
        {
            if (checkStop())
                return;
            y += yDir * kRailStepMm;
            qRail(0) = y;
            path1.push_back(qRail);
        }
        if (std::abs(yTarget - y) > 1e-9)
        {
            qRail(0) = yTarget;
            path1.push_back(qRail);
        }

        reportProgress(20);

        // ========== 固定 Joint0，后续 IK / RRT 不再采样地轨 ==========
        if (railJoint && railJoint->getDofPosition() == 1)
        {
            const double eps = 1e-6;
            rl::math::Vector lo(1), hi(1);
            lo << (yTarget - eps);
            hi << (yTarget + eps);
            railJoint->setMinimum(lo);
            railJoint->setMaximum(hi);
        }

        // ========== 焊接起点关节解（path2 终点）==========
        rl::mdl::JacobianInverseKinematics ik(kinematic.get());
        ik.setDuration(std::chrono::milliseconds(params.timeoutMs));
        ik.addGoal(T_world_tcp_start * T_tcp_to_flange, 0);
        kinematic->setPosition(path1.back());
        if (!ik.solve())
        {
            restoreRail();
            m_running.store(false);
            emit failed(QStringLiteral("IK failed: weld start configuration."));
            return;
        }
        rl::math::Vector qGoal = kinematic->getPosition();
        qGoal(0) = yTarget;

        reportProgress(30);

        // ========== path2：Joint0 锁定后的 RRT（path1 终点 → 焊接起点）==========
        std::vector<rl::math::Vector> path2;
        {
            QMutexLocker lock(&mw->mutex);

            if (!mw->start)
                mw->start = std::make_shared<rl::math::Vector>(path1.back());
            else
                *mw->start = path1.back();
            if (!mw->goal)
                mw->goal = std::make_shared<rl::math::Vector>(qGoal);
            else
                *mw->goal = qGoal;

            mw->planner->start = mw->start.get();
            mw->planner->goal = mw->goal.get();
            mw->planner->viewer = nullptr;

            if (!mw->planner->verify())
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Invalid start or goal configuration."));
                return;
            }

            if (checkStop())
                return;

            const bool solved = mw->planner->solve();
            if (!solved)
            {
                restoreRail();
                m_running.store(false);
                emit failed(QStringLiteral("Planner failed: home-rail pose to weld start."));
                return;
            }

            rl::plan::VectorList sparse = mw->planner->getPath();
            if (mw->optimizer)
            {
                mw->optimizer->setViewer(nullptr);
                mw->optimizer->process(sparse);
            }

            const double delta = (params.cartStepLen > 0.0) ? params.cartStepLen : 1.0;
            rl::math::Vector inter(mw->model->getDofPosition());
            if (!sparse.empty())
                path2.push_back(*sparse.begin());

            rl::plan::VectorList::iterator it = sparse.begin();
            rl::plan::VectorList::iterator jt = sparse.begin();
            if (jt != sparse.end())
                ++jt;
            for (; it != sparse.end() && jt != sparse.end(); ++it, ++jt)
            {
                if (checkStop())
                    return;
                const rl::math::Real steps =
                    std::ceil(mw->model->distance(*it, *jt) / delta);
                const rl::math::Real n = (steps < 1.0) ? 1.0 : steps;
                for (std::size_t k = 1; k < static_cast<std::size_t>(n) + 1; ++k)
                {
                    mw->model->interpolate(*it, *jt, static_cast<rl::math::Real>(k) / n, inter);
                    path2.push_back(inter);
                }
            }
        }

        // 地轨保持 yTarget，避免限位窗口 eps 漂移
        for (rl::math::Vector& q : path2)
            q(0) = yTarget;

        restoreRail();
        reportProgress(90);

        // ========== 拼接 path1 + path2 ==========
        std::vector<rl::math::Vector> jointTrajectory = path1;
        std::size_t p2Begin = 0;
        if (!path2.empty() && !path1.empty()
            && (path2.front() - path1.back()).cwiseAbs().maxCoeff() < 1e-6)
        {
            p2Begin = 1;
        }
        jointTrajectory.insert(jointTrajectory.end(), path2.begin() + static_cast<std::ptrdiff_t>(p2Begin), path2.end());

        std::vector<rl::math::Vector3> toolPoints;
        toolPoints.reserve(jointTrajectory.size());
        for (const rl::math::Vector& q : jointTrajectory)
        {
            if (checkStop()) return;
            toolPoints.push_back(toolXyz(q));
        }

        reportProgress(100);

        kinematic->setPosition(kinematic->getHomePosition());
        kinematic->forwardPosition();

        m_running.store(false);
        emit finished_start(jointTrajectory, toolPoints, 100.0);
    }
    catch (const std::exception& e)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QString::fromUtf8(e.what()));
    }
    catch (...)
    {
        if (railJoint && railMin0.size() > 0)
        {
            railJoint->setMinimum(railMin0);
            railJoint->setMaximum(railMax0);
        }
        m_running.store(false);
        emit failed(QStringLiteral("Unknown exception in IK worker (go to start)."));
    }
}