/**
 * @file IKWorker.h
 * @brief 运行于工作线程的 RL 逆运动学与路径插补 Worker。
 */
#pragma once
#ifndef IK_WORKER_H
#define IK_WORKER_H

#include <QObject>
#include <QMutex>
#include <atomic>
#include <memory>
#include <vector>

#include "GlobalDefs.h"

Q_DECLARE_METATYPE(rl::math::Vector)
Q_DECLARE_METATYPE(std::vector<rl::math::Vector>)
Q_DECLARE_METATYPE(IKSolveParams)
Q_DECLARE_METATYPE(IKReturnHomeParams)
Q_DECLARE_METATYPE(DiscretePoint)
Q_DECLARE_METATYPE(IKGoToStartParams)
Q_DECLARE_METATYPE(std::vector<rl::math::Vector3>)

/**
 * @brief 异步 IK / 回零 / 起始点规划；通过 Qt 信号返回关节轨迹。
 */
class IKWorker : public QObject
{
    Q_OBJECT
public:
    explicit IKWorker(QObject* parent = nullptr);
    ~IKWorker() override;

    /**
     * @brief 注入共享运动学模型（与主线程共享 shared_ptr）。
     * @note 求解运行期间主线程不得读写该模型。
     */
    void setKinematic(const std::shared_ptr<rl::mdl::Dynamic>& mdl);

    /** @brief 请求中止当前任务。 */
    void requestStop();
    /** @brief 是否正在执行 doSolve / doReturnHome / doGoToStart。 */
    bool isRunning() const { return m_running.load(); }

public slots:
    /** @brief 沿离散路径点求逆解并生成关节轨迹。 */
    void doSolve(const IKSolveParams& params);
    /** @brief 从当前配置规划回零/安全位关节路径。 */
    void doReturnHome(const IKReturnHomeParams& params);
    /** @brief 规划移动到路径起始点的关节轨迹。 */
    void doGoToStart(const IKGoToStartParams& params);

signals:
    void started();
    void progress(int percent);
    void finished(const std::vector<rl::math::Vector>& jointTrajectory,
        const double& ratio, const DiscretePoint& start);
    void finished_return(const std::vector<rl::math::Vector>& jointTrajectory,
        const std::vector<rl::math::Vector3>& toolPoints,
        const double& ratio);
    void finished_start(const std::vector<rl::math::Vector>& jointTrajectory,
        const std::vector<rl::math::Vector3>& toolPoints,
        const double& ratio);
    void failed(const QString& errorMessage);
    void aborted();

private:
    /** @brief 将离散点位置与 XYZ 轴向构造成 rl::math::Transform。 */
    rl::math::Transform pointToTransform(const DiscretePoint& pt);

    std::shared_ptr<rl::mdl::Kinematic> m_kinematic;
    mutable QMutex m_kinMutex;
    std::atomic<bool> m_stopRequested{ false };
    std::atomic<bool> m_running{ false };
};

#endif