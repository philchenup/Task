#ifndef FR5_H
#define FR5_H

#include "device_robot/IRobot.h"
#include "libfairino/robot.h"
#include "utils/utils.h"

#include <QTimer>
#include <QMutex>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <utility>

// ─────────────────────────────────────────────────────────────────────────────
// FR5 — 法奥 FR5，接口与 JAKAZU12 对齐
// ─────────────────────────────────────────────────────────────────────────────
class FR5 : public IRobot
{
    Q_OBJECT

public:
    explicit FR5(QObject* parent = nullptr);
    ~FR5() override;

    bool isConnected() const override;
    bool isEnabled() const override;

    void calcOffMatrix(const Eigen::Affine3f tcp, double offInter, std::string rotType, std::vector<double>& off_vec);

    bool quat2eulerFr5(const std::vector<double>& quat_in, std::vector<double>& euler_out);

    std::vector<double> toDouble(const utils::Joint& j);

    std::vector<std::vector<double>> densifyJoints(const std::vector<std::vector<double>>& joints);

    int runTrajectoryPoint(const std::vector<std::vector<double>>& path);

    int getPlaceJoint(const std::vector<double>& cur_place_joint, const utils::ArrayConfig& cfg,
        int n, bool x_first, std::vector<double>& new_place_joint);

    std::vector<double> toVector(const JointPos& jv);

    JointPos toJoint(const std::vector<double>& joint_vec);

public slots:
    void connectRobot(const RobotPara& para) override;
    void disconnectRobot() override;
    void enableRobot(bool enable) override;
    void clearError() override;

    void resetPlace() override;

    void moveJoint(const std::vector<double>& joint, const QString& tag = QString()) override;
    void movePose(const std::vector<double>& pose, const QString& tag = QString()) override;

    void sigUpdatePose() override;

    void setSpeedFactor(int percent) override;

    bool getEndPose(std::vector<double>& pose) override;
    bool getCurrentJoint(std::vector<double>& j) override;

    void startGraspSequence(const std::vector<std::vector<double>>& path, const Eigen::Affine3f& tcp) override;

    void startScrewSequence(const std::vector<std::pair<std::vector<double>, std::vector<double>>>& pose,
        const Eigen::Affine3f& tcp, const std::string rotType) override;

    void onDrillStep() override;

    void robotStop() override;
    void robotStart() override;

    void setDownSpeed(const double speed) override { downSpeed = speed; }

    void getScrewStatus(const std::string& status) override;

    void moveNextPose(bool ok, const QString& tag) override;

    void loadPlaceConfig(const std::string& path) override;

private slots:
    void onFeedbackTimerTimeout();
    void startFeedbackTimer();
    void stopFeedbackTimer();

signals:
    void _startFeedbackTimer();
    void _stopFeedbackTimer();

private:
    void connectTask(const std::string& ip);
    void applySpeed();

    void waitArriveAndNotify(const std::vector<double>& targetPose,
        const QString& tag,
        int timeoutMs = 30000);
    void waitArriveJointAndNotify(const std::vector<double>& targetJoint,
        const QString& tag,
        int timeoutMs = 30000);

    QMutex m_mutex_recv;
    std::string recvRet = "";
    Eigen::Affine3f m_tcp;
    double m_offInter = 0.0;
    std::string m_rotType;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> m_grasp_pose;
    std::vector<std::pair<std::vector<double>, std::vector<double>>> m_screw_pose;
    int currentIndex = 0;
    bool isStop = false;
    std::vector<double> capJoint;
    double downSpeed = 0.0;

    bool loadPlaceStatus = false;
    utils::PlaceConfig place_config;
    std::vector<std::vector<double>> m_path;
    int count_grasp = 0;

    void updateSnapshot();
    bool readCurrentPose(std::vector<double>& pose);
    bool readCurrentJoint(std::vector<double>& joint);

    struct FeedbackSnapshot {
        std::vector<double> endVec{ 0, 0, 0, 0, 0, 0 };
        std::vector<double> joint{ 0, 0, 0, 0, 0, 0 };
        bool valid = false;
    };
    mutable std::mutex m_snapMutex;
    FeedbackSnapshot m_snap;

    FRRobot m_robot;

    std::atomic<bool> m_connected{ false };
    std::atomic<bool> m_enabled{ false };

    double m_globalSpeed = 20.0;

    QTimer* m_feedbackTimer{ nullptr };
    QTimer* m_drillTimer{ nullptr };
};

#endif // FR5_H
