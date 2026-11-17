#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTranslator>
#include <QMessageBox>
#include "ui_mainwindow.h"

#include "GlobalDefs.h"

#include "device_cam/ICamera_factory.h"
#include "device_laser/ILaser_factory.h"

#include "base/customdock.h"
#include "base/customdialog.h"

#include "device_robot/Profinet.h"

class ConfigurationDelegate;
class ConfigurationModel;
class OperationalDelegate;
class OperationalModel;
class ReadModel;
class CustomDialog;
class CustomDock;
class OccView;
class IKWorker;
class ProfinetComm;
class ProcessWorker;
class MvCameraWorker;

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    /**
     * @brief 获取主窗口单例。
     * @return 全局 MainWindow 指针；若尚未创建则自动构造。
     */
    static MainWindow* instance();

    /**
     * @brief 构造主窗口并完成各子系统初始化。
     * @param parent 父控件，默认为 nullptr。
     */
    explicit MainWindow(QWidget* parent = nullptr);

    /**
     * @brief 析构主窗口并释放线程与 UI 资源。
     */
    ~MainWindow();

    /**
     * @brief 创建并显示指定类型的自定义对话框。
     * @tparam T 对话框类型。
     * @param label 对话框注册名称。
     * @param center 是否在父窗口居中显示。
     */
    template <class T>
    void createDialog(const QString& label, const bool& center) {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console, center);
    }

    /**
     * @brief 切换应用界面主题。
     * @param index 0=暗色，1=MacOS，2=Ubuntu。
     */
    void changeTheme(int);

    /**
     * @brief 切换界面语言。
     * @param index 0=英文，1=中文。
     */
    void changeLanguage(int);

    /**
     * @brief 处理机器人工具/用户坐标系单选切换。
     * @param checked 单选钮是否选中。
     */
    void changeRobotCoorBtn(bool);

    /**
     * @brief 安全地将图像写入磁盘。
     * @param savePath 目标文件路径。
     * @param image OpenCV 图像矩阵。
     */
    void safeImwrite(const QString& savePath, const cv::Mat& image);

    /**
     * @brief 加载机器人连杆 STL 并在 OCCT 中显示。
     * @param type 机器人型号目录名（如 kuka16、kuka20）。
     */
    void loadOcctModel(std::string type);

    /**
     * @brief 初始化全部 Qt 信号与槽连接。
     */
    void connect_init();

    /**
     * @brief 在 3D 视图中可视化离散焊缝轨迹点。
     * @param trajPt 离散轨迹点序列（含位姿与方向）。
     */
    void showWeldTrajPt(const std::vector<DiscretePoint>& trajPt);

    /**
     * @brief 在 3D 视图中绘制工具中心路径折线。
     * @param toolPoints 工具点在基坐标系下的位置序列（米）。
     */
    void showToolPoints(const std::vector<rl::math::Vector3>& toolPoints);

    /**
     * @brief 初始化逆解工作线程及轨迹仿真相关连接。
     */
    void initSimulation();

    /**
     * @brief 由点云中两点计算起止焊接位姿。
     * @param start_end 至少包含起止两点的点云。
     * @param scene 场景点云，用于法向估计。
     * @param radius 法向邻域搜索半径。
     * @param pose_start 输出起点齐次位姿。
     * @param pose_end 输出终点齐次位姿。
     * @return 成功为 true；点不足或几何退化为 false。
     */
    bool computeTwoPointPoses(const ct::Cloud::Ptr& start_end,
        const ct::Cloud::Ptr& scene,
        float radius,
        Eigen::Affine3f& pose_start,
        Eigen::Affine3f& pose_end);

    /**
     * @brief 显示点云及起止位姿坐标轴。
     * @param cloud 彩色点云，可为空则仅显示坐标轴。
     * @param poseStart 起点位姿。
     * @param poseEnd 终点位姿。
     * @param axisLength 坐标轴显示长度。
     */
    void DisplayCloudAndAxes(
        const ct::Cloud::Ptr& cloud,
        const Eigen::Affine3f& poseStart,
        const Eigen::Affine3f& poseEnd,
        Standard_Real axisLength);

    /**
     * @brief 初始化熔池相机线程与 UI 绑定。
     */
    void InitializedMeltCam();

    /**
     * @brief 初始化激光传感器及工作线程通信。
     */
    void InitializeLaser();

    /**
     * @brief 初始化 Profinet 机器人通信与控制面板。
     */
    void InitializeRobot();

    /**
     * @brief 初始化 3D 结构光/线扫相机及采集逻辑。
     */
    void InitializeCamera();

    /**
     * @brief 输出 INFO 日志到控制台与 glog。
     * @param msg 日志文本。
     */
    void logInfo(const QString& msg) {
        ui->console->print(ct::LOG_INFO, msg);
        LOG(INFO) << msg.toStdString();
    }

    /**
     * @brief 输出 WARNING 日志到控制台与 glog。
     * @param msg 日志文本。
     */
    void logWarning(const QString& msg) {
        ui->console->print(ct::LOG_WARNING, msg);
        LOG(WARNING) << msg.toStdString();
    }

    /**
     * @brief 输出 ERROR 日志到控制台与 glog。
     * @param msg 日志文本。
     */
    void logError(const QString& msg) {
        ui->console->print(ct::LOG_ERROR, msg);
        LOG(ERROR) << msg.toStdString();
    }

    /**
     * @brief 从配置文件加载机器人型号并初始化控制部件。
     */
    void robotWidgetInit();

    /**
     * @brief 从 RL XML 加载运动学、场景与规划组件。
     * @param filename 机器人描述 XML 路径。
     */
    void loadRobotWidget(const QString& filename);

signals:
    /** @brief 通知 ProcessWorker 执行视觉处理流程。 */
    void triggerVisionPro();

    /**
     * @brief 向机器人通信层下发轨迹关节点。
     * @param joint 当前插补点的关节向量。
     */
    void sendTrajectory(const rl::math::Vector& joint);

protected:
    /**
     * @brief 关闭窗口前弹出退出确认。
     * @param event 关闭事件，确认时 accept，否则 ignore。
     */
    void closeEvent(QCloseEvent* event) override;

public:
    QString engine;

    std::vector<std::shared_ptr<rl::plan::WorkspaceSphereExplorer>> explorers;

    std::shared_ptr<rl::math::Vector> goal;

    std::shared_ptr<rl::mdl::Dynamic> mdl;

    std::shared_ptr<rl::plan::SimpleModel> model;

    QMutex mutex;

    std::vector<std::shared_ptr<rl::plan::NearestNeighbors>> nearestNeighbors;

    std::shared_ptr<rl::plan::Optimizer> optimizer;

    std::shared_ptr<rl::plan::Planner> planner;

    std::shared_ptr<rl::math::Vector> q;

    std::shared_ptr<rl::plan::Sampler> sampler;

    std::shared_ptr<rl::math::Vector> sigma;

    std::shared_ptr<rl::sg::Scene> scene;

    rl::sg::Model* sceneModel;

    rl::sg::Body* sceneBody;

    rl::sg::Body* rlWorkpiece;

    std::shared_ptr<rl::math::Vector> start;

    std::shared_ptr<rl::plan::Verifier> verifier;

    bool isConnectToRobot = false;

    Handle(AIS_Shape) tool_aWire;

    rl::math::Transform tcp_transform;
    rl::math::Transform laser_transform;

    ConfigurationDelegate* configurationDelegates;
    OperationalDelegate* operationalDelegates;

    OperationalModel* operationalModel;
    ConfigurationModel* configurationModel;

private slots:

    /**
     * @brief 根据关节角刷新 OCCT 机器人连杆显示。
     */
    void occtUpdate();

    /** @brief 导入 STEP/STP 工件并绑定碰撞体。 */
    void load3DModel();

    /** @brief 导出当前选中形体为 STL 或 STEP。 */
    void saveModel();

    /** @brief 移除选中对象并同步删除 RL 工件体。 */
    void removeModel();

    /**
     * @brief 处理相机采集回调。
     * @param data 点云、彩色图等相机数据。
     * @param id 点云/帧标识。
     */
    void GetCamData(const CamData& data, const QString& id);

    /** @brief 启动单次视觉处理（相机、点云树或本地文件）。 */
    void actionStart_triggered();

    /**
     * @brief 刷新焊缝列表并触发轨迹规划。
     * @param selectShape 当前选中工件形体。
     * @param weldEdges 提取的焊缝边集合。
     */
    void flushWeldList(const TopoDS_Shape& selectShape, const std::vector<TopoDS_Edge>& weldEdges);

    /**
     * @brief 焊缝表选中行变化时高亮焊缝方向。
     * @param selected 新选中项。
     * @param deselected 取消选中的项。
     */
    void onWeldSelected(const QItemSelection& selected, const QItemSelection& deselected);

    /** @brief 异步规划并执行回 Home 关节轨迹。 */
    void robotGoHome();

    /**
     * @brief 仿真定时器：按索引播放 wholeTrajectory。
     * @param inter 当前轨迹点索引（引用，调用方递增）。
     */
    void execSimulation(int& inter);

    /**
     * @brief 回零动画定时器：播放 m_jointTrajectory_home。
     * @param inter 当前回零路径索引（引用，调用方递增）。
     */
    void execReturnHome(int& inter);

    /** @brief 生成焊接离散轨迹并请求 IK 移动到起点。 */
    void Trajectory();

    /** @brief 同步 AIS 工件位姿到 RL 碰撞体。 */
    void updateAisRL();

    /** @brief 工件几何变更后重建碰撞体绑定。 */
    void rebuildWorkpieceCollision();

    /**
     * @brief 控制场景刷新定时器启停。
     * @param enabled true 启动 50ms 刷新，false 暂停。
     */
    void setSceneFlushEnabled(bool enabled);
private:
    static MainWindow* singleton;
    QVector<TopoDS_Shape> links;
    QVector<AIS_Shape*> displinks;
    bool isTheFirstDraw = true;

    Ui::MainWindow* ui;
    QTranslator* translator;

    QMutex mapMutex;
    OccView* myOccView;
    Handle(AIS_Shape) m_weldArrow;

    QTimer* handwareStatusUpdateTimer;

    QTimer* flushSceneTimer;

    QTimer* flushTrajTimer;
    bool isPauseSim = false;

    QTimer* flushHomeTimer;

    IKWorker* ikwork;
    QThread* m_thread_ikwork = nullptr;

    std::unique_ptr<ProcessWorker> proworker;
    QThread* m_thread_prowork = nullptr;

    bool meltCamConnected = false;
    bool meltCamGraping = false;
    QThread* m_thread_melt = nullptr;
    MvCameraWorker* m_worker_melt = nullptr;

    QThread* m_thread_rbworker = nullptr;

    std::unique_ptr<ProfinetComm> m_comm;

    CameraFactory::CameraType camType;
    std::unique_ptr<ICamera> camera;
    int robot_heart_beat_num = 0;
    int robot_heart_beat_num_last = 0;
    int heart_beat_different_times = 0;
    QTimer* heartMonitorTimer;
    bool robotConnected = false;

    LaserFactory::LaserType laserType;
    std::unique_ptr<ILaser> laser;
    QThread* m_thread_laserwork = nullptr;
    int laserIndex = 0;

    bool camConnected = false;
    bool laserConnected = false;

    bool camCaptureRunning = false;

    bool isRunOnce = false;
    bool isSimulation = false;
    int m_counter = 0;
    int m_counter_home = 0;
    std::vector<rl::math::Vector> m_jointTrajectory_home;
    std::vector<rl::math::Vector> wholeTrajectory;
    TopoDS_Shape m_selectShape;
    std::vector<EdgeCluster> sortedClusters;
    std::vector<DiscretePoint> mergedTraj;

    ct::Cloud::Ptr cam_captured;

    int calibNumCount = 0;
    QString current_cloud_id = "";
    AIS_Shape* loadShape;
    QMutex g_robotMutex;
    RobotGetData g_robotData;
    bool m_IsCartMove = false;

    bool weldMachineConneted = false;
};
#endif // MAINWINDOW_H