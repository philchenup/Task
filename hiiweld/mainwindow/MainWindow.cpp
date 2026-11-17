#include "mainwindow.h"

#include "edit/cutting.h"
#include "edit/sampling.h"
#include "edit/color.h"

#include "tool/pickpoints.h"

#include "help/about.h"
#include "help/shortcutkey.h"

#include <QDebug>
#include <QDesktopWidget>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QRadioButton>

#include "base/customdock.h"
#include "base/customdialog.h"

#include "tool/readmodel.h"
#include "tool/occView.h"
#include "tool/TcpCalib.h"
#include "tool/WeldSeamTransition.h"
#include "tool/IKWorker.h"
#include "tool/CamConfig.h"
#include "tool/HandEyeCalib.h"
#include "tool/VisionConfig.h"
#include "tool/AisRlBodyBinding.h"
#include "tool/expertProcess.h"
#include "tool/record.h"

#include "utils/processWorker.h"

#include "robot_control/ConfigurationDelegate.h"
#include "robot_control/ConfigurationModel.h"
#include "robot_control/OperationalDelegate.h"
#include "robot_control/OperationalModel.h"

#include "device_melt/meltCam.h"

#include "utils/ImageLabelZoom.h"

namespace
{
	Quantity_Color grayColor(194 / 255.0f, 186 / 255.0f, 181 / 255.0f, Quantity_TOC_RGB);
	Quantity_Color kukaColor(1.0, 0.5, 0.0, Quantity_TOC_RGB);
	Quantity_Color balckColor(0.2, 0.2, 0.2, Quantity_TOC_RGB);
	Quantity_Color silveryColor(0.7725, 0.7843, 0.7843, Quantity_TOC_RGB);
	Quantity_Color redColor(0.8196, 0.247, 0.0745, Quantity_TOC_RGB);
	Quantity_Color yellowColor(0.7529, 0.74117, 0.3647, Quantity_TOC_RGB);
}

/**
 * @brief 构造主窗口并完成各子系统初始化。
 * @param parent 父控件。
 */
MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	ui(new Ui::MainWindow),
	translator(nullptr),
	myOccView(new OccView(this)),
	configurationDelegates(),
	operationalDelegates(),
	flushSceneTimer(new QTimer),
	flushTrajTimer(new QTimer),
	flushHomeTimer(new QTimer),
	handwareStatusUpdateTimer(new QTimer),
	ikwork(new IKWorker),
	m_thread_laserwork(new QThread),
	m_thread_ikwork(new QThread),
	m_thread_prowork(new QThread),
	m_thread_rbworker(new QThread),
	m_thread_melt(new QThread),
	m_worker_melt(new MvCameraWorker),
	cam_captured(new ct::Cloud)
{
	MainWindow::singleton = this;

	ui->setupUi(this);

	SoDB::init();

	// init qrc
	Q_INIT_RESOURCE(res);

	ui->sceneWidget->addWidget(myOccView);
	ui->tabWidget->setCurrentIndex(0);

	ui->weldTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	ui->weldTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	ui->weldTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
	ui->weldTableWidget->setColumnWidth(0, 100);
	ui->weldTableWidget->setColumnWidth(1, 100);
	ui->mainConsoleWidget->setCurrentIndex(0);
	ui->laserTabWidget->setCurrentIndex(0);

	ui->cloudview->setBackgroundColor({ 127, 127, 127 });
	ui->laser3DWidget->setBackgroundColor({ 20, 20, 20 });
	ui->cloudtree->setCloudView(ui->cloudview);
	ui->cloudtree->setConsole(ui->console);
	ui->cloudtree->setParentIcon(QIcon(":/res/icon/document-open.svg"));
	ui->cloudtree->setChildIcon(QIcon(":/res/icon/view-calendar.svg"));

	ui->fileLayout->setContentsMargins(0, 0, 0, 0);
	ui->fileLayout->setSpacing(6);
	ui->setLayout->setContentsMargins(0, 0, 0, 0);
	ui->setLayout->setSpacing(6);
	ui->algorithmLayout->setContentsMargins(0, 0, 0, 0);
	ui->algorithmLayout->setSpacing(6);
	ui->viewLayout->setContentsMargins(0, 0, 0, 0);
	ui->viewLayout->setSpacing(6);
	ui->statusLayout->setContentsMargins(0, 0, 0, 0);
	ui->statusLayout->setSpacing(6);
	ui->globalFuncLayout->setContentsMargins(0, 0, 0, 0);
	ui->globalFuncLayout->setSpacing(6);

	ui->menuWidget->setCurrentIndex(0);
	ui->mainConsoleWidget->setCurrentIndex(1);

	connect_init();

	robotWidgetInit();

	initSimulation();

	this->changeTheme(0);
	std::string result = "HWIWeld Start!";
	logInfo(QString::fromLocal8Bit(result.c_str()));

	InitializeCamera();

	InitializedMeltCam();

	InitializeLaser();

	InitializeRobot();
}

/** @brief 停止定时器与工作线程，释放 UI 并清空单例指针。 */
MainWindow::~MainWindow() {
	logInfo("MainWindow destroying; shutting down worker threads.");
	MainWindow::singleton = nullptr;

	handwareStatusUpdateTimer->stop();
	flushSceneTimer->stop();
	flushTrajTimer->stop();
	flushHomeTimer->stop();

	camera = nullptr;

	if (m_thread_prowork) {
		m_thread_prowork->quit();
		m_thread_prowork->wait(3000);
		delete m_thread_prowork;
		m_thread_prowork = nullptr;
	}

	if (m_thread_laserwork) {
		m_thread_laserwork->quit();
		m_thread_laserwork->wait(3000);
		delete m_thread_laserwork;
		m_thread_laserwork = nullptr;
	}

	if (ikwork && ikwork->isRunning()) ikwork->requestStop();
	if (m_thread_ikwork) {
		m_thread_ikwork->quit();
		m_thread_ikwork->wait(3000);
		delete m_thread_ikwork;
		m_thread_ikwork = nullptr;
	}
	ikwork = nullptr;

	if (m_thread_rbworker) {
		m_thread_rbworker->quit();
		m_thread_rbworker->wait(3000);
		delete m_thread_rbworker;
		m_thread_rbworker = nullptr;
	}

	if (m_thread_melt) {
		m_thread_melt->quit();
		m_thread_melt->wait(3000);
		delete m_thread_melt;
		m_thread_melt = nullptr;
	}

	delete ui;
}

/**
 * @brief 获取主窗口单例。
 * @return 全局 MainWindow 指针；不存在时先创建。
 */
MainWindow* MainWindow::instance()
{
	if (nullptr == MainWindow::singleton)
	{
		new MainWindow();
	}
	return MainWindow::singleton;
}

/** @brief 绑定菜单、设备、仿真、机器人及 ProcessWorker 等全部信号槽。 */
void MainWindow::connect_init() {

	// 轨迹刷新
	connect(this->myOccView, &OccView::sendExtractEdges, this, &MainWindow::flushWeldList, Qt::QueuedConnection);
	connect(this->myOccView, &OccView::updateAis2RL, this, &MainWindow::updateAisRL, Qt::QueuedConnection);
	connect(this->myOccView, &OccView::workpieceGeometryChanged, this, &MainWindow::rebuildWorkpieceCollision, Qt::QueuedConnection);

	// 3D模型操作
	connect(ui->importModelBtn, &QToolButton::clicked, this, &MainWindow::load3DModel, Qt::QueuedConnection);
	connect(ui->saveModelBtn, &QToolButton::clicked, this, &MainWindow::saveModel, Qt::QueuedConnection);
	connect(ui->removeModelBtn, &QToolButton::clicked, this, &MainWindow::removeModel, Qt::QueuedConnection);

	// 点云文件操作
	connect(ui->importCloudBtn, &QToolButton::clicked, this, [=] {  
		ui->cloudtree->addCloud(); 
		ui->tabWidget->setCurrentIndex(1);
		});
	connect(ui->saveCloudBtn, &QToolButton::clicked, this, [=] { 
		ui->cloudtree->saveSelectedClouds(); 
		ui->tabWidget->setCurrentIndex(1); 
		});
	connect(ui->removeCloudBtn, &QToolButton::clicked, this, [=] {
		ui->cloudtree->removeSelectedClouds(); 
		ui->tabWidget->setCurrentIndex(1); 
		});

	connect(ui->exitBtn, &QToolButton::clicked, this, [=] { this->close(); });

	// 配置类
	connect(ui->tcpCalibBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Tcp Calibration");
		this->createDialog<TcpCalib>("TcpCalib", true);
		});

	connect(ui->handeyeBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: HandEye Calibration");
		this->createDialog<HandEyeCalib>("HandEyeCalib", true);
		});

	connect(ui->camsetBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: CamParaSet");
		this->createDialog<CamConfig>("CamConfig", true);
		});

	connect(ui->laserCalibBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Laser Calibration");
		
		});

	connect(ui->stopLimitBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Stop Limitation Setting");

		});

	connect(ui->systemSetBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: System Setting");

		});

	// 算法类
	connect(ui->cutCloudBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Cutting");
		this->createDialog<Cutting>("Cutting", false);
		});

	connect(ui->cloudSampleBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Sample");
		this->createDialog<Sampling>("Sampling", false);
		});

	connect(ui->colorRenderBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Color");
		this->createDialog<Color>("Color", false);
		});

	connect(ui->pickPointBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: PickPoint");
		this->createDialog<PickPoints>("PickPoints", false);
		/*if (PickPoints* dlg = ct::getDialog<PickPoints>("PickPoints"))
		{
			connect(dlg, &PickPoints::sendPointsPicked, this, [=](const ct::Cloud::Ptr& pick) {
				Eigen::Affine3f start_pose, end_pose;
				bool ret = computeTwoPointPoses(pick, cam_captured, 5.0, start_pose, end_pose);
				if (ret) {
					DisplayCloudAndAxes(cam_captured, start_pose, end_pose, 50.0);
				}
				else {
					ui->console->print(ct::LOG_ERROR, "Compute weld start pose or end pose wrong!");
					return;
				}
				}, Qt::QueuedConnection);
		}*/
		});

	connect(ui->meltPoolBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Melt pool");

		});

	connect(ui->weldFeatureBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: weld Feature");

		});

	// 视图类
	connect(ui->reportAnalysisBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Report Analysis");

		});

	connect(ui->openDailyBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open Daily");

		});

	connect(ui->workStatisBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Work Statis");

		});

	connect(ui->imageSaveBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open Image Save Path");

		});

	connect(ui->dataSaveBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open Data Save Path");

		});

	connect(ui->helpdocBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open help docment Path");

		});

	// 状态监控
	QObject::connect(handwareStatusUpdateTimer, &QTimer::timeout, this, [=]() {
		// 机器人状态检测
		if (robotConnected) {
			ui->robotConBtn->setIcon(QIcon(":/res/icon/comm.svg"));
		}
		else {
			ui->robotConBtn->setIcon(QIcon(":/res/icon/notcomm.svg"));
		}

		// 相机状态检测
		if (camConnected) {
			ui->camConBtn->setIcon(QIcon(":/res/icon/comm.svg"));
		}
		else {
			ui->camConBtn->setIcon(QIcon(":/res/icon/notcomm.svg"));
		}

		// 熔池相机状态检测
		if (meltCamConnected) {
			ui->meltConStatusBtn->setIcon(QIcon(":/res/icon/comm.svg"));
		}
		else {
			ui->meltConStatusBtn->setIcon(QIcon(":/res/icon/notcomm.svg"));
		}

		// 激光状态检测
		if (laserConnected) {
			ui->laserConBtn->setIcon(QIcon(":/res/icon/comm.svg"));
		}
		else {
			ui->laserConBtn->setIcon(QIcon(":/res/icon/notcomm.svg"));
		}

		// 焊机状态检测
		if (weldMachineConneted) {
			ui->weldConBtn->setIcon(QIcon(":/res/icon/comm.svg"));
		}
		else {
			ui->weldConBtn->setIcon(QIcon(":/res/icon/notcomm.svg"));
		}

		});
	flushSceneTimer->start(500);

	// 全局功能类
	connect(ui->globalParaBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open Global parameter setting");

		});
	
	connect(ui->weldParaBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open Weld process parameter setting");
		this->createDialog<ExpertProcess>("ExpertProcess", true);
		});

	connect(ui->databaseBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Open data base parameter setting");

		});

	connect(ui->startSimBtn, &QToolButton::clicked, this, [=] {
		logInfo(QString("Action triggered: Simulation, isSimulation=%1").arg(isSimulation));
		if (wholeTrajectory.size() == 0) {
			logWarning("Trajectory points are not ready.");
			return;
		}

		if (!isSimulation) {
			ui->startSimBtn->setIcon(QIcon(":/res/icon/simStop.svg"));
			ui->startSimBtn->setText(QStringLiteral("停止仿真"));
			flushTrajTimer->start(10);
			isSimulation = true;
			m_counter = 0;
			logInfo("Simulation started.");
		}
		else {
			ui->startSimBtn->setIcon(QIcon(":/res/icon/simulation.svg"));
			ui->startSimBtn->setText(QStringLiteral("开始仿真"));
			flushTrajTimer->stop();
			isSimulation = false;
			m_counter = 0;
			logInfo("Simulation stopped.");
		}
	}, Qt::QueuedConnection);

	connect(ui->pauseSimBtn, &QToolButton::clicked, this, [=]() {
		logInfo(QString("Action triggered: Pause"));
		if (isPauseSim == false && flushTrajTimer->isActive()) {
			flushTrajTimer->stop();
			ui->pauseSimBtn->setIcon(QIcon(":/res/icon/continue.svg"));
			ui->pauseSimBtn->setText(QStringLiteral("继续仿真"));
			isPauseSim = true;
		}
		else if(isPauseSim == true && !flushTrajTimer->isActive()){
			flushTrajTimer->start(10);
			ui->pauseSimBtn->setIcon(QIcon(":/res/icon/pause_1.svg"));
			ui->pauseSimBtn->setText(QStringLiteral("暂停仿真"));
			isPauseSim = false;
		}
	}, Qt::QueuedConnection);

	connect(ui->resetSimBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: reset Simulation");
		configurationModel->setData(this->mdl->getHomePosition());
		});

	connect(ui->downGifBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Record");
		this->createDialog<Record>("Record", true);
		});

	connect(ui->startWeldBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Start Welding flow");

		});

	connect(ui->pauseWeldBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Pause Welding flow");

		});

	connect(ui->stopWeldBtn, &QToolButton::clicked, [=] {
		logInfo("Action triggered: Stop Welding flow");

		});

	connect(ui->goHomeBtn, &QToolButton::clicked, this, [this]() {
		logInfo("Action triggered: RobotGoHome");
		if (flushTrajTimer->isActive()) {
			logInfo("Robot running weld trajectory, cannot go home now, please stop first!");
			return;
		}
		if (flushHomeTimer && flushHomeTimer->isActive()) {
			flushHomeTimer->stop();
			ui->actionPause->setIcon(QIcon(":/res/icon/continue.svg"));
		}
		this->robotGoHome();
		}, Qt::QueuedConnection);

	//***********************************************************************
	connect(ui->actionExecTraj, &QAction::triggered, this, [=]() {
		logInfo("Action triggered: ExecTraj");
		if (!flushHomeTimer->isActive()) {
			logInfo("Robot going home now, please wait robot arrive home first!");
			return;
		}
		if (!ui->realRobotCheck->isChecked()) {
			logWarning("Not connected to the real robot.");
			return;
		}
		if (wholeTrajectory.size() == 0) {
			logWarning("Trajectory points are not ready.");
			return;
		}
		});

	connect(ui->weldTableWidget->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onWeldSelected, Qt::QueuedConnection);

	connect(flushTrajTimer, &QTimer::timeout, this, [=]() { execSimulation(m_counter); m_counter++; });

	connect(flushHomeTimer, &QTimer::timeout, this, [=]() { execReturnHome(m_counter_home); m_counter_home++; });


	// help
	About* about = new About(this);
	ShortcutKey* shortcutKey = new ShortcutKey(this);
	connect(ui->actionAbout, &QAction::triggered, [this, about]() {
		logInfo("Action triggered: About");
		about->show();
		});

	// robot control panel
	connect(ui->toolCoordinateBtn, &QRadioButton::toggled, this, &MainWindow::changeRobotCoorBtn);
	connect(ui->userCoordinateBtn, &QRadioButton::toggled, this, &MainWindow::changeRobotCoorBtn);

	connect(ui->calibCheckBtn, &QCheckBox::toggled, [=](bool checked) {
		logInfo(QString("Button toggled: CalibCheck checked=%1").arg(checked));
		if (checked) calibNumCount = 0;
		});

	proworker = std::make_unique<ProcessWorker>();
	proworker->moveToThread(m_thread_prowork);
	connect(m_thread_prowork, &QThread::finished, this, &MainWindow::deleteLater);
	connect(this, &MainWindow::triggerVisionPro, this->proworker.get(), &ProcessWorker::triggerVisionPro);
	connect(this->proworker.get(), &ProcessWorker::sendTargetTransform, this, [=](const Eigen::Affine3f& targetTrans) {
		if (!loadShape) return;
		const Eigen::Matrix4f& mat = targetTrans.matrix();
		gp_Trsf T0;
		T0.SetValues(
			mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
			mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
			mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3)
		);
		loadShape->SetLocalTransformation(T0);
		myOccView->getContext()->Display(loadShape, Standard_False);
		myOccView->Redraw();
		updateAisRL();
		});
	connect(this->proworker.get(), &ProcessWorker::sendMsQ, [this](int level, const QString& errorMsg) {
		switch (level)
		{
		case 0:
			logInfo(errorMsg);
			break;
		case 1:
			logWarning(errorMsg);
			break;
		case 2:
			logError(errorMsg);
			break;
		default:
			break;
		}
		});
	connect(ui->loadVisionConfigBtn, &QPushButton::clicked, this, [this]() {
		logInfo("Button clicked: LoadVisionConfig");
		QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/visionConfig/", tr("Json File (*.Json );所有文件 (*)"));
		if (!filePath.isEmpty()) {
			ui->visionConfigEdit->setText(filePath);
		}
		else {
			logWarning(tr("No valid JSON file selected."));
			return;
		}
		proworker->loadVisionConfig(filePath.toStdString());
		}, Qt::QueuedConnection);

	m_thread_prowork->start();
}

/** @brief 重建 SD_M 相机实例并重连 UI 与点云/标定回调。 */
void MainWindow::InitializeCamera() {

	if (camera != nullptr) {
		disconnect(camera.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, camera.get(), nullptr);
		camera->reset();
		camera.release()->deleteLater();
	}

	camType = CameraFactory::CameraType::SD_M;
	camera = CameraFactory::createCamera(camType);

	// 相机信号绑定（因为上面已经断开了旧相机的连接，这里直接绑定新相机即可，不会重复）
	connect(this->camera.get(), &ICamera::sendCamModel, this, [=](const QString& camModel) {
		if (camModel == "") {
			logError(tr("Camera model list is empty."));
			return;
		}
		ui->cam_cbox_device->addItem(camModel);
		ui->cam_cbox_device->setCurrentIndex(0);
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamInfo, this, [=](const QString& info) {
		if (info.isEmpty()) {
			logError(tr("Camera info is empty."));
			return;
		}
		// ui->cam_txt_info->append(info);
		logInfo(info);
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamConnectStatus, this, [=](const bool& status) {
		if (status == false) {
			ui->cam_btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->cam_btn_connect->setText("connect");
			ui->cam_btn_search->setEnabled(true);
			camConnected = false;
		}
		else {
			ui->cam_btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->cam_btn_connect->setText("discon");
			ui->cam_btn_search->setEnabled(false);
			camConnected = true;
		}
		}, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamCloud, this, &MainWindow::GetCamData, Qt::QueuedConnection);

	connect(this->camera.get(), &ICamera::sendCamRunningStatus, this, [=](const bool& status) {
		this->camCaptureRunning = status;
		if (status) {
			ui->cam_btn_capture->setCheckable(true);
			ui->cam_btn_capture->setChecked(true);
			ui->cam_btn_capture->setStyleSheet(R"(
                QPushButton:checked {
                    background-color: #6a6a6a;
                }
            )");
		}
		else {
			ui->cam_btn_capture->setCheckable(false);
			ui->cam_btn_capture->setChecked(false);
		}
		}, Qt::QueuedConnection);

	// 【核心修改 2】：UI 按钮的 Lambda 连接，采用“先断开，再连接”策略
	// 无论 InitializeCamera 被点击多少次，按钮点击事件都只会触发一次

	disconnect(ui->cam_btn_search, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_search, &QPushButton::clicked, this->camera.get(), [&]() {
		logInfo("Button clicked: Camera Search");
		if (this->camera) {
			this->camera->searchDevice();
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_connect, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_connect, &QPushButton::clicked, this->camera.get(), [&]() {
		logInfo("Button clicked: Camera Connect");
		if (this->camera) {
			int index = ui->cam_cbox_device->currentIndex();
			this->camera->connectDevice(ui->camJsonEdit->text().toStdString());
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_capture, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_capture, &QPushButton::clicked, this, [=]() {
		logInfo("Button clicked: Camera Capture");
		json camConfig;
		std::ifstream file(ui->camJsonEdit->text().toStdString().c_str());
		try {
			file >> camConfig;
			file.close();
		}
		catch (const std::exception& e) {
			file.close();
			logError(tr("Failed to open camera config file: %1").arg(e.what()));
			return;
		}
		file.close();
		if (this->camCaptureRunning == false) {
			// 硬触发模式
			if (camConfig["trigger_mode"] == 1 && camConfig["capture_mode"] == 64) {
				this->camera->captureFixedLineScan(ui->camJsonEdit->text().toStdString());
			}
			else { // 面阵或摆动线扫
				this->camera->captureDevice(camConfig["capture_mode"] <= 16);
			}
		}
		else {
			if (camConfig["trigger_mode"] == 1 && camConfig["capture_mode"] == 64) {
				this->camera->stopFixedLineScan();
			}
		}
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_add, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_add, &QPushButton::clicked, this, [this]() {
		logInfo("Button clicked: Camera Add");
		if (this->camera) this->camera->add();
		}, Qt::QueuedConnection);

	disconnect(ui->cam_btn_reset, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_reset, &QPushButton::clicked, this->camera.get(), &ICamera::reset, Qt::QueuedConnection);

	disconnect(ui->cam_btn_cover, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->cam_btn_cover, &QPushButton::clicked, this, [=]() {
		QString btn_text = ui->cam_btn_cover->text();
		if (btn_text == "opencover") {
			this->camera.get()->opencover();
			ui->cam_btn_cover->setText("closecover");
			ui->cam_btn_cover->setIcon(QIcon(":/res/icon/closecover.svg"));
		}
		else if (btn_text == "closecover") {
			this->camera.get()->closecover();
			ui->cam_btn_cover->setText("opencover");
			ui->cam_btn_cover->setIcon(QIcon(":/res/icon/opencover.svg"));
		}
		}, Qt::QueuedConnection);

	disconnect(ui->loadCamJsonBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->loadCamJsonBtn, &QPushButton::clicked, this, [=]() {
		logInfo("Button clicked: LoadCamJson");
		QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "./config/cam/", tr("Json File (*.Json );所有文件 (*)"));
		if (!filePath.isEmpty()) {
			ui->camJsonEdit->setText(filePath);
		}
		else {
			logWarning(tr("No valid JSON file selected."));
			return;
		}
		if (this->camConnected) {
			ui->cam_btn_connect->clicked();
		}
		}, Qt::QueuedConnection);

	this->camera->setCloudTree(ui->cloudtree);
	this->camera->setCloudView(ui->cloudview);
	this->camera->setConsole(ui->console);

	ui->cam_cbox_device->clear();

	logInfo(tr("Camera initialized successfully."));
}

/** @brief 创建激光设备在工作线程中运行并绑定采集与图像显示。 */
void MainWindow::InitializeLaser() {
	if (laser != nullptr) {
		disconnect(laser.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, laser.get(), nullptr);
		laser.release()->deleteLater();
	}

	if (m_thread_laserwork->isRunning()) {
		m_thread_laserwork->quit();
		m_thread_laserwork->wait();
	}

	laserType = LaserFactory::LaserType::ILaser_PF400;
	laser = LaserFactory::createLaser(laserType);

	laser->moveToThread(m_thread_laserwork);
	// connect(m_thread_laserwork, &QThread::finished, this->laser.get(), &QObject::deleteLater);

	disconnect(ui->laserSearchBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->laserSearchBtn, &QPushButton::clicked, laser.get(), &ILaser::searchDevice, Qt::QueuedConnection);

	disconnect(ui->connectLaserBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->connectLaserBtn, &QPushButton::clicked, this, [this]() {
		logInfo(QString("Button clicked: Laser Connect, ip=%1").arg(ui->laserIpComb->currentText()));
		const std::string ip = ui->laserIpComb->currentText().toStdString();
		if (laser) {
			QMetaObject::invokeMethod(laser.get(), [laser = laser.get(), ip]() {
				laser->connectDevice(ip);
				}, Qt::QueuedConnection);
		}
		}, Qt::QueuedConnection);

	disconnect(ui->openLaserBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->openLaserBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		logInfo(QString("Button clicked: OpenLaser, text=%1").arg(ui->openLaserBtn->text()));
		if (this->laser) {
			this->laser->openLaser(ui->openLaserBtn->text().toStdString() == "Laser");
		}
		}, Qt::QueuedConnection);

	disconnect(ui->openCamBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->openCamBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		logInfo(QString("Button clicked: OpenLaserCam, text=%1").arg(ui->openCamBtn->text()));
		if (this->laser) {
			this->laser->openSensor(ui->openCamBtn->text().toStdString() == "Cam");
		}
		}, Qt::QueuedConnection);

	disconnect(ui->capDataBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->capDataBtn, &QPushButton::clicked, this->laser.get(), [&]() {
		logInfo(QString("Button clicked: CapLaserData, text=%1").arg(ui->capDataBtn->text()));
		if (this->laser) {
			this->laser->captureDevice(ui->capDataBtn->text().toStdString() == "Capture");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendConStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->connectLaserBtn->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->connectLaserBtn->setText("connect");
			ui->laserSearchBtn->setEnabled(true);
			laserConnected = false;
		}
		else {
			ui->connectLaserBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->connectLaserBtn->setText("discon");
			ui->laserSearchBtn->setEnabled(false);
			laserConnected = true;
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendLaserStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->openLaserBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->openLaserBtn->setText("Laser");
		}
		else {
			ui->openLaserBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->openLaserBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendSensorStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->openCamBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->openCamBtn->setText("Cam");
		}
		else {
			ui->openCamBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->openCamBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendCaptureStatus, this, [&](const bool& m_enable) {
		if (!m_enable) {
			ui->capDataBtn->setIcon(QIcon(":/res/icon/start_.svg"));
			ui->capDataBtn->setText("Capture");
		}
		else {
			ui->capDataBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
			ui->capDataBtn->setText("stop");
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendMsg, this, [&](const int& level, const QString& msg) {
		switch (level) {
		case 0:
			logInfo(msg);
			break;
		case 1:
			logWarning(msg);
			break;
		case 2:
			logError(msg);
			break;
		default:
			break;
		}
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendLaserIp, this, [&](const QString& camIp) {
		ui->laserIpComb->clear();
		ui->laserIpComb->addItem(camIp);
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendCaptureImage, this, [=](const cv::Mat& image) {
		cv::Mat rgbMat;
		cv::cvtColor(image, rgbMat, cv::COLOR_BGR2RGB);
		QImage qImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
		QPixmap pixmap = QPixmap::fromImage(qImage);
		QPixmap scaledPixmap = pixmap.scaled(ui->CaptureImageLabel->size() - QSize(2, 2), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		/*ui->CaptureImageLabel->setPixmap(scaledPixmap);
		ui->CaptureImageLabel->setAlignment(Qt::AlignCenter);*/
		setZoomableImage(ui->CaptureImageLabel, scaledPixmap.toImage(), QStringLiteral("激光相机"));
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::sendMonitorImage, this, [=](const cv::Mat& image) {
		cv::Mat rgbMat;
		cv::cvtColor(image, rgbMat, cv::COLOR_BGR2RGB);
		QImage qImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
		QPixmap pixmap = QPixmap::fromImage(qImage);
		QPixmap scaledPixmap = pixmap.scaled(ui->MonitorImageLabel->size() - QSize(2, 2), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		/*ui->MonitorImageLabel->setPixmap(scaledPixmap);
		ui->MonitorImageLabel->setAlignment(Qt::AlignCenter);*/
		setZoomableImage(ui->MonitorImageLabel, scaledPixmap.toImage(), QStringLiteral("监控相机"));
		}, Qt::QueuedConnection);

	connect(this->laser.get(), &ILaser::send3DContour, this, [=](const ct::Cloud::Ptr& contour) {
		ui->laser3DWidget->removeAllCoordinateSystems();
		ui->laser3DWidget->removeAllPointClouds();
		ui->laser3DWidget->addPointCloud(contour);
		ui->laser3DWidget->addCoordinateSystem(ct::Coord("cloud", 100.0, Eigen::Affine3f::Identity()));
		ui->laser3DWidget->update();

		Eigen::Vector3f translation(91.0675, -32.789, -157.778);
		Eigen::Vector3f rot_vec(1.12473, -1.59023, -1.33706);
		Eigen::Affine3f pose = Eigen::Translation3f(translation) * Eigen::AngleAxisf(rot_vec.norm(), rot_vec.normalized());
		ui->laser3DWidget->setViewerPose(pose);
		}, Qt::QueuedConnection);

	m_thread_laserwork->start();

	ui->laserIpComb->clear();

	logInfo(tr("Laser sensor initialized successfully."));
}

/** @brief 初始化 Profinet 通信、真机状态反馈与关节/笛卡尔点动。 */
void MainWindow::InitializeRobot()
{
	logInfo("Button clicked: InitializeRobot");
	m_IsCartMove = false;
	disconnect(this, &MainWindow::sendTrajectory, this, nullptr);
	disconnect(operationalModel, &OperationalModel::sendConfigurationData, this, nullptr);

	if (m_comm != nullptr) {
		disconnect(m_comm.get(), nullptr, this, nullptr);
		disconnect(this, nullptr, m_comm.get(), nullptr);
		if (m_thread_rbworker->isRunning()) {
			ProfinetComm* raw = m_comm.release();
			QMetaObject::invokeMethod(raw, [raw]() {
				raw->releaseProfinetCard();
				raw->deleteLater();
				}, Qt::BlockingQueuedConnection);
			m_thread_rbworker->quit();
			m_thread_rbworker->wait();
		}
		else if (m_comm->thread() == QThread::currentThread()) {
			m_comm.reset();
		}
	}
	else if (m_thread_rbworker->isRunning()) {
		m_thread_rbworker->quit();
		m_thread_rbworker->wait();
	}

	m_comm = std::make_unique<ProfinetComm>();
	m_comm->moveToThread(m_thread_rbworker);

	auto applyRobotLinkUi = [this](bool connected) {
		if (connected) {
			ui->robotConnectBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
			ui->robotConnectBtn->setText("discon");
			logInfo("Robot connected successfully.");
			ui->realRobotCheck->setChecked(true);
			ui->virtualRobotCheck->setChecked(false);
		}
		else {
			ui->robotConnectBtn->setIcon(QIcon(":/res/icon/connect.svg"));
			ui->robotConnectBtn->setText("connect");
			logInfo("Robot disconnected successfully.");
			ui->realRobotCheck->setChecked(false);
			ui->virtualRobotCheck->setChecked(true);
		}
	};

	// 更新机器人连接状态
	connect(m_comm.get(), &ProfinetComm::sigConnected, this, [this, applyRobotLinkUi](const bool& m_connected) {
		robotConnected = m_connected;
		applyRobotLinkUi(robotConnected);
		});

	// 返回机械臂当前状态量
	connect(m_comm.get(), &ProfinetComm::sigTaskAck, this, [this](const TaskAck& data) {
		
		});

	connect(m_comm.get(), &ProfinetComm::sigRobotData, this, [this](const RobotGetData& data) {
		if (m_IsCartMove)
			return;
		QMutexLocker locker(&g_robotMutex);
		g_robotData = data;
		rl::math::Vector rlVec = this->mdl->getHomePosition();
		rlVec[0] = g_robotData.E1;
		rlVec[1] = g_robotData.SJ1 / 180.0 * M_PI;
		rlVec[2] = g_robotData.SJ2 / 180.0 * M_PI;
		rlVec[3] = g_robotData.SJ3 / 180.0 * M_PI;
		rlVec[4] = g_robotData.SJ4 / 180.0 * M_PI;
		rlVec[5] = g_robotData.SJ5 / 180.0 * M_PI;
		rlVec[6] = g_robotData.SJ6 / 180.0 * M_PI;
		this->configurationModel->setData(rlVec);
		}, Qt::QueuedConnection);

	connect(m_comm.get(), &ProfinetComm::sigInfoMsg, this, [this](const QString& msg) {
		logInfo(msg);
		}, Qt::QueuedConnection);

	connect(m_comm.get(), &ProfinetComm::sigErrMsg, this, [this](const QString& msg) {
		logError(msg);
		}, Qt::QueuedConnection);

	disconnect(ui->robotConnectBtn, &QPushButton::clicked, nullptr, nullptr);
	connect(ui->robotConnectBtn, &QPushButton::clicked, this, [this]() {
		logInfo("Button clicked: RobotConnect");
		ProfinetComm* comm = m_comm.get();
		if (!comm) {
			logWarning("RobotConnect ignored: communicator is null.");
			return;
		}

		if (ui->robotConnectBtn->text() == QLatin1String("discon")) {
			comm->releaseProfinetCard();
			return;
		}

		comm->initProfinet();
		});

	auto commandFromModel = [this](const rl::math::Vector& joints) {
		
	};

	struct JointBtn { QPushButton* btn; int axis; int dir; };
	const JointBtn items[] = {
		{ ui->E1SubBtn, 0, -1 }, { ui->E1AddBtn, 0, +1 },
		{ ui->J1SubBtn, 1, -1 }, { ui->J1AddBtn, 1, +1 },
		{ ui->J2SubBtn, 2, -1 }, { ui->J2AddBtn, 2, +1 },
		{ ui->J3SubBtn, 3, -1 }, { ui->J3AddBtn, 3, +1 },
		{ ui->J4SubBtn, 4, -1 }, { ui->J4AddBtn, 4, +1 },
		{ ui->J5SubBtn, 5, -1 }, { ui->J5AddBtn, 5, +1 },
		{ ui->J6SubBtn, 6, -1 }, { ui->J6AddBtn, 6, +1 },
	};

	auto stepJoint = [this](int axis, int dir) {
		ProfinetComm* comm = m_comm.get();
		if (!comm)
			return;
	};
	for (const auto& it : items) {
		disconnect(it.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(it.btn, &QPushButton::clicked, this, [stepJoint, it]() {
			stepJoint(it.axis, it.dir);
			});
	}

	struct CartBtn { QPushButton* btn; int axis; int dir; };
	const CartBtn cartItems[] = {
		{ ui->XSubBtn, 0, -1 }, { ui->XAddBtn, 0, +1 },
		{ ui->YSubBtn, 1, -1 }, { ui->YAddBtn, 1, +1 },
		{ ui->ZSubBtn, 2, -1 }, { ui->ZAddBtn, 2, +1 },
		{ ui->ASubBtn, 3, -1 }, { ui->AAddBtn, 3, +1 },
		{ ui->BSubBtn, 4, -1 }, { ui->BAddBtn, 4, +1 },
		{ ui->CSubBtn, 5, -1 }, { ui->CAddBtn, 5, +1 },
	};
	auto stepCart = [this](int axis, int dir) {
		m_IsCartMove = true;
		this->operationalModel->stepAxis(axis, dir, ui->transStepSpinbox->value());
	};
	for (const CartBtn& item : cartItems) {
		disconnect(item.btn, &QPushButton::clicked, nullptr, nullptr);
		connect(item.btn, &QPushButton::clicked, this, [stepCart, item]() {
			stepCart(item.axis, item.dir);
			});
	}

	connect(this, &MainWindow::sendTrajectory, this, [this, commandFromModel](const rl::math::Vector& cur_joint) {
		ProfinetComm* comm = m_comm.get();
		if (!comm)
			return;
		if (cur_joint.size() < 7) {
			logWarning("Trajectory ignored: joint vector size < 7.");
			return;
		}
		//comm->slotSendRobotData(commandFromModel(cur_joint));
		});

	QObject::connect(operationalModel, &OperationalModel::sendConfigurationData, this, [this, commandFromModel](const rl::math::Vector& conData) {
		m_IsCartMove = false;
		ProfinetComm* comm = m_comm.get();
		if (!comm)
			return;
		if (conData.size() < 7) {
			logWarning("Cartesian command ignored: joint vector size < 7.");
			return;
		}
		//comm->slotSendRobotData(commandFromModel(conData));
		}, Qt::QueuedConnection);

	m_thread_rbworker->start();
	logInfo("Robot initialized successfully (Profinet worker ready).");
}

/** @brief 启动熔池相机线程并连接搜索、连接与抓图显示。 */
void MainWindow::InitializedMeltCam() {

	m_thread_melt->setObjectName(QStringLiteral("MvCameraThread"));
	m_worker_melt->moveToThread(m_thread_melt);

	connect(ui->meltCamSearchBtn, &QPushButton::clicked, m_worker_melt, &MvCameraWorker::searchDevices);

	connect(ui->meltCamGrabBtn, &QPushButton::clicked, this, [=]() {
		if (meltCamGraping) {
			m_worker_melt->stopGrabbing();
		}
		else {
			m_worker_melt->startGrabbing();
		}
		}, Qt::QueuedConnection);

	connect(ui->meltConBtn, &QPushButton::clicked, this, [=]() {
		if (meltCamConnected) {
			m_worker_melt->disconnectDevice();
		}
		else {
			m_worker_melt->connectDevice(ui->meltCamComb->currentIndex());
		}
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::imageReady, this, [=](const cv::Mat& image) {
		cv::Mat rgbMat;
		cv::cvtColor(image, rgbMat, cv::COLOR_BGR2RGB);
		QImage qImage(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
		QPixmap pixmap = QPixmap::fromImage(qImage);
		QPixmap scaledPixmap = pixmap.scaled(ui->meltCamLabel->size() - QSize(2, 2), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		/*ui->meltCamLabel->setPixmap(scaledPixmap);
		ui->meltCamLabel->setAlignment(Qt::AlignCenter);*/
		setZoomableImage(ui->meltCamLabel, scaledPixmap.toImage(), QStringLiteral("熔池相机"));
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::devicesFound, this, [=](const std::vector<QString>& device_info) {
		for (auto model : device_info) {
			ui->meltCamComb->addItem(model);
		}
		ui->meltCamComb->setCurrentIndex(0);
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::deviceConnected, this, [=]() {
		ui->meltConBtn->setIcon(QIcon(":/res/icon/disconnect.svg"));
		ui->meltConBtn->setText("discon");
		ui->meltCamComb->setEnabled(false);
		ui->meltCamSearchBtn->setEnabled(false);
		meltCamConnected = false;
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::deviceDisconnected, this, [=]() {
		ui->meltConBtn->setIcon(QIcon(":/res/icon/connect.svg"));
		ui->meltConBtn->setText("connect");
		ui->meltCamComb->setEnabled(true);
		ui->meltCamSearchBtn->setEnabled(true);
		meltCamConnected = true;
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::grabbingStarted, this, [=]() {
		ui->meltCamGrabBtn->setIcon(QIcon(":/res/icon/stop_1.svg"));
		ui->meltCamGrabBtn->setText("stop");
		meltCamGraping = false;
		}, Qt::QueuedConnection);
	connect(m_worker_melt, &MvCameraWorker::grabbingStopped, this, [=]() {
		ui->meltCamGrabBtn->setIcon(QIcon(":/res/icon/start_.svg"));
		ui->meltCamGrabBtn->setText("start");
		meltCamGraping = true;
		}, Qt::QueuedConnection);

	connect(m_worker_melt, &MvCameraWorker::statusMessage, this, [=](const QString& msg) {
		ui->console->print(ct::LOG_INFO, msg);
		}, Qt::QueuedConnection);
	connect(m_worker_melt, &MvCameraWorker::errorOccurred, this, [=](const QString& msg) {
		ui->console->print(ct::LOG_ERROR, msg);
		}, Qt::QueuedConnection);

	m_thread_melt->start();
	if (!m_thread_melt->isRunning()) {
		logInfo(QStringLiteral("熔池相机线程启动失败"));
		delete m_worker_melt;
		delete m_thread_melt;
		m_worker_melt = nullptr;
		m_thread_melt = nullptr;
	}
}

/**
 * @brief 处理相机回调数据。
 * @param data 点云与彩色图等。
 * @param id 点云/帧标识。
 */
void MainWindow::GetCamData(const CamData& data, const QString& id) {

	if (data.cloud != nullptr) {
		cam_captured = data.cloud->makeShared();
	}

	if (isRunOnce) {
		isRunOnce = false;
		try
		{
			logInfo(QString("Camera feedback: RunOnce capture received, cloudId=%1").arg(id));
			proworker->setInputSceneData(data.cloud);
			current_cloud_id = id;
			emit triggerVisionPro();
			logInfo("Vision process triggered from camera capture.");
		}
		catch (const std::exception& e)
		{
			logError(QString("Point-cloud process flow failed: %1").arg(e.what()));
			return;
		}
		return;
	}

	if (ui->checkSaveImageBtn->isChecked() && !data.color_Image.empty()) {
		QString timestamp = "./camera_data/" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
		safeImwrite(timestamp + ".png", data.color_Image);;
	}

	if (ui->calibCheckBtn->isChecked()) { // 记录图片和机械臂位姿数据
		std::string savePath;
		if (calibNumCount < 10) {
			savePath = "./config/Calibration/CalibImage/color_0" + std::to_string(calibNumCount) + ".bmp";
		}
		else {
			savePath = "./config/Calibration/CalibImage/color_" + std::to_string(calibNumCount) + ".bmp";
		}

		if (data.color_Image.empty()) {
			logError("Capture camera color image is empty, check the capture mode!");
			return;
		}
		safeImwrite(QString::fromStdString(savePath), data.color_Image);
		const std::string fileName = "./config/Calibration/calibPose.json";

		json root;
		{
			std::ifstream fin(fileName);
			if (fin.is_open()) {
				try {
					fin >> root;
					fin.close();
				}
				catch (...) {
					root = json::object(); 
					logError("Robot pose file not exist or broken, check the ./config/Calibration/calibPose.json!");
					return;
				}
			}
		}
		if (!root.is_object()) {
			root = json::object();
		}

		json record;

		QMutexLocker locker(&g_robotMutex);
		record = { g_robotData.X, g_robotData.Y, g_robotData.Z, g_robotData.A, g_robotData.B, g_robotData.C };
		root[(calibNumCount < 10 ? "0" : "") + std::to_string(calibNumCount)] = record;
		std::ofstream fout(fileName);
		if (fout.is_open()) {
			fout << root.dump(4); // 缩进 4 空格，便于阅读
			fout.close();
		}
		calibNumCount++;
		if (calibNumCount == 17) {
			logWarning("Calibration capture limit reached (17 frames).");
			calibNumCount = 0;
			return;
		}
		logInfo(QString("The %1 camera frame and robot data record done, please adjust next robot!").arg(calibNumCount + 1));
	}

	return;
}

/**
 * @brief 切换应用主题样式表。
 * @param index 0=暗色，1=MacOS，2=Ubuntu。
 */
void MainWindow::changeTheme(int index)
{
	QFile qss;
	switch (index)
	{
	case 0:
		qss.setFileName(":/res/theme/dark.qss");
		qss.open(QFile::ReadOnly);
		qApp->setStyleSheet(qss.readAll());
		qss.close();
		ui->statusBar->showMessage(tr("Dark Theme"), 2000);
		logInfo("Theme changed to Dark.");
		break;
	case 1:
		qss.setFileName(":/res/theme/MacOS.qss");
		qss.open(QFile::ReadOnly);
		qApp->setStyleSheet(qss.readAll());
		qss.close();
		ui->statusBar->showMessage(tr("MacOS Theme"), 2000);
		logInfo("Theme changed to MacOS.");
		break;
	case 2:
		qss.setFileName(":/res/theme/Ubuntu.qss");
		qss.open(QFile::ReadOnly);
		qApp->setStyleSheet(qss.readAll());
		qss.close();
		ui->statusBar->showMessage(tr("Ubuntu Theme"), 2000);
		logInfo("Theme changed to Ubuntu.");
		break;
	}
}

/**
 * @brief 切换界面语言。
 * @param index 0=英文，1=中文。
 */
void MainWindow::changeLanguage(int index)
{
	switch (index)
	{
	case 0:
		if (translator != nullptr) {
			qApp->removeTranslator(translator);
			ui->retranslateUi(this);
			logInfo("Language switched to English.");
		}
		break;
	case 1:
		if (translator == nullptr) {
			translator = new QTranslator;
			translator->load(":/res/trans/zh_CN.qm");
		}
		qApp->installTranslator(translator);
		ui->retranslateUi(this);
		logInfo("Language switched to Chinese.");
		break;
	}
}

/**
 * @brief 记录工具/用户坐标系切换。
 * @param checked 触发切换的单选钮是否为选中状态。
 */
void MainWindow::changeRobotCoorBtn(bool checked) {
	if (!checked) return;

	QRadioButton* senderBtn = qobject_cast<QRadioButton*>(sender());
	if (!senderBtn) return;

	int index = senderBtn == ui->toolCoordinateBtn ? 1 : 2;
	std::string result;
	switch (index) {
	case 1:
		result = "切换工具坐标系.";
		logInfo(QString::fromLocal8Bit(result.c_str()));
		break;
	case 2:
		result = "切换用户坐标系.";
		logInfo(QString::fromLocal8Bit(result.c_str()));
		break;
	default:
		break;
	}
}

/**
 * @brief 关闭前弹出确认对话框。
 * @param event 关闭事件。
 */
void MainWindow::closeEvent(QCloseEvent* event)
{
	logInfo("Close requested: Exit confirmation dialog shown.");
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this,
		tr(u8"退出程序"),
		tr(u8"确定要退出程序吗？"),
		QMessageBox::Yes | QMessageBox::No,
		QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		logInfo("Exit confirmed; closing application.");
		event->accept();
	}
	else {
		logInfo("Exit cancelled by user.");
		event->ignore();
	}
}

/** @brief 单次视觉：触发采集或从点云树/PLY 文件启动 ProcessWorker。 */
void MainWindow::actionStart_triggered() {
	logInfo("Action triggered: Start vision process");
	isRunOnce = true;
	if (camConnected) {
		ui->cam_btn_capture->click();
	}
	else if (ui->cloudtree->getSelectedClouds().size() != 0) {
		logInfo(tr("Using selected cloud-tree data for vision test flow."));
		std::vector<ct::Cloud::Ptr> selectPcd = ui->cloudtree->getSelectedClouds();
		proworker->setInputSceneData(selectPcd[0]);
		emit triggerVisionPro();
		isRunOnce = false;
	}
	else {
		logInfo(tr("Using local file data for vision test flow."));
		QString pcd_path = QFileDialog::getOpenFileName(this, tr("Open pcd File"), "", tr("pcd File (*.ply);所有文件 (*)"));
		ct::Cloud::Ptr cloud_in(new ct::Cloud);
		pcl::io::loadPLYFile(pcd_path.toStdString(), *cloud_in);
		proworker->setInputSceneData(cloud_in);
		emit triggerVisionPro();
		isRunOnce = false;
	}
}

/**
 * @brief 加载机器人 STL 连杆模型。
 * @param type 型号子目录名。
 */
void MainWindow::loadOcctModel(std::string type) {
	//return;
	links.clear();

	Interface_Static::SetIVal("read.step.product.mode", 0);  // 禁用产品结构严格检查
	Interface_Static::SetIVal("read.step.sequence.mode", 0); // 禁用序列化检查

	Interface_Static::SetIVal("read.step.memory.optimize", 1);  // 启用内存优化
	Interface_Static::SetIVal("read.step.progressive.mode", 1); // 启用渐进加载
	AIS_Shape* myais_shape;

	for (int i = 0; i < 4; ++i) {
		QString robot_file = QString::fromStdString("./robot_env/" + type + "/step/base" + std::to_string(i) + ".stl");
		TopoDS_Shape link = ReadModel::readStlModel(robot_file.toStdString().c_str());
		links.append(link);
		myais_shape = new AIS_Shape(link);
		displinks.append(myais_shape);
	}

	for (int i = 0; i < 9; i++)
	{
		QString linkPath = QString::fromStdString("./robot_env/" + type + "/step/link" + std::to_string(i) + ".stl");
		TopoDS_Shape link = ReadModel::readStlModel(linkPath.toStdString().c_str());
		links.append(link);
		myais_shape = new AIS_Shape(link);
		displinks.append(myais_shape);
	}

	occtUpdate();
}

/** @brief 正运动学更新 OCCT 连杆位姿与颜色并重绘视图。 */
void MainWindow::occtUpdate() {

	if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(this->mdl.get())) {
		kinematic->forwardPosition();
		for (std::size_t i = 0; i < this->sceneModel->getNumBodies(); ++i)
		{
			rl::math::Transform tf = kinematic->getBodyFrame(i);

			gp_Trsf T0;
			T0.SetValues(tf(0, 0), tf(0, 1), tf(0, 2), tf(0, 3) * 1000.0,
				tf(1, 0), tf(1, 1), tf(1, 2), tf(1, 3) * 1000.0,
				tf(2, 0), tf(2, 1), tf(2, 2), tf(2, 3) * 1000.0);

			displinks[i]->SetLocalTransformation(T0);

			if (i == 0) displinks[i]->SetColor(redColor);
			else if (i == 1 || i == 2 || i == 3) displinks[i]->SetColor(yellowColor);
			else if (i == 11) displinks[i]->SetColor(silveryColor);
			else if (i == 12) displinks[i]->SetColor(balckColor);
			else  displinks[i]->SetColor(kukaColor);

			myOccView->getContext()->Display(displinks[i], Standard_False);
			myOccView->getContext()->SetSelectionModeActive(displinks[i], -1, Standard_False);
		}
		if (isTheFirstDraw) {
			myOccView->fitAll();
			isTheFirstDraw = false;
		}
		else {
			myOccView->Redraw();
		}
	}
}

/** @brief 从 config.json 加载机型、TCP/激光标定并初始化机器人 OCCT 模型。 */
void MainWindow::robotWidgetInit() {
	logInfo("Robot widget initialization started.");

	/*tcp_transform(0, 0) = 0.70710678; tcp_transform(0, 1) = 0.0; tcp_transform(0, 2) = 0.70710678; tcp_transform(0, 3) = 79.90306;
	tcp_transform(1, 0) = 0.0; tcp_transform(1, 1) = 1.0; tcp_transform(1, 2) = 0.0; tcp_transform(1, 3) = 0.0;
	tcp_transform(2, 0) = -0.70710678; tcp_transform(2, 1) = 0.0; tcp_transform(2, 2) = 0.7071067; tcp_transform(2, 3) = 517.903;
	tcp_transform(3, 0) = 0.0; tcp_transform(3, 1) = 0.0; tcp_transform(3, 2) = 0.0; tcp_transform(3, 3) = 1.0;

	QString filename = QFileDialog::getOpenFileName(this, "", "", "All Formats (*.xml)");
	this->loadRobotWidget(filename);*/

	std::ifstream fin("./config/config.json");
	if (!fin) {
		logError("Failed to open config.json.");
		return;
	}
	json config = json::parse(fin);

	// 2. 读取机器人型号
	std::string ro_type = config["RoType"].get<std::string>();

	// 3. 读取 4x4 矩阵 -> rl::math::Transform
	tcp_transform.setIdentity();
	const auto& m = config["tcp_transform"];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			tcp_transform.matrix()(i, j) = m[i][j].get<double>();
		}
	}
	tcp_transform.translation() *= 0.001;

	laser_transform.setIdentity();
	const auto& laser_t = config["laser_transform"];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			laser_transform.matrix()(i, j) = laser_t[i][j].get<double>();
		}
	}
	laser_transform.translation() *= 0.001;

	QString filename;
	if (ro_type == "kuka16" || ro_type == "kuka20") {
		filename = QDir::currentPath() + QString::fromStdString("/robot_env/" + ro_type + "/" + ro_type + ".xml");
	}
	else {
		logError("Invalid robot type in config.json; please check the configuration.");
		return;
	}
	this->loadRobotWidget(filename);

	std::string type;
	if (filename.contains("kuka16")) {
		type = "kuka16";
	}
	else if (filename.contains("kuka20")) {
		type = "kuka20";
	}
	loadOcctModel(type);
}

/**
 * @brief 解析 RL XML 并构建规划场景与控制表格。
 * @param filename 机器人 XML 文件路径。
 */
void MainWindow::loadRobotWidget(const QString& filename) {

	QMutexLocker lock(&this->mutex);

	rl::xml::DomParser parser;
	rl::xml::Document document = parser.readFile(filename.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	document.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);

	rl::xml::Path path(document);
	this->scene = std::make_shared<rl::sg::ode::Scene>();

	rl::xml::NodeSet modelScene = path.eval("(/rl/plan|/rlplan)//model/scene").getValue<rl::xml::NodeSet>();
	this->scene->load(modelScene[0].getUri(modelScene[0].getProperty("href")));
	this->sceneModel = this->scene->getModel(0);
	if (this->scene->getNumModels() > 1) {
		this->sceneBody = this->scene->getModel(1)->create();
	}
	else {
		logWarning(u8"Scene model count is less than 2.");
		return;
	}

	rl::xml::NodeSet mdl = path.eval("(/rl/plan|/rlplan)//model/kinematics")
		.getValue<rl::xml::NodeSet>();
	rl::mdl::XmlFactory factory;

	std::shared_ptr<rl::mdl::Model> model =
		factory.create(mdl[0].getUri(mdl[0].getProperty("href")));

	this->mdl = std::dynamic_pointer_cast<rl::mdl::Dynamic>(model);

	this->mdl->setHomePosition(this->mdl->getHomePosition());
	this->mdl->setPosition(this->mdl->getHomePosition());
	this->mdl->forwardPosition();

	if (rl::sg::DistanceScene* scene = dynamic_cast<rl::sg::DistanceScene*>(this->scene.get())) {
		this->model = std::make_shared<rl::plan::DistanceModel>();
	}
	else if (rl::sg::SimpleScene* scene = dynamic_cast<rl::sg::SimpleScene*>(this->scene.get())) {
		this->model = std::make_shared<rl::plan::SimpleModel>();
	}
	else {
		throw std::runtime_error("selected engine does not support collision queries");
	}

	if (nullptr != this->mdl) {
		this->model->mdl = this->mdl.get();
	}

	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();

	this->verifier = std::make_shared<rl::plan::RecursiveVerifier>();
	this->verifier->delta = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta)").getValue<rl::math::Real>(1);
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/recursiveVerifier/delta/@unit)").getValue<std::string>()) {
		this->verifier->delta *= rl::math::DEG2RAD;
	}

	if (nullptr != this->verifier) {
		this->verifier->model = this->model.get();
	}

	this->optimizer.reset();
	this->optimizer = std::make_shared<rl::plan::AdvancedOptimizer>();
	rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast<rl::plan::AdvancedOptimizer*>(this->optimizer.get());
	advancedOptimizer->length = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/length)").getValue<rl::math::Real>(1);
	if ("deg" == path.eval("string((/rl/plan|/rlplan)//advancedOptimizer/length/@unit)").getValue<std::string>()) {
		advancedOptimizer->length *= rl::math::DEG2RAD;
	}
	advancedOptimizer->ratio = path.eval("number((/rl/plan|/rlplan)//advancedOptimizer/ratio)").getValue<rl::math::Real>(0.1f);

	if (nullptr != this->optimizer) {
		this->optimizer->model = this->model.get();
		this->optimizer->verifier = this->verifier.get();
	}

	this->sampler = std::make_shared<rl::plan::UniformSampler>();
	rl::plan::UniformSampler* uniformSampler = static_cast<rl::plan::UniformSampler*>(this->sampler.get());
	if (nullptr != this->sampler) {
		this->sampler->model = this->model.get();
	}

	rl::xml::NodeSet planners = path.eval("(/rl/plan|/rlplan)//addRrtConCon|(/rl/plan|/rlplan)//eet|(/rl/plan|/rlplan)//rrt|(/rl/plan|/rlplan)//rrtCon|(/rl/plan|/rlplan)//rrtConCon|(/rl/plan|/rlplan)//rrtConExt|(/rl/plan|/rlplan)//rrtDual|(/rl/plan|/rlplan)//rrtExtCon").getValue<rl::xml::NodeSet>();
	for (int i = 0; i < std::min(1, planners.size()); ++i) {
		rl::xml::Path path(document, planners[i]);
		this->planner = std::make_shared<rl::plan::AddRrtConCon>();
		rl::plan::AddRrtConCon* addRrtConCon = static_cast<rl::plan::AddRrtConCon*>(this->planner.get());
		addRrtConCon->alpha = path.eval("number(alpha)").getValue<rl::math::Real>(0.5f);
		addRrtConCon->delta = path.eval("number(delta)").getValue<rl::math::Real>(20);

		if ("deg" == path.eval("string(delta/@unit)").getValue<std::string>()) {
			addRrtConCon->delta *= rl::math::DEG2RAD;
		}

		addRrtConCon->epsilon = path.eval("number(epsilon)").getValue<rl::math::Real>(1.0e-1f);

		if ("deg" == path.eval("string(epsilon/@unit)").getValue<std::string>()) {
			addRrtConCon->epsilon *= rl::math::DEG2RAD;
		}

		addRrtConCon->lower = path.eval("number(lower)").getValue<rl::math::Real>(20);

		if ("deg" == path.eval("string(lower/@unit)").getValue<std::string>()) {
			addRrtConCon->lower *= rl::math::DEG2RAD;
		}

		addRrtConCon->radius = path.eval("number(radius)").getValue<rl::math::Real>(200);

		if ("deg" == path.eval("string(radius/@unit)").getValue<std::string>()) {
			addRrtConCon->radius *= rl::math::DEG2RAD;
		}

		addRrtConCon->sampler = this->sampler.get();
	}

	std::size_t nearestNeighborsSize = 1;
	if (rl::plan::RrtDual* rrtDual = dynamic_cast<rl::plan::RrtDual*>(this->planner.get())) {
		nearestNeighborsSize = 2;
	}

	for (::std::size_t i = 0; i < nearestNeighborsSize; ++i)
	{
		std::shared_ptr<rl::plan::NearestNeighbors> nearestNeighbors;
		std::shared_ptr<rl::plan::GnatNearestNeighbors> gnatNearestNeighbors = std::make_shared<rl::plan::GnatNearestNeighbors>(this->model.get());
		nearestNeighbors = gnatNearestNeighbors;
		this->nearestNeighbors.push_back(nearestNeighbors);
		if (rl::plan::Prm* prm = dynamic_cast<rl::plan::Prm*>(this->planner.get())) {
			prm->setNearestNeighbors(nearestNeighbors.get());
		}
		if (rl::plan::Rrt* rrt = dynamic_cast<rl::plan::Rrt*>(this->planner.get())) {
			rrt->setNearestNeighbors(nearestNeighbors.get(), i);
		}
	}
	this->planner->duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(100));
	this->planner->model = this->model.get();

	ConfigurationDelegate* configurationDelegate = new ConfigurationDelegate(this);
	this->configurationDelegates = configurationDelegate;

	this->configurationModel = new ConfigurationModel(this);

	QTableView* configurationView = new QTableView(this);
	configurationView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	configurationView->horizontalHeader()->hide();
	configurationView->setAlternatingRowColors(true);
	configurationView->setItemDelegate(configurationDelegate);
	configurationView->setModel(configurationModel);

	OperationalDelegate* operationalDelegate = new OperationalDelegate(this);
	this->operationalDelegates = operationalDelegate;

	this->operationalModel = new OperationalModel(this);

	QTableView* operationalView = new QTableView(this);
	operationalView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	operationalView->horizontalHeader()->hide();
	operationalView->setAlternatingRowColors(true);
	operationalView->setItemDelegate(operationalDelegate);
	operationalView->setModel(operationalModel);

	QFont tabFont = ui->robotControlWidget->tabBar()->font();
	tabFont.setFamily("Microsoft YaHei");
	tabFont.setPointSize(9);
	ui->robotControlWidget->tabBar()->setFont(tabFont);

	ui->robotControlWidget->tabBar()->setUsesScrollButtons(true);
	ui->robotControlWidget->addTab(configurationView, QStringLiteral("轴空间"));
	ui->robotControlWidget->addTab(operationalView, QStringLiteral("笛卡尔"));
	ui->robotControlWidget->addTab(ui->scrollArea, QStringLiteral("控制"));

	QObject::connect(
		configurationModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		operationalModel,
		SLOT(configurationChanged(const QModelIndex&, const QModelIndex&))
	);

	QObject::connect(
		operationalModel,
		SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
		configurationModel,
		SLOT(operationalChanged(const QModelIndex&, const QModelIndex&))
	);

	QObject::connect(ui->rotateStepSpinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		configurationDelegate, &ConfigurationDelegate::setSingleStep);

	QObject::connect(ui->transStepSpinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
		operationalDelegate, &OperationalDelegate::setSingleStep);

	//tcp_transform
	operationalModel->setToolTransform(tcp_transform);

	connect(ui->toolCoordinateBtn, &QRadioButton::toggled,
		operationalModel, &OperationalModel::setDisplayTcp);
	ui->userCoordinateBtn->setChecked(true);
	operationalModel->setDisplayTcp(false);

	configurationModel->setData(this->mdl->getHomePosition());

	QObject::connect(flushSceneTimer, &QTimer::timeout, this, &MainWindow::occtUpdate);
	flushSceneTimer->start(50);
}

/** @brief 导入 STEP/STP 工件并绑定 RL 碰撞体后显示。 */
void MainWindow::load3DModel() {
	logInfo("Action triggered: Load3DModel");

	ui->tabWidget->setCurrentIndex(0);

	QString filename = QFileDialog::getOpenFileName(this, "", "", "All Formats (*.stp | *.step)");
	QFileInfo fileInfo(filename);
	QString suffix = fileInfo.suffix().toLower();

	QMutexLocker locker(&mapMutex);
	Handle(AIS_ColoredShape) loadColorShape;
	if (suffix.compare(QStringLiteral("step"), Qt::CaseInsensitive) == 0 ||
		suffix.compare(QStringLiteral("stp"), Qt::CaseInsensitive) == 0) {
		ReadModel::ColoredModel colored = ReadModel::readStepModelWithColors(filename.toStdString().c_str());
		if (colored.shape.IsNull()) {
			return;
		}
		loadColorShape = ReadModel::makeDisplayShape(colored, grayColor);
	}
	else {
		logWarning(QStringLiteral("Unsupported import format; only STP or STEP is allowed."));
		return;
	}

	if (loadColorShape.IsNull()) {
		logError(QStringLiteral("Failed to read the imported model file."));
		return;
	}
	
	loadShape = loadColorShape.get();

	rlWorkpiece = bindAisShapeToRlBody(loadShape, this->scene->getModel(1));
	syncAisPoseToRlBody(loadShape, rlWorkpiece);

	myOccView->getContext()->Display(loadShape, Standard_False);
	if (isTheFirstDraw) {
		myOccView->fitAll();
		isTheFirstDraw = false;
	}
	else {
		myOccView->Redraw();
	}

	ui->console->print(ct::LOG_INFO, QString("Current scene model bodies num is %1.").arg(this->scene->getModel(1)->getNumBodies()));
}

/** @brief 将当前选中形体导出为 STL 或 STEP。 */
void MainWindow::saveModel() {
	logInfo("Action triggered: SaveModel");

	TopoDS_Shape currentSelectionShape = this->myOccView->getSelectShape();
	if (currentSelectionShape.IsNull()) {
		logWarning(QStringLiteral("No entity selected."));
		return;
	}

	ReadModel rm;
	QString fileName = QFileDialog::getSaveFileName(this, QStringLiteral("模型导出"), "", tr("STEP Files (*.step *.stp);;STL Files (*.stl)"));
	if (fileName.isEmpty()) {
		return;
	}

	QFileInfo fileInfo(fileName);
	QString suffix = fileInfo.suffix().toLower();
	bool success = false;
	if (suffix == "stl") {
		rm.writeStlModel(currentSelectionShape, fileName.toUtf8().constData());
		success = true;
	}
	else if (suffix == "step" || suffix == "stp") {
		rm.writeStepModel(currentSelectionShape, fileName.toUtf8().constData());
		success = true;
	}
	else {
		logWarning(QStringLiteral("Unsupported file format; please choose .stl or .step."));
		return;
	}
	if (success) {
		logInfo(QStringLiteral("File saved successfully: ") + fileName);
	}
	else {
		logError(QStringLiteral("Failed to save file: ") + fileName);
	}
}

/** @brief 移除 OCCT 选中对象并同步删除 RL 工件体。 */
void MainWindow::removeModel() {
	logInfo("Action triggered: RemoveModel");
	std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
	myOccView->getContext()->InitSelected();

	// 选取线段，获取端点
	while (myOccView->getContext()->MoreSelected())
	{
		Handle(AIS_InteractiveObject) aAis = myOccView->getContext()->SelectedInteractive();
		selectedObjects.push_back(aAis);
		myOccView->getContext()->NextSelected();
	}

	for (auto aAis : selectedObjects)
	{
		if (aAis.IsNull()) continue;
		if (loadShape != nullptr && rlWorkpiece != nullptr && aAis.get() == loadShape)
		{
			this->scene->getModel(1)->remove(rlWorkpiece);
			delete rlWorkpiece;
			rlWorkpiece = nullptr;
			loadShape = nullptr;
		}
		myOccView->getContext()->Remove(aAis, false);
	}
	myOccView->getContext()->UpdateCurrentViewer();

	ui->console->print(ct::LOG_INFO, QString("Current scene model bodies num is %1.").arg(this->scene->getModel(1)->getNumBodies()));
}

/**
 * @brief 排序焊缝边并填充参数表后调用轨迹规划。
 * @param selectShape 选中工件形体。
 * @param weldEdges 提取的焊缝边。
 */
void MainWindow::flushWeldList(const TopoDS_Shape& selectShape, const std::vector<TopoDS_Edge>& weldEdges) {
	logInfo(QString("Weld extract feedback: edgeCount=%1").arg(static_cast<int>(weldEdges.size())));
	ui->weldTableWidget->setRowCount(0);

	if (weldEdges.size() == 0) {
		logWarning(tr("No valid weld edges extracted."));
		return;
	}
	if (selectShape.IsNull()) {
		logWarning(tr("No valid model selected."));
		return;
	}

	m_selectShape = std::move(selectShape);
	std::vector<TopoDS_Edge> m_weldEdges = std::move(weldEdges);

	WeldingPathBuilder getPath(10.0, 1e-3);
	std::vector<ProcessedEdge> sortedEdges;
	sortedClusters.clear();
	getPath.SortEdges(m_weldEdges, sortedEdges, sortedClusters);
	if (sortedEdges.size() < 1) return;

	auto Pt2Qstring = [](const gp_Pnt& point, QString& result) {
		double x = point.X();
		double y = point.Y();
		double z = point.Z();
		QString xStr = QString::number(x, 'f', 3);
		QString yStr = QString::number(y, 'f', 3);
		QString zStr = QString::number(z, 'f', 3);
		result = QString("(%1, %2, %3)").arg(xStr).arg(yStr).arg(zStr);
	};

	int dataCount = sortedEdges.size();
	for (int i = 0; i < dataCount; i++) {
		ui->weldTableWidget->insertRow(i);

		QComboBox* pathCombo = new QComboBox();
		pathCombo->addItems({ "Line", "Circle" });
		pathCombo->setCurrentIndex(0);
		ui->weldTableWidget->setCellWidget(i, 0, pathCombo);

		QComboBox* craftCombo = new QComboBox();
		craftCombo->addItems({ QStringLiteral("I口"), QStringLiteral("V口"), QStringLiteral("角焊"), QStringLiteral("多层多道") });
		ui->weldTableWidget->setCellWidget(i, 1, craftCombo);

		ProcessedEdge current_edge = sortedEdges[i];

		QString ptString;
		Pt2Qstring(current_edge.StartPoint, ptString);
		QTableWidgetItem* coordItem_start = new QTableWidgetItem(ptString);
		coordItem_start->setTextAlignment(Qt::AlignCenter);
		coordItem_start->setFlags(coordItem_start->flags() & ~Qt::ItemIsEditable);
		ui->weldTableWidget->setItem(i, 2, coordItem_start);

		QDoubleSpinBox* StartOffset = new QDoubleSpinBox();
		StartOffset->setMaximum(200);
		StartOffset->setMinimum(30);
		StartOffset->setValue(50);
		StartOffset->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 3, StartOffset);

		Pt2Qstring(current_edge.EndPoint, ptString);
		QTableWidgetItem* coordItem_end = new QTableWidgetItem(ptString);
		coordItem_end->setTextAlignment(Qt::AlignCenter);
		coordItem_end->setFlags(coordItem_end->flags() & ~Qt::ItemIsEditable);
		ui->weldTableWidget->setItem(i, 4, coordItem_end);

		QDoubleSpinBox* EndOffset = new QDoubleSpinBox();
		EndOffset->setMaximum(200);
		EndOffset->setMinimum(30);
		EndOffset->setValue(50);
		EndOffset->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 5, EndOffset);

		QDoubleSpinBox* TransitionSpin = new QDoubleSpinBox();
		TransitionSpin->setMaximum(180);
		TransitionSpin->setMinimum(-180);
		TransitionSpin->setValue(-45);
		TransitionSpin->setDecimals(2);
		ui->weldTableWidget->setCellWidget(i, 6, TransitionSpin);
	}

	logInfo(tr("Weld extraction completed."));

	this->Trajectory();
	logInfo(tr("Exec trajectory planning."));
}

/**
 * @brief 根据表格选中行在 3D 视图绘制焊缝方向箭头。
 * @param selected 新选中区域。
 * @param deselected 取消选中的区域。
 */
void MainWindow::onWeldSelected(const QItemSelection& selected, const QItemSelection& deselected)
{
	Q_UNUSED(deselected);
	if (selected.indexes().isEmpty())
		return;

	int row = selected.indexes().first().row();
	logInfo(QString("Weld table selection feedback: row=%1").arg(row));

	QTableWidget* table = qobject_cast<QTableWidget*>(sender()->parent());
	if (!table)
		return;

	QTableWidgetItem* itemStart = table->item(row, 2); // 起点
	QTableWidgetItem* itemEnd = table->item(row, 4);   // 终点
	const QString start_point_string = itemStart ? itemStart->text() : "";
	const QString end_point_string = itemEnd ? itemEnd->text() : "";

	QRegularExpression re(R"(\(([\d.+-]+),\s*([\d.+-]+),\s*([\d.+-]+)\))");
	QRegularExpressionMatch match_start = re.match(start_point_string);
	QRegularExpressionMatch match_end = re.match(end_point_string);
	if (!match_start.hasMatch() || !match_end.hasMatch())
		return;

	const gp_Pnt m_start_point(match_start.captured(1).toDouble(),
		match_start.captured(2).toDouble(),
		match_start.captured(3).toDouble());
	const gp_Pnt m_end_point(match_end.captured(1).toDouble(),
		match_end.captured(2).toDouble(),
		match_end.captured(3).toDouble());

	Handle(AIS_Shape) blueArrow =
		myOccView->CreateBlueArrow(m_start_point, m_end_point, 2, 6, 4);

	Handle(AIS_InteractiveContext) ctx = myOccView->getContext();
	if (!m_weldArrow.IsNull()) {
		ctx->Remove(m_weldArrow, Standard_False);
		m_weldArrow.Nullify();
	}

	m_weldArrow = blueArrow;
	ctx->SetDisplayMode(m_weldArrow, 1, Standard_False);
	ctx->Display(m_weldArrow, Standard_False);
	myOccView->Redraw();
}

/** @brief 配置 IKWorker 线程与轨迹/回零/仿真相关信号连接。 */
void MainWindow::initSimulation() {
	ikwork->moveToThread(m_thread_ikwork);
	QObject::connect(m_thread_ikwork, &QThread::finished, ikwork, &QObject::deleteLater);
	ikwork->setKinematic(this->mdl);

	connect(ikwork, &IKWorker::progress, this, [=](int percent) {
		ui->trajProgressBar->setValue(percent);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::started, this, [=]() {
		setSceneFlushEnabled(false);
		ui->trajProgressBar->setRange(0, 100);
		ui->trajProgressBar->setValue(0);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished_return, this, [=](const std::vector<rl::math::Vector>& jointTrajectory,
		const std::vector<rl::math::Vector3>& toolPoints,
		const double& ratio) {
			m_jointTrajectory_home = jointTrajectory;
			showToolPoints(toolPoints);
			flushHomeTimer->start(20);
			logInfo(QString("Trajectory return-to-home completed, progress %1%").arg(QString::number(ratio)));
			setSceneFlushEnabled(true);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished_start, this,
		[=](const std::vector<rl::math::Vector>& jointTrajectory,
			const std::vector<rl::math::Vector3>& toolPoints,
			const double& ratio) {
				if (jointTrajectory.size() < 1) { return; }
				wholeTrajectory.clear();
				wholeTrajectory.insert(wholeTrajectory.end(), jointTrajectory.begin(), jointTrajectory.end());
				IKSolveParams params;
				params.trajectory = mergedTraj;
				params.T_flange_to_tcp = tcp_transform;
				params.q_initial = jointTrajectory.back();
				if (params.q_initial.size() == 0) {
					params.q_initial = this->mdl->getPosition();
				}
				params.constrainRail = true;
				params.railWindow = 0.01;
				params.timeoutMs = 500;
				setSceneFlushEnabled(true);
				QMetaObject::invokeMethod(ikwork, "doSolve", Qt::QueuedConnection, Q_ARG(IKSolveParams, params));
				showToolPoints(toolPoints);
				logInfo(QString("Trajectory home-to-start completed, progress %1%, points %2").arg(QString::number(ratio)).arg(static_cast<int>(toolPoints.size())));
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::finished, this, [=](const std::vector<rl::math::Vector>& jointTrajectory,
		const double& ratio, const DiscretePoint& start) {
			if (jointTrajectory.size() < 1) { return; }
			setSceneFlushEnabled(true);
			ui->trajProgressBar->setValue(100);
			logInfo(QString("Trajectory weld motion completed, progress %1%").arg(QString::number(ratio)));
			if (ui->virtualRobotCheck->isChecked()) {
				wholeTrajectory.insert(wholeTrajectory.end(), jointTrajectory.begin(), jointTrajectory.end());
			}
			else if (ui->realRobotCheck->isChecked()) {
				wholeTrajectory.push_back(jointTrajectory[0]);
				wholeTrajectory.push_back(jointTrajectory[jointTrajectory.size() - 1]);
			}
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::failed, this, [=](const QString& errorMessage) {
		setSceneFlushEnabled(true);
		logError(errorMessage);
		}, Qt::QueuedConnection);

	connect(ikwork, &IKWorker::aborted, this, [=]() {
		setSceneFlushEnabled(true);
		logError("Trajectory progress aborted.");
		}, Qt::QueuedConnection);

	m_thread_ikwork->start();
}

/**
 * @brief 按索引播放整条轨迹（仿真或下发真机）。
 * @param inter 当前轨迹点索引，由定时器槽递增。
 */
void MainWindow::execSimulation(int& inter) {

	if (wholeTrajectory.size() == 0 || inter >= wholeTrajectory.size()) {
		if (ui->virtualRobotCheck->isChecked()) {
			ui->startSimBtn->setIcon(QIcon(":/res/icon/simulation.svg"));
			ui->startSimBtn->setText(QStringLiteral("开始仿真"));
		}
		if (ui->realRobotCheck->isChecked()) {
			ui->actionExecTraj->setIcon(QIcon(":/res/icon/execTrajectory.svg"));
		}

		flushTrajTimer->stop();
		isSimulation = false;
		m_counter = 0;
		return;
	}

	rl::math::Vector current_joint = wholeTrajectory[inter];
	if (ui->realRobotCheck->isChecked()) {
		emit sendTrajectory(current_joint);
	}
	else if (ui->virtualRobotCheck->isChecked()) {
		configurationModel->setData(current_joint);
	}
}

/** @brief 异步请求 IKWorker 规划回 Home 关节轨迹。 */
void MainWindow::robotGoHome() {
	m_jointTrajectory_home.clear();

	IKReturnHomeParams p;
	p.q_current = this->mdl->getPosition();
	p.q_home = this->mdl->getHomePosition();
	p.T_flange_to_tcp = tcp_transform;
	p.jointStepRad = 2 * M_PI / 180.0;
	p.railStepLen = 0.01;
	p.timeoutMs = 500;
	// setSceneFlushEnabled(false);
	QMetaObject::invokeMethod(ikwork, "doReturnHome", Qt::QueuedConnection, Q_ARG(IKReturnHomeParams, p));
}

/**
 * @brief 按索引播放回零关节轨迹。
 * @param inter 当前回零路径索引，由定时器槽递增。
 */
void MainWindow::execReturnHome(int& inter) {
	if (m_jointTrajectory_home.size() == 0 || inter >= m_jointTrajectory_home.size()) {
		inter = 0;
		flushHomeTimer->stop();
		return;
	}
	rl::math::Vector current_joint = m_jointTrajectory_home[inter];

	if (ui->realRobotCheck->isChecked()) {
		emit sendTrajectory(current_joint);
	}
	else if (ui->virtualRobotCheck->isChecked()) {
		configurationModel->setData(current_joint);
	}
}

/**
 * @brief 为离散轨迹点显示局部坐标系与点标记。
 * @param trajPt 离散焊接轨迹点序列。
 */
void MainWindow::showWeldTrajPt(const std::vector<DiscretePoint>& trajPt) {
	if (trajPt.size() < 2) return;
	if (this->myOccView == nullptr) return;

	for (const auto& info : trajPt)
	{
		// 用 Trihedron 显示局部坐标系
		// gp_Ax2 需要 (origin, mainDir=Z, xDir)
		gp_Ax2 anAx2(info.position, info.zDir, info.xDir);
		Handle(Geom_Axis2Placement) anAxisPlacement =
			new Geom_Axis2Placement(anAx2);

		Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxisPlacement);
		aTrihedron->SetSize(0.005);

		// 设置坐标轴颜色：X-红 Y-绿 Z-蓝
		Handle(Prs3d_DatumAspect) aDatumAspect = new Prs3d_DatumAspect();
		aDatumAspect->LineAspect(Prs3d_DatumParts_XAxis)
			->SetColor(Quantity_NOC_RED);
		aDatumAspect->LineAspect(Prs3d_DatumParts_YAxis)
			->SetColor(Quantity_NOC_GREEN);
		aDatumAspect->LineAspect(Prs3d_DatumParts_ZAxis)
			->SetColor(Quantity_NOC_BLUE1);
		aTrihedron->Attributes()->SetDatumAspect(aDatumAspect);
		aDatumAspect->SetAxisLength(50, 50, 50);

		// 同时显示一个点标记（可选）
		Handle(Geom_CartesianPoint) aGeomPnt =
			new Geom_CartesianPoint(info.position);
		Handle(AIS_Point) anAISPoint = new AIS_Point(aGeomPnt);
		Handle(Prs3d_PointAspect) aPointAspect = new Prs3d_PointAspect(
			Aspect_TOM_O_PLUS, Quantity_NOC_YELLOW, 2);
		anAISPoint->Attributes()->SetPointAspect(aPointAspect);

		this->myOccView->getContext()->Display(aTrihedron, Standard_False);
		this->myOccView->getContext()->Display(anAISPoint, Standard_False);
	}
	this->myOccView->getContext()->UpdateCurrentViewer();
}

/** @brief 生成焊接离散轨迹、可视化并请求 IK 移至起点。 */
void MainWindow::Trajectory() {
	logInfo("Action triggered: Trajectory planning");

	if (m_selectShape.IsNull()) {
		logWarning(tr("No valid model selected for trajectory planning."));
		return;
	}
	mergedTraj.clear();
	std::vector<std::vector<DiscretePoint>> wholeTrajPt;
	WeldingPathBuilder getPath(10, 1e-3);
	constexpr double kSpacingMm = 5;
	constexpr double kRetractMm = 50;
	constexpr double kVerticalTiltDeg = 45.0;
	if (!getPath.BuildWeldingTrajectory(m_selectShape,
		sortedClusters,
		mergedTraj,
		kSpacingMm,
		kRetractMm,
		kVerticalTiltDeg)) {
		mergedTraj.clear();
		logError(tr("Trajectory plan failed!"));
		return;
	}
	showWeldTrajPt(mergedTraj);
	logInfo(tr("Trajectory point num: %1").arg(static_cast<int>(mergedTraj.size())));

	for (std::size_t i = 0; i < mergedTraj.size(); ++i)
	{
		const gp_Pnt& p = mergedTraj[i].position;
		mergedTraj[i].position.SetCoord(
			p.X() / 1000.0,
			p.Y() / 1000.0,
			p.Z() / 1000.0);
	}

	if (mergedTraj.size() > 2) {
		IKGoToStartParams p;
		p.q_home = this->mdl->getHomePosition();
		p.startPoint = mergedTraj[0];

		p.T_flange_to_tcp = tcp_transform;
		p.jointStepRad = 1 * M_PI / 180.0;
		p.railStepLen = 0.01;
		p.cartStepLen = 0.01;
		p.timeoutMs = 500;

		QMetaObject::invokeMethod(ikwork, "doGoToStart", Qt::QueuedConnection, Q_ARG(IKGoToStartParams, p));
	}
}

/**
 * @brief 确保目录存在后保存图像文件。
 * @param savePath 输出路径。
 * @param image 待保存图像。
 */
void MainWindow::safeImwrite(const QString& savePath, const cv::Mat& image) {
	QString dirPath = QFileInfo(savePath).absolutePath();

	QDir dir;
	if (!dir.exists(dirPath)) {
		if (!dir.mkpath(dirPath)) {
			logError(QStringLiteral("Failed to create output directory."));
			return;
		}
	}

	bool success = cv::imwrite(savePath.toStdString(), image);
	if (!success) {
		logError(QStringLiteral("Failed to save image."));
	}
	else {
		logInfo(QStringLiteral("Image saved successfully: %1").arg(savePath));
	}
}

/**
 * @brief 绘制工具中心路径线框。
 * @param toolPoints 工具点序列（米）。
 */
void MainWindow::showToolPoints(const std::vector<rl::math::Vector3>& toolPoints)
{
	if (this->myOccView == nullptr || toolPoints.size() < 2) return;

	if (!tool_aWire.IsNull()) {
		this->myOccView->getContext()->Remove(tool_aWire, Standard_False);
		tool_aWire.Nullify();
	}

	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);

	gp_Pnt prev(toolPoints[0](0) * 1000.0, toolPoints[0](1) * 1000.0, toolPoints[0](2) * 1000.0);
	int edgeCount = 0;
	for (std::size_t i = 1; i < toolPoints.size(); ++i) {
		const gp_Pnt p(toolPoints[i](0) * 1000.0, toolPoints[i](1) * 1000.0, toolPoints[i](2) * 1000.0);
		if (prev.Distance(p) <= Precision::Confusion())
			continue;
		BRepBuilderAPI_MakeEdge mk(prev, p);
		if (mk.IsDone()) {
			builder.Add(compound, mk.Edge());
			++edgeCount;
		}
		prev = p;
	}
	if (edgeCount < 1)
		return;

	tool_aWire = new AIS_Shape(compound);
	tool_aWire->SetColor(Quantity_NOC_GREEN);
	tool_aWire->SetWidth(2.0);
	tool_aWire->SetDisplayMode(AIS_WireFrame);
	this->myOccView->getContext()->Display(tool_aWire, Standard_False);
	this->myOccView->Redraw();
}

/** @brief 同步 AIS 工件位姿到 RL 碰撞体并打印平移。 */
void MainWindow::updateAisRL()
{
	if (loadShape == nullptr || rlWorkpiece == nullptr)
		return;
	QMutexLocker locker(&mutex);
	rl::math::Transform t = syncAisPoseToRlBody(loadShape, rlWorkpiece);
	ui->console->print(ct::LOG_INFO, QString("Current Model Position:%1, %2, %3.")
		.arg(t.translation().x()).arg(t.translation().y()).arg(t.translation().z()));
}

/** @brief 按当前 AIS 形体重建 RL 工件碰撞体。 */
void MainWindow::rebuildWorkpieceCollision()
{
	if (loadShape == nullptr || !scene || scene->getNumModels() < 2)
		return;
	QMutexLocker locker(&mutex);
	rl::sg::Model* env = scene->getModel(1);
	if (rlWorkpiece != nullptr) {
		env->remove(rlWorkpiece);
		delete rlWorkpiece;
		rlWorkpiece = nullptr;
	}
	rlWorkpiece = bindAisShapeToRlBody(loadShape, env);
}

/**
 * @brief 启停 OCCT 场景刷新定时器。
 * @param enabled true 启动 50ms 刷新，false 停止。
 */
void MainWindow::setSceneFlushEnabled(bool enabled)
{
	if (!flushSceneTimer) return;
	if (enabled) {
		if (!flushSceneTimer->isActive())
			flushSceneTimer->start(50);
	}
	else {
		flushSceneTimer->stop();
	}
}

/**
 * @brief 由两点及场景法向计算起止位姿。
 * @param start_end 起止点云（≥2 点）。
 * @param scene 场景点云。
 * @param radius 法向估计邻域半径。
 * @param pose_start 输出起点位姿。
 * @param pose_end 输出终点位姿。
 * @return 成功 true，否则 false。
 */
bool MainWindow::computeTwoPointPoses(const ct::Cloud::Ptr& start_end,
	const ct::Cloud::Ptr& scene,
	float radius,
	Eigen::Affine3f& pose_start,
	Eigen::Affine3f& pose_end)
{
	if (!start_end || start_end->size() < 2 || !scene || scene->empty())
		return false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*scene, *xyz);

	pcl::PointCloud<pcl::Normal> nrm;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyz);
	ne.setRadiusSearch(radius * 2);
	ne.setViewPoint(0.f, 0.f, 0.f);
	ne.compute(nrm);
	for (size_t i = 0; i < scene->size(); ++i) {
		scene->points[i].normal_x = nrm[i].normal_x;
		scene->points[i].normal_y = nrm[i].normal_y;
		scene->points[i].normal_z = nrm[i].normal_z;
	}

	auto axisZ = [&](const ct::PointXYZRGBN& q) {
		pcl::KdTreeFLANN<ct::PointXYZRGBN> tree;
		tree.setInputCloud(scene);
		std::vector<int> ids;
		std::vector<float> d;
		Eigen::Vector3f sum(0, 0, 0);
		if (tree.radiusSearch(q, radius, ids, d) > 0) {
			for (int i : ids)
				sum += Eigen::Vector3f(scene->points[i].normal_x,
					scene->points[i].normal_y,
					scene->points[i].normal_z);
		}
		else {
			sum = Eigen::Vector3f(q.normal_x, q.normal_y, q.normal_z);
		}
		if (!std::isfinite(sum.x()) || sum.squaredNorm() < 1e-12f)
			sum = Eigen::Vector3f::UnitZ();
		return Eigen::Vector3f(-sum.normalized());
	};
	auto makePose = [](const Eigen::Vector3f& t,
		const Eigen::Vector3f& z_in,
		const Eigen::Vector3f& y_in) {
			Eigen::Vector3f z = z_in.normalized();
			Eigen::Vector3f x = y_in.cross(z);
			if (x.squaredNorm() < 1e-12f)
				x = ((std::fabs(z.z()) < 0.9f) ? Eigen::Vector3f::UnitZ()
					: Eigen::Vector3f::UnitX()).cross(z);
			x.normalize();
			Eigen::Vector3f y = z.cross(x).normalized();
			Eigen::Affine3f T = Eigen::Affine3f::Identity();
			T.linear().col(0) = x;
			T.linear().col(1) = y;
			T.linear().col(2) = z;
			T.translation() = t;
			return T;
	};
	const auto& p0 = start_end->points[0];
	const auto& p1 = start_end->points[1];
	Eigen::Vector3f t0(p0.x, p0.y, p0.z), t1(p1.x, p1.y, p1.z);
	Eigen::Vector3f y_dir = t1 - t0;
	if (y_dir.squaredNorm() < 1e-12f) return false;
	pose_start = makePose(t0, axisZ(p0), y_dir);
	pose_end = makePose(t1, axisZ(p1), y_dir);
	return true;
}

/**
 * @brief 在 OCCT 中显示点云与起止坐标轴。
 * @param cloud 彩色点云，可空。
 * @param poseStart 起点位姿。
 * @param poseEnd 终点位姿。
 * @param axisLength 坐标轴长度。
 */
void MainWindow::DisplayCloudAndAxes(
	const ct::Cloud::Ptr& cloud,
	const Eigen::Affine3f& poseStart,
	const Eigen::Affine3f& poseEnd,
	Standard_Real axisLength)
{
	auto makeAxes = [axisLength](const Eigen::Affine3f& pose) {
		const Eigen::Vector3f t = pose.translation();
		const Eigen::Matrix3f R = pose.linear();
		Handle(AIS_Trihedron) tri = new AIS_Trihedron(
			new Geom_Axis2Placement(gp_Ax2(
				gp_Pnt(t.x(), t.y(), t.z()),
				gp_Dir(R(0, 2), R(1, 2), R(2, 2)),
				gp_Dir(R(0, 0), R(1, 0), R(2, 0)))));
		tri->SetSize(axisLength);
		tri->SetDrawArrows(Standard_True);
		const Standard_Real w = 3.0; // 线宽，按需改
		Handle(Prs3d_DatumAspect) asp = tri->Attributes()->DatumAspect();
		asp->LineAspect(Prs3d_DP_XAxis)->SetWidth(w);
		asp->LineAspect(Prs3d_DP_YAxis)->SetWidth(w);
		asp->LineAspect(Prs3d_DP_ZAxis)->SetWidth(w);
		return tri;
	};

	if (cloud && !cloud->empty()) {
		const int n = static_cast<int>(cloud->size());
		Handle(Graphic3d_ArrayOfPoints) pts =
			new Graphic3d_ArrayOfPoints(n, Graphic3d_ArrayFlags_VertexColor);
		for (int i = 0; i < n; ++i) {
			const auto& p = cloud->points[i];
			pts->AddVertex(gp_Pnt(p.x, p.y, p.z),
				Quantity_Color(p.r / 255.0, p.g / 255.0, p.b / 255.0, Quantity_TOC_sRGB));
		}
		Handle(AIS_PointCloud) aisCloud = new AIS_PointCloud();
		aisCloud->SetPoints(pts);
		aisCloud->Attributes()->SetPointAspect(
			new Prs3d_PointAspect(Aspect_TOM_POINT, Quantity_NOC_WHITE, 2.0));
		myOccView->getContext()->Display(aisCloud, Standard_False);
	}
	myOccView->getContext()->Display(makeAxes(poseStart), Standard_False);
	myOccView->getContext()->Display(makeAxes(poseEnd), Standard_False);
}


