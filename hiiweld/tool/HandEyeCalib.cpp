/**
 * @file HandEyeCalib.cpp
 * @brief HandEyeCalib 实现：OpenCV 标定、3D 一致性、VTK 预览。
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */
#include "HandEyeCalib.h"
#include "ui_HandEyeCalib.h"
#include "MathUtils.h"
#include <QDesktopServices> 
#include <QUrl>
#include <fstream>
#include "vtk_nlohmannjson.h"
#include VTK_NLOHMANN_JSON(json.hpp)
#include <QDateTime>
#include <QFileDialog>
#include <QCheckbox>

using json = nlohmann::json;

namespace {
    /** @brief cv R,t 转 Eigen Affine3d。 */
    Eigen::Affine3d cvRt2Affine(const cv::Mat& R, const cv::Mat& t) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c)
                T.linear()(r, c) = R.at<double>(r, c);
            T.translation()(r) = t.at<double>(r, 0);
        }
        return T;
    }

    Eigen::Affine3d cv4x4ToAffine(const cv::Mat& M) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                T.matrix()(r, c) = M.at<double>(r, c);
        return T;
    }

    Eigen::Affine3d cvRtToAffineNormalized(const cv::Mat& Rin, const cv::Mat& tin)
    {
        cv::Mat R, t;
        Rin.copyTo(R);
        tin.copyTo(t);
        if (R.rows == 3 && R.cols == 1) {
            cv::Mat R3;
            cv::Rodrigues(R, R3);
            R = R3;
        }
        if (t.rows == 1 && t.cols == 3)
            t = t.t();
        return cvRt2Affine(R, t);
    }

    bool Collinear(const Eigen::Matrix3Xd& pts)
    {
        const Eigen::Matrix3Xd c = pts.colwise() - pts.rowwise().mean();
        Eigen::JacobiSVD<Eigen::Matrix3Xd> svd(c, Eigen::ComputeThinU);
        return svd.singularValues().size() < 2 ||
            svd.singularValues()(1) < 1e-6 * svd.singularValues()(0);
    }

    /** @brief 点对点刚体变换 SVD。 */
    bool EstimateRigid(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& dst,
        Eigen::Affine3d& T)
    {
        if (src.cols() < 3) {
            return false;
        }
        if (Collinear(src) || Collinear(dst)) {
            return false;
        }

        const Eigen::Vector3d p = src.rowwise().mean();
        const Eigen::Vector3d q = dst.rowwise().mean();
        const Eigen::Matrix3d H = (src.colwise() - p) * (dst.colwise() - q).transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
        S(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
        const Eigen::Matrix3d R = svd.matrixV() * S * svd.matrixU().transpose();

        T = Eigen::Affine3d::Identity();
        T.linear() = R;
        T.translation() = q - R * p;
        if (!T.matrix().allFinite() || T.linear().determinant() < 0.0) {
            return false;
        }
        return true;
    }

    double FillErrors(const std::vector<Eigen::Vector3d>& pred,
        const std::vector<Eigen::Vector3d>& gt,
        std::vector<double>& errors)
    {
        errors.resize(pred.size());
        double ss = 0.0;
        for (std::size_t i = 0; i < pred.size(); ++i) {
            errors[i] = (pred[i] - gt[i]).norm();
            ss += errors[i] * errors[i];
        }
        return std::sqrt(ss / static_cast<double>(pred.size()));
    }
}

/** @brief 初始化 UI、VTK 视图与眼在手上默认选项。 */
HandEyeCalib::HandEyeCalib(QWidget* parent) :CustomDialog(parent),
ui(new Ui::HandEyeCalib),
m_viewer(nullptr)
{
    ui->setupUi(this);

    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    renderWindow->AddRenderer(renderer);
    ui->qvtkWidget->setRenderWindow(renderWindow);
    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    m_viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
    m_viewer->setBackgroundColor(0.5, 0.5, 0.5);
    m_viewer->resetCamera();
    ui->qvtkWidget->update();

    ui->eulerBtn->setChecked(true);
    ui->eulerorderComb->setCurrentIndex(0);

	ui->stackedWidget->setCurrentIndex(0);
    connect(ui->calibPanleTypeComb, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) {
        ui->stackedWidget->setCurrentIndex((index == 2) ? 1 : 0); });

    connect(ui->EyeInHandbtn, &QRadioButton::toggled, this, [this](bool checked) {
        if (checked) {
            ui->EyeInHandbtn->setChecked(true);
            ui->EyeOnHandbtn->setChecked(false);

            ui->capFlangeEdit->setEnabled(true);
            ui->loadCapFlangeBtn->setEnabled(true);
        }
        else {
            ui->EyeInHandbtn->setChecked(false);
            ui->EyeOnHandbtn->setChecked(true);

            ui->capFlangeEdit->setEnabled(false);
            ui->loadCapFlangeBtn->setEnabled(false);
        }
		});

    ui->EyeInHandbtn->setChecked(true);
}

/** @brief 析构。 */
HandEyeCalib::~HandEyeCalib()
{
    delete ui;
}

void /** @brief 加载图像文件夹。 */
HandEyeCalib::on_loadImageBtn_clicked() {
    QString ImageFolderPath = QFileDialog::getExistingDirectory(
        this,                          // 父窗口
        tr("Choose File"),               // 对话框标题
        "",                           // 初始目录，空字符串表示使用上次访问的目录
        QFileDialog::ShowDirsOnly     // 选项：只显示文件夹
        | QFileDialog::DontResolveSymlinks // 选项：不解析符号链接
    );

    if (!ImageFolderPath.isEmpty()) {
        ui->ImageFilePath->setText(ImageFolderPath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
    try
    {
        std::vector<std::string> imagePaths;
        cv::glob(ImageFolderPath.toStdString() + "/*", imagePaths, false);

        Point3DConsistency tempCons;
        tempCons.perPoseMeanAbsError.resize(imagePaths.size(), Eigen::Vector3d(0.0, 0.0, 0.0));
        std::vector<bool> validIndex(imagePaths.size(), true);
        updateTable(ui->validTable, validIndex, tempCons);
    }
    catch (const std::exception& e)
    {
        printW(QString("Image file path error happened, cause by %1").arg(e.what()));
        return;
    }
    
}

void /** @brief 加载位姿文件。 */
HandEyeCalib::on_loadPoseBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "", tr("Json File (*.Json);;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->PosePath->setText(filePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
}

void HandEyeCalib::on_loadCamTgtPointBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "", tr("Json File (*.Json);;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->camTgtPointEdit->setText(filePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
}

void HandEyeCalib::on_loadCapFlangeBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "", tr("Json File (*.Json);;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->capFlangeEdit->setText(filePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
}

void HandEyeCalib::on_loadTcpTouchBtn_clicked() {
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open Json File"), "", tr("Json File (*.Json);;所有文件 (*)"));
    if (!filePath.isEmpty()) {
        ui->tcpTouchEdit->setText(filePath);
    }
    else {
        printW(tr("No choose any file!"));
        return;
    }
}

void /** @brief 根据面板类型执行标定。 */
HandEyeCalib::on_execCalibBtn_clicked() {

    if(ui->calibPanleTypeComb->currentIndex() == 2) {
        tcpCalibMethod();
    }
    else {
        markCalibMethod();
	}
}

void /** @brief 标定板手眼标定入口。 */
HandEyeCalib::markCalibMethod() {
    QString PoseFilePath = ui->PosePath->text();
    QString ImageFolderPath = ui->ImageFilePath->text();

    if (PoseFilePath == "" || ImageFolderPath == "") {
        printE(tr("File path (%l) or (%2) not right load!").arg(PoseFilePath).arg(ImageFolderPath));
        return;
    }

    bool handInEye = ui->EyeInHandbtn->isChecked();

    int typeCalibrationPanel = ui->calibPanleTypeComb->currentIndex();
    bool chess_board = typeCalibrationPanel == 0;
    bool symmetric = false;

    int squareX = ui->calibPanelXSpin->value();
    int squareY = ui->calibPanelYSpin->value();

    double squareInter = ui->P2PDisSpin->value();
    Point3DConsistency calibRet;
    if (handInEye) {
        transform = eye_in_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), squareInter, squareX, squareY,
            chess_board, symmetric, "./config/Calibration/", calibRet);
    }
    else {

        transform = eye_on_hand(PoseFilePath.toStdString(), ImageFolderPath.toStdString(), squareInter, squareX, squareY,
            chess_board, symmetric, "./config/Calibration/", calibRet);
    }

    updateVtkWindow(transform, calibRet, handInEye);
}

void /** @brief TCP 点触标定入口。 */
HandEyeCalib::tcpCalibMethod() {

    QString camTgtPointPath = ui->camTgtPointEdit->text();
    QString capFlangePath = ui->capFlangeEdit->text();
    QString tcpTouchPath = ui->tcpTouchEdit->text();

	EyeHandCalibResult calibResult;

    if (ui->EyeInHandbtn->isChecked()) {
		std::vector<Eigen::Vector3d> cam = readXYZFromJson(camTgtPointPath.toStdString());
		std::vector<Eigen::Vector3d> tcp = readXYZFromJson(tcpTouchPath.toStdString());
		std::vector<Eigen::Affine3d> flange = readFlangePoseFromJson(capFlangePath.toStdString());

        calibResult = CalibrateEyeInHand(cam, flange, tcp);
    }
    else {
        std::vector<Eigen::Vector3d> cam_eoh = readXYZFromJson(camTgtPointPath.toStdString());
        std::vector<Eigen::Vector3d> tcp_eoh = readXYZFromJson(tcpTouchPath.toStdString());
        calibResult = CalibrateEyeOnHand(cam_eoh, tcp_eoh);
    }

	transform = calibResult.T_calib.cast<float>();

    updateVtkWindow(transform, calibResult, ui->EyeInHandbtn->isChecked());
}

std::vector<Eigen::Vector3d> /** @brief 从 JSON 读取三维点列表。 */
HandEyeCalib::readXYZFromJson(const std::string& pose_path) {

	std::vector<Eigen::Vector3d> xyz;

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return xyz;
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return xyz;
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }
    std::vector<std::vector<double>> pose_vectors;

    for (std::string key : indices) {
        std::vector<double> pose_vec;
		Eigen::Vector3d pose;
        if (j.contains(key) && j[key].is_array()) {
            std::vector<double> pose_vec = j[key].get<std::vector<double>>();
			pose = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
            xyz.push_back(pose);
        }
    }

	return xyz;
}

std::vector<Eigen::Affine3d> /** @brief 从 JSON 读取法兰位姿序列。 */
HandEyeCalib::readFlangePoseFromJson(const std::string& pose_path) {
    std::vector<Eigen::Affine3d> flange_pose;

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return flange_pose;
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return flange_pose;
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }
    std::vector<std::vector<double>> pose_vectors;

    
    for (std::string key : indices) {
        std::vector<double> pose_vec;
        if (j.contains(key) && j[key].is_array()) {
            pose_vec = j[key].get<std::vector<double>>();
            pose_vectors.push_back(pose_vec);
        }
    }

    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->quatBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

	MathUtils mu;
    for (const auto& pose_vec : pose_vectors) {
        Eigen::Vector3d pos = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == RotateType::Euler) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d euler_angle = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d euler_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                euler_rad = mu.deg2radVec(euler_angle);
            }
            else {
                euler_rad = euler_angle;
            }
            matrix = mu.eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == RotateType::Quat) {
            assert(pose_vec.size() == 7); // w x y z
            Eigen::Quaterniond q = Eigen::Quaterniond(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
            matrix = mu.quaternionToMatrix(q);
        }
        else if (rt == RotateType::RotateVec) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d rot_vec_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                rot_vec_rad = mu.deg2radVec(rotate_vec);
            }
            else {
                rot_vec_rad = rotate_vec;
            }
            matrix = mu.rotVecToMatrix(rot_vec_rad);
        }

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = matrix;
        transform.translation() = pos;
		flange_pose.push_back(transform);
    }

	return flange_pose;
}

void HandEyeCalib::on_saveCalibBtn_clicked() {
    writeCalibration(transform);
}

void /** @brief 导出平移与四元数 JSON。 */
HandEyeCalib::writeCalibration(const Eigen::Affine3f& result) {
    QString pre;
    if (ui->EyeInHandbtn->isChecked()) {
        pre = "./config/calibration/HandInEyeCalib-";
    }
    else {
        pre = "./config/calibration/HandOnEyeCalib-";
    }

    QString filename = pre + QDateTime::currentDateTime().toString("yyyy-MM-dd-hh-mm-ss");

    QString filepath = QFileDialog::getSaveFileName(this, tr("Save Hand"), filename, "JSON(*.json)");
    if (filepath.isEmpty()) return;

    Eigen::Vector3f translation = result.translation();
    std::vector<double> translation_ = { translation.x(), translation.y(), translation.z() };
    nlohmann::json j;
    j["EIH"] = ui->EyeInHandbtn->isChecked() ? 1 : 2;
    j["Translation"] = translation_;
    Eigen::Matrix3f rotation_matrix = result.rotation();
    Eigen::Quaternionf quaternion(rotation_matrix);
    std::vector<float> quat = { quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z() };
    j["Quaternion"] = quat;
    std::ofstream file(filepath.toStdString());
    if (!file.is_open()) {
        printE(tr("The HanEye Calibration data (%1) save failed!").arg(filepath));
        return;
    }
    printI(tr("The HanEye Calibration data (%1) save done!").arg(filepath));
    file << j.dump(4);
    file.close();
}

double /** @brief Laplacian 方差评估清晰度。 */
HandEyeCalib::AssessImageQuality(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    double variance = stddev.val[0] * stddev.val[0];
    return variance;
}

void /** @brief 生成棋盘/圆点标定板 3D 角点。 */
HandEyeCalib::GenerateW3DPoints(bool symmetric, bool chess_board, int pattern_x, int pattern_y, double square_size, std::vector<cv::Point3f>& objp) {
    // 生成3D世界坐标
    objp.clear();
    if (symmetric || chess_board) {
        for (int j = 0; j < pattern_y; j++) {
            for (int i = 0; i < pattern_x; i++) {
                objp.push_back(cv::Point3f(float(i * square_size), float(j * square_size), 0));
            }
        }
    }
    else { // Asymmetric
        double offset = square_size / 2.0;
        for (int j = 0; j < pattern_y; j++) {
            double start_x, start_y;
            if (j % 2 == 0) {
                start_x = 0.0;
                start_y = (j > 1) ? j * square_size / 2.0 : 0.0;
            }
            else {
                start_x = offset;
                start_y = (j > 2) ? offset + (j - 1) * square_size / 2.0 : offset;
            }
            for (int i = 0; i < pattern_x; i++) {
                double x = start_x + i * square_size;
                objp.push_back(cv::Point3f(x, start_y, 0.0));
            }
        }
    }
}

Eigen::Affine3f /** @brief 眼在手外：检测角点、标定相机、calibrateHandEye。 */
HandEyeCalib::eye_on_hand(
    const std::string& pose_path,
    const std::string& image_path,
    double square_size,
    int pattern_x,
    int pattern_y,
    bool chess_board,
    bool symmetric,
    const std::string& save_path,
    Point3DConsistency& cons) {

    cv::Size pattern_size(pattern_x, pattern_y);

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }

    std::vector<std::vector<double>> pose_vectors;

    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->quatBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

    for (std::string key : indices) {
        std::vector<double> pose_vec;
        if (j.contains(key) && j[key].is_array()) {
            pose_vec = j[key].get<std::vector<double>>();
            pose_vectors.push_back(pose_vec);
        }
    }

    if (pose_vectors.size() != indices.size()) {
        printE(tr("Input file num not equal!"));
        return Eigen::Affine3f::Identity();
    }

    int rowCount = ui->validTable->rowCount();
    std::vector<bool> keptIndices(rowCount, true);
    for (int i = 0; i < rowCount; ++i) {
        QWidget* w = ui->validTable->cellWidget(i, Col::COL_CHECK);
        if (!w) continue;
        QCheckBox* cb = w->findChild<QCheckBox*>();
        if (cb && !cb->isChecked())
            keptIndices[i] = false;
    }

    std::vector<std::string> image_paths;
    cv::glob(image_path + "/*", image_paths, false);

    if (image_paths.size() != pose_vectors.size()) {
        printE("Image Number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
    }

    if (keptIndices.size() < image_paths.size())
        keptIndices.resize(image_paths.size(), true);

    std::vector<cv::Point3f> objp;
    GenerateW3DPoints(symmetric, chess_board, pattern_x, pattern_y, square_size, objp);

    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<std::vector<double>> filtered_pose_vectors;
    int det_success_num = 0;
    cv::Mat gray;

    for (size_t i = 0; i < image_paths.size(); ++i) {
        if (!keptIndices[i]) continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) {
            keptIndices[i] = false;
            continue;
        }

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;
        if (quality_score < quality_threshold) {
            keptIndices[i] = false;
            printW(tr("Image Quality low!"));
            continue;
        }

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        gray = 255 - gray;

        std::vector<cv::Point2f> corners;
        bool found = false;

        if (chess_board) {
            found = cv::findChessboardCornersSB(gray, pattern_size, corners);
        }
        else {
            cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

            if (symmetric) {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
            }
            else {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
                if (found) {
                    found = (corners.size() == static_cast<size_t>(pattern_x * pattern_y));
                }
            }
        }

        if (found) {
            det_success_num++;
            obj_points_list.push_back(objp);
            img_points_list.push_back(corners);
            filtered_pose_vectors.push_back(pose_vectors[i]);

            cv::drawChessboardCorners(img, pattern_size, corners, found);
            cv::imwrite(save_path + "/calib_image/calib_" + std::to_string(i) + ".png", img);
        }
        else {
            keptIndices[i] = false;
        }
    }

    if (obj_points_list.size() < 3) {
        printE(tr("Not Found Calibration Panel Image, please check the setting para!"));
        return Eigen::Affine3f::Identity();
    }

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double ret = cv::calibrateCamera(obj_points_list, img_points_list, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    std::vector<cv::Mat> R_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);
            R_board2cameras.push_back(R_board2camera.clone());
            t_board2cameras.push_back(tvec.clone());
        }
        else {
            printE(tr("Pnp slove failed!"));
            return Eigen::Affine3f::Identity();
        }
    }

    MathUtils mu;
    std::vector<cv::Mat> R_end2bases;
    std::vector<cv::Mat> t_end2bases;
    for (const auto& pose_vec : filtered_pose_vectors) {
        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == RotateType::Euler) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d euler_angle = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d euler_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                euler_rad = mu.deg2radVec(euler_angle);
            }
            else {
                euler_rad = euler_angle;
            }
            matrix = mu.eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == RotateType::Quat) {
            assert(pose_vec.size() == 7);
            Eigen::Quaterniond q = Eigen::Quaterniond(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
            matrix = mu.quaternionToMatrix(q);
        }
        else if (rt == RotateType::RotateVec) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d rot_vec_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                rot_vec_rad = mu.deg2radVec(rotate_vec);
            }
            else {
                rot_vec_rad = rotate_vec;
            }
            matrix = mu.rotVecToMatrix(rot_vec_rad);
        }

        cv::Mat R_from_pose = cv::Mat::eye(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_from_pose.at<double>(r, c) = matrix.matrix()(r, c);
            }
        }
        cv::Mat t_from_pose = (cv::Mat_<double>(3, 1) << pose_vec[0], pose_vec[1], pose_vec[2]);

        // OpenCV 眼在手外需要 base->end；必须 clone，不能存 .t() 视图
        cv::Mat R_base2end = R_from_pose.t();
        cv::Mat t_base2end = (-R_base2end * t_from_pose);
        R_end2bases.push_back(R_base2end);
        t_end2bases.push_back(t_base2end);
    }

    std::vector<cv::Mat> R_calib, t_calib;
    R_calib.reserve(R_end2bases.size());
    t_calib.reserve(t_end2bases.size());
    for (size_t i = 0; i < R_end2bases.size(); ++i) {
        R_calib.push_back(R_end2bases[i].clone());
        t_calib.push_back(t_end2bases[i].clone());
    }

    cv::Mat R_camera2base, t_camera2base;
    cv::calibrateHandEye(R_calib, t_calib, R_board2cameras, t_board2cameras,
        R_camera2base, t_camera2base, cv::CALIB_HAND_EYE_TSAI);

    cv::Mat T_camera2base = cv::Mat::eye(4, 4, CV_64F);
    R_camera2base.copyTo(T_camera2base(cv::Rect(0, 0, 3, 3)));
    t_camera2base.copyTo(T_camera2base(cv::Rect(3, 0, 1, 3)));

    double total_error = 0.0;
    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(obj_points_list[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
        double error = cv::norm(img_points_list[i], projected_points, cv::NORM_L2) / static_cast<double>(projected_points.size());
        total_error += error;
    }

    cons = evaluate3DPointConsistencyEyeOnHand(
        R_board2cameras, t_board2cameras,
        R_end2bases, t_end2bases,
        T_camera2base, objp);
    cons.reproject_error_pixel = total_error / static_cast<double>(obj_points_list.size());

    Eigen::Affine3d trans;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            trans.matrix()(i, j) = T_camera2base.at<double>(i, j);
        }
    }

    updateTable(ui->validTable, keptIndices, cons);

    return trans.cast<float>();
}

Eigen::Affine3f /** @brief 眼在手上：同上，相机相对法兰。 */
HandEyeCalib::eye_in_hand(
    const std::string& pose_path,
    const std::string& image_path,
    double square_size,
    int pattern_x,
    int pattern_y,
    bool chess_board,
    bool symmetric,
    const std::string& save_path, 
    Point3DConsistency& cons) {

    cv::Size pattern_size(pattern_x, pattern_y);

    std::ifstream file(pose_path);
    if (!file.is_open()) {
        printE(tr("Failed to open input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    nlohmann::json j;
    try {
        file >> j;
        file.close();
    }
    catch (const json::parse_error& e) {
        file.close();
        printE(tr("Failed to parser input file (%1)!").arg(QString::fromStdString(pose_path)));
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::string> indices;
    for (auto& element : j.items()) {
        indices.push_back(element.key());
    }
    std::vector<std::vector<double>> pose_vectors;

    RotateType rt;
    if (ui->eulerBtn->isChecked()) {
        rt = RotateType::Euler;
    }
    else if (ui->quatBtn->isChecked()) {
        rt = RotateType::Quat;
    }
    else {
        rt = RotateType::RotateVec;
    }

    for (std::string key : indices) {
        std::vector<double> pose_vec;
        if (j.contains(key) && j[key].is_array()) {
            pose_vec = j[key].get<std::vector<double>>();
            pose_vectors.push_back(pose_vec);
        }
    }

    if (pose_vectors.size() != indices.size()) {
        printE(tr("Input file num not equal!"));
        return Eigen::Affine3f::Identity();
    }

    int rowCount = ui->validTable->rowCount();
    std::vector<bool> keptIndices(rowCount, true);
    for (int i = 0; i < rowCount; ++i) {
        QWidget* w = ui->validTable->cellWidget(i, Col::COL_CHECK);
        if (!w) continue;
        QCheckBox* cb = w->findChild<QCheckBox*>();
        if (cb && !cb->isChecked())
            keptIndices[i] = false;
    }

    std::vector<std::string> image_paths;
    cv::glob(image_path + "/*", image_paths, false);

    if (image_paths.size() != pose_vectors.size()) {
        printE("Image Number not equal to Robot Pose Number, check the input data file!");
        return Eigen::Affine3f::Identity();
    }

    std::vector<std::vector<cv::Point3f>> obj_points_list;
    std::vector<std::vector<cv::Point2f>> img_points_list;
    std::vector<std::vector<double>> filtered_pose_vectors; // 修正：使用正确的类型

    std::vector<cv::Point3f> objp;
    GenerateW3DPoints(symmetric, chess_board, pattern_x, pattern_y, square_size, objp);

    int det_success_num = 0;
    cv::Mat gray;

    for (size_t i = 0; i < image_paths.size(); ++i) {

        if (!keptIndices[i]) continue;

        cv::Mat img = cv::imread(image_paths[i]);
        if (img.empty()) continue;

        double quality_score = AssessImageQuality(img);
        double quality_threshold = 20.0;

        if (quality_score < quality_threshold) {
            printW("Image Quality Low, Pass away!");
            continue;
        }

        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        gray = 255 - gray;

        std::vector<cv::Point2f> corners;
        bool found = false;

        if (chess_board) {
            found = cv::findChessboardCornersSB(gray, pattern_size, corners);
        }
        else {
            //cv::SimpleBlobDetector::Params bp;
            //bp.filterByColor = true;
            //bp.blobColor = 0;          // 黑圆白底；白圆黑底改成 255
            //bp.filterByArea = true;
            //bp.minArea = 100;           // 按实际圆面积改
            //bp.maxArea = 20000;
            //bp.filterByCircularity = true;
            //bp.minCircularity = 0.5f;
            //bp.filterByConvexity = true;
            //bp.minConvexity = 0.8f;
            //bp.filterByInertia = true;
            //bp.minInertiaRatio = 0.4f;
            //bp.minDistBetweenBlobs = 8.f;
            //auto detector = cv::SimpleBlobDetector::create(bp);

            if (symmetric) {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
            }
            else {
                found = cv::findCirclesGrid(gray, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            }
        }

        if (found) {
            det_success_num++;
            obj_points_list.push_back(objp);
            img_points_list.push_back(corners);
            filtered_pose_vectors.push_back(pose_vectors[i]); // 修正：类型匹配

            cv::drawChessboardCorners(img, pattern_size, corners, found);
            cv::imwrite(save_path + "/calib_image/calib_" + std::to_string(i) + ".png", img);
        }
        else {
            keptIndices[i] = false;
        }
    }

    if (obj_points_list.size() == 0) {
        printE(tr("Not Found Calibration Panel Image, please check the setting para!"));
        return Eigen::Affine3f::Identity();
    }

    // --- 相机标定 ---
    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double ret = cv::calibrateCamera(obj_points_list, img_points_list, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

    // --- 求解标定板在相机坐标系中的位姿 ---
    std::vector<cv::Mat> R_board2cameras;
    std::vector<cv::Mat> t_board2cameras;

    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_list[i], img_points_list[i], camera_matrix, dist_coeffs, rvec, tvec);
        if (success) {
            cv::Mat R_board2camera;
            cv::Rodrigues(rvec, R_board2camera);

            R_board2cameras.push_back(R_board2camera);
            t_board2cameras.push_back(tvec);

            /*cv::Mat T_board2camera = cv::Mat::eye(4, 4, CV_64F);
            R_board2camera.copyTo(T_board2camera(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T_board2camera(cv::Rect(3, 0, 1, 3)));
            Eigen::Affine3d t1e;
            cv::cv2eigen(T_board2camera, t1e.matrix());
            *cam2target_it = t1e.inverse();*/
        }
        else {
            printE(tr("Pnp slove failed!"));
            return Eigen::Affine3f::Identity();
        }
    }

    // --- 转换机器人位姿数据为末端的变换 (与eye_on_hand不同) ---
    MathUtils mu;
    std::vector<cv::Mat> R_end2bases;
    std::vector<cv::Mat> t_end2bases;

    for (const auto& pose_vec : filtered_pose_vectors) { 
        Eigen::Vector3d pos = Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]);
        QString order = ui->eulerorderComb->currentText();
        Eigen::Matrix3d matrix;
        if (rt == RotateType::Euler) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d euler_angle = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d euler_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                euler_rad = mu.deg2radVec(euler_angle);
            }
            else {
                euler_rad = euler_angle;
            }
            matrix = mu.eulerToMatrix(euler_rad, order.toStdString());
        }
        else if (rt == RotateType::Quat) {
            assert(pose_vec.size() == 7); // w x y z
            Eigen::Quaterniond q = Eigen::Quaterniond(pose_vec[3], pose_vec[4], pose_vec[5], pose_vec[6]);
            matrix = mu.quaternionToMatrix(q);
        }
        else if (rt == RotateType::RotateVec) {
            assert(pose_vec.size() == 6);
            Eigen::Vector3d rotate_vec = Eigen::Vector3d(pose_vec[3], pose_vec[4], pose_vec[5]);
            Eigen::Vector3d rot_vec_rad;
            if (ui->angleUnitComb->currentIndex() == 0) {
                rot_vec_rad = mu.deg2radVec(rotate_vec);
            }
            else {
                rot_vec_rad = rotate_vec;
            }
            matrix = mu.rotVecToMatrix(rot_vec_rad);
        }

        /*Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = matrix;
        transform.translation() = pos;
        *base2end_it = transform;*/

        cv::Mat R_end = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_end.at<double>(i, j) = matrix.matrix()(i, j);
            }
        }
        cv::Mat t_end = (cv::Mat_<double>(3, 1) << pose_vec[0], pose_vec[1], pose_vec[2]);
        R_end2bases.push_back(R_end);
        t_end2bases.push_back(t_end);
    }

    cv::Mat R_camera2end, t_camera2end;
    cv::calibrateHandEye(R_end2bases, t_end2bases, R_board2cameras, t_board2cameras, R_camera2end, t_camera2end, cv::CALIB_HAND_EYE_TSAI);

    cv::Mat T_camera2end = cv::Mat::eye(4, 4, CV_64F);
    R_camera2end.copyTo(T_camera2end(cv::Rect(0, 0, 3, 3)));
    t_camera2end.copyTo(T_camera2end(cv::Rect(3, 0, 1, 3)));

    double total_error = 0.0;
    for (size_t i = 0; i < obj_points_list.size(); ++i) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(obj_points_list[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
        double error = cv::norm(img_points_list[i], projected_points, cv::NORM_L2) / static_cast<double>(projected_points.size());
        total_error += error;
    }

    cons = evaluate3DPointConsistencyEyeInHand(
        R_board2cameras, t_board2cameras,
        R_end2bases, t_end2bases,
        T_camera2end, objp);
    cons.reproject_error_pixel = total_error / static_cast<double>(obj_points_list.size());;

    Eigen::Affine3d trans;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            trans.matrix()(i, j) = T_camera2end.at<double>(i, j);
        }
    }

    updateTable(ui->validTable, keptIndices, cons);

    return trans.cast<float>();
}

Point3DConsistency /** @brief 眼在手上 3D 点重投影一致性。 */
HandEyeCalib::evaluate3DPointConsistencyEyeInHand(
    const std::vector<cv::Mat>& R_board2cams,
    const std::vector<cv::Mat>& t_board2cams,
    const std::vector<cv::Mat>& R_end2bases,
    const std::vector<cv::Mat>& t_end2bases,
    const cv::Mat& T_cam2end,
    const std::vector<cv::Point3f>& objp)
{
    // 眼在手上：标定板固定在基座侧。将棋盘格角点经
    //   棋盘 -> 相机 -> 法兰 -> 基座
    // 变到机械臂基座系。各姿态下同一角点应重合，对全部位姿求均值后算偏差，不跳过第 0 帧。
    // p_cam  = R_board2cam * p_board + t_board2cam
    // p_end  = R_cam2end   * p_cam   + t_cam2end
    // p_base = R_end2base  * p_end   + t_end2base
    Point3DConsistency res;
    const size_t nPose = R_board2cams.size();
    const size_t nPts = objp.size();
    res.numPoses = static_cast<int>(nPose);
    if (nPose < 2 || nPts == 0 ||
        R_end2bases.size() != nPose || t_end2bases.size() != nPose ||
        t_board2cams.size() != nPose) {
        printW(tr("3D consistency eval skipped: need >=2 poses and >=1 point."));
        return res;
    }
    const Eigen::Affine3d T_c2e = cv4x4ToAffine(T_cam2end);
    const Eigen::Matrix3d R_cam2end = T_c2e.linear();
    const Eigen::Vector3d t_cam2end = T_c2e.translation();
    std::vector<eigenVector> ptsInBase(nPts, eigenVector(nPose, Eigen::Vector3d::Zero()));
    for (size_t i = 0; i < nPose; ++i) {
        const Eigen::Affine3d T_b2c = cvRtToAffineNormalized(R_board2cams[i], t_board2cams[i]);
        const Eigen::Affine3d T_e2b = cvRtToAffineNormalized(R_end2bases[i], t_end2bases[i]);
        const Eigen::Matrix3d R_board2cam = T_b2c.linear();
        const Eigen::Vector3d t_board2cam = T_b2c.translation();
        const Eigen::Matrix3d R_end2base = T_e2b.linear();
        const Eigen::Vector3d t_end2base = T_e2b.translation();
        for (size_t j = 0; j < nPts; ++j) {
            const Eigen::Vector3d p_board(objp[j].x, objp[j].y, objp[j].z);
            const Eigen::Vector3d p_cam = R_board2cam * p_board + t_board2cam;
            const Eigen::Vector3d p_end = R_cam2end * p_cam + t_cam2end;
            ptsInBase[j][i] = R_end2base * p_end + t_end2base;
        }
    }
    res.perPointDev.assign(nPose, std::vector<Eigen::Vector3d>(nPts, Eigen::Vector3d::Zero()));
    res.pointMeanInBase.assign(nPts, Eigen::Vector3d::Zero());
    res.perPoseMeanAbsError.assign(nPose, Eigen::Vector3d::Zero());
    res.perPoseError.assign(nPose, 0.0);
    Eigen::Vector3d sumAbs = Eigen::Vector3d::Zero();
    const double invPts = 1.0 / static_cast<double>(nPts);
    const double invPose = 1.0 / static_cast<double>(nPose);
    for (size_t j = 0; j < nPts; ++j) {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < nPose; ++i)
            mean += ptsInBase[j][i];
        mean *= invPose;
        res.pointMeanInBase[j] = mean;
        for (size_t i = 0; i < nPose; ++i) {
            const Eigen::Vector3d d = ptsInBase[j][i] - mean;
            res.perPointDev[i][j] = d;
            res.perPoseMeanAbsError[i] += d.cwiseAbs();
            res.perPoseError[i] += d.norm();
            sumAbs += d.cwiseAbs();
        }
    }
    for (size_t i = 0; i < nPose; ++i) {
        res.perPoseMeanAbsError[i] *= invPts;
        res.perPoseError[i] *= invPts;
    }
    res.meanAbsError = sumAbs * (invPts * invPose);
    return res;
}

Point3DConsistency /** @brief 眼在手外 3D 点一致性（法兰系）。 */
HandEyeCalib::evaluate3DPointConsistencyEyeOnHand(
    const std::vector<cv::Mat>& R_board2cams,
    const std::vector<cv::Mat>& t_board2cams,
    const std::vector<cv::Mat>& R_end2bases,
    const std::vector<cv::Mat>& t_end2bases,
    const cv::Mat& T_cam2base,
    const std::vector<cv::Point3f>& objp)
{
    // 眼在手外：标定板固连法兰。OpenCV 传入的 R_end2bases 已是 base->end。
    // 将棋盘格角点经
    //   棋盘 -> 相机 -> 基座 -> 法兰
    // 变到法兰系。相机到法兰的观测链在各姿态下应给出同一组法兰系坐标，
    // 对全部位姿求均值后算偏差，不跳过第 0 帧。
    // p_cam  = R_board2cam * p_board + t_board2cam
    // p_base = R_cam2base  * p_cam   + t_cam2base
    // p_end  = R_base2end  * p_base  + t_base2end
    Point3DConsistency res;
    const size_t nPose = R_board2cams.size();
    const size_t nPts = objp.size();
    res.numPoses = static_cast<int>(nPose);

    if (nPose < 2 || nPts == 0 ||
        R_end2bases.size() != nPose || t_end2bases.size() != nPose ||
        t_board2cams.size() != nPose) {
        printW(tr("3D consistency eval skipped: need >=2 poses and >=1 point."));
        return res;
    }

    const Eigen::Affine3d T_c2b = cv4x4ToAffine(T_cam2base);
    const Eigen::Matrix3d R_cam2base = T_c2b.linear();
    const Eigen::Vector3d t_cam2base = T_c2b.translation();

    std::vector<eigenVector> ptsInEnd(nPts, eigenVector(nPose, Eigen::Vector3d::Zero()));

    for (size_t i = 0; i < nPose; ++i) {
        const Eigen::Affine3d T_b2c = cvRtToAffineNormalized(R_board2cams[i], t_board2cams[i]);
        const Eigen::Affine3d T_b2e = cvRtToAffineNormalized(R_end2bases[i], t_end2bases[i]);
        const Eigen::Matrix3d R_board2cam = T_b2c.linear();
        const Eigen::Vector3d t_board2cam = T_b2c.translation();
        const Eigen::Matrix3d R_base2end = T_b2e.linear();
        const Eigen::Vector3d t_base2end = T_b2e.translation();

        for (size_t j = 0; j < nPts; ++j) {
            const Eigen::Vector3d p_board(objp[j].x, objp[j].y, objp[j].z);
            const Eigen::Vector3d p_cam = R_board2cam * p_board + t_board2cam;
            const Eigen::Vector3d p_base = R_cam2base * p_cam + t_cam2base;
            ptsInEnd[j][i] = R_base2end * p_base + t_base2end;
        }
    }

    res.perPointDev.assign(nPose, std::vector<Eigen::Vector3d>(nPts, Eigen::Vector3d::Zero()));
    res.pointMeanInBase.assign(nPts, Eigen::Vector3d::Zero());
    res.perPoseMeanAbsError.assign(nPose, Eigen::Vector3d::Zero());
    res.perPoseError.assign(nPose, 0.0);

    Eigen::Vector3d sumAbs = Eigen::Vector3d::Zero();
    const double invPts = 1.0 / static_cast<double>(nPts);
    const double invPose = 1.0 / static_cast<double>(nPose);

    for (size_t j = 0; j < nPts; ++j) {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < nPose; ++i)
            mean += ptsInEnd[j][i];
        mean *= invPose;
        res.pointMeanInBase[j] = mean;

        for (size_t i = 0; i < nPose; ++i) {
            const Eigen::Vector3d d = ptsInEnd[j][i] - mean;
            res.perPointDev[i][j] = d;
            res.perPoseMeanAbsError[i] += d.cwiseAbs();
            res.perPoseError[i] += d.norm();
            sumAbs += d.cwiseAbs();
        }
    }

    for (size_t i = 0; i < nPose; ++i) {
        res.perPoseMeanAbsError[i] *= invPts;
        res.perPoseError[i] *= invPts;
    }
    res.meanAbsError = sumAbs * (invPts * invPose);
    return res;
}

void /** @brief 填充有效位姿与 XYZ 误差表。 */
HandEyeCalib::updateTable(QTableWidget* table,
    std::vector<bool>& validIndex,
    const Point3DConsistency& res)
{
    const int rowCount = static_cast<int>(validIndex.size());          // 总索引数，如 17

    // 防御：res 中有效结果个数应等于 validIndex 中 true 的个数
    const int validTotal = static_cast<int>(
        std::count(validIndex.begin(), validIndex.end(), true));
    if (static_cast<int>(res.perPoseMeanAbsError.size()) != validTotal) {
        printW(tr("updateTable: res size(%1) != valid count(%2), display may misalign.")
            .arg(res.perPoseMeanAbsError.size()).arg(validTotal));
    }

    table->clearContents();
    table->setColumnCount(Col::COL_COUNT);
    table->setRowCount(rowCount);
    table->setHorizontalHeaderLabels({
        tr("Index"),
        tr("Check"),
        tr("X Err(mm)"),
        tr("Y Err(mm)"),
        tr("Z Err(mm)")
        });
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->verticalHeader()->setVisible(false);

    int validCnt = 0;

    for (int i = 0; i < rowCount; ++i) {
        const bool isValid = validIndex[i];

        // 第 1 列：索引序号（始终是总索引 i）
        QTableWidgetItem* idxItem = new QTableWidgetItem();
        idxItem->setData(Qt::DisplayRole, i);
        idxItem->setTextAlignment(Qt::AlignCenter);
        table->setItem(i, COL_INDEX, idxItem);

        // 第 2 列：QCheckBox（居中）
        QWidget* cellWidget = new QWidget(table);
        QHBoxLayout* layout = new QHBoxLayout(cellWidget);
        QCheckBox* checkBox = new QCheckBox(cellWidget);
        checkBox->setChecked(isValid);
        checkBox->setProperty("poseIndex", i);   // 绑定总索引，回调时反查
        layout->addWidget(checkBox);
        layout->setAlignment(Qt::AlignCenter);
        layout->setContentsMargins(0, 0, 0, 0);
        cellWidget->setLayout(layout);
        table->setCellWidget(i, COL_CHECK, cellWidget);

        // 第 3/4/5 列：X / Y / Z 误差
        const int cols[3] = { COL_X, COL_Y, COL_Z };

        if (isValid && validCnt < static_cast<int>(res.perPoseMeanAbsError.size())) {
            // 有效行：从 res 取对应结果，validCnt 推进
            const Eigen::Vector3d& e = res.perPoseMeanAbsError[validCnt];
            const double vals[3] = { e.x(), e.y(), e.z() };
            for (int k = 0; k < 3; ++k) {
                QTableWidgetItem* item = new QTableWidgetItem();
                item->setData(Qt::DisplayRole, QString::number(vals[k], 'f', 4));
                item->setTextAlignment(Qt::AlignCenter);
                table->setItem(i, cols[k], item);
            }
            ++validCnt;
        }
        else {
            // 无效行：误差列留空占位，保持行结构对齐
            for (int k = 0; k < 3; ++k) {
                QTableWidgetItem* item = new QTableWidgetItem(QStringLiteral("-"));
                item->setTextAlignment(Qt::AlignCenter);
                item->setForeground(Qt::gray);
                table->setItem(i, cols[k], item);
            }
        }
    }
}

EyeHandCalibResult /** @brief 点触式眼在手上 SVD 求 ^{F}T_{C}。 */
HandEyeCalib::CalibrateEyeInHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const std::vector<Eigen::Affine3d>& flanges_in_base,
    const std::vector<Eigen::Vector3d>& tcps_in_base)
{
    EyeHandCalibResult out;
    const std::size_t n = points_in_camera.size();
    if (n != flanges_in_base.size() || n != tcps_in_base.size()) {
        return out;
    }

    Eigen::Matrix3Xd src(3, static_cast<int>(n)), dst(3, static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        src.col(static_cast<int>(i)) = points_in_camera[i];
        dst.col(static_cast<int>(i)) = flanges_in_base[i].inverse() * tcps_in_base[i];
    }

    if (!EstimateRigid(src, dst, out.T_calib)) {
        return out;
    }

    std::vector<Eigen::Vector3d> pred(n);
    for (std::size_t i = 0; i < n; ++i) {
        pred[i] = flanges_in_base[i] * (out.T_calib * points_in_camera[i]);
    }
    out.rmse = FillErrors(pred, tcps_in_base, out.per_sample_error);
    out.success = true;

    Eigen::Vector3d t = out.T_calib.translation();
    Eigen::Quaterniond q(out.T_calib.rotation());
    out.quat_vec << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();

    return out;
}

EyeHandCalibResult /** @brief 点触式眼在手外刚体配准。 */
HandEyeCalib::CalibrateEyeOnHand(
    const std::vector<Eigen::Vector3d>& points_in_camera,
    const std::vector<Eigen::Vector3d>& tcps_in_base)
{
    EyeHandCalibResult out;
    const std::size_t n = points_in_camera.size();
    if (n != tcps_in_base.size()) {
        return out;
    }

    Eigen::Matrix3Xd src(3, static_cast<int>(n)), dst(3, static_cast<int>(n));
    for (std::size_t i = 0; i < n; ++i) {
        src.col(static_cast<int>(i)) = points_in_camera[i];
        dst.col(static_cast<int>(i)) = tcps_in_base[i];
    }

    if (!EstimateRigid(src, dst, out.T_calib)) {
        return out;
    }

    std::vector<Eigen::Vector3d> pred(n);
    for (std::size_t i = 0; i < n; ++i) {
        pred[i] = out.T_calib * points_in_camera[i];
    }
    out.rmse = FillErrors(pred, tcps_in_base, out.per_sample_error);
    out.success = true;

    Eigen::Vector3d t = out.T_calib.translation();
    Eigen::Quaterniond q(out.T_calib.rotation());
    out.quat_vec << t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z();

    return out;
}

void /** @brief 控制台输出标定结果并在 VTK 显示坐标系。 */
HandEyeCalib::updateVtkWindow(const Eigen::Affine3f& transform, const Point3DConsistency& cons, bool handineye) {

    if (handineye) {
        printI("Hand In Eye Calibration calc done!");
    }
    else {
        printI("Hand On Eye Calibration calc done!");
    }

    Eigen::Affine3f trans;
    trans.translation() = transform.translation();
    trans.linear() = transform.rotation();
    Eigen::Quaternionf quatTcp(transform.rotation());
    quatTcp.normalize();

    printI(QString("Translation [x:%1, y:%2, z:%3] Quaternion [w:%4, x:%5, y:%6, z:%7] done.")
        .arg(trans.translation().x())
        .arg(trans.translation().y())
        .arg(trans.translation().z())
        .arg(quatTcp.w())
        .arg(quatTcp.x())
        .arg(quatTcp.y())
        .arg(quatTcp.z()));

    if (handineye) {
        if (ui->calibPanleTypeComb->currentIndex() == 2) {
            printI(QString("[EyeInHand] 3D consistency mean error[mm](in Calib panel frame) calib error:%1 mm").arg(cons.reproject_error_pixel, 0, 'f', 4));
        }
        else {
            printI(QString("[EyeInHand] 3D consistency mean error[mm](in Calib panel frame)  X:%1  Y:%2  Z:%3  "
                "reproject pixel error %4 (valid poses:%5)")
                .arg(cons.meanAbsError.x(), 0, 'f', 4).arg(cons.meanAbsError.y(), 0, 'f', 4).arg(cons.meanAbsError.z(), 0, 'f', 4)
                .arg(cons.reproject_error_pixel, 0, 'f', 4).arg(cons.numPoses));
        }

    }
    else {
        printI(QString("[EyeOnHand] 3D consistency mean error[mm](in End frame)  X:%1  Y:%2  Z:%3  "
            "reproject pixel error %4 (valid poses:%5)")
            .arg(cons.meanAbsError.x(), 0, 'f', 4).arg(cons.meanAbsError.y(), 0, 'f', 4).arg(cons.meanAbsError.z(), 0, 'f', 4)
            .arg(cons.reproject_error_pixel, 0, 'f', 4).arg(cons.numPoses));
    }

    m_viewer->removeAllCoordinateSystems();

    double ratio = 1.0;
    if (ui->EyeInHandbtn->isChecked()) {
        ratio = 2.0;
    }

    double scale = 100 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale, "coordinate_base");

    double scale_tcp = 50 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale_tcp, trans, "coordinate_tcp");

    m_viewer->resetCamera();
    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}

void HandEyeCalib::updateVtkWindow(const Eigen::Affine3f& transform, const EyeHandCalibResult& cons, bool handineye) {

    if (handineye) {
        printI("Hand In Eye Calibration calc done!");
    }
    else {
        printI("Hand On Eye Calibration calc done!");
    }

    Eigen::Affine3f trans;
    trans.translation() = transform.translation();
    trans.linear() = transform.rotation();
    Eigen::Quaternionf quatTcp(transform.rotation());
    quatTcp.normalize();

    printI(QString("Translation [x:%1, y:%2, z:%3] Quaternion [w:%4, x:%5, y:%6, z:%7] done.")
        .arg(trans.translation().x())
        .arg(trans.translation().y())
        .arg(trans.translation().z())
        .arg(quatTcp.w())
        .arg(quatTcp.x())
        .arg(quatTcp.y())
        .arg(quatTcp.z()));

    if (handineye) {
        printI(QString("[EyeInHand] 3D consistency mean error[mm](in Calib panel frame) calib error:%1 mm").arg(cons.rmse, 0, 'f', 4));
    }
    else {
        printI(QString("[EyeOnHand] 3D consistency mean error[mm](in Cam base frame) calib error:%1 mm").arg(cons.rmse, 0, 'f', 4));
    }

    m_viewer->removeAllCoordinateSystems();

    double ratio = 1.0;
    if (ui->EyeInHandbtn->isChecked()) {
        ratio = 2.0;
    }

    double scale = 100 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale, "coordinate_base");

    double scale_tcp = 50 * ratio; // 坐标轴的长度，单位米
    m_viewer->addCoordinateSystem(scale_tcp, trans, "coordinate_tcp");

    m_viewer->resetCamera();
    ui->qvtkWidget->renderWindow()->Render();
    ui->qvtkWidget->update();
}
