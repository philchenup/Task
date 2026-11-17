/**
 * @file MathUtils.cpp
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */

#include "MathUtils.h"
#include <cmath>
#include <algorithm>
#include <cctype>

#define M_PI_ 3.1415926

using namespace std;
using namespace Eigen;

/** @brief 工具类构造。
MathUtils::MathUtils() {}

/** @brief 析构。
MathUtils::~MathUtils(){}

/** @brief 度转弧度。 */
double MathUtils::deg2rad(double deg) {
    return deg * M_PI_ / 180.0;
}

/** @brief 弧度转度。 */
double MathUtils::rad2deg(double rad) {
    return rad * 180.0 / M_PI_;
}

/** @brief 向量逐分量 deg2rad。 */
Eigen::Vector3d MathUtils::deg2radVec(const Eigen::Vector3d& deg) {
    return Eigen::Vector3d(deg2rad(deg.x()), deg2rad(deg.y()), deg2rad(deg.z()));
}

/** @brief 检查欧拉序列为三字母 XYZ。 */
void MathUtils::validateOrder(const std::string& order) {
    if (order.length() != 3) {
        throw std::invalid_argument("Order string must be exactly 3 characters (e.g., 'ZYX').");
    }
    for (char c : order) {
        char upper = std::toupper(c);
        if (upper != 'X' && upper != 'Y' && upper != 'Z') {
            throw std::invalid_argument("Order string must only contain 'X', 'Y', or 'Z'.");
        }
    }
}

/** @brief 构造单轴旋转。 */
Eigen::AngleAxisd MathUtils::getSingleAxis(char axis, double angle) {
    char upper = std::toupper(axis);
    if (upper == 'X') {
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
    }
    else if (upper == 'Y') {
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    }
    else if (upper == 'Z') {
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    }
    throw std::invalid_argument("Invalid axis character.");
}

/** @brief 欧拉角合成旋转矩阵。 */
Eigen::Matrix3d MathUtils::eulerToMatrix(const Eigen::Vector3d& angles,
    const std::string& order,
    bool intrinsic) {
    validateOrder(order);

    std::string seq = "";
    for (char c : order) seq += std::toupper(c);

    Eigen::Vector3d rad_switch;
    if (seq == "XYZ") {
        rad_switch = angles;
    }
    else if (seq == "ZYX") {
        rad_switch[0] = angles[2];
        rad_switch[1] = angles[1];
        rad_switch[2] = angles[0];
    }
    else if (seq == "ZYZ") {
        rad_switch[0] = angles[2];
        rad_switch[1] = angles[1];
        rad_switch[2] = angles[2];
    }

    Eigen::AngleAxisd R1 = getSingleAxis(seq[0], rad_switch[0]);
    Eigen::AngleAxisd R2 = getSingleAxis(seq[1], rad_switch[1]);
    Eigen::AngleAxisd R3 = getSingleAxis(seq[2], rad_switch[2]);

    Eigen::Quaterniond q;

    if (intrinsic) {
        // 内旋 (Intrinsic): 乘法顺序与旋转顺序一致
        // R = R1 * R2 * R3
        q = R1 * R2 * R3;
    }
    else {
        // 外旋 (Extrinsic): 乘法顺序与旋转顺序相反
        // R = R3 * R2 * R1
        q = R3 * R2 * R1;
    }
    Eigen::Matrix3d R = q.toRotationMatrix();
    return R;
}

/** @brief 旋转矩阵分解为欧拉角。 */
Eigen::Vector3d MathUtils::matrixToEuler(const Eigen::Matrix3d& matrix,
    const std::string& order) {
    validateOrder(order);
    std::string seq = "";
    for (char c : order) seq += std::toupper(c);

    if (seq == "ZYX") {
        return matrix.eulerAngles(2, 1, 0);
    }
    else if (seq == "XYZ") {
        return matrix.eulerAngles(0, 1, 2);
    }
    else if (seq == "YXZ") {
        return matrix.eulerAngles(1, 0, 2);
    }
    else if (seq == "YZY") {
        return matrix.eulerAngles(1, 2, 1);
    }
    else {
        return Eigen::Vector3d();
    }
}

/** @brief 欧拉角转四元数。 */
Eigen::Quaterniond MathUtils::eulerToQuaternion(const Eigen::Vector3d& angles,
    const std::string& order,
    bool intrinsic) {
    Eigen::Matrix3d mat = eulerToMatrix(angles, order, intrinsic);
    return Eigen::Quaterniond(mat);
}

/** @brief 四元数转欧拉角。 */
Eigen::Vector3d MathUtils::quaternionToEuler(const Eigen::Quaterniond& q,
    const std::string& order,
    bool intrinsic) {
    Eigen::Matrix3d mat = q.toRotationMatrix();
    return matrixToEuler(mat, order);
}

/** @brief 旋转矩阵转四元数。 */
Eigen::Quaterniond MathUtils::matrixToQuaternion(const Eigen::Matrix3d& matrix) {
    return Eigen::Quaterniond(matrix);
}

/** @brief 四元数转旋转矩阵。 */
Eigen::Matrix3d MathUtils::quaternionToMatrix(const Eigen::Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

/** @brief 旋转向量（轴角）转矩阵。 */
Eigen::Matrix3d MathUtils::angleAxisToMatrix(const Eigen::Vector3d& axis_angle) {
    double angle = axis_angle.norm();
    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Vector3d axis = axis_angle / angle;
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

/** @brief 矩阵转轴角向量。 */
Eigen::Vector3d MathUtils::matrixToAngleAxis(const Eigen::Matrix3d& matrix) {
    Eigen::AngleAxisd aa(matrix);
    return aa.angle() * aa.axis();
}

/** @brief 旋转矢量（度）转矩阵。 */
Eigen::Matrix3d MathUtils::rotVecToMatrix(const Eigen::Vector3d& rot_vec) {
    double angle = rot_vec.norm();

    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d unit_axis = rot_vec.normalized();
    Eigen::AngleAxisd rotation_vector_to_matrix(angle, unit_axis);

    Eigen::Matrix3d rotation_matrix_from_av = rotation_vector_to_matrix.toRotationMatrix();

    return rotation_matrix_from_av;
}

/** @brief 矩阵转旋转矢量。 */
Eigen::Vector3d MathUtils::matrixToRotVec(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);

    return aa.angle() * aa.axis();
}

/** @brief 联立方程求 TCP；六点法补姿态。 */
int MathUtils::TcpCalc(const std::vector<Eigen::Vector3d>& pos,
    const std::vector<Eigen::Matrix3d>& rot, Eigen::Matrix4d& ret) {

    std::vector<MatrixXd> A_list;
    std::vector<VectorXd> B_list;

    int base_idx = 3;
    for (int i = 0; i < 3; i++) {
        Matrix3d R_i = rot[i];
        Matrix3d R_base = rot[base_idx];
        Vector3d P_i = pos[i];
        Vector3d P_base = pos[base_idx];

        MatrixXd A_block = R_base - R_i;
        VectorXd B_block = P_i - P_base;

        A_list.push_back(A_block);
        B_list.push_back(B_block);
    }

    MatrixXd A(9, 3);  // 3 equations * 3 rows each = 9 total rows
    VectorXd B(9);     // 9 right-hand side values

    int row_offset = 0;
    for (int i = 0; i < A_list.size(); i++) {
        A.block(row_offset, 0, 3, 3) = A_list[i];
        B.segment(row_offset, 3) = B_list[i];
        row_offset += 3;
    }
    Vector3d tcpPos = (A.transpose() * A).inverse() * A.transpose() * B;

    ret(0, 3) = tcpPos.x(); // Tx
    ret(1, 3) = tcpPos.y(); // Ty
    ret(2, 3) = tcpPos.z(); // Tz

    if (pos.size() != 6 || rot.size() != 6) {
        return 0;
    }

    Vector3d P4 = pos[3];
    Vector3d P5 = pos[4];
    Vector3d P6 = pos[5];
    Matrix3d R4 = rot[3];

    Vector3d vec_x_base = P5 - P4;
    if (vec_x_base.norm() < 1e-6) return -1;

    Vector3d dir_x_base = vec_x_base / vec_x_base.norm();

    Vector3d x_tool = R4.transpose() * (-dir_x_base);

    Vector3d vec_z_base = P6 - P4;
    if (vec_z_base.norm() < 1e-6) return -1;

    Vector3d dir_z_base = vec_z_base / vec_z_base.norm();
    Vector3d z_tool = R4.transpose() * dir_z_base;

    Vector3d x_final = x_tool / x_tool.norm();
    Vector3d y_final = z_tool.cross(x_final);
    y_final = y_final / y_final.norm();
    Vector3d z_final = x_final.cross(y_final);

    Matrix3d R_tcp;
    R_tcp.col(0) = x_final;
    R_tcp.col(1) = y_final;
    R_tcp.col(2) = z_final;

    ret.block<3, 3>(0, 0) = R_tcp;

    return 0;
}