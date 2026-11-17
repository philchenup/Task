/**
 * @file MathUtils.h
 * @brief 姿态变换、欧拉/四元数/旋转向量互转与 TCP 标定解算。
 * @author philchen
 * @version 3.0
 * @date 2026-03-10
 */

#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <Eigen/Geometry>
#include <string>
#include <stdexcept>
#include <vector>

namespace Ui
{
    class MathUtils;
}


class MathUtils
{
public:
    explicit MathUtils();

    ~MathUtils();

public:
    /** @brief 常用欧拉顺序（与字符串 order 对照，便于 IDE 提示）。 */
    enum class EulerOrder {
        ZYX, XYZ, XZY, YXZ, YZX, ZXY, // Tait-Bryan (非对称)
        XYX, XZX, YXY, YZY, ZXZ, ZYZ  // Proper Euler (对称)
    };

    /**
     * @brief 四点法求 TCP 平移；六点法再估计 TCP 姿态。
     * @param pos 各标定点的法兰位置
     * @param rot 对应旋转矩阵
     * @param ret 输出 4×4 齐次矩阵（平移在末列）
     * @return 0 成功，-1 输入几何退化
     */
    static int TcpCalc(const std::vector<Eigen::Vector3d>& pos,
        const std::vector<Eigen::Matrix3d>& rot, Eigen::Matrix4d& ret);

    /**
     * @brief 将欧拉角转换为旋转矩阵
     * @param angles 包含3个角度的向量 [angle1, angle2, angle3] (弧度)
     * @param order 旋转顺序字符串，例如 "ZYX", "XYZ", "ZYZ" (大小写不敏感)
     * @param intrinsic 是否为内旋 (Intrinsic)。默认为 true (符合 ROS/机器人学标准)。
     *                  若为 false，则视为绕固定轴的外旋 (Extrinsic)。
     * @return 3x3 旋转矩阵
     */
    static Eigen::Matrix3d eulerToMatrix(const Eigen::Vector3d& angles,
        const std::string& order,
        bool intrinsic = true);

    /**
     * @brief 将旋转矩阵转换为欧拉角
     * @param matrix 3x3 旋转矩阵
     * @param order 期望的欧拉角顺序字符串 (例如 "ZYX")
     * @param intrinsic 是否与输入矩阵的生成方式一致 (通常保持 true)
     * @return 包含3个角度的向量 [angle1, angle2, angle3] (弧度)
     * @note 存在万向节死锁问题，在奇异点附近解可能不唯一。
     */
    static Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d& matrix,
        const std::string& order);

    /**
     * @brief 欧拉角转四元数
     */
    static Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& angles,
        const std::string& order,
        bool intrinsic = true);

    /**
     * @brief 四元数转欧拉角
     */
    static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q,
        const std::string& order,
        bool intrinsic = true);

    /**
     * @brief 旋转矩阵转四元数
     */
    static Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& matrix);

    /**
     * @brief 四元数转旋转矩阵
     */
    static Eigen::Matrix3d quaternionToMatrix(const Eigen::Quaterniond& q);

    /**
     * @brief 旋转向量 (Angle-Axis) 转旋转矩阵
     * @param axis_angle 前3维为轴方向(可非单位)，模长为旋转角度
     */
    static Eigen::Matrix3d angleAxisToMatrix(const Eigen::Vector3d& axis_angle);

    /**
     * @brief 旋转矩阵转旋转向量
     */
    static Eigen::Vector3d matrixToAngleAxis(const Eigen::Matrix3d& matrix);

    /**
     * @brief 角度转弧度
     */
    static double deg2rad(double deg);

    /**
     * @brief 弧度转角度
     */
    static double rad2deg(double rad);

    /**
     * @brief 向量角度转弧度
     */
    static Eigen::Vector3d deg2radVec(const Eigen::Vector3d& deg);

    /**
     * @brief 旋转矢量转旋转矩阵
     */
    static Eigen::Matrix3d rotVecToMatrix(const Eigen::Vector3d& rot_vec);

    /**
     * @brief 旋转矩阵转旋转矢量
     */
    Eigen::Vector3d matrixToRotVec(const Eigen::Matrix3d& R);

private:
    // 内部辅助：获取单轴旋转矩阵
    static Eigen::AngleAxisd getSingleAxis(char axis, double angle);

    // 内部辅助：验证顺序字符串
    static void validateOrder(const std::string& order);

};

#endif // MATHUTILS_H
