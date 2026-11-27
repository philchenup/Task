#pragma once

#ifndef PROCESSWPRKER_H
#define PROCESSWPRKER_H

#include <cstdlib>
#include <QMetaType>
#include <QObject>  
#include <QPushButton> 
#include <QTimer>
#include <QDateTime>
#include <QDir>
#include <QString>
#include <QThread>
#include <QMutex>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "base/cloud.h"
#include "utils/utils.h"

Q_DECLARE_METATYPE(std::vector<Eigen::Affine3f>);
Q_DECLARE_METATYPE(Eigen::Affine3f);

class ProcessWorker : public QObject
{
	Q_OBJECT
public:
	explicit ProcessWorker(QObject* parent = nullptr);
	~ProcessWorker();

	void setInputSceneData(const ct::Cloud::Ptr&);

	// 输入为相机坐标系下的点云，单位毫米，按点云中的全部有限点计算。
	// 少于 30 个点，或没有法向与相机 +Z 夹角不超过 10° 的平面时返回 false，且不修改 pose。
	// 成功时对内点做最小二乘精细拟合，输出平面质心沿该法向反向平移 300mm 的 Eigen::Affine3d。
	// 位姿 Z 轴与相机 +Z 同向，X/Y 朝向相机坐标轴。
	bool computeRetreatPlanePose(const ct::Cloud::ConstPtr& cloud, Eigen::Affine3d& pose) const;

	void loadVisionConfig(const std::string& visionconfig_path);

signals:
	void sendMsQ(int level, const QString& status);

	void sendTargetTransform(const Eigen::Affine3f&);

	void updateProgress(const int value);

public slots:
	void triggerVisionPro();

private:

	bool visionConfigLoadOk = false;

	utils::VisionConfig config_vision;

	ct::Cloud::Ptr m_model_cloud;

	ct::Cloud::Ptr m_scene_cloud;

	ct::Cloud::Ptr m_ground_cloud;

	QMutex mutex;
};


#endif // !PROCESSWPRKER_H
