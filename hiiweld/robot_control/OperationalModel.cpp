//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
/**
 * @file OperationalModel.cpp
 * @brief OperationalModel 实现（雅可比 IK）
 */

#include <QStatusBar>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Exception.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/sg/Body.h>

#include "ConfigurationModel.h"
#include "OperationalModel.h"
#include "MainWindow.h"

/** @brief 构造。 */
OperationalModel::OperationalModel(QObject* parent) :
	QAbstractTableModel(parent),
	id(0)
{
}

/** @brief 析构。 */
OperationalModel::~OperationalModel()
{
}

int
/** @brief 列数 1。 */
OperationalModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

void
/** @brief 关节变化刷新。 */
OperationalModel::configurationChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->beginResetModel();
	this->endResetModel();
}

void
/** @brief 切换 TCP 显示。 */
OperationalModel::setDisplayTcp(bool tcp)
{
	if (showTcp == tcp)
	{
		return;
	}
	showTcp = tcp;
	this->beginResetModel();
	this->endResetModel();
}

void
/** @brief 设置 TCP 工具变换。 */
OperationalModel::setToolTransform(const rl::math::Transform& T)
{
	T_flange_tcp = T;
	if (showTcp)
	{
		this->beginResetModel();
		this->endResetModel();
	}
}

rl::math::Transform
/** @brief 法兰或 TCP 位姿。 */
OperationalModel::displayedTransform(std::size_t column) const
{
	const rl::math::Transform& T_flange = MainWindow::instance()->mdl->getOperationalPosition(column);
	return showTcp ? flangeToTcp(T_flange, T_flange_tcp) : T_flange;
}

QVariant
/** @brief 显示/编辑笛卡尔分量。 */
OperationalModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return QVariant();
	}

	if (!index.isValid())
	{
		return QVariant();
	}

	int num = index.column();

	const rl::math::Transform transform = displayedTransform(index.column());
	rl::math::Transform::ConstTranslationPart position = transform.translation();
	rl::math::Vector3 orientation = transform.rotation().eulerAngles(2, 1, 0).reverse();

	switch (role)
	{
	case Qt::DisplayRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return QString::number(position(index.row()), 'f', 4) + QString(" m");
			break;
		case 3:
		case 4:
		case 5:
			return QString::number(orientation(index.row() - 3) * rl::math::RAD2DEG, 'f', 4) + QChar(176);
			break;
		default:
			break;
		}
		break;
	case Qt::EditRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return position(index.row());
			break;
		case 3:
		case 4:
		case 5:
			return orientation(index.row() - 3) * rl::math::RAD2DEG;
			break;
		default:
			break;
		}
		break;
	case Qt::TextAlignmentRole:
		return QVariant(Qt::AlignRight | Qt::AlignVCenter);
		break;
	default:
		break;
	}

	return QVariant();
}

Qt::ItemFlags
/** @brief 可编辑。 */
OperationalModel::flags(const QModelIndex& index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}

	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
/** @brief XYZABC 与 TCP/Flange 列名。 */
OperationalModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return QVariant();
	}

	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		switch (section)
		{
		case 0:
			return "X";
			break;
		case 1:
			return "Y";
			break;
		case 2:
			return "Z";
			break;
		case 3:
			return "A";
			break;
		case 4:
			return "B";
			break;
		case 5:
			return "C";
			break;
		default:
			break;
		}
	}

	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		return showTcp ? QString("TCP") : QString("Flange");
	}

	return QVariant();
}

int
/** @brief 固定 6 行。 */
OperationalModel::rowCount(const QModelIndex& parent) const
{
	/*if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return 0;
	}

	return MainWindow::instance()->kinematicModels[this->id]->getOperationalDof();*/
	return 6;
}

bool
/** @brief 增量运动 + 雅可比 IK 更新。 */
OperationalModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return false;
	}

	if (index.isValid() && Qt::EditRole == role)
	{
		if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->mdl.get()))
		{
			rl::math::Transform transform = displayedTransform(index.column());
			rl::math::Vector3 orientation = transform.linear().eulerAngles(2, 1, 0).reverse();

			double current = 0.0;
			if (index.row() < 3)
			{
				current = transform.translation()(index.row());
			}
			else if (index.row() < 6)
			{
				current = orientation(index.row() - 3) * rl::math::RAD2DEG;
			}
			else
			{
				return false;
			}

			const double delta = value.value<rl::math::Real>() - current;
			rl::math::Transform offset = rl::math::Transform::Identity();
			switch (index.row())
			{
			case 0:
				offset.translation() = rl::math::Vector3(delta, 0, 0);
				break;
			case 1:
				offset.translation() = rl::math::Vector3(0, delta, 0);
				break;
			case 2:
				offset.translation() = rl::math::Vector3(0, 0, delta);
				break;
			case 3:
				offset.linear() = rl::math::AngleAxis(delta * rl::math::DEG2RAD, rl::math::Vector3::UnitX()).toRotationMatrix();
				break;
			case 4:
				offset.linear() = rl::math::AngleAxis(delta * rl::math::DEG2RAD, rl::math::Vector3::UnitY()).toRotationMatrix();
				break;
			case 5:
				offset.linear() = rl::math::AngleAxis(delta * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()).toRotationMatrix();
				break;
			default:
				return false;
			}

						// TCP 模式沿 TCP 轴偏移，否则沿法兰轴偏移，再变回法兰求 IK
			transform = transform * offset;
			rl::math::Transform goal = showTcp ? tcpToFlange(transform, T_flange_tcp) : transform;

			rl::math::Vector q = kinematic->getPosition();

			std::shared_ptr<rl::mdl::InverseKinematics> ik;

			ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematic);
			rl::mdl::JacobianInverseKinematics* jacobianIk = static_cast<rl::mdl::JacobianInverseKinematics*>(ik.get());
			jacobianIk->setMethod(rl::mdl::JacobianInverseKinematics::Method::svd);

			for (std::size_t i = 0; i < kinematic->getOperationalDof(); ++i)
			{
				ik->addGoal(i == index.column() ? goal : kinematic->getOperationalPosition(i), i);
			}

			std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
			bool solved = ik->solve();
			std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();

			if (solved) {
				kinematic->forwardPosition();
				rl::math::Vector q_after = kinematic->getPosition();
				double sum_of_squares = (q_after - q).squaredNorm();
				if (std::sqrt(sum_of_squares) > 20) solved = false;
			}

			if (solved)
			{
				MainWindow::instance()->statusBar()->showMessage("IK solved in " + QString::number(std::chrono::duration<double>(stop - start).count() * rl::math::UNIT2MILLI) + " ms", 2000);

				kinematic->forwardPosition();

				for (std::size_t i = 0; i < MainWindow::instance()->sceneModel->getNumBodies(); ++i)
				{
					MainWindow::instance()->sceneModel->getBody(i)->setFrame(kinematic->getBodyFrame(i));
				}

				emit dataChanged(this->createIndex(0, 0), this->createIndex(this->rowCount(), this->columnCount()));

				if (IsCartMove) {
					rl::math::Vector cur_joint = kinematic->getPosition();
					emit sendConfigurationData(cur_joint);
					IsCartMove = false;
				}

				return true;
			}
			else
			{
				MainWindow::instance()->statusBar()->showMessage("IK failed", 2000);

				kinematic->setPosition(q);
				kinematic->forwardPosition();
			}
		}
	}

	return false;
}

bool
/** @brief 点动接口。 */
OperationalModel::stepAxis(int axis, int dir, double stepSize)
{
	IsCartMove = true;
	if (axis < 0 || axis >= rowCount() || dir == 0 || stepSize <= 0.0)
		return false;
	const QModelIndex idx = index(axis, 0);
	const double current = data(idx, Qt::EditRole).toDouble();
	return setData(idx, current + dir * stepSize, Qt::EditRole);
}