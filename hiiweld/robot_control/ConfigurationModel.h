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
 * @file ConfigurationModel.h
 * @brief 轴空间关节表 Qt 模型
 */
#ifndef CONFIGURATIONMODEL_H
#define CONFIGURATIONMODEL_H

#include <QAbstractTableModel>
#include <rl/math/Vector.h>

/**
 * @brief 关节空间配置表模型：绑定 MainWindow::mdl 显示与编辑关节向量 q。
 */
class ConfigurationModel : public QAbstractTableModel
{
	Q_OBJECT
	
public:
	/** @brief 构造空模型。 */
	ConfigurationModel(QObject* parent = nullptr);
	
	/** @brief 析构。 */
	virtual ~ConfigurationModel();
	
	/** @brief 列数恒为 1。 */
	int columnCount(const QModelIndex& parent = QModelIndex()) const;
	
	/**
	 * @brief 显示/编辑关节值；超限标红；弧度轴以度显示。
	 * @param role DisplayRole/EditRole/ForegroundRole 等。
	 */
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	
	/** @brief 单元格可编辑。 */
	Qt::ItemFlags flags(const QModelIndex &index) const;
	
	/** @brief 垂直表头为关节名称。 */
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
	
	/** @brief 行数 = 位置自由度 DOF。 */
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	/**
	 * @brief 编辑单格：写回 mdl、正运动学并更新场景 Body 位姿。
	 */
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole); 
	
	/**
	 * @brief 批量设置关节向量 q 并刷新整表。
	 * @param q 关节位置（与 mdl 一致）。
	 */
	bool setData(const rl::math::Vector& q); 
	
	std::size_t id; /**< 模型索引（预留）。 */
	
public slots:
	/** @brief 笛卡尔模型变更时重置本模型视图。 */
	void operationalChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);	
	
protected:
	
private:

};

#endif // CONFIGURATIONMODEL_H