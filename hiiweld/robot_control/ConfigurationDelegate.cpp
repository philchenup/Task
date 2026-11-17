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
 * @file ConfigurationDelegate.cpp
 * @brief ConfigurationDelegate 实现
 */
/**
 * @file ConfigurationDelegate.cpp
 * @brief ConfigurationDelegate 实现
 */

#include <QDoubleSpinBox>
#include <QModelIndex>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Revolute.h>

#include "ConfigurationDelegate.h"
#include "MainWindow.h"

/** @brief 构造。 */
ConfigurationDelegate::ConfigurationDelegate(QObject* parent) :
	QItemDelegate(parent),
	id(0)
{
}

/** @brief 析构。 */
ConfigurationDelegate::~ConfigurationDelegate()
{
}

QWidget*
/** @brief 创建带 joint 限位的编辑器。 */
ConfigurationDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QDoubleSpinBox* editor = new QDoubleSpinBox(parent);
	
	rl::math::Vector maximum = MainWindow::instance()->mdl->getMaximum();
	rl::math::Vector minimum = MainWindow::instance()->mdl->getMinimum();
	Eigen::Matrix<rl::math::Unit, Eigen::Dynamic, 1> qUnits = MainWindow::instance()->mdl->getPositionUnits();
	
	editor->setDecimals(4);

	switch (index.row())
	{
	case 0:
		editor->setSingleStep(stepSize);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		editor->setSingleStep(stepSize);
		break;
	default:
		break;
	}

	if (rl::math::UNIT_RADIAN == qUnits(index.row()))
	{
		editor->setMinimum(minimum(index.row()) * rl::math::RAD2DEG);
		editor->setMaximum(maximum(index.row()) * rl::math::RAD2DEG);
	}
	else
	{
		editor->setMinimum(minimum(index.row()));
		editor->setMaximum(maximum(index.row()));
	}
	
	QObject::connect(editor, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));

	return editor;
}

void
/** @brief 初始化编辑器数值。 */
ConfigurationDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	QDoubleSpinBox* doubleSpinBox = static_cast<QDoubleSpinBox*>(editor);
	doubleSpinBox->setValue(index.model()->data(index, Qt::EditRole).toDouble());
}

void
/** @brief 提交到模型。 */
ConfigurationDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	QDoubleSpinBox* doubleSpinBox = static_cast<QDoubleSpinBox*>(editor);
	doubleSpinBox->interpretText();
	
	if (!model->setData(index, doubleSpinBox->value(), Qt::EditRole))
	{
		doubleSpinBox->setValue(index.model()->data(index, Qt::EditRole).toDouble());
	}
}

void
/** @brief 布局。 */
ConfigurationDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	editor->setGeometry(option.rect);
}	

void
/** @brief 即时提交。 */
ConfigurationDelegate::valueChanged(double d)
{
	emit commitData(static_cast<QWidget*>(QObject::sender()));
}
