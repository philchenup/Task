/**
 * @file AisRlBodyBinding.h
 * @brief OCCT AIS_Shape 与 RL 碰撞体 Body 的网格绑定与位姿同步。
 */
﻿#ifndef AIS_RL_BODY_BINDING_H
#define AIS_RL_BODY_BINDING_H

#include <AIS_Shape.hxx>
#include <Standard_Real.hxx>
#include <gp_Trsf.hxx>

#include <rl/math/Transform.h>
#include <rl/sg/Body.h>
#include <rl/sg/Model.h>

/**
 * OpenCASCADE 的 loadShape 长度是毫米；Coin3D / Robotics Library 场景长度是米。
 * 绑定与位姿同步默认乘这个系数。不要去缩放 AIS_Shape 本身，OCC 视图仍按毫米显示。
 */
/** @brief OCCT 毫米到 RL 米的长度换算系数。 */
static const Standard_Real kOccMillimeterToMeter = 0.001;

/**
 * 把 OpenCASCADE 的 AIS_Shape 网格化后绑定为 Robotics Library 场景里的一个 Body。
 *
 * 网格写在 Body 局部坐标系（AIS_Shape::Shape()，不含显示位姿），
 * 顶点坐标会乘 lengthScale（默认毫米 → 米）。
 * OCCT 里拖动/SetLocalTransformation 之后，调用 syncAisPoseToRlBody()
 * 把当前显示位姿写到 body->setFrame()，平移同样乘 lengthScale，碰撞模型会跟着动。
 *
 * linearDeflection 仍是形状单位（毫米）。<=0 时按包围盒对角线的 1% 自动取。
 * 网格在缩放前生成，所以不要把这个偏差改成米。
 *
 * load3DModel() 保持原调用即可：
 *   rlWorkpiece = bindAisShapeToRlBody(loadShape, scene->getModel(1));
 * 拖动后：
 *   syncAisPoseToRlBody(loadShape, rlWorkpiece);
 *
 * @param ais           已有几何的 AIS_Shape（STEP 读入后的显示对象）
 * @param sgModel       RL 场景中的环境模型（不要用机器人那一组 Model 0）
 * @param linearDeflection  网格线偏差，单位毫米；<=0 时自动取
 * @param lengthScale   形状单位到场景单位的长度系数，默认 0.001（mm → m）
 * @return              新创建的 rl::sg::Body*，由 sgModel 持有寿命
 */
rl::sg::Body* bindAisShapeToRlBody(
	AIS_Shape* ais,
	rl::sg::Model* sgModel,
	Standard_Real linearDeflection = -1.0,
	Standard_Real lengthScale = kOccMillimeterToMeter);

/**
 * @brief 将 AIS 当前世界位姿同步到已绑定的 RL Body。
 * @param ais 显示用 AIS_Shape。
 * @param body 已绑定 Body。
 * @param lengthScale 平移缩放（默认 mm→m）。
 * @return 写入 RL 的变换。
 */
rl::math::Transform syncAisPoseToRlBody(
	const AIS_Shape* ais,
	rl::sg::Body* body,
	Standard_Real lengthScale = kOccMillimeterToMeter);

/**
 * gp_Trsf → rl::math::Transform
 *
 * OCCT: p' = ScaleFactor * GetRotation * p + TranslationPart
 * RL:   Eigen::Transform<Real,3,Affine>，写入 linear() 与 translation()
 *
 * 3x3（旋转，以及 gp_Trsf 自带的缩放）原样写入。
 * 只有平移乘 lengthScale。默认 1，不改变单位；绑定路径传入 kOccMillimeterToMeter。
 */
rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf, Standard_Real lengthScale = 1.0);

gp_Trsf GetAisShapeWorldTrsf(const AIS_Shape* ais);

void unbindAisShapesFromRlModel(rl::sg::Model* sgModel);
#endif
