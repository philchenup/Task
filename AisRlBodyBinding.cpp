#include "AisRlBodyBinding.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>

namespace {

	SoVRMLShape* makeVrmlMesh(const TopoDS_Shape& shape, Standard_Real deflection, Standard_Real lengthScale)
	{
		Bnd_Box box;
		BRepBndLib::Add(shape, box);
		Standard_Real linDefl = deflection;
		if (linDefl <= 0.0)
		{
			if (box.IsVoid())
				linDefl = 1.0;
			else
				linDefl = 0.01 * std::sqrt(box.SquareExtent());
		}

		// 网格化仍在 OCCT 毫米空间里做。换成米只发生在写入 Coin3D 顶点时。
		BRepMesh_IncrementalMesh mesher(shape, linDefl, Standard_False, 0.5, Standard_True);
		(void)mesher;

		std::vector<SbVec3f> points;
		std::vector<int32_t> indices;
		points.reserve(4096);
		indices.reserve(8192);

		for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
		{
			const TopoDS_Face face = TopoDS::Face(exp.Current());
			TopLoc_Location loc;
			Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
			if (tri.IsNull() || tri->NbTriangles() < 1)
				continue;

			const gp_Trsf trsf = loc.Transformation();
			const Standard_Integer i0 = static_cast<Standard_Integer>(points.size());
			points.reserve(points.size() + static_cast<std::size_t>(tri->NbNodes()));

			for (Standard_Integer n = 1; n <= tri->NbNodes(); ++n)
			{
				const gp_Pnt p = tri->Node(n).Transformed(trsf);
				points.emplace_back(
					static_cast<float>(p.X() * lengthScale),
					static_cast<float>(p.Y() * lengthScale),
					static_cast<float>(p.Z() * lengthScale));
			}

			const Standard_Boolean reversed = (face.Orientation() == TopAbs_REVERSED);
			for (Standard_Integer t = 1; t <= tri->NbTriangles(); ++t)
			{
				Standard_Integer n1, n2, n3;
				tri->Triangle(t).Get(n1, n2, n3);
				if (reversed)
					std::swap(n2, n3);
				indices.push_back(i0 + n1 - 1);
				indices.push_back(i0 + n2 - 1);
				indices.push_back(i0 + n3 - 1);
				indices.push_back(-1);
			}
		}

		if (points.empty() || indices.size() < 4)
			return nullptr;

		SoVRMLCoordinate* coord = new SoVRMLCoordinate();
		coord->point.setNum(static_cast<int>(points.size()));
		SbVec3f* dst = coord->point.startEditing();
		for (std::size_t i = 0; i < points.size(); ++i)
			dst[i] = points[i];
		coord->point.finishEditing();

		SoVRMLIndexedFaceSet* faces = new SoVRMLIndexedFaceSet();
		faces->coord.setValue(coord);
		faces->ccw = TRUE;
		faces->solid = TRUE;
		faces->convex = FALSE;
		faces->coordIndex.setValues(0, static_cast<int>(indices.size()), indices.data());

		SoVRMLAppearance* app = new SoVRMLAppearance();
		app->material.setValue(new SoVRMLMaterial());

		SoVRMLShape* vrml = new SoVRMLShape();
		vrml->appearance.setValue(app);
		vrml->geometry.setValue(faces);
		return vrml;
	}

} // namespace


rl::math::Transform gpTrsfToRl(const gp_Trsf& trsf, Standard_Real lengthScale)
{
	NCollection_Mat4<Standard_Real> m;
	trsf.GetMat4(m);
	rl::math::Transform T;
	T.setIdentity();
	T.linear()(0, 0) = m.GetValue(0, 0);
	T.linear()(0, 1) = m.GetValue(0, 1);
	T.linear()(0, 2) = m.GetValue(0, 2);
	T.linear()(1, 0) = m.GetValue(1, 0);
	T.linear()(1, 1) = m.GetValue(1, 1);
	T.linear()(1, 2) = m.GetValue(1, 2);
	T.linear()(2, 0) = m.GetValue(2, 0);
	T.linear()(2, 1) = m.GetValue(2, 1);
	T.linear()(2, 2) = m.GetValue(2, 2);
	// 平移是长度；3x3 是旋转（以及 gp_Trsf 自己的无量纲缩放），不能乘 lengthScale。
	T.translation() = rl::math::Vector3(
		m.GetValue(0, 3) * lengthScale,
		m.GetValue(1, 3) * lengthScale,
		m.GetValue(2, 3) * lengthScale);

	return T;
}

gp_Trsf GetAisShapeWorldTrsf(const AIS_Shape* ais)
{
	if (ais == nullptr) {
		return gp_Trsf();
	}
	const gp_Trsf aisTrsf = ais->Transformation();
	const gp_Trsf shapeLoc = ais->Shape().Location().Transformation();
	return aisTrsf * shapeLoc;
}

rl::math::Transform syncAisPoseToRlBody(const AIS_Shape* ais, rl::sg::Body* body, Standard_Real lengthScale)
{
	rl::math::Transform t;
	t.setIdentity();
	if (ais == nullptr || body == nullptr)
		return t;

	t = gpTrsfToRl(GetAisShapeWorldTrsf(ais), lengthScale);

	body->setFrame(t);

	return t;
}

rl::sg::Body* bindAisShapeToRlBody(
	AIS_Shape* ais,
	rl::sg::Model* sgModel,
	Standard_Real linearDeflection,
	Standard_Real lengthScale)
{
	if (ais == nullptr || sgModel == nullptr)
		return nullptr;

	const TopoDS_Shape shape = ais->Shape();
	if (shape.IsNull())
		return nullptr;

	SoVRMLShape* vrml = makeVrmlMesh(shape, linearDeflection, lengthScale);
	if (vrml == nullptr)
		return nullptr;

	vrml->ref();
	rl::sg::Body* body = sgModel->create();
	body->setName("occ_ais_body");
	body->create(vrml);
	vrml->unref();

	syncAisPoseToRlBody(ais, body, lengthScale);
	return body;
}

void unbindAisShapesFromRlModel(rl::sg::Model* sgModel)
{
	if (sgModel == nullptr)
		return;
	for (std::size_t i = sgModel->getNumBodies(); i > 0; )
	{
		--i;
		rl::sg::Body* body = sgModel->getBody(i);
		if (body != nullptr && body->getName() == "occ_ais_body") {
			sgModel->remove(body);
			delete body;
		}
	}
}
