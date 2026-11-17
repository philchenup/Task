/**
 * @file readmodel.cpp
 * @brief STL/STEP/XCAF 读写与 AIS 着色显示。
 */
#include "readmodel.h"
#include <Interface_Static.hxx>
#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <OSD_Path.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <StepData_StepModel.hxx>
#include <StepBasic_ConversionBasedUnitAndLengthUnit.hxx>
#include <StepBasic_SiPrefix.hxx>
#include <StepBasic_SiUnitAndLengthUnit.hxx>
#include <StepBasic_SiUnitName.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace {

    Handle(TDocStd_Document) newXcafDocument()
    {
        Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
        Handle(TDocStd_Document) doc;
        app->NewDocument("MDTV-XCAF", doc);
        return doc;
    }

    bool tryGetColor(const Handle(XCAFDoc_ColorTool)& colorTool,
        const TDF_Label& label,
        const TopoDS_Shape& shape,
        Quantity_Color& color)
    {
        if (colorTool.IsNull()) {
            return false;
        }
        if (colorTool->GetColor(label, XCAFDoc_ColorSurf, color)) {
            return true;
        }
        if (colorTool->GetColor(label, XCAFDoc_ColorGen, color)) {
            return true;
        }
        if (colorTool->GetColor(label, XCAFDoc_ColorCurv, color)) {
            return true;
        }
        if (!shape.IsNull()) {
            if (colorTool->GetColor(shape, XCAFDoc_ColorSurf, color)) {
                return true;
            }
            if (colorTool->GetColor(shape, XCAFDoc_ColorGen, color)) {
                return true;
            }
        }
        return false;
    }

    void applyLabelColors(const Handle(XCAFDoc_ShapeTool)& shapeTool,
        const Handle(XCAFDoc_ColorTool)& colorTool,
        const TDF_Label& label,
        const Handle(AIS_ColoredShape)& ais)
    {
        if (shapeTool.IsNull() || colorTool.IsNull() || ais.IsNull() || label.IsNull()) {
            return;
        }

        TopoDS_Shape shape = shapeTool->GetShape(label);
        Quantity_Color color;
        if (!shape.IsNull() && tryGetColor(colorTool, label, shape, color)) {
            ais->SetCustomColor(shape, color);
        }

        if (shapeTool->IsAssembly(label)) {
            TDF_LabelSequence comps;
            shapeTool->GetComponents(label, comps);
            for (Standard_Integer i = 1; i <= comps.Length(); ++i) {
                applyLabelColors(shapeTool, colorTool, comps.Value(i), ais);
            }
            return;
        }

        if (shapeTool->IsReference(label)) {
            TDF_Label ref;
            if (shapeTool->GetReferredShape(label, ref)) {
                applyLabelColors(shapeTool, colorTool, ref, ais);
            }
        }

        TDF_LabelSequence subs;
        shapeTool->GetSubShapes(label, subs);
        for (Standard_Integer i = 1; i <= subs.Length(); ++i) {
            applyLabelColors(shapeTool, colorTool, subs.Value(i), ais);
        }

        if (!shape.IsNull()) {
            for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
                const TopoDS_Shape face = exp.Current();
                Quantity_Color faceColor;
                if (colorTool->GetColor(face, XCAFDoc_ColorSurf, faceColor) ||
                    colorTool->GetColor(face, XCAFDoc_ColorGen, faceColor)) {
                    ais->SetCustomColor(face, faceColor);
                }
            }
        }
    }

    Handle(AIS_ColoredShape) shadeDefaults(const Handle(AIS_ColoredShape)& ais,
        const Quantity_Color& color)
    {
        if (ais.IsNull()) {
            return ais;
        }
        ais->SetColor(color);
        ais->SetDisplayMode(AIS_Shaded);
        ais->Attributes()->SetFaceBoundaryDraw(Standard_True);
        ais->Attributes()->SetFaceBoundaryAspect(
            new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, 1.0));
        return ais;
    }

    // true = 米（不换算）；false = 毫米（默认，需缩放到米）
    bool stepUnitIsMetre(const Handle(StepData_StepModel)& model)
    {
        if (model.IsNull()) {
            return false;
        }
        bool sawMetre = false;
        bool sawMilli = false;
        for (Standard_Integer i = 1; i <= model->NbEntities(); ++i) {
            Handle(StepBasic_ConversionBasedUnitAndLengthUnit) conv =
                Handle(StepBasic_ConversionBasedUnitAndLengthUnit)::DownCast(model->Value(i));
            if (!conv.IsNull() && !conv->Name().IsNull()) {
                TCollection_AsciiString n(conv->Name()->ToCString());
                n.UpperCase();
                if (n.Search("MILLI") >= 1 || n.IsEqual("MM")) {
                    return false;
                }
                if (n.IsEqual("METRE") || n.IsEqual("METER") || n.IsEqual("M")) {
                    return true;
                }
            }
            Handle(StepBasic_SiUnitAndLengthUnit) si =
                Handle(StepBasic_SiUnitAndLengthUnit)::DownCast(model->Value(i));
            if (!si.IsNull() && si->Name() == StepBasic_sunMetre) {
                if (si->HasPrefix() && si->Prefix() == StepBasic_spMilli) {
                    sawMilli = true;
                }
                else if (!si->HasPrefix()) {
                    sawMetre = true;
                }
            }
        }
        if (sawMilli) {
            return false;
        }
        return sawMetre;
    }

    gp_Pnt shapeCenter(const TopoDS_Shape& shape)
    {
        Bnd_Box box;
        BRepBndLib::Add(shape, box);
        if (box.IsVoid()) {
            return gp_Pnt(0, 0, 0);
        }
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        return gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
    }

} // namespace

/** @brief 占位构造（接口均为静态）。
ReadModel::ReadModel()
{

}

/** @brief 对形状施加均匀缩放（文件内辅助函数）。
TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape, double scaleFactor)
{
    if (inputShape.IsNull()) {
        return TopoDS_Shape();
    }

    gp_Trsf scaleTransform;
    // 以包围盒圆心 (x,y,z) 为缩放中心，等比例缩放
    scaleTransform.SetScale(shapeCenter(inputShape), scaleFactor);

    BRepBuilderAPI_Transform transformTool(inputShape, scaleTransform, Standard_True);
    transformTool.Build();

    TopoDS_Shape scaledShape = transformTool.Shape();
    if (scaledShape.IsNull()) {
        return TopoDS_Shape();
    }
    return scaledShape;
}

/** @brief 通过 StlAPI_Reader 读 STL。 */
TopoDS_Shape ReadModel::readStlModel(Standard_CString filePath)
{
    Handle(Poly_Triangulation) tri = RWStl::ReadFile(OSD_Path(filePath));

    TopoDS_Shape shape_Stl;
    if (tri.IsNull())
        return shape_Stl;

    TopoDS_Face face;
    BRep_Builder builder;
    builder.MakeFace(face, tri);

    shape_Stl = face;
    return shape_Stl;
}

/** @brief 通过 STEPControl_Reader 读 STEP。 */
TopoDS_Shape ReadModel::readStepModel(Standard_CString filePath) {

    STEPControl_Reader stepReader;
    IFSelect_ReturnStatus readStatus = stepReader.ReadFile(filePath);
    if (readStatus != IFSelect_RetDone) {
        return TopoDS_Shape();
    }

    const bool isMetre = stepUnitIsMetre(stepReader.StepModel());
    Interface_Static::SetCVal("read.step.unit", isMetre ? "M" : "MM");
    stepReader.TransferRoots();
    TopoDS_Shape shape = stepReader.OneShape();
    // mm → 绕圆心缩放到 m；m → 直接读取
    if (!isMetre && !shape.IsNull()) {
        shape = ScaleShape(shape, 0.001);
    }
    return shape;
}

/** @brief STEPCAF 读入几何以与面颜色。 */
ReadModel::ColoredModel ReadModel::readStepModelWithColors(Standard_CString filePath)
{
    ColoredModel out;
    if (filePath == nullptr) {
        return out;
    }

    Handle(TDocStd_Document) doc = newXcafDocument();
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);

    if (reader.ReadFile(filePath) != IFSelect_RetDone) {
        out.shape = readStepModel(filePath);
        return out;
    }

    const bool isMetre = stepUnitIsMetre(reader.ChangeReader().StepModel());
    Interface_Static::SetCVal("read.step.unit", isMetre ? "M" : "MM");
    if (!reader.Transfer(doc)) {
        out.shape = readStepModel(filePath);
        return out;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(doc->Main());

    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        return out;
    }

    if (freeShapes.Length() == 1) {
        out.shape = shapeTool->GetShape(freeShapes.Value(1));
    }
    else {
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
            builder.Add(compound, shapeTool->GetShape(freeShapes.Value(i)));
        }
        out.shape = compound;
    }

    TDF_LabelSequence colorLabels;
    colorTool->GetColors(colorLabels);
    out.hasColors = colorLabels.Length() > 0;
    out.xcafDoc = doc;
    /*if (!isMetre && !out.shape.IsNull()) {
        out.shape = ScaleShape(out.shape, 1);
    }*/
    return out;
}

/** @brief 根据 XCAF 颜色或 fallback 构建 AIS_ColoredShape。 */
Handle(AIS_ColoredShape) ReadModel::makeDisplayShape(
    const ColoredModel& model,
    const Quantity_Color& fallbackColor)
{
    if (model.shape.IsNull()) {
        return Handle(AIS_ColoredShape)();
    }

    Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(model.shape);
    shadeDefaults(ais, fallbackColor);

    if (model.hasColors && !model.xcafDoc.IsNull()) {
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(model.xcafDoc->Main());
        Handle(XCAFDoc_ColorTool) colorTool =
            XCAFDoc_DocumentTool::ColorTool(model.xcafDoc->Main());

        TDF_LabelSequence freeShapes;
        shapeTool->GetFreeShapes(freeShapes);
        for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
            applyLabelColors(shapeTool, colorTool, freeShapes.Value(i), ais);
        }
    }

    return ais;
}

/** @brief 单色 AIS_ColoredShape 封装。 */
Handle(AIS_ColoredShape) ReadModel::makeDisplayShape(
    const TopoDS_Shape& shape,
    const Quantity_Color& color)
{
    if (shape.IsNull()) {
        return Handle(AIS_ColoredShape)();
    }
    Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(shape);
    return shadeDefaults(ais, color);
}

/** @brief STEP 写出。
void ReadModel::writeStepModel(TopoDS_Shape shape, Standard_CString filePath)
{
    if (shape.IsNull()) return;

    Interface_Static::SetCVal("write.step.unit", "M");
    STEPControl_Writer write;

    IFSelect_ReturnStatus stat1 = write.Transfer(shape, STEPControl_AsIs);

    IFSelect_ReturnStatus stat2 = write.Write(filePath);

    return;
}

/** @brief STL 写出。
void ReadModel::writeStlModel(TopoDS_Shape shape, Standard_CString filePath) {
    if (shape.IsNull()) return;

    StlAPI_Writer stlWriter;

    stlWriter.Write(shape, filePath);

    return;
}