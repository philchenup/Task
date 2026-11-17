/**
 * @file readmodel.h
 * @brief OCCT 几何读写：STL/STEP 及 XCAF 颜色导入，AIS 显示封装。
 */
#ifndef READMODEL_H
#define READMODEL_H

#include <TopoDS.hxx>
#include <StlAPI_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ColoredShape.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TDocStd_Document.hxx>
#include <Standard_Handle.hxx>

/** @brief OCCT 模型导入导出（静态工具类）。 */
class ReadModel
{
public:
    /** @brief STEP 带颜色结果：shape 为几何；xcafDoc 非空时含 XCAF 颜色。 */
    struct ColoredModel {
        TopoDS_Shape shape;
        Handle(TDocStd_Document) xcafDoc;
        bool hasColors = false;
    };

    ReadModel();

    /** @brief 读取 STL 三角网格为 TopoDS_Shape。 */
    static TopoDS_Shape readStlModel(Standard_CString filePath);

    /** @brief 读取 STEP（仅几何，不含颜色）。 */
    static TopoDS_Shape readStepModel(Standard_CString filePath);

    /** @brief 读取 STEP + XCAF 面/部件颜色。 */
    static ColoredModel readStepModelWithColors(Standard_CString filePath);

    /** @brief 由 ColoredModel 生成可着色 AIS 对象。 */
    static Handle(AIS_ColoredShape) makeDisplayShape(
        const ColoredModel& model,
        const Quantity_Color& fallbackColor = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    /** @brief 整形状单色 AIS 显示封装。 */
    static Handle(AIS_ColoredShape) makeDisplayShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    /** @brief 写出 STEP 文件。 */
    static void writeStepModel(TopoDS_Shape shape, Standard_CString filePath);

    /** @brief 写出 STL 文件。 */
    static void writeStlModel(TopoDS_Shape shape, Standard_CString filePath);
};

#endif // READMODEL_H