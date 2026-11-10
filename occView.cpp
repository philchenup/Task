#include <OpenGl_GraphicDriver.hxx>

#include "occView.h"
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>
#include <QMenu>
#include <QMouseEvent> //橡皮筋类
#include <QRubberBand>
#include <QStyleFactory>
#include <windows.h>
#include <QWidget>
#include <BRepMesh_IncrementalMesh.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopExp.hxx>
#include <Aspect_Handle.hxx> //OpenCASCADE中的窗口句柄类
#include <Aspect_DisplayConnection.hxx> //OpenCASCADE中的显示连接类
#include <AIS_Shape.hxx>
#include <AIS_ViewCube.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopExp_Explorer.hxx>
#include <AIS_TextLabel.hxx>
#include <Resource_Unicode.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Pln.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Geom_Plane.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <QKeyEvent>
#include <AIS_Trihedron.hxx>
#include <AIS_Point.hxx>

#ifdef WNT
#include <WNT_Window.hxx>
#else
#undef Bool
#undef CursorShape
#undef None
#undef KeyPress
#undef KeyRelease
#undef FocusIn
#undef FocusOut
#undef FontChange
#undef Expose
#include <Xw_Window.hxx>
#endif

static Handle(Graphic3d_GraphicDriver)& GetGraphicDriver()
{
    static Handle(Graphic3d_GraphicDriver) aGraphicDriver;
    return aGraphicDriver; //获取图像驱动句柄
}

OccView::OccView(QWidget* parent)
    : QWidget(parent),
    myXmin(0),
    myYmin(0),
    myXmax(0),
    myYmax(0),
    myCurrentMode(CurAction3d_DynamicRotation),
    myDegenerateModeIsOn(Standard_True),
    myRectBand(NULL)
{
    // No Background
    setBackgroundRole(QPalette::NoRole);

    // set focus policy to threat QContextMenuEvent from keyboard
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);

    resize(800, 600);

    // Enable the mouse tracking, by default the mouse tracking is disabled.
    setMouseTracking(true);

    init();
}

void OccView::init()
{
    // 创建Aspect_DisplayConnection对象，用于处理与显示系统的连接。
    Handle(Aspect_DisplayConnection) aDisplayConnection =
        new Aspect_DisplayConnection();

    // 检查图形驱动是否已存在，如果不存在则创建一个新的OpenGl_GraphicDriver对象，并将其与Aspect_DisplayConnection关联。
    if (GetGraphicDriver().IsNull())
    {
        GetGraphicDriver() = new OpenGl_GraphicDriver(aDisplayConnection);
    }

    // 获取窗口的句柄. This returns something suitable for all platforms.
    WId window_handle = (WId)winId();


    // 根据平台选择性的创建相应类型的窗口对象
#ifdef WNT
    Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle)window_handle);
#elif defined(__APPLE__) && !defined(MACOSX_USE_GLX)
    Handle(Cocoa_Window) wind = new Cocoa_Window((NSView*)window_handle);
#else
    Handle(Xw_Window) wind = new Xw_Window(aDisplayConnection, (Window)window_handle);
#endif

    // 创建 V3dViewer and V3d_View对象，设置视图窗口，并检查窗口是否以及映射
    // myViewer = new V3d_Viewer(GetGraphicDriver(), Standard_ExtString("viewer3d")); 这一行会报错，应该改成下一行
    myViewer = new V3d_Viewer(GetGraphicDriver());

    myView = myViewer->CreateView();

    myView->SetWindow(wind);
    if (!wind->IsMapped()) wind->Map();

    // 创建AIS_InteractiveContext对象，用于处理交互式上下文。
    myContext = new AIS_InteractiveContext(myViewer);

    // 设置默认光源，打开光照
    myViewer->SetDefaultLights();
    myViewer->SetLightOn();

    // 背景设置为渐变色
    Standard_Real R1 = 0;
    Standard_Real G1 = 0;
    Standard_Real B1 = 0;
    Quantity_Color aColor1(R1, G1, B1, Quantity_TOC_RGB);

    Standard_Real R2 = 50 / 255.;
    Standard_Real G2 = 50 / 255.;
    Standard_Real B2 = 50 / 255.;
    Quantity_Color aColor2(R2, G2, B2, Quantity_TOC_RGB);

    Aspect_GradientFillMethod aMethod = Aspect_GradientFillMethod(2);
    myView->SetBgGradientColors(aColor1, aColor2, aMethod, 1);

    /*Standard_Real R2 = 50 / 255.;
    Standard_Real G2 = 50 / 255.;
    Standard_Real B2 = 50 / 255.;
    Quantity_Color aColor2(R2, G2, B2, Quantity_TOC_RGB);
    myView->SetBackgroundColor(aColor2);*/

    myView->Redraw();

    myView->MustBeResized();//窗口大小发生变化时，视图会被要求重新调整大小以适应新的窗口尺寸

    Handle(AIS_ViewCube) H_AisViewCube = new AIS_ViewCube();

    H_AisViewCube->SetBoxColor(Quantity_NOC_WHITE); //设置视方体为白色
    H_AisViewCube->SetTransparency(0.5);   //设置透明度
    //设置视方体每个面的字体为黑色
    H_AisViewCube->SetTextColor(Quantity_Color(Quantity_NOC_BLACK));
    H_AisViewCube->SetFontHeight(18); //设置视方体每个面字体的高度

    //设置视方体每个面的边界线
    //首先创建一个Prs3d_Drawer 对象，一切属性的修改在myDraw 下进行
    const Handle(Prs3d_Drawer)& myDrawer = H_AisViewCube->Attributes();
    myDrawer->SetupOwnFaceBoundaryAspect(); //设置的预前工作（十分重要）
    myDrawer->SetFaceBoundaryDraw(true);

    //设置边界线的颜色为蓝色
    myDrawer->FaceBoundaryAspect()->SetColor(Quantity_NameOfColor::Quantity_NOC_LIGHTSKYBLUE);
    myDrawer->FaceBoundaryAspect()->SetWidth(2.0);//设置边界线的宽度
    // H_AisViewCube->SetMaterial(Graphic3d_MaterialAspect(Graphic3d_NOM_DEFAULT));//设置材质

    //设置视方体基准线
    myDrawer->SetDatumAspect(new Prs3d_DatumAspect()); //设置的预前工作（十分重要）
    const Handle_Prs3d_DatumAspect& datumAspect = H_AisViewCube->Attributes()->DatumAspect();

    //设置轴颜色
    datumAspect->ShadingAspect(Prs3d_DP_XAxis)->SetColor(Quantity_NOC_RED);
    datumAspect->ShadingAspect(Prs3d_DP_YAxis)->SetColor(Quantity_NOC_GREEN);
    datumAspect->ShadingAspect(Prs3d_DP_ZAxis)->SetColor(Quantity_NOC_BLUE);

    //设置X,Y,Z文本颜色
    datumAspect->TextAspect(Prs3d_DP_XAxis)->SetColor(Quantity_NOC_RED);
    datumAspect->TextAspect(Prs3d_DP_YAxis)->SetColor(Quantity_NOC_GREEN);
    datumAspect->TextAspect(Prs3d_DP_ZAxis)->SetColor(Quantity_NOC_BLUE);

    H_AisViewCube->SetTransformPersistence(
        new Graphic3d_TransformPers(
            Graphic3d_TMF_TriedronPers,
            Aspect_TOTP_LEFT_LOWER/*设置在左下角*/,
            Graphic3d_Vec2i(100, 100)));
    myContext->Display(H_AisViewCube, Standard_True);

    const Standard_Real kGroundHalf = 10.0;   // 地面半边长，按你的场景单位(m)调整
    gp_Pln aGroundPln(gp_Pnt(0.0, 0.0, 3.10), gp_Dir(0.0, 0.0, 1.0));
    TopoDS_Face aGroundFace = BRepBuilderAPI_MakeFace(aGroundPln, -kGroundHalf, kGroundHalf, -kGroundHalf, kGroundHalf).Face();

    Handle(AIS_Shape) aGround = new AIS_Shape(aGroundFace);
    aGround->SetColor(Quantity_NameOfColor::Quantity_NOC_GRAY50);
    aGround->SetMaterial(Graphic3d_NameOfMaterial_Stone);
    aGround->SetTransparency(0.0f);
    myContext->Display(aGround, Standard_False);
    myContext->SetSelectionModeActive(aGround, -1, Standard_False);

    myView->SetProj(1.0, -1.0, -1.0);
    myView->SetUp(0.0, 0.0, -1.0);

    myView->Redraw();

    myContext->SetDisplayMode(AIS_Shaded, Standard_True);
}

const Handle(AIS_InteractiveContext)& OccView::getContext() const
{
    return myContext;
}

const Handle(V3d_View)& OccView::getV3dView() const
{
    return myView;
}
/*!
Get paint engine for the OpenGL viewer. [ virtual public ]
*/
//OccView 小部件不使用 Qt 的默认绘图引擎。这可能是因为在使用 OpenGL 进行绘图的情况下，直接与 OpenGL 交互更为有效，而不需要使用 Qt 的绘图引擎。
QPaintEngine* OccView::paintEngine() const
{
    return 0;
}

// 覆盖了 Qt 的虚函数 paintEvent。这个函数在小部件需要重绘时被调用。
void OccView::paintEvent(QPaintEvent* /*theEvent*/)
{
    // myView表示OPENCASCADE中的三维视图
    myView->Redraw(); //三维视图进行重新绘制。这通常是为了在视图中显示更新后的图形内容。
}

// 覆盖了 Qt 的虚函数 resizeEvent。这个函数在小部件的大小调整时被调用。
void OccView::resizeEvent(QResizeEvent* /*theEvent*/)
{
    if (!myView.IsNull())
    {
        myView->MustBeResized(); //要求三维视图进行必要的调整，以适应新的窗口大小
    }
}

//调整三维视图以适应所有显示的对象，并在调整后重新绘制
void OccView::fitAll(void)
{
    myView->FitAll();  //调整视图以适应所有显示的对象，使它们都可见
    myView->ZFitAll(); //调整视图的深度范围以确保所有对象都在深度范围内
    myView->Redraw();  //重新绘制视图，以反映上述调整
}

void OccView::Redraw(void)
{
    myView->Redraw();
}

//重置三维视图的状态，将其恢复至初始状态
void OccView::reset(void)
{
    myView->Reset(); //重置视图的状态
}

//平移
void OccView::pan(void)
{
    //更改当前操作模式为动态平移
    myCurrentMode = CurAction3d_DynamicPanning;
}

//缩放
void OccView::zoom(void)
{
    //更改当前操作模式为动态缩放
    myCurrentMode = CurAction3d_DynamicZooming;
}

//旋转
void OccView::rotate(void)
{
    //更改当前操作为动态旋转
    myCurrentMode = CurAction3d_DynamicRotation;
}

//鼠标按下事件回调函数
void OccView::mousePressEvent(QMouseEvent* theEvent)
{
    if (theEvent->button() == Qt::LeftButton) //左键操作
    {
        onLButtonDown((theEvent->buttons() | theEvent->modifiers()), theEvent->pos());
    }
    else if (theEvent->button() == Qt::MiddleButton) //中键操作
    {
        onMButtonDown((theEvent->buttons() | theEvent->modifiers()), theEvent->pos());
    }
    else if (theEvent->button() == Qt::RightButton) //右键操作
    {
        onRButtonDown((theEvent->buttons() | theEvent->modifiers()), theEvent->pos());
    }
}

//鼠标释放事件回调函数
void OccView::mouseReleaseEvent(QMouseEvent* theEvent)
{
    if (theEvent->button() == Qt::LeftButton) //左键释放
    {
        onLButtonUp(theEvent->buttons() | theEvent->modifiers(), theEvent->pos());
    }
    else if (theEvent->button() == Qt::MiddleButton) //中键释放
    {
        onMButtonUp(theEvent->buttons() | theEvent->modifiers(), theEvent->pos());
    }
    else if (theEvent->button() == Qt::RightButton) //右键释放
    {
        onRButtonUp(theEvent->buttons() | theEvent->modifiers(), theEvent->pos());
    }
}

//鼠标移动回调函数
void OccView::mouseMoveEvent(QMouseEvent* theEvent)
{
    onMouseMove(theEvent->buttons(), theEvent->pos());
}

//鼠标滚轮回调函数
void OccView::wheelEvent(QWheelEvent* theEvent)
{
    onMouseWheel(theEvent->buttons(), theEvent->delta(), theEvent->pos());
}

//处理左键按下事件
void OccView::onLButtonDown(const int /*theFlags*/, const QPoint thePoint)
{
    // 在min中保存当前的鼠标坐标.
    myXmin = thePoint.x();
    myYmin = thePoint.y();
    myXmax = thePoint.x();
    myYmax = thePoint.y();

    if (!myManipulator.IsNull() && myManipulator->HasActiveMode())
    {
        myManipulator->StartTransform(thePoint.x(), thePoint.y(), myView);
        myIsManipulating = Standard_True;
    }
}

//处理中键按下事件
void OccView::onMButtonDown(const int theFlags, const QPoint thePoint)
{
    // Save the current mouse coordinate in min.
    myXmin = thePoint.x();
    myYmin = thePoint.y();
    myXmax = thePoint.x();
    myYmax = thePoint.y();

    if (theFlags & Qt::ControlModifier)
    {
        pan();
    }
    else
    {
        rotate();
    }
    if (myCurrentMode == CurAction3d_DynamicRotation) //当前模式为动态旋转
    {
        myView->StartRotation(thePoint.x(), thePoint.y()); //启动动态旋转
    }
}

//处理右键按下事件
void OccView::onRButtonDown(const int theFlags, const QPoint thePoint)
{
    this->setContextMenuPolicy(Qt::CustomContextMenu);
    QMenu menu(this);
    // 添加“焊缝选择”菜单项
    QAction* pointSelectAction = menu.addAction(u8"点选模式");
    QAction* faceSelectAction = menu.addAction(u8"面选模式");
    QAction* singleWeldSelectAction = menu.addAction(u8"单焊缝提取");
    QAction* allWeldSelectAction = menu.addAction(u8"全焊缝提取");
    QAction* activeAction = menu.addAction(u8"场景激活");

    QAction* selectedAction = menu.exec(this->mapToGlobal(thePoint));

    if (selectedAction == pointSelectAction) {
        setSelectMode(SelectMode::Mode_Vertex);
    }
    else if (selectedAction == faceSelectAction) {
        setSelectMode(SelectMode::Mode_Face);
    }
    else if (selectedAction == singleWeldSelectAction) {
        setSelectMode(SelectMode::Mode_Edge);
    }
    else if (selectedAction == allWeldSelectAction) {
        setSelectMode(SelectMode::Mode_AllEdge);
    }
    else if (selectedAction == activeAction) {
        this->getContext()->Deactivate();
        this->getContext()->Activate(0);
    }

    return;
}

std::vector<TopoDS_Edge> OccView::extractCommonEdgesWithConditions(const TopoDS_Shape& shape)
{
    std::vector<TopoDS_Edge> resultEdges;
    if (shape.IsNull()) {
        return resultEdges;
    }

    constexpr double kMinWeldLength = 10; // 模型单位，只去掉碎边
    constexpr double kParallelDot = 0.9962; // 约 5°，共面分割不算焊缝

    // 外法向指向 face1 内部时，材料张角大于 180°，是角焊缝凹边
    auto isConcaveEdge = [](const TopoDS_Edge& edge,
        const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const gp_Vec& n1,
        const gp_Vec& n2) -> bool {
            BRepAdaptor_Curve curve(edge);
            const double mid = 0.5 * (curve.FirstParameter() + curve.LastParameter());
            gp_Pnt point;
            gp_Vec tangent;
            curve.D1(mid, point, tangent);
            if (tangent.SquareMagnitude() <= gp::Resolution()) {
                return false;
            }
            tangent.Normalize();

            TopAbs_Orientation orientationInFace = TopAbs_FORWARD;
            bool found = false;
            for (TopExp_Explorer exp(face1, TopAbs_EDGE); exp.More(); exp.Next()) {
                if (exp.Current().IsSame(edge)) {
                    orientationInFace = exp.Current().Orientation();
                    found = true;
                    break;
                }
            }
            if (!found) {
                return false;
            }
            if (orientationInFace == TopAbs_REVERSED) {
                tangent.Reverse();
            }

            gp_Vec intoFace1 = n1.Crossed(tangent);
            if (intoFace1.SquareMagnitude() <= gp::Resolution()) {
                return false;
            }
            intoFace1.Normalize();
            return n2.Dot(intoFace1) > 1.0e-6;
        };

    TopTools_IndexedDataMapOfShapeListOfShape edgeMap;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeMap);

    for (int i = 1; i <= edgeMap.Extent(); ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeMap.FindKey(i));
        if (BRep_Tool::Degenerated(edge)) {
            continue;
        }

        const TopTools_ListOfShape& faceList = edgeMap.FindFromIndex(i);
        if (faceList.Extent() != 2) {
            continue;
        }

        auto faceIt = faceList.cbegin();
        const TopoDS_Face& face1 = TopoDS::Face(*faceIt);
        ++faceIt;
        const TopoDS_Face& face2 = TopoDS::Face(*faceIt);

        BRepAdaptor_Surface surf1(face1);
        BRepAdaptor_Surface surf2(face2);
        if (surf1.GetType() != GeomAbs_Plane || surf2.GetType() != GeomAbs_Plane) {
            continue;
        }

        gp_Vec normal1(surf1.Plane().Axis().Direction());
        gp_Vec normal2(surf2.Plane().Axis().Direction());
        if (face1.Orientation() == TopAbs_REVERSED) normal1.Reverse();
        if (face2.Orientation() == TopAbs_REVERSED) normal2.Reverse();
        normal1.Normalize();
        normal2.Normalize();

        if (std::abs(normal1.Dot(normal2)) > kParallelDot) {
            continue;
        }
        if (!isConcaveEdge(edge, face1, face2, normal1, normal2)) {
            continue;
        }

        GProp_GProps edgeProps;
        BRepGProp::LinearProperties(edge, edgeProps);
        if (edgeProps.Mass() < kMinWeldLength) {
            continue;
        }

        resultEdges.push_back(edge);
    }

    return resultEdges;
}

void OccView::setSelectMode(SelectMode mode)
{
    mySelectMode = mode;

    this->getContext()->Deactivate();

    std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
    this->myContext->InitSelected();

    while (this->myContext->MoreSelected())
    {
        Handle(AIS_InteractiveObject) aAis = this->myContext->SelectedInteractive();
        selectedObjects.push_back(aAis);
        this->myContext->NextSelected();
    }

    if (selectedObjects.size() == 0) return;

    for (auto aAis : selectedObjects)
    {
        if (mode == SelectMode::Mode_Vertex) {
            this->getContext()->Activate(aAis, AIS_Shape::SelectionMode(TopAbs_VERTEX));
        }
        else if (mode == SelectMode::Mode_Face) {
            this->getContext()->Activate(aAis, AIS_Shape::SelectionMode(TopAbs_FACE));
        }
        else if (mode == SelectMode::Mode_Edge) {
            this->getContext()->Activate(aAis, AIS_Shape::SelectionMode(TopAbs_EDGE));
        }
        else if (mode == SelectMode::Mode_AllEdge) {
            Handle(AIS_Shape) shapeObj = Handle(AIS_Shape)::DownCast(aAis);
            if (!shapeObj.IsNull()) {
                TopoDS_Shape selectedShape = shapeObj->Shape();
                std::vector<TopoDS_Edge> weldEdges = extractCommonEdgesWithConditions(selectedShape);
                if (weldEdges.size() < 1) return;
                emit sendExtractEdges(selectedShape, weldEdges);
            }
        }
    }
    this->myContext->UpdateCurrentViewer();
}

TopoDS_Shape OccView::getSelectShape() {
    TopoDS_Shape currentSelectionShape;

    std::vector<Handle(AIS_InteractiveObject)> selectedObjects;
    this->myContext->InitSelected();

    while (this->myContext->MoreSelected())
    {
        Handle(AIS_InteractiveObject) aAis = this->myContext->SelectedInteractive();
        selectedObjects.push_back(aAis);
        this->myContext->NextSelected();
    }

    if (selectedObjects.size() == 0) return currentSelectionShape;
    for (auto aAis : selectedObjects)
    {
        Handle(AIS_Shape) shapeObj = Handle(AIS_Shape)::DownCast(aAis);
        if (!shapeObj.IsNull()) {
            currentSelectionShape = shapeObj->Shape();
        }
    }

    return currentSelectionShape;
}

void OccView::ComputeAngleBetweenFaces(const TopoDS_Shape& contextShape, const TopoDS_Shape& SelectedShape)
{
    if (contextShape.IsNull()) {
        return;
    }
    if (SelectedShape.ShapeType() != TopAbs_ShapeEnum::TopAbs_EDGE) {
        return;
    }
    TopoDS_Edge selectedEdge = TopoDS::Edge(SelectedShape);
    TopoDS_Vertex FirstV = TopExp::FirstVertex(selectedEdge);
    TopoDS_Vertex LastV = TopExp::LastVertex(selectedEdge);
    gp_Pnt pntFirst = BRep_Tool::Pnt(FirstV);
    gp_Pnt pntLast = BRep_Tool::Pnt(LastV);

    Handle(AIS_Shape) blueArrow = (std::pow(pntFirst.X(), 2) + std::pow(pntFirst.Y(), 2) + std::pow(pntFirst.Z(), 2))
        < (std::pow(pntLast.X(), 2) + std::pow(pntLast.Y(), 2) + std::pow(pntLast.Z(), 2)) ?
        CreateBlueArrow(pntFirst, pntLast, 2.0, 10.0, 5.0) : CreateBlueArrow(pntLast, pntFirst, 2.0, 10.0, 5.0);

    this->getContext()->SetDisplayMode(blueArrow, 1, Standard_True);
    this->getContext()->Display(blueArrow, Standard_False);
    this->Redraw();
}

Handle(AIS_Shape) OccView::CreateBlueArrow(const gp_Pnt& P0, const gp_Pnt& P1,
    double shaftRadius,
    double headLength,
    double headRadius)
{
    gp_Vec vec(P0, P1);
    double totalLength = vec.Magnitude();

    if (totalLength < Precision::Confusion()) {
        throw std::runtime_error("Invalid arrow: zero length");
    }

    // 箭头杆长度 = 总长 - 头长（确保头不超出）
    double shaftLength = std::max(0.0, totalLength - headLength);
    gp_Dir direction = vec.Normalized();

    // 局部坐标系：原点在 P0，Z 轴沿方向
    gp_Ax2 axis(P0, direction);

    // 1. 创建杆（圆柱）
    TopoDS_Shape shaft;
    if (shaftLength > Precision::Confusion()) {
        shaft = BRepPrimAPI_MakeCylinder(axis, shaftRadius, shaftLength);
    }

    // 2. 创建头（圆锥），位于杆末端
    gp_Pnt headBase = P0.Translated(vec * (shaftLength / totalLength));
    gp_Ax2 headAxis(headBase, direction);
    TopoDS_Shape head = BRepPrimAPI_MakeCone(headAxis, headRadius, 0.0, headLength);

    // 3. 合并杆和头
    TopoDS_Shape arrowShape;
    if (shaftLength > Precision::Confusion()) {
        BRepAlgoAPI_Fuse fuse(shaft, head);
        if (!fuse.IsDone()) {
            arrowShape = head;
        }
        else {
            arrowShape = fuse.Shape();
        }
    }
    else {
        arrowShape = head;
    }

    Handle(AIS_Shape) aisArrow = new AIS_Shape(arrowShape);

    aisArrow->SetColor(Quantity_NOC_BLUE);

    return aisArrow;
}

//处理鼠标滚轮滚动事件（根据鼠标滚轮的滚动方向，调整缩放中心点的坐标，并对场景进行缩放操作）
void OccView::onMouseWheel(const int /*theFlags*/, const int theDelta, const QPoint thePoint)
{
    //使用Standard_Integer是为了确保 OpenCASCADE 在不同的计算机体系结构和操作系统上都能够以一致的方式表示整数
    Standard_Integer aFactor = 16; //缩放因子

    Standard_Integer aX = thePoint.x();
    Standard_Integer aY = thePoint.y();

    if (theDelta > 0)
    {
        aX += aFactor;
        aY += aFactor;
    }
    else
    {
        aX -= aFactor;
        aY -= aFactor;
    }
    //以 (thePoint.x(), thePoint.y()) 为中心进行缩放，并根据 aX 和 aY 的值来调整缩放的中心点坐标
    myView->Zoom(thePoint.x(), thePoint.y(), aX, aY);
}

//占位符函数，用于在弹出菜单中添加项
void OccView::addItemInPopup(QMenu* /*theMenu*/)
{
}

//用于处理在指定坐标 (x, y) 处触发的弹出菜单
void OccView::popup(const int /*x*/, const int /*y*/)
{
}

//处理鼠标左键释放事件
void OccView::onLButtonUp(const int theFlags, const QPoint thePoint)
{
    if (myIsManipulating && !myManipulator.IsNull())
    {
        myManipulator->StopTransform(Standard_True);
        myIsManipulating = Standard_False;

        Handle(AIS_Shape) aisShape =
            Handle(AIS_Shape)::DownCast(myManipulatorTarget);

        if (!aisShape.IsNull())
        {
            // 1. 取出本次拖动累积的局部变换（平移+旋转）
            const gp_Trsf& trsf = aisShape->LocalTransformation();

            // 2. 仅当确实发生了变换才烘焙，避免无意义的几何复制
            if (trsf.Form() != gp_Identity)
            {
                // 刚体变换传 Standard_False：共享底层、更省内存，结果一致
                TopoDS_Shape movedShape =
                    BRepBuilderAPI_Transform(aisShape->Shape(), trsf, Standard_False).Shape();

                aisShape->SetShape(movedShape);       // 写回真实几何
                aisShape->ResetTransformation();      // 清显示层变换，防止叠加
                myContext->Redisplay(aisShape, Standard_False);

                // 3. 几何已变，操控器需重新绑定到新位置，否则坐标系会错位
                myManipulator->Detach();
                myManipulator->Attach(aisShape);
                myManipulator->SetSize(120.0f);
                myManipulator->SetModeActivationOnDetection(Standard_True);
            }
        }

        myView->Redraw();
        // 烘焙之后 LocalTransformation 已清空，位姿在 Shape().Location() 里。
        // 通知 MainWindow 把同一位姿写入 rlWorkpiece，网格不用重建。
        emit updateAis2RL();
        return;
    }

    // 如果 myRectBand 存在（非空），则隐藏橡皮筋（QRubberBand 对象）。
    if (myRectBand)
    {
        myRectBand->hide();
    }

    // 如果鼠标释放的位置与按下时的位置相同，且同时按下了 Ctrl 键，那么触发多重选择（multi selection）
    if (thePoint.x() == myXmin && thePoint.y() == myYmin)
    {
        //如果按下了 Ctrl 键，处理多选事件
        if (theFlags & Qt::ControlModifier)
        {
            multiInputEvent(thePoint.x(), thePoint.y());
        }
        else
        {
            inputEvent(thePoint.x(), thePoint.y());
        }
    }
}

//处理鼠标中键释放事件
void OccView::onMButtonUp(const int /*theFlags*/, const QPoint thePoint)
{
    //鼠标释放的位置是否与按下时的位置相同
    if (thePoint.x() == myXmin && thePoint.y() == myYmin)
    {
        panByMiddleButton(thePoint);
    }
}

//处理鼠标右键释放事件
void OccView::onRButtonUp(const int /*theFlags*/, const QPoint thePoint)
{
    popup(thePoint.x(), thePoint.y()); //显示弹出菜单
}

//处理鼠标移动事件
void OccView::onMouseMove(const int theFlags, const QPoint thePoint)
{
    if (myIsManipulating && !myManipulator.IsNull())
    {
        myManipulator->Transform(thePoint.x(), thePoint.y(), myView);
        myView->Redraw();
        return;   // 拖动操控器期间不执行视图旋转/框选
    }

    // 如果鼠标左键被按下，则绘制橡皮筋
    if (theFlags & Qt::LeftButton)
    {
        drawRubberBand(myXmin, myYmin, thePoint.x(), thePoint.y());

        dragEvent(thePoint.x(), thePoint.y());
    }

    // 如果按下了 Ctrl 键，执行多重选择的移动事件
    if (theFlags & Qt::ControlModifier)
    {
        multiMoveEvent(thePoint.x(), thePoint.y());
    }
    else
    {
        moveEvent(thePoint.x(), thePoint.y());
    }

    // 如果鼠标中键被按下
    if (theFlags & Qt::MiddleButton)
    {
        //根据当前的操作模式，实行相应的操作
        switch (myCurrentMode)
        {
            //旋转
        case CurAction3d_DynamicRotation:
            myView->Rotation(thePoint.x(), thePoint.y());
            break;
            //动态缩放
        case CurAction3d_DynamicZooming:
            myView->Zoom(myXmin, myYmin, thePoint.x(), thePoint.y());
            break;
            //动态平移
        case CurAction3d_DynamicPanning:
            myView->Pan(thePoint.x() - myXmax, myYmax - thePoint.y());
            myXmax = thePoint.x();
            myYmax = thePoint.y();
            break;

        default:
            break;
        }
    }

}

//处理拖拽事件
void OccView::dragEvent(const int x, const int y)
{
    //在指定的矩形区域内进行选择操作，并返回选择操作的状态
    //myContext->Select(myXmin, myYmin, x, y, myView, Standard_True);
    myContext->SelectRectangle(Graphic3d_Vec2i(myXmin, myYmin), Graphic3d_Vec2i(x, y), myView, AIS_SelectionScheme_Replace);
    emit selectionChanged();
}

//处理鼠标拖动事件
void OccView::multiDragEvent(const int x, const int y)
{
    //使用交互上下文中的 ShiftSelect 方法进行多选（框选）操作
    //myContext->ShiftSelect(myXmin, myYmin, x, y, myView, Standard_True);
    myContext->SelectRectangle(Graphic3d_Vec2i(myXmin, myYmin), Graphic3d_Vec2i(x, y), myView, AIS_SelectionScheme_XOR);
    emit selectionChanged();

}

//处理单个对象选择的事件
//void OccView::inputEvent(const int x, const int y)
//{
//    Q_UNUSED(x);
//    Q_UNUSED(y);
//
//    //清除之前的选择，执行全新的选择操作
//    myContext->Select(Standard_True);
//
//    emit selectionChanged();
//
//    if (mySelectMode == SelectMode::Mode_Edge)
//    {
//        myContext->InitSelected();
//        if (myContext->MoreSelected())
//        {
//            const TopoDS_Shape& selected = myContext->SelectedShape();
//            if (selected.ShapeType() == TopAbs_EDGE)
//            {
//                Handle(AIS_Shape) shapeObj =
//                    Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
//                if (!shapeObj.IsNull())
//                {
//                    TopoDS_Shape selectedShape = shapeObj->Shape();
//                    std::vector<TopoDS_Edge> weldEdges;
//                    weldEdges.push_back(TopoDS::Edge(selected));
//                    emit sendExtractEdges(selectedShape, weldEdges);
//                }
//            }
//        }
//    }
//}
void OccView::inputEvent(const int x, const int y)
{
    Q_UNUSED(x);
    Q_UNUSED(y);

    myContext->Select(Standard_True);

    emit selectionChanged();

    if (mySelectMode != SelectMode::Mode_Edge)
        return;

    myContext->InitSelected();
    if (!myContext->MoreSelected())
        return;

    Handle(AIS_InteractiveObject) picked = myContext->SelectedInteractive();
    if (picked.IsNull())
        return;

    // ViewCube、焊缝坐标系、点标记、操控器都没有 TopoDS_TShape
    if (picked->IsKind(STANDARD_TYPE(AIS_ViewCube))
        || picked->IsKind(STANDARD_TYPE(AIS_Trihedron))
        || picked->IsKind(STANDARD_TYPE(AIS_Point))
        || picked->IsKind(STANDARD_TYPE(AIS_Manipulator))
        || picked->IsKind(STANDARD_TYPE(AIS_TextLabel)))
        return;

    Handle(AIS_Shape) shapeObj =
        Handle(AIS_Shape)::DownCast(picked);
    if (shapeObj.IsNull())
        return;

    const TopoDS_Shape selected = myContext->SelectedShape();
    if (selected.IsNull())
        return;
    if (selected.ShapeType() != TopAbs_EDGE)
        return;

    TopoDS_Shape selectedShape = shapeObj->Shape();
    if (selectedShape.IsNull())
        return;

    std::vector<TopoDS_Edge> weldEdges;
    weldEdges.push_back(TopoDS::Edge(selected));
    emit sendExtractEdges(selectedShape, weldEdges);
}

//处理多个对象选择的事件
void OccView::multiInputEvent(const int x, const int y)
{
    Q_UNUSED(x);
    Q_UNUSED(y);
    //在现有选择集的基础上进行多选，即向已选择的对象集合中添加或移除对象
    myContext->ShiftSelect(Standard_True);

    emit selectionChanged();

    Handle(AIS_InteractiveObject) picked;
    myContext->InitSelected();
    if (myContext->MoreSelected())
        picked = myContext->SelectedInteractive();

    AttachManipulator(picked);   // picked 为空时会自动清理已有操控器
}

void OccView::moveEvent(const int x, const int y)
{
    myContext->MoveTo(x, y, myView, Standard_True);
}

void OccView::multiMoveEvent(const int x, const int y)
{
    myContext->MoveTo(x, y, myView, Standard_True);
}

//在视图中绘制一个橡皮筋
void OccView::drawRubberBand(const int minX, const int minY, const int maxX, const int maxY)
{
    QRect aRect;

    // Set the rectangle correctly.
    (minX < maxX) ? (aRect.setX(minX)) : (aRect.setX(maxX));
    (minY < maxY) ? (aRect.setY(minY)) : (aRect.setY(maxY));

    aRect.setWidth(abs(maxX - minX));
    aRect.setHeight(abs(maxY - minY));

    if (!myRectBand)
    {
        myRectBand = new QRubberBand(QRubberBand::Rectangle, this);

        // setStyle is important, set to windows style will just draw
        // rectangle frame, otherwise will draw a solid rectangle.
        myRectBand->setStyle(QStyleFactory::create("windows"));
    }

    myRectBand->setGeometry(aRect);
    myRectBand->show();
}

void OccView::panByMiddleButton(const QPoint& thePoint)
{
    Standard_Integer aCenterX = 0;
    Standard_Integer aCenterY = 0;

    QSize aSize = size();
    //计算视图中心坐标
    aCenterX = aSize.width() / 2;
    aCenterY = aSize.height() / 2;
    //实现平移操作，其中平移的距离是当前鼠标位置与视图中心的偏移
    myView->Pan(aCenterX - thePoint.x(), thePoint.y() - aCenterY);
}

/*模型平移+旋转交互*/
void OccView::AttachManipulator(const Handle(AIS_InteractiveObject)& selectedModel)
{
    // ── 1. 无有效模型：清理已有操控器并返回 ──────────────────────────
    if (selectedModel.IsNull())
    {
        DetachManipulator();
        return;
    }

    // ── 2. 点击的是同一个模型：不重复创建，仅刷新视图 ────────────────
    if (!myManipulator.IsNull() && myManipulatorTarget == selectedModel)
    {
        myView->Redraw();
        return;
    }

    // ── 3. 切换到新模型：先移除旧操控器 ──────────────────────────────
    DetachManipulator();

    // ── 4. 创建新的 AIS_Manipulator ──────────────────────────────────
    myManipulator = new AIS_Manipulator();

    // ── 5. 功能裁剪：只保留平移箭头 + 旋转环，禁用缩放块和平面扇形 ──
    for (int axis = 0; axis < 3; ++axis)
    {
        myManipulator->SetPart(axis, AIS_MM_Scaling, Standard_False); // 关掉缩放方块
        myManipulator->SetPart(axis, AIS_MM_TranslationPlane, Standard_False); // 关掉平面扇形
        // AIS_MM_Translation（箭头）与 AIS_MM_Rotation（旋转环）默认开启，保留
    }

    // ── 6. 外观与交互设置 ────────────────────────────────────────────
    myManipulator->SetSize(120.0f);                          // 控制器显示尺寸
    myManipulator->SetModeActivationOnDetection(Standard_True); // 鼠标悬停自动高亮对应轴

    // ── 7. 绑定到模型并激活 平移 + 旋转 两种模式 ─────────────────────
    myManipulator->Attach(selectedModel);
    myManipulator->EnableMode(AIS_MM_Translation);
    myManipulator->EnableMode(AIS_MM_Rotation);

    // ── 8. 显示到场景，记录目标，让视图获得键盘焦点以便接收 ESC ─────
    myContext->Display(myManipulator, Standard_True);
    myManipulatorTarget = selectedModel;

    setFocus();          // 确保后续 ESC 键事件能进入本控件
    myView->Redraw();
}

void OccView::DetachManipulator()
{
    if (myManipulator.IsNull())
        return;

    // Detach 解除与模型的绑定关系，Remove 从场景移除显示
    myManipulator->Detach();
    myContext->Remove(myManipulator, Standard_True);

    myManipulator.Nullify();          // 释放操控器句柄
    myManipulatorTarget.Nullify();    // 清空目标记录
    myIsManipulating = Standard_False;

    myView->Redraw();
}

void OccView::keyPressEvent(QKeyEvent* theEvent)
{
    if (theEvent->key() == Qt::Key_Escape)
    {
        DetachManipulator();
        emit updateAis2RL();
        return;
    }
    QWidget::keyPressEvent(theEvent);   // 其余按键交还基类处理
}