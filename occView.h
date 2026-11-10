#pragma once
#ifndef _OCCVIEW_H_

#include <QGLWidget>
#include <AIS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Curve.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopExp.hxx>
#include <Interface_Static.hxx>
#include <AIS_Manipulator.hxx>

class QMenu;
class QRubberBand;

//! Adapted a QWidget for OpenCASCADE viewer.
class OccView : public QWidget
{
    Q_OBJECT

public:
    //鼠标操作的枚举类型，表示鼠标的不同动作状态，比如旋转、缩放等
    enum CurrentAction3d
    {
        CurAction3d_Nothing,
        CurAction3d_DynamicZooming,
        CurAction3d_WindowZooming,
        CurAction3d_DynamicPanning,
        CurAction3d_GlobalPanning,
        CurAction3d_DynamicRotation
    };

    enum class SelectMode {
        Mode_Vertex,  // 点
        Mode_Edge,    // 边
        Mode_Face,    // 面
        Mode_AllEdge    // 实体
    };

public:
    //构造函数，用于创建 OccView 对象
    OccView(QWidget* parent);
    //获取 AIS_InteractiveContext 上下文对象的方法
    const Handle(AIS_InteractiveContext)& getContext() const;

    const Handle(V3d_View)& getV3dView() const;

    void ComputeAngleBetweenFaces(const TopoDS_Shape& contextShape, const TopoDS_Shape& SelectedShape);

    std::vector<TopoDS_Edge> extractCommonEdgesWithConditions(const TopoDS_Shape& shape);

    Handle(AIS_Shape) CreateBlueArrow(const gp_Pnt& P0, const gp_Pnt& P1,
        double shaftRadius,
        double headLength,
        double headRadius);

    TopoDS_Shape getSelectShape();

    void AttachManipulator(const Handle(AIS_InteractiveObject)& selectedModel);

    void DetachManipulator();

signals:
    void selectionChanged(void);

    void sendExtractEdges(const TopoDS_Shape& selectShape, const std::vector<TopoDS_Edge>& weldEdges);

    // 操控器一次拖动结束（鼠标松开）或 ESC 退出时发出。
    // MainWindow 接到后只同步 rlWorkpiece 的位姿，不要再次 bind。
    void updateAis2RL();

public slots:
    //! operations for the view.
    void pan(void);
    void fitAll(void);
    void Redraw(void);
    void reset(void);
    void zoom(void);
    void rotate(void);

protected:
    virtual QPaintEngine* paintEngine() const;

    virtual void keyPressEvent(QKeyEvent* theEvent);

    // Paint events.
    virtual void paintEvent(QPaintEvent* theEvent);
    virtual void resizeEvent(QResizeEvent* theEvent);

    // Mouse events.
    virtual void mousePressEvent(QMouseEvent* theEvent);
    virtual void mouseReleaseEvent(QMouseEvent* theEvent);
    virtual void mouseMoveEvent(QMouseEvent* theEvent);
    virtual void wheelEvent(QWheelEvent* theEvent);

    // Button events.
    virtual void onLButtonDown(const int theFlags, const QPoint thePoint);
    virtual void onMButtonDown(const int theFlags, const QPoint thePoint);
    virtual void onRButtonDown(const int theFlags, const QPoint thePoint);
    virtual void onMouseWheel(const int theFlags, const int theDelta, const QPoint thePoint);
    virtual void onLButtonUp(const int theFlags, const QPoint thePoint);
    virtual void onMButtonUp(const int theFlags, const QPoint thePoint);
    virtual void onRButtonUp(const int theFlags, const QPoint thePoint);
    virtual void onMouseMove(const int theFlags, const QPoint thePoint);

    // Popup menu.
    virtual void addItemInPopup(QMenu* theMenu);

protected:
    void init(void);
    void popup(const int x, const int y);
    void dragEvent(const int x, const int y);
    void inputEvent(const int x, const int y);
    void moveEvent(const int x, const int y);
    void multiMoveEvent(const int x, const int y);
    void multiDragEvent(const int x, const int y);
    void multiInputEvent(const int x, const int y);
    void drawRubberBand(const int minX, const int minY, const int maxX, const int maxY);
    void panByMiddleButton(const QPoint& thePoint);
    
    void setSelectMode(SelectMode mode);
public:

    //! the occ viewer.
    Handle(V3d_Viewer) myViewer;

    //! the occ view.
    Handle(V3d_View) myView;

    //! the occ context.
    Handle(AIS_InteractiveContext) myContext;

    //! save the mouse position.
    Standard_Integer myXmin;
    Standard_Integer myYmin;
    Standard_Integer myXmax;
    Standard_Integer myYmax;

    //! the mouse current mode.
    CurrentAction3d myCurrentMode;

    //! save the degenerate mode state.
    Standard_Boolean myDegenerateModeIsOn;

    //! rubber rectangle for the mouse selection.
    QRubberBand* myRectBand;

    SelectMode mySelectMode = SelectMode::Mode_Face;
private:
    Handle(AIS_Manipulator)       myManipulator;
    Handle(AIS_InteractiveObject) myManipulatorTarget;
    Standard_Boolean              myIsManipulating = Standard_False;  // 是否正在拖动操控器

};

#endif // _OCCVIEW_H_
