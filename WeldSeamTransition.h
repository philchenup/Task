#pragma once

#include "GlobalDefs.h"

#include <Precision.hxx>

#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class WeldingPathBuilder
{
public:
    WeldingPathBuilder(double standoff = 3.0,
        double coincidenceTol = Precision::Confusion() * 10.0,
        double gapFitTangentScale = 5.0)
        : m_standoff(standoff),
        m_tol(coincidenceTol),
        m_gapScale(gapFitTangentScale) {
    }

    void GetCirclePath(const std::vector<ProcessedEdge>& edges, TopoDS_Wire& wires);

    bool SortEdges(const std::vector<TopoDS_Edge>& weldEdges,
        std::vector<ProcessedEdge>& sortedEdges,
        std::vector<EdgeCluster>& sortedClusters);

    //! 用 SortEdges 输出的 sortedClusters 生成焊枪轨迹（平焊 / 立焊）。
    //!
    //! 平焊（EdgeCluster::isVertical == false）
    //! - 按簇内顺序，沿每条焊缝从 StartPoint 走到 EndPoint，合成一条轨迹
    //! - Y：焊缝前进方向
    //! - Z：两侧相邻面外法向夹角平分线的反向（指向坡口根部）
    //! - X：Y × Z，满足右手系 X × Y = Z
    //! - 两条焊缝首尾相接处采样圆弧过渡（共点用倒角圆弧，间隙用切向插值曲线）
    //!
    //! 立焊（EdgeCluster::isVertical == true）
    //! - 位置仍沿焊缝从高处走到低处
    //! - Z：水平分量指向坡口内侧，并相对 XOY 面向上倾斜 verticalTiltDeg（默认 45°）
    //! - Y：水平，且与 Z 垂直
    //! - X：Y × Z
    //! - 顺序：焊完退焊位置 → 预焊位置 → 焊缝 → 焊完退焊位置。
    //!   退焊位置是焊缝终点沿 -Z 偏移 retractMm，预焊位置是焊缝起点沿 -Z 偏移 retractMm。
    //!
    //! 平焊每个簇保留进入点与退出点：焊缝起点之前、终点之后各一点，沿该点 -Z 偏移 retractMm。
    //! 平焊与立焊分簇，簇与簇之间不在工件表面上直接连姿态。
    bool BuildWeldingTrajectory(const TopoDS_Shape& selectShape,
        const std::vector<EdgeCluster>& sortedClusters,
        std::vector<DiscretePoint>& trajectory,
        double spacingMm = 10.0,
        double retractMm = 50.0,
        double verticalTiltDeg = 45.0);

    std::vector<DiscretePoint> DiscretizeWireWithCustomFrame(
        const TopoDS_Wire& wire,
        const TopoDS_Shape& parentShape,
        double stepSize,
        double tolerance,
        double maxAngleStep = 5.0 * M_PI / 180.0);

private:
    double m_standoff;
    double m_tol;
    double m_gapScale;

    bool GetTangentAt(const ProcessedEdge& pe, gp_Vec& tangent);

    bool BuildInterpolationCurve(const ProcessedEdge& pe1,
        const ProcessedEdge& pe2,
        TopoDS_Edge& outEdge);

    TopoDS_Wire BuildWiresFromUnorderedEdges(const std::vector<TopoDS_Edge>& edges, double tolerance = 1e-6);

    bool BuildFilletArc(const ProcessedEdge& pe1, const ProcessedEdge& pe2, TopoDS_Edge& filletEdge);

    bool IsVerticalByEndpoints(const TopoDS_Edge& edge, double vecEps);

    bool ComputeFaceNormalAt(const TopoDS_Face& face,
        const gp_Pnt& p,
        double vecEps,
        gp_Vec& outNormal);

    bool BuildClusterTrajectory(const TopoDS_Shape& selectShape,
        const EdgeCluster& cluster,
        double spacingMm,
        double retractMm,
        double verticalTiltDeg,
        std::vector<DiscretePoint>& out);
};

//! 兼容原 WeldCoordinateSystem.h。Z 默认取两侧面外法向夹角平分线；
//! reverseZ 为 true 时取反向。新轨迹请用 WeldingPathBuilder::BuildWeldingTrajectory，
//! 该函数的 Z 固定指向坡口内侧。
Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    gp_Ax3& weldAxis,
    Standard_Boolean reverseZ = Standard_False);

Standard_Boolean DiscretizeWeldTrajectory(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real spacingMm = 10.0,
    Standard_Boolean reverseZ = Standard_False,
    Standard_Real retractMm = 50.0);
