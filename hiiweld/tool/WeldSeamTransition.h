/**
 * @file WeldSeamTransition.h
 * @brief 焊缝几何边排序、过渡圆弧/插值与离散步态焊接轨迹（DiscretePoint）。
 */
#pragma once

#include "GlobalDefs.h"

#include <Precision.hxx>

#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 从已选焊缝边构建 Wire、排序/聚类，并在角缝两侧面上生成带焊枪姿态的离散点列。
 *
 * 参数含义：
 * - standoff：焊枪与焊缝的默认法向偏置（mm，用于部分过渡与间隙处理）。
 * - coincidenceTol：端点共点判定容差（OCCT Precision 量级）。
 * - gapFitTangentScale：大间隙桥接时切向拟合缩放系数。
 */
class WeldingPathBuilder
{
public:
  /**
   * @param standoff 默认法向偏置距离（mm）。
   * @param coincidenceTol 端点重合判定容差。
   * @param gapFitTangentScale 间隙段切向拟合比例。
   */
  WeldingPathBuilder(double standoff = 3.0,
    double coincidenceTol = Precision::Confusion() * 10.0,
    double gapFitTangentScale = 0.02)
    : m_standoff(standoff),
      m_tol(coincidenceTol),
      m_gapScale(gapFitTangentScale) {
  }

  /**
   * @brief 将多条 ProcessedEdge 用倒角/插值连接成闭合或开口 Wire（圆角焊缝等场景）。
   * @param edges 已定向的焊缝边集合。
   * @param wires 输出 TopoDS_Wire。
   */
  void GetCirclePath(const std::vector<ProcessedEdge>& edges, TopoDS_Wire& wires);

  /**
   * @brief 对无序焊缝边拓扑排序并聚类为 EdgeCluster（水平段/立焊段分组）。
   * @param weldEdges 输入 OCCT 边。
   * @param sortedEdges 输出定向后的 ProcessedEdge。
   * @param sortedClusters 输出边簇（连续焊接单元）。
   * @return 成功且非空为 true。
   */
  bool SortEdges(const std::vector<TopoDS_Edge>& weldEdges,
    std::vector<ProcessedEdge>& sortedEdges,
    std::vector<EdgeCluster>& sortedClusters);

  /**
   * @brief 在整块选中工件上，按聚类生成合并焊接轨迹（含起收弧安全点）。
   * @param selectShape 焊缝所在父形状（用于取邻接面法向）。
   * @param sortedClusters SortEdges 的聚类结果。
   * @param trajectory 输出全部离散点（多簇串联）。
   * @param spacingMm 沿焊缝弧长采样步距（mm）。
   * @param retractMm 起弧/收弧沿焊枪 Z 后退距离（mm）。
   * @param verticalTiltDeg 立焊时焊枪绕局部 Y 倾角（度，典型 45°）。
   * @param clusterTrajectories 若非空，则每簇单独保存一条点列。
   * @return 至少生成一个有效点即为 true。
   */
  bool BuildWeldingTrajectory(const TopoDS_Shape& selectShape,
    const std::vector<EdgeCluster>& sortedClusters,
    std::vector<DiscretePoint>& trajectory,
    double spacingMm = 10.0,
    double retractMm = 50.0,
    double verticalTiltDeg = 45.0,
    std::vector<std::vector<DiscretePoint>>* clusterTrajectories = nullptr);

  /**
   * @brief 对已成型 Wire 按固定步长离散，并为每点构造自定义焊枪坐标系（相对 parentShape）。
   * @param wire 焊缝 Wire。
   * @param parentShape 面法向查询用父形状。
   * @param stepSize 弧长步长。
   * @param tolerance 几何容差。
   * @param maxAngleStep 相邻采样点 Z 轴最大夹角（弧度），用于平滑立焊翻转。
   */
  std::vector<DiscretePoint> DiscretizeWireWithCustomFrame(
    const TopoDS_Wire& wire,
    const TopoDS_Shape& parentShape,
    double stepSize,
    double tolerance,
    double maxAngleStep = 5.0 * M_PI / 180.0);

private:
  double m_standoff;   ///< 法向偏置
  double m_tol;        ///< 共点/重合容差
  double m_gapScale;   ///< 间隙切向缩放

  /** @brief 在 ProcessedEdge 中点求单位切向（沿焊接方向）。 */
  bool GetTangentAt(const ProcessedEdge& pe, gp_Vec& tangent);

  /**
   * @brief 两段不相接焊缝间用 Hermite/样条类插值生成过渡边。
   * @param pe1 前一段焊缝。
   * @param pe2 后一段焊缝。
   * @param outEdge 输出过渡 TopoDS_Edge。
   */
  bool BuildInterpolationCurve(const ProcessedEdge& pe1,
    const ProcessedEdge& pe2,
    TopoDS_Edge& outEdge);

  /**
   * @brief 将无序边按端点连通性串成 Wire（tolerance 为端点匹配距离）。
   */
  TopoDS_Wire BuildWiresFromUnorderedEdges(const std::vector<TopoDS_Edge>& edges, double tolerance = 1e-6);

  /**
   * @brief 两面成角处生成圆角过渡圆弧边（fillet），连接 pe1 与 pe2。
   */
  bool BuildFilletArc(const ProcessedEdge& pe1, const ProcessedEdge& pe2, TopoDS_Edge& filletEdge);

  /** @brief 用端点弦向量与世界 Z 夹角判断是否近似立焊（小于 kVerticalAngleDeg）。 */
  bool IsVerticalByEndpoints(const TopoDS_Edge& edge, double vecEps);

  /** @brief 将空间点投影到面上，取外向法向。 */
  bool ComputeFaceNormalAt(const TopoDS_Face& face,
    const gp_Pnt& p,
    double vecEps,
    gp_Vec& outNormal);

  /**
   * @brief 单簇内：排序边、插入过渡/立焊处理、离散为 DiscretePoint。
   * @param out 该簇离散点（含立面摆动与入刀出刀姿态修正）。
   */
  bool BuildClusterTrajectory(const TopoDS_Shape& selectShape,
    const EdgeCluster& cluster,
    double spacingMm,
    double retractMm,
    double verticalTiltDeg,
    std::vector<DiscretePoint>& out);
};

/**
 * @brief 对单条焊缝边，在两侧邻接面上构造焊枪坐标系 gp_Ax3（X 沿前进，Z 指向内侧）。
 * @param selectShape 工件形状。
 * @param edge 焊缝边。
 * @param weldAxis 输出坐标系。
 * @param reverseZ 为 true 时 Z 取反（外侧焊/调试）。
 */
Standard_Boolean ComputeWeldCoordinateSystem(
  const TopoDS_Shape& selectShape,
  const TopoDS_Edge& edge,
  gp_Ax3& weldAxis,
  Standard_Boolean reverseZ = Standard_False);

/**
 * @brief 单条边等弧长离散 + 邻面法向建姿态；可选起收弧 retract 点。
 * @param spacing 采样步距（mm）。
 * @param reverseZ 是否翻转 Z。
 * @param retract 起弧前/收弧后沿 Z 退离距离（mm）。
 */
Standard_Boolean DiscretizeWeldTrajectory(
  const TopoDS_Shape& selectShape,
  const TopoDS_Edge& edge,
  std::vector<DiscretePoint>& trajectory,
  Standard_Real spacingMm = 10.0,
  Standard_Boolean reverseZ = Standard_False,
  Standard_Real retractMm = 50.0);