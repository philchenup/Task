// 替换 MainWindow.cpp 里的 Trajectory()。不要和原来的定义同时编译。
//
// MainWindow.h 里、sortedClusters 旁边增加：
//     std::vector<TopoDS_Edge> m_weldEdges;
//
// flushWeldList(...) 里保存本次焊缝：
//     m_selectShape = selectShape;
//     m_weldEdges = weldEdges;
//
// 工程里只保留合并后的 WeldSeamTransition.cpp。
// 原来的 WeldCoordinateSystem.cpp 从工程中移除，头文件 WeldCoordinateSystem.h 只做转发。

#include "MainWindow.h"

#include "WeldSeamTransition.h"

void MainWindow::Trajectory()
{
    if (m_selectShape.IsNull() || m_weldEdges.empty()) {
        logWarning(tr("未选择工件或焊缝为空，无法规划轨迹。"));
        return;
    }

    WeldingPathBuilder getPath;
    std::vector<ProcessedEdge> sortedEdges;
    if (!getPath.SortEdges(m_weldEdges, sortedEdges, sortedClusters) || sortedClusters.empty()) {
        logWarning(tr("焊缝排序失败，无法规划轨迹。"));
        return;
    }

    // 平焊：Y 沿焊缝前进，Z 指向坡口内侧，X = Y × Z，交接处采样圆弧。
    // 立焊：Z 相对水平面向上 45° 指向坡口，Y 保持水平，X = Y × Z。
    // 每个簇的首尾带沿 -Z 偏移 50 mm 的进入点和退出点。
    constexpr double kSpacingMm = 10.0;
    constexpr double kRetractMm = 50.0;
    constexpr double kVerticalTiltDeg = 45.0;
    if (!getPath.BuildWeldingTrajectory(m_selectShape,
            sortedClusters,
            mergedTraj,
            kSpacingMm,
            kRetractMm,
            kVerticalTiltDeg)) {
        mergedTraj.clear();
        logError(tr("焊缝轨迹规划失败。"));
        return;
    }

    showWeldTrajPt(mergedTraj);
    logInfo(tr("轨迹点数：%1").arg(static_cast<int>(mergedTraj.size())));
}
