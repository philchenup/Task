#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从「平板 + 曲线工件」点云中提取弯曲焊缝轨迹。点云单位为毫米，使用 Open3D 读取。

做法来自两个仓库，并按毫米单位改了尺度：

1. romi-lab/robotic-welding-demo
   法向不对称特征找出交界（坡口/焊根），DBSCAN 收成焊缝簇。
   https://github.com/romi-lab/robotic-welding-demo
2. maggielovedd/3D-Point-Cloud-Curve-Extraction-and-Optimization
   邻域 PCA 把焊缝带收成中心线，再沿局部切向排序。
   https://github.com/maggielovedd/3D-Point-Cloud-Curve-Extraction-and-Optimization

弯曲焊缝不使用 demo 里的 splprep(s=inf)，那个平滑因子会把曲线拉直。
这里用有限平滑，并把轨迹投回底板，得到焊根曲线。

依赖: open3d, numpy, scipy

示例:
  python extract_weld_trajectory.py workpiece.pcd
  python extract_weld_trajectory.py workpiece.ply --band 20 --no-show
  python extract_weld_trajectory.py --demo
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree


def load_cloud(path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(path)
    if pcd.is_empty():
        raise RuntimeError(f"点云为空或格式无法读取: {path}")
    return pcd


def preprocess(pcd: o3d.geometry.PointCloud, voxel: float) -> o3d.geometry.PointCloud:
    cloud = pcd.voxel_down_sample(voxel)
    cloud.remove_non_finite_points()
    if len(cloud.points) < 100:
        raise RuntimeError("降采样后点数过少，检查点云单位是否为毫米、体素是否过大。")
    filtered, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return filtered


def fit_base_plane(points: np.ndarray, thresh: float):
    """RANSAC 底板。返回单位法向 n、平面常数 d（n·x + d = 0），以及有符号高度。"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    model, inliers = pcd.segment_plane(
        distance_threshold=thresh, ransac_n=3, num_iterations=2000
    )
    n = np.asarray(model[:3], dtype=float)
    norm = np.linalg.norm(n)
    n /= norm
    d = float(model[3]) / norm
    dist = points @ n + d
    above = np.count_nonzero(dist > thresh * 2)
    below = np.count_nonzero(dist < -thresh * 2)
    if below > above:
        n = -n
        d = -d
        dist = -dist
    return n, d, dist, np.asarray(inliers, dtype=int)


def project_to_plane(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    return points - (points @ normal + d)[:, None] * normal


def largest_clusters(points: np.ndarray, eps: float, min_points: int, keep_ratio: float = 0.25):
    if len(points) < min_points:
        return []
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    labels = np.asarray(cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    if labels.size == 0 or labels.max() < 0:
        return []
    groups = []
    for label in range(labels.max() + 1):
        idx = np.flatnonzero(labels == label)
        groups.append(points[idx])
    groups.sort(key=len, reverse=True)
    cutoff = max(min_points, int(len(groups[0]) * keep_ratio))
    return [group for group in groups if len(group) >= cutoff]


def asymmetry_feature(points: np.ndarray, normals: np.ndarray, neighbor: int) -> np.ndarray:
    """romi-lab demo_all.py find_feature_value('asymmetry') 的向量化版本。

    邻域平均法向里，垂直于当前点法向的分量。交界处法向突变，该值大。
    """
    neighbor = int(np.clip(neighbor, 8, max(8, len(points) // 20)))
    _, indices = cKDTree(points).query(points, k=min(neighbor, len(points)))
    mean_normal = normals[indices].mean(axis=1)
    dots = np.sum(mean_normal * normals, axis=1)
    scale = dots / (np.sum(normals * normals, axis=1) + 1e-12)
    return np.linalg.norm(mean_normal - normals * scale[:, None], axis=1)


def estimate_normals(points: np.ndarray, radius: float, plate_normal: np.ndarray) -> np.ndarray:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=40)
    )
    normals = np.asarray(cloud.normals)
    flip = normals @ plate_normal < 0
    normals[flip] *= -1
    return normals


def _span(points: np.ndarray) -> float:
    if len(points) == 0:
        return 0.0
    return float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))


def select_junction_points(
    points: np.ndarray,
    height: np.ndarray,
    plane_inliers: np.ndarray,
    normal: np.ndarray,
    d: float,
    gap: float,
    band: float,
    margin: float,
    asym_ratio: float,
    bridge: float,
):
    """焊根候选。

    先用底板把范围限制在接触带里，避免把工件立边（图里爬升的那条绿线）当成焊缝。
    再在接触带内用法向不对称特征（romi-lab）留下交界，并用接触带把空洞补上。
    特征把曲线截断时，退回整条接触带。
    """
    workpiece_mask = height > gap
    if np.count_nonzero(workpiece_mask) < 50:
        raise RuntimeError("底板上几乎没有工件点。调小 --gap，或确认点云单位是毫米。")

    workpiece = points[workpiece_mask]
    clusters = largest_clusters(workpiece, eps=max(8.0, margin), min_points=30, keep_ratio=0.2)
    part = workpiece if not clusters else clusters[0]

    part_height = (part @ normal) + d
    band_pts = part[(part_height > gap) & (part_height < band)]
    if len(band_pts) < 30:
        raise RuntimeError("接触带点数过少。增大 --band，或减小 --gap。")

    on_plane = np.zeros(len(points), dtype=bool)
    on_plane[plane_inliers] = True
    plate_pts = points[on_plane & (np.abs(height) <= gap)]
    projected_band = project_to_plane(band_pts, normal, d)
    if len(plate_pts) > 0:
        near_dist, _ = cKDTree(projected_band).query(project_to_plane(plate_pts, normal, d), k=1)
        plate_near = plate_pts[near_dist < margin]
    else:
        plate_near = np.empty((0, 3))

    junction = np.vstack([band_pts, plate_near]) if len(plate_near) else band_pts
    normals = estimate_normals(junction, radius=max(6.0, band * 0.5), plate_normal=normal)
    score = asymmetry_feature(junction, normals, neighbor=24)
    band_score = score[: len(band_pts)]
    threshold = np.quantile(band_score, 1.0 - asym_ratio)
    high = band_pts[band_score >= threshold]

    selected = band_pts
    if len(high) >= 30:
        distance, _ = cKDTree(project_to_plane(high, normal, d)).query(projected_band, k=1)
        closed = band_pts[distance <= bridge]
        if len(closed) >= 30 and _span(project_to_plane(closed, normal, d)) >= 0.7 * _span(projected_band):
            selected = closed
    return selected, band_pts


def thin_line(points: np.ndarray, radius: float):
    """把焊缝带上的点投到邻域主方向上，得到中心线点及切向。"""
    tree = cKDTree(points)
    neighbor_ids = tree.query_ball_point(points, radius)
    moved = np.empty_like(points)
    directions = np.zeros_like(points)
    for index, neighbors in enumerate(neighbor_ids):
        if len(neighbors) < 3:
            moved[index] = points[index]
            directions[index] = np.array([1.0, 0.0, 0.0])
            continue
        patch = points[neighbors]
        center = patch.mean(axis=0)
        _, _, vt = np.linalg.svd(patch - center, full_matrices=False)
        direction = vt[0]
        directions[index] = direction
        offset = points[index] - center
        moved[index] = center + np.dot(offset, direction) * direction
    return moved, directions


def voxel_average(points: np.ndarray, voxel: float) -> np.ndarray:
    """同一体素内的点取平均。切向不在这里平均，符号相反的主方向会互相抵消。"""
    keys = np.floor(points / voxel).astype(np.int64)
    buckets: dict[tuple[int, int, int], list[int]] = {}
    for index, key in enumerate(map(tuple, keys)):
        buckets.setdefault(key, []).append(index)
    return np.asarray([points[np.asarray(indices)].mean(axis=0) for indices in buckets.values()])


def _walk_curve(points: np.ndarray, directions: np.ndarray, start: int, step: float) -> np.ndarray:
    """沿局部切向走完整条开曲线。排序规则与 maggielovedd 的 min-angle 相同。"""
    tree = cKDTree(points)
    visited = np.zeros(len(points), dtype=bool)
    visited[start] = True
    ordered = [start]
    centroid = points.mean(axis=0)
    heading = directions[start].copy()
    if np.dot(heading, centroid - points[start]) < 0:
        heading = -heading
    index = start
    search = step * 0.85
    while len(ordered) < len(points):
        proposed = directions[index]
        if np.dot(heading, proposed) < 0:
            proposed = -proposed
        # 局部主方向偶尔会转到焊缝横截面上，这时沿用上一步切向。
        if np.dot(heading, proposed) > 0.45:
            heading = proposed
        predicted = points[index] + heading * step
        candidates = tree.query_ball_point(predicted, search)
        candidates = [item for item in candidates if not visited[item]]
        if not candidates:
            candidates = tree.query_ball_point(predicted, search * 2.0)
            candidates = [item for item in candidates if not visited[item]]
        if not candidates:
            break
        vectors = points[candidates] - points[index]
        lengths = np.linalg.norm(vectors, axis=1)
        valid = lengths > 1e-6
        if not np.any(valid):
            break
        cosine = (vectors @ heading) / ((lengths + 1e-12) * (np.linalg.norm(heading) + 1e-12))
        forward = np.flatnonzero(valid & (cosine > 0.25))
        if len(forward) == 0:
            break
        predicted_distance = np.linalg.norm(points[np.asarray(candidates)[forward]] - predicted, axis=1)
        score = cosine[forward] - 0.2 * predicted_distance / step
        chosen = candidates[forward[int(np.argmax(score))]]
        visited[chosen] = True
        ordered.append(chosen)
        index = chosen
    return points[np.asarray(ordered)]


def sort_open_curve(points: np.ndarray, directions: np.ndarray, step: float) -> np.ndarray:
    if len(points) < 4:
        return points
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    projection = (points - center) @ vt[0]
    starts = [int(np.argmin(projection)), int(np.argmax(projection))]
    best = None
    best_length = -1.0
    for start in starts:
        ordered = _walk_curve(points, directions, start, step)
        if len(ordered) < 2:
            continue
        length = float(np.linalg.norm(np.diff(ordered, axis=0), axis=1).sum())
        if length > best_length:
            best = ordered
            best_length = length
    return points if best is None else best


def clean_polyline(points: np.ndarray, min_step: float) -> np.ndarray:
    if len(points) == 0:
        return points
    kept = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - kept[-1]) < min_step:
            continue
        if len(kept) >= 2 and np.dot(kept[-1] - kept[-2], point - kept[-1]) <= 0:
            continue
        kept.append(point)
    return np.asarray(kept)


def fit_spline(points: np.ndarray, smooth_mm: float, sample_step: float) -> np.ndarray:
    """三次 B 样条。smooth_mm 是允许的大致偏差，不能用 inf。"""
    points = clean_polyline(points, min_step=max(0.5, sample_step * 0.4))
    if len(points) < 4:
        return points
    # 弧长参数，避免点距不均把样条拉偏。
    chord = np.linalg.norm(np.diff(points, axis=0), axis=1)
    parameter = np.concatenate([[0.0], np.cumsum(chord)])
    if parameter[-1] < 1e-6:
        return points
    parameter /= parameter[-1]
    smoothing = max(len(points) * smooth_mm ** 2, 1e-6)
    degree = min(3, len(points) - 1)
    try:
        tck, _ = splprep(
            [points[:, 0], points[:, 1], points[:, 2]],
            u=parameter,
            s=smoothing,
            k=degree,
        )
    except Exception:
        return points
    count = max(int(parameter.size * 4), 50)
    dense = np.column_stack(splev(np.linspace(0.0, 1.0, count), tck))
    return resample_polyline(dense, sample_step)


def resample_polyline(points: np.ndarray, step: float) -> np.ndarray:
    if len(points) < 2:
        return points
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    station = np.concatenate([[0.0], np.cumsum(segment)])
    total = station[-1]
    if total < step:
        return points
    targets = np.arange(0.0, total, step)
    if total - targets[-1] > step * 0.3:
        targets = np.append(targets, total)
    else:
        targets[-1] = total
    return np.column_stack([np.interp(targets, station, points[:, axis]) for axis in range(3)])


def trace_cluster(points_on_plane: np.ndarray, thin_radius: float, sample_step: float, smooth_mm: float):
    if len(points_on_plane) < 15:
        return None
    moved, _ = thin_line(points_on_plane, thin_radius)
    centers = voxel_average(moved, voxel=max(sample_step, thin_radius * 0.35))
    if len(centers) < 8:
        return None
    # 在已经收细的中心线上重新估计切向，避免体素内方向互相抵消。
    _, directions = thin_line(centers, radius=max(thin_radius, sample_step * 4.0))
    ordered = sort_open_curve(centers, directions, step=max(sample_step * 2.0, thin_radius * 0.45))
    ordered = clean_polyline(ordered, min_step=sample_step * 0.8)
    if len(ordered) < 4:
        return None
    return fit_spline(ordered, smooth_mm=smooth_mm, sample_step=sample_step)


def extract_weld_trajectories(pcd: o3d.geometry.PointCloud, args) -> dict:
    raw_count = len(pcd.points)
    cloud = preprocess(pcd, args.voxel)
    points = np.asarray(cloud.points)
    print(f"读取 {raw_count} 点，降采样后 {len(points)} 点，体素 {args.voxel:.2f} mm")

    normal, plane_d, height, inliers = fit_base_plane(points, args.plane_thresh)
    inlier_ratio = len(inliers) / len(points)
    print(f"底板内点 {len(inliers)} ({inlier_ratio:.1%})，法向 {np.array2string(normal, precision=3)}")
    if inlier_ratio < 0.15:
        print("警告: 底板内点偏少。若工件表面被当成了平面，减小 --plane-thresh 或先裁掉远离底板的区域。")

    selected, band = select_junction_points(
        points,
        height,
        inliers,
        normal,
        plane_d,
        gap=args.gap,
        band=args.band,
        margin=args.margin,
        asym_ratio=args.asym_ratio,
        bridge=max(args.cluster_eps, args.thin_radius * 0.6),
    )
    print(f"接触带 {len(band)} 点，参与轨迹提取 {len(selected)} 点")

    on_plane = project_to_plane(selected, normal, plane_d)
    clusters = largest_clusters(on_plane, eps=args.cluster_eps, min_points=args.min_cluster, keep_ratio=0.3)
    if not clusters:
        clusters = [on_plane]
    print(f"焊缝簇 {len(clusters)} 条")

    trajectories = []
    used_clusters = []
    for cluster in clusters:
        trajectory = trace_cluster(cluster, args.thin_radius, args.step, args.smooth)
        if trajectory is None or len(trajectory) < 4:
            continue
        length = float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())
        if length < args.min_length:
            continue
        trajectories.append(trajectory)
        used_clusters.append(cluster)
        print(f"  轨迹 {len(trajectories)}: {len(trajectory)} 点, 长度 {length:.1f} mm")

    if not trajectories:
        raise RuntimeError("没有形成足够长的焊缝轨迹。尝试增大 --band、--cluster-eps，或减小 --asym-ratio。")

    return {
        "cloud": cloud,
        "normal": normal,
        "plane_d": plane_d,
        "inliers": inliers,
        "band": band,
        "clusters": used_clusters,
        "trajectories": trajectories,
    }


def save_trajectory(path: Path, trajectories: list[np.ndarray]):
    path.parent.mkdir(parents=True, exist_ok=True)
    blocks = []
    obj_lines = []
    vertex_base = 0
    for index, trajectory in enumerate(trajectories):
        tagged = np.column_stack([np.full(len(trajectory), index), trajectory])
        blocks.append(tagged)
        for point in trajectory:
            obj_lines.append(f"v {point[0]:.6f} {point[1]:.6f} {point[2]:.6f}")
        for offset in range(1, len(trajectory)):
            obj_lines.append(f"l {vertex_base + offset} {vertex_base + offset + 1}")
        vertex_base += len(trajectory)
    data = np.vstack(blocks)
    txt_path = path.with_suffix(".txt")
    np.savetxt(txt_path, data, fmt="%.6f", header="seam_id x_mm y_mm z_mm")
    obj_path = path.with_suffix(".obj")
    obj_path.write_text("\n".join(obj_lines) + "\n", encoding="utf-8")
    print(f"轨迹已写入 {txt_path}")
    print(f"折线已写入 {obj_path} （CloudCompare 可直接打开）")


def make_path_mesh(points: np.ndarray, radius: float, color: list[float]) -> o3d.geometry.TriangleMesh:
    meshes = []
    z_axis = np.array([0.0, 0.0, 1.0])
    for start, end in zip(points[:-1], points[1:]):
        direction = end - start
        length = np.linalg.norm(direction)
        if length < 1e-4:
            continue
        direction = direction / length
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=8, split=1)
        axis = np.cross(z_axis, direction)
        sine = np.linalg.norm(axis)
        cosine = float(np.dot(z_axis, direction))
        if sine < 1e-8:
            rotation = np.eye(3) if cosine > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            axis = axis / sine
            skew = np.array(
                [
                    [0.0, -axis[2], axis[1]],
                    [axis[2], 0.0, -axis[0]],
                    [-axis[1], axis[0], 0.0],
                ]
            )
            rotation = np.eye(3) + skew * sine + skew @ skew * (1.0 - cosine)
        cylinder.rotate(rotation, center=np.zeros(3))
        cylinder.translate((start + end) * 0.5)
        meshes.append(cylinder)
    if not meshes:
        mesh = o3d.geometry.TriangleMesh()
    else:
        mesh = meshes[0]
        for piece in meshes[1:]:
            mesh += piece
    mesh.paint_uniform_color(color)
    mesh.compute_vertex_normals()
    return mesh


def build_geometries(result: dict) -> list:
    points = np.asarray(result["cloud"].points)
    colors = np.tile(np.array([0.62, 0.64, 0.68]), (len(points), 1))
    colors[result["inliers"]] = np.array([0.28, 0.32, 0.40])
    scene = o3d.geometry.PointCloud()
    scene.points = o3d.utility.Vector3dVector(points)
    scene.colors = o3d.utility.Vector3dVector(colors)

    band = o3d.geometry.PointCloud()
    band.points = o3d.utility.Vector3dVector(result["band"])
    band.paint_uniform_color([0.95, 0.45, 0.12])

    geometries = [scene, band]
    palette = (
        [0.15, 0.95, 0.25],
        [0.95, 0.25, 0.85],
        [0.95, 0.85, 0.15],
        [0.20, 0.75, 1.0],
    )
    lift = result["normal"] * 3.0
    for index, trajectory in enumerate(result["trajectories"]):
        color = palette[index % len(palette)]
        visible = trajectory + lift
        geometries.append(make_path_mesh(visible, radius=2.0, color=color))
        for endpoint, endpoint_color in ((visible[0], [1.0, 0.15, 0.1]), (visible[-1], [0.1, 0.35, 1.0])):
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
            sphere.translate(endpoint)
            sphere.paint_uniform_color(endpoint_color)
            sphere.compute_vertex_normals()
            geometries.append(sphere)
    return geometries


def show_geometries(geometries: list, screenshot: str | None):
    if screenshot:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="weld trajectory", width=1280, height=960, visible=False)
        for geometry in geometries:
            vis.add_geometry(geometry)
        view = vis.get_view_control()
        view.set_front([0.55, -0.72, 0.42])
        view.set_up([0.0, 0.0, 1.0])
        view.set_zoom(0.55)
        for _ in range(8):
            vis.poll_events()
            vis.update_renderer()
        vis.capture_screen_image(screenshot, do_render=True)
        vis.destroy_window()
        print(f"截图已写入 {screenshot}")
        return
    o3d.visualization.draw_geometries(
        geometries,
        window_name="弯曲焊缝轨迹  灰=工件  深色=底板  橙=接触带  绿=焊根",
        width=1280,
        height=960,
    )


def synthesize_demo(path: Path) -> tuple[np.ndarray, float, float, float]:
    """平板 z=0，上面立一段圆柱壳。返回点云、圆心平面坐标和内外半径。"""
    rng = np.random.default_rng(7)
    plate_x = np.linspace(-220, 220, 180)
    plate_y = np.linspace(-180, 180, 150)
    xx, yy = np.meshgrid(plate_x, plate_y)
    plate = np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])

    radius_outer = 150.0
    radius_inner = 140.0
    angles = np.linspace(np.deg2rad(-55), np.deg2rad(55), 160)
    heights = np.linspace(0.0, 110.0, 80)
    angle_grid, height_grid = np.meshgrid(angles, heights)
    shells = []
    for radius in (radius_outer, radius_inner):
        shells.append(
            np.column_stack(
                [
                    radius * np.sin(angle_grid.ravel()),
                    radius * np.cos(angle_grid.ravel()) - 40.0,
                    height_grid.ravel(),
                ]
            )
        )
    cloud = np.vstack([plate, *shells])
    cloud += rng.normal(0.0, 0.35, cloud.shape)
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(cloud)
    o3d.io.write_point_cloud(str(path), out)
    return cloud, radius_outer, radius_inner, -40.0


def evaluate_demo(trajectories: list[np.ndarray], radius_outer: float, radius_inner: float, center_y: float):
    center = np.array([0.0, center_y, 0.0])
    print("与设计圆弧的偏差（毫米）:")
    for index, trajectory in enumerate(trajectories):
        radial = np.linalg.norm(trajectory[:, :2] - center[:2], axis=1)
        height = np.abs(trajectory[:, 2])
        err_outer = np.sqrt(np.mean((radial - radius_outer) ** 2 + height ** 2))
        err_inner = np.sqrt(np.mean((radial - radius_inner) ** 2 + height ** 2))
        nearest = min(err_outer, err_inner)
        which = "外圆" if err_outer <= err_inner else "内圆"
        print(f"  轨迹 {index + 1}: 最近{which} RMSE {nearest:.2f} mm, 高度均值 {height.mean():.2f} mm")


def parse_args(argv: list[str]):
    parser = argparse.ArgumentParser(description="提取平板上曲线工件的弯曲焊缝轨迹（单位 mm）")
    parser.add_argument("cloud", nargs="?", help="Open3D 可读的点云，如 .pcd / .ply")
    parser.add_argument("--demo", action="store_true", help="生成平板+圆柱壳并提取，用于核对算法")
    parser.add_argument("--voxel", type=float, default=1.0, help="体素，mm")
    parser.add_argument("--plane-thresh", type=float, default=1.5, help="底板 RANSAC 距离，mm")
    parser.add_argument("--gap", type=float, default=2.0, help="高于底板多少算工件，mm")
    parser.add_argument("--band", type=float, default=12.0, help="焊根接触带高度，mm")
    parser.add_argument("--margin", type=float, default=12.0, help="保留工件周围的底板宽度，mm")
    parser.add_argument("--asym-ratio", type=float, default=0.22, help="保留不对称度最高的比例")
    parser.add_argument("--cluster-eps", type=float, default=6.0, help="焊缝簇 DBSCAN 半径，mm")
    parser.add_argument("--min-cluster", type=int, default=25)
    parser.add_argument("--thin-radius", type=float, default=10.0, help="中心线细化邻域半径，mm")
    parser.add_argument("--step", type=float, default=2.0, help="输出轨迹点间距，mm")
    parser.add_argument("--smooth", type=float, default=1.5, help="样条允许偏差，mm")
    parser.add_argument("--min-length", type=float, default=30.0, help="短于该长度的轨迹丢弃，mm")
    parser.add_argument("--out", default="weld_trajectory_mm.txt")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--screenshot", default="", help="可选，保存可视化 png")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.demo:
        demo_path = Path("demo_plate_curved_shell.ply")
        _, radius_outer, radius_inner, center_y = synthesize_demo(demo_path)
        args.cloud = str(demo_path)
        print(f"已生成示例点云 {demo_path}")
    if not args.cloud:
        print("请传入点云路径，或使用 --demo。单位必须是毫米。", file=sys.stderr)
        return 2

    result = extract_weld_trajectories(load_cloud(args.cloud), args)
    save_trajectory(Path(args.out), result["trajectories"])
    if args.demo:
        evaluate_demo(result["trajectories"], radius_outer, radius_inner, center_y)
    if args.screenshot or not args.no_show:
        show_geometries(build_geometries(result), args.screenshot or None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
