#!/usr/bin/env python3
"""
管-板角焊缝（圆管立在平板上）3D 点云寻位。

输入 : PLY 点云（单位 mm，结构光 / 3D 相机单视角或多视角拼接结果）
输出 : 焊缝轨迹点 + 每点焊枪坐标系（X=焊缝切向, Y=向上沿管壁, Z=向上远离工件）
       CSV（位置 + 旋转矩阵 + 四元数）、PLY（焊缝线 + 坐标系）、Open3D 可视化

算法:
  1. 预处理（裁剪 / 去离群 / 下采样 / 法向）
  2. RANSAC 平面  -> 平板 (n, d)
  3. 圆柱拟合     -> 轴向(法向高斯球 PCA) + 2D 圆 RANSAC + 非线性精修 (a, c, R)
  4. 解析交线     -> 圆柱 ∩ 平面, 给出整圈焊缝初值（遮挡区也有）
  5. 径向截面精修 -> 每个角度取过轴线的截面, 板线 ∩ 管壁线 = 真实焊缝点
  6. 鲁棒平滑     -> 相对解析模型的偏移做环形中值滤波 + MAD 剔野, 遮挡区回退模型
  7. 焊枪位姿     -> X 切向, Z 由板法向向管径向旋转 work_angle, Y = Z × X 且 Y·n > 0

用法:
  python weld_seam_pipe_plate.py scan.ply --radius 30 --work-angle 45
  python weld_seam_pipe_plate.py scan.ply --no-vis --out-csv seam.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass

import numpy as np
import open3d as o3d
from scipy.optimize import least_squares


# ----------------------------------------------------------------------------
# 基础工具
# ----------------------------------------------------------------------------
def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def any_perpendicular(a: np.ndarray) -> np.ndarray:
    """返回一个与 a 垂直的单位向量。"""
    helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    return unit(np.cross(a, helper))


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数 (w, x, y, z)。"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# ----------------------------------------------------------------------------
# 2D RANSAC: 直线 / 圆
# ----------------------------------------------------------------------------
@dataclass
class Line2D:
    p: np.ndarray  # 直线上一点
    d: np.ndarray  # 单位方向
    inliers: np.ndarray  # bool mask

    def dist(self, pts: np.ndarray) -> np.ndarray:
        rel = pts - self.p
        return np.abs(rel[:, 0] * self.d[1] - rel[:, 1] * self.d[0])


def fit_line_lsq(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    c = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    return c, unit(vt[0])


def ransac_line_2d(pts: np.ndarray, thr: float, iters: int = 200, rng=None) -> Line2D | None:
    rng = rng or np.random.default_rng(0)
    n = len(pts)
    if n < 2:
        return None
    best_mask, best_cnt = None, 0
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        d = pts[j] - pts[i]
        if np.linalg.norm(d) < 1e-6:
            continue
        d = unit(d)
        rel = pts - pts[i]
        dist = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])
        mask = dist < thr
        cnt = int(mask.sum())
        if cnt > best_cnt:
            best_cnt, best_mask = cnt, mask
    if best_mask is None or best_cnt < 2:
        return None
    c, d = fit_line_lsq(pts[best_mask])
    line = Line2D(c, d, best_mask)
    line.inliers = line.dist(pts) < thr
    if line.inliers.sum() >= 2:
        c, d = fit_line_lsq(pts[line.inliers])
        line = Line2D(c, d, line.inliers)
    return line


def intersect_lines_2d(l1: Line2D, l2: Line2D) -> np.ndarray | None:
    A = np.array([[l1.d[0], -l2.d[0]], [l1.d[1], -l2.d[1]]])
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    t = np.linalg.solve(A, l2.p - l1.p)
    return l1.p + t[0] * l1.d


def circle_from_3pts(p1, p2, p3):
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    return np.array([ux, uy]), r


def fit_circle_lsq(pts: np.ndarray, c0: np.ndarray, r0: float):
    """几何距离最小二乘圆拟合。"""

    def resid(x):
        return np.linalg.norm(pts - x[:2], axis=1) - x[2]

    sol = least_squares(resid, np.r_[c0, r0], loss="soft_l1", f_scale=0.5)
    return sol.x[:2], float(sol.x[2])


def ransac_circle_2d(pts: np.ndarray, thr: float, r_range=None, iters: int = 1000, rng=None):
    rng = rng or np.random.default_rng(1)
    n = len(pts)
    if n < 3:
        raise RuntimeError("圆拟合点数不足")
    best, best_cnt = None, 0
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        res = circle_from_3pts(pts[idx[0]], pts[idx[1]], pts[idx[2]])
        if res is None:
            continue
        c, r = res
        if r_range is not None and not (r_range[0] <= r <= r_range[1]):
            continue
        dist = np.abs(np.linalg.norm(pts - c, axis=1) - r)
        cnt = int((dist < thr).sum())
        if cnt > best_cnt:
            best_cnt, best = cnt, (c, r)
    if best is None:
        raise RuntimeError("圆 RANSAC 失败（检查 --radius / --radius-tol 是否合理）")
    c, r = best
    mask = np.abs(np.linalg.norm(pts - c, axis=1) - r) < thr
    c, r = fit_circle_lsq(pts[mask], c, r)
    mask = np.abs(np.linalg.norm(pts - c, axis=1) - r) < thr
    return c, r, mask


# ----------------------------------------------------------------------------
# 几何基元
# ----------------------------------------------------------------------------
@dataclass
class Plane:
    n: np.ndarray  # 单位法向（指向管子一侧 = "上"）
    d: float  # n·p = d

    def dist(self, pts: np.ndarray) -> np.ndarray:
        return pts @ self.n - self.d


@dataclass
class Cylinder:
    a: np.ndarray  # 单位轴向（指向远离平板一侧）
    c: np.ndarray  # 轴上一点
    R: float
    u: np.ndarray  # 与 a 垂直的正交基 u, v
    v: np.ndarray

    def radial_dist(self, pts: np.ndarray) -> np.ndarray:
        rel = pts - self.c
        rel -= np.outer(rel @ self.a, self.a)
        return np.linalg.norm(rel, axis=1)


def segment_plane(pcd: o3d.geometry.PointCloud, thr: float) -> tuple[Plane, np.ndarray]:
    model, idx = pcd.segment_plane(distance_threshold=thr, ransac_n=3, num_iterations=3000)
    n = np.asarray(model[:3])
    s = np.linalg.norm(n)
    n, d = n / s, -model[3] / s
    inl = np.zeros(len(pcd.points), bool)
    inl[idx] = True
    pts = np.asarray(pcd.points)
    # 让法向指向点数更多的那一侧（管子所在侧）
    off = pts[~inl] @ n - d
    if len(off) and np.median(off) < 0:
        n, d = -n, -d
    return Plane(n, d), inl


def fit_cylinder(pts: np.ndarray, normals: np.ndarray, thr: float,
                 r_nominal: float | None, r_tol: float, plane: Plane) -> tuple[Cylinder, np.ndarray]:
    # --- 1. 轴向：圆柱面法向都垂直于轴 -> 法向高斯球最小奇异方向 ---
    nrm = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-9)
    _, _, vt = np.linalg.svd(nrm, full_matrices=False)
    a = unit(vt[-1])
    if a @ plane.n < 0:
        a = -a
    u = any_perpendicular(a)
    v = np.cross(a, u)

    # --- 2. 投影到 ⊥a 平面做 2D 圆 RANSAC ---
    p2 = np.c_[pts @ u, pts @ v]
    r_range = (r_nominal - r_tol, r_nominal + r_tol) if r_nominal else None
    c2, R, mask = ransac_circle_2d(p2, thr, r_range)
    c = c2[0] * u + c2[1] * v

    # --- 3. 非线性精修 (轴向两角 + 圆心2D + R)，仅用内点 ---
    inl_pts = pts[mask]

    def params_to_cyl(x):
        th, ph, cu, cv, r = x
        a_ = unit(a + th * u + ph * v)
        u_ = any_perpendicular(a_)
        v_ = np.cross(a_, u_)
        c_ = c + cu * u + cv * v
        return Cylinder(a_, c_, r, u_, v_)

    def resid(x):
        return params_to_cyl(x).radial_dist(inl_pts) - x[4]

    sol = least_squares(resid, np.array([0, 0, 0, 0, R]), loss="soft_l1", f_scale=thr)
    cyl = params_to_cyl(sol.x)
    if cyl.a @ plane.n < 0:
        cyl = Cylinder(-cyl.a, cyl.c, cyl.R, cyl.u, -cyl.v)
    mask = np.abs(cyl.radial_dist(pts) - cyl.R) < thr
    return cyl, mask


# ----------------------------------------------------------------------------
# 焊缝：解析交线 + 截面精修 + 平滑
# ----------------------------------------------------------------------------
def analytic_seam(cyl: Cylinder, plane: Plane, step_deg: float):
    t = np.deg2rad(np.arange(0.0, 360.0, step_deg))
    radial = np.cos(t)[:, None] * cyl.u + np.sin(t)[:, None] * cyl.v
    ring = cyl.c + cyl.R * radial
    s = (plane.d - ring @ plane.n) / (plane.n @ cyl.a)
    return ring + s[:, None] * cyl.a, t, radial


def refine_by_sections(raw: np.ndarray, kdtree: o3d.geometry.KDTreeFlann, seam0: np.ndarray,
                       radial: np.ndarray, cyl: Cylinder, plane: Plane,
                       half_thk: float, window: float, line_thr: float, min_pts: int):
    """每个角度取过轴线的径向截面，板线 ∩ 管壁线。返回精修点及有效标志。"""
    rng = np.random.default_rng(2)
    out = seam0.copy()
    ok = np.zeros(len(seam0), bool)
    a = cyl.a
    for i, (p0, r) in enumerate(zip(seam0, radial)):
        m = unit(np.cross(a, r))  # 截面法向
        _, idx, _ = kdtree.search_radius_vector_3d(p0, window)
        if len(idx) < min_pts:
            continue
        nb = raw[idx]
        rel = nb - p0
        sec = nb[np.abs(rel @ m) < half_thk]
        if len(sec) < min_pts:
            continue
        rel = sec - p0
        xy = np.c_[rel @ r, rel @ a]  # x: 径向(向外为正), y: 轴向(向上为正)
        # 根部 1.5mm 以内常是死区/焊脚，不参与直线拟合
        xy = xy[np.linalg.norm(xy, axis=1) > 1.5]
        if len(xy) < min_pts:
            continue
        l1 = ransac_line_2d(xy, line_thr, rng=rng)
        if l1 is None or l1.inliers.sum() < min_pts // 2:
            continue
        rest = xy[~l1.inliers]
        if len(rest) < min_pts // 2:
            continue
        l2 = ransac_line_2d(rest, line_thr, rng=rng)
        if l2 is None or l2.inliers.sum() < min_pts // 2:
            continue
        # 两线夹角太小说明没同时抓到板和管
        if abs(l1.d @ l2.d) > math.cos(math.radians(25)):
            continue
        q = intersect_lines_2d(l1, l2)
        if q is None or np.linalg.norm(q) > 0.5 * window:
            continue
        out[i] = p0 + q[0] * r + q[1] * a
        ok[i] = True
    return out, ok


def smooth_seam(seam0: np.ndarray, refined: np.ndarray, ok: np.ndarray,
                radial: np.ndarray, cyl: Cylinder, win: int = 7, mad_k: float = 3.0):
    """相对解析模型的 (径向, 轴向) 偏移做环形中值滤波 + MAD 剔野，无效处回退模型。"""
    n = len(seam0)
    rel = refined - seam0
    off = np.c_[np.einsum("ij,ij->i", rel, radial), rel @ cyl.a]
    off[~ok] = np.nan
    if ok.sum() < max(5, n // 10):
        return seam0.copy(), np.zeros(n, bool)

    # MAD 剔野
    for k in range(2):
        v = off[ok, k]
        med, mad = np.median(v), np.median(np.abs(v - np.median(v))) + 1e-6
        bad = np.abs(off[:, k] - med) > mad_k * 1.4826 * mad
        ok = ok & ~bad
    off[~ok] = np.nan

    # 环形中值 + 线性插值填补
    half = win // 2
    sm = np.full_like(off, np.nan)
    for i in range(n):
        idx = [(i + j) % n for j in range(-half, half + 1)]
        w = off[idx]
        w = w[~np.isnan(w[:, 0])]
        if len(w) >= 3:
            sm[i] = np.median(w, axis=0)
    valid = ~np.isnan(sm[:, 0])
    for k in range(2):
        if valid.sum() >= 2:
            ang = np.arange(n)
            sm[~valid, k] = np.interp(ang[~valid], ang[valid], sm[valid, k], period=n)
    sm = np.nan_to_num(sm)
    seam = seam0 + sm[:, [0]] * radial + sm[:, [1]] * cyl.a
    return seam, valid


# ----------------------------------------------------------------------------
# 焊枪坐标系
# ----------------------------------------------------------------------------
def build_frames(seam: np.ndarray, radial: np.ndarray, plane: Plane, work_angle_deg: float):
    """
    X: 焊缝切向（行走方向）
    Z: 由板法向 n 朝管外径向 r 旋转 work_angle，远离工件向上
    Y: Z × X，并保证 Y·n > 0（沿管壁向上）；若不满足则反转行走方向
    """
    n = plane.n
    wa = math.radians(work_angle_deg)

    def z_dir(r):
        r_perp = unit(r - (r @ n) * n)  # 径向在板面内的分量
        return unit(math.cos(wa) * n + math.sin(wa) * r_perp)

    def frames_for(order):
        P = seam[order]
        Rd = radial[order]
        m = len(P)
        X = np.zeros_like(P)
        for i in range(m):
            X[i] = unit(P[(i + 1) % m] - P[(i - 1) % m])
        Zs, Ys = np.zeros_like(P), np.zeros_like(P)
        for i in range(m):
            z = z_dir(Rd[i])
            z = unit(z - (z @ X[i]) * X[i])
            y = unit(np.cross(z, X[i]))
            Zs[i], Ys[i] = z, y
        return P, X, Ys, Zs

    def y_score(Y, Rd):
        # Y 应"向上且朝管内"：与 (n - r_perp) 同向。work_angle=0 时 Y⊥n，仍可由朝管内判定
        r_perp = Rd - np.outer(Rd @ n, n)
        return float(np.mean(np.einsum("ij,ij->i", Y, n - r_perp)))

    order = np.arange(len(seam))
    P, X, Y, Z = frames_for(order)
    if y_score(Y, radial[order]) < 0:
        order = order[::-1]
        P, X, Y, Z = frames_for(order)
    rots = np.stack([X, Y, Z], axis=2)  # 列向量 = 轴
    return P, rots, order


# ----------------------------------------------------------------------------
# 输出 / 可视化
# ----------------------------------------------------------------------------
def write_csv(path: str, P: np.ndarray, rots: np.ndarray):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "z",
                    "xx", "xy", "xz", "yx", "yy", "yz", "zx", "zy", "zz",
                    "qw", "qx", "qy", "qz"])
        for i, (p, R) in enumerate(zip(P, rots)):
            q = rot_to_quat(R)
            w.writerow([i, *np.round(p, 4), *np.round(R[:, 0], 6), *np.round(R[:, 1], 6),
                        *np.round(R[:, 2], 6), *np.round(q, 6)])


def seam_lineset(P: np.ndarray, color=(1, 0, 0)) -> o3d.geometry.LineSet:
    m = len(P)
    lines = [[i, (i + 1) % m] for i in range(m)]
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(P), o3d.utility.Vector2iVector(lines))
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (m, 1)))
    return ls


def frames_lineset(P: np.ndarray, rots: np.ndarray, every: int, length: float) -> o3d.geometry.LineSet:
    pts, lines, cols = [], [], []
    palette = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # X红 Y绿 Z蓝
    for i in range(0, len(P), every):
        base = len(pts)
        pts.append(P[i])
        for k in range(3):
            pts.append(P[i] + rots[i][:, k] * length)
            lines.append([base, base + 1 + k])
            cols.append(palette[k])
    ls = o3d.geometry.LineSet(o3d.utility.Vector3dVector(np.asarray(pts)),
                              o3d.utility.Vector2iVector(np.asarray(lines)))
    ls.colors = o3d.utility.Vector3dVector(np.asarray(cols, float))
    return ls


def visualize(pcd, plane_mask, cyl_mask, P, rots, every, axis_len, screenshot: str | None = None):
    pts = np.asarray(pcd.points)
    col = np.full((len(pts), 3), 0.7)
    col[plane_mask] = (0.75, 0.75, 0.55)
    col[cyl_mask] = (0.55, 0.7, 0.85)
    show = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    show.colors = o3d.utility.Vector3dVector(col)
    geos = [show, seam_lineset(P), frames_lineset(P, rots, every, axis_len),
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_len * 3)]
    # 起点标记
    sp = o3d.geometry.TriangleMesh.create_sphere(radius=axis_len * 0.25)
    sp.translate(P[0])
    sp.paint_uniform_color((1, 0.5, 0))
    geos.append(sp)
    title = "Pipe-plate weld seam  (X红=切向  Y绿=向上沿管壁  Z蓝=向上离开工件)"
    if screenshot is None:
        o3d.visualization.draw_geometries(geos, window_name=title, width=1280, height=900)
        return
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=900, visible=False)
    for g in geos:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.point_size = 1.5
    opt.line_width = 3.0
    up = unit(rots[:, :, 2].mean(0))  # 近似板法向
    ctr = vis.get_view_control()
    ctr.set_lookat(P.mean(0))
    ctr.set_front(unit(up + 1.2 * any_perpendicular(up)))  # 斜上方约 40° 俯视
    ctr.set_up(up)
    ctr.set_zoom(0.12)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(screenshot, do_render=True)
    vis.destroy_window()
    print(f"[可视化] 截图已保存: {screenshot}")


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="管-板角焊缝 3D 点云寻位（单位 mm）")
    ap.add_argument("ply", help="输入 PLY 点云")
    ap.add_argument("--radius", type=float, default=None, help="管外半径名义值 mm（强烈建议给）")
    ap.add_argument("--radius-tol", type=float, default=2.0, help="半径容差 mm")
    ap.add_argument("--voxel", type=float, default=0.5, help="分割用下采样体素 mm")
    ap.add_argument("--outlier", choices=["radius", "stat", "none"], default="radius",
                    help="去离群方式: radius=半径滤波(默认,保留稀疏的交界处点) stat=统计滤波 none=不滤")
    ap.add_argument("--outlier-radius", type=float, default=2.0, help="半径滤波搜索半径 mm")
    ap.add_argument("--outlier-min-pts", type=int, default=3, help="半径内至少点数")
    ap.add_argument("--sor-std", type=float, default=4.0, help="统计滤波 std_ratio（仅 --outlier stat）")
    ap.add_argument("--cluster-eps", type=float, default=None,
                    help="管子聚类 DBSCAN 半径 mm，默认 max(4*voxel, 2.5)；根部稀疏时适当加大")
    ap.add_argument("--plane-thr", type=float, default=0.4, help="平面 RANSAC 阈值 mm")
    ap.add_argument("--cyl-thr", type=float, default=0.5, help="圆柱内点阈值 mm")
    ap.add_argument("--step-deg", type=float, default=1.0, help="焊缝角度步长 deg")
    ap.add_argument("--section-thk", type=float, default=0.8, help="截面半厚 mm")
    ap.add_argument("--section-window", type=float, default=15.0, help="截面取点半径 mm")
    ap.add_argument("--line-thr", type=float, default=0.3, help="截面直线 RANSAC 阈值 mm")
    ap.add_argument("--min-section-pts", type=int, default=40)
    ap.add_argument("--work-angle", type=float, default=45.0, help="Z 相对板法向的工作角 deg（0=垂直板）")
    ap.add_argument("--no-refine", action="store_true", help="跳过截面精修，只用解析交线")
    ap.add_argument("--roi", type=float, nargs=6, metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"))
    ap.add_argument("--out-csv", default="seam_poses.csv")
    ap.add_argument("--out-ply", default="seam_result.ply")
    ap.add_argument("--frame-every", type=int, default=10, help="每隔几个点画一个坐标系")
    ap.add_argument("--axis-len", type=float, default=6.0, help="坐标系轴长 mm")
    ap.add_argument("--no-vis", action="store_true")
    ap.add_argument("--screenshot", default=None, help="离屏渲染并保存 PNG（无显示器时使用）")
    args = ap.parse_args()

    # ---------- 1. 读取与预处理 ----------
    pcd = o3d.io.read_point_cloud(args.ply)
    if pcd.is_empty():
        sys.exit(f"读取失败或为空: {args.ply}")
    pcd.remove_non_finite_points()
    if args.roi:
        bb = o3d.geometry.AxisAlignedBoundingBox(np.array(args.roi[:3]), np.array(args.roi[3:]))
        pcd = pcd.crop(bb)
    n_in = len(pcd.points)
    print(f"[输入] {n_in} 点")
    # 注意: 统计离群点滤波(SOR)是按"平均邻距"全局判定的, 管子根部在掠射角下点密度骤降,
    # 整段会被当成离群点删掉(交界处点云消失)。默认用半径滤波, 只删孤立飞点, 不删稀疏面。
    if args.outlier == "radius":
        pcd, _ = pcd.remove_radius_outlier(nb_points=args.outlier_min_pts, radius=args.outlier_radius)
    elif args.outlier == "stat":
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=args.sor_std)
    print(f"[去离群] 模式={args.outlier}  移除 {n_in - len(pcd.points)} 点 "
          f"({100 * (n_in - len(pcd.points)) / max(n_in, 1):.1f}%)")
    raw = np.asarray(pcd.points)
    kdtree_raw = o3d.geometry.KDTreeFlann(pcd)

    ds = pcd.voxel_down_sample(args.voxel)
    ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 6, max_nn=50))
    pts = np.asarray(ds.points)
    nrm = np.asarray(ds.normals)
    print(f"[下采样] {len(pts)} 点")

    # ---------- 2. 平板 ----------
    plane, plane_mask = segment_plane(ds, args.plane_thr)
    plane_rms = float(np.sqrt(np.mean(plane.dist(pts[plane_mask]) ** 2)))
    print(f"[平面] n={np.round(plane.n, 4)} d={plane.d:.3f}  内点={plane_mask.sum()}  RMS={plane_rms:.3f} mm")

    # ---------- 3. 圆管 ----------
    above = (~plane_mask) & (plane.dist(pts) > 3 * args.plane_thr)
    cand = ds.select_by_index(np.where(above)[0])
    eps = args.cluster_eps or max(args.voxel * 4, 2.5)
    labels = np.asarray(cand.cluster_dbscan(eps=eps, min_points=5))
    if labels.max() < 0:
        sys.exit("平板之上没有找到管子点云")
    big = np.argmax(np.bincount(labels[labels >= 0]))
    cand_idx = np.where(above)[0][labels == big]
    cyl, cyl_mask_local = fit_cylinder(pts[cand_idx], nrm[cand_idx], args.cyl_thr,
                                       args.radius, args.radius_tol, plane)
    cyl_mask = np.zeros(len(pts), bool)
    cyl_mask[cand_idx[cyl_mask_local]] = True
    cyl_rms = float(np.sqrt(np.mean((cyl.radial_dist(pts[cyl_mask]) - cyl.R) ** 2)))
    tilt = math.degrees(math.acos(np.clip(cyl.a @ plane.n, -1, 1)))
    print(f"[圆柱] 轴向={np.round(cyl.a, 4)}  R={cyl.R:.3f} mm  内点={cyl_mask.sum()}  "
          f"RMS={cyl_rms:.3f} mm  管轴相对板法向倾角={tilt:.2f}°")
    if args.radius and abs(cyl.R - args.radius) > args.radius_tol:
        print(f"[警告] 拟合半径与名义值偏差 {cyl.R - args.radius:+.2f} mm")

    # ---------- 4. 解析交线 ----------
    seam0, t, radial = analytic_seam(cyl, plane, args.step_deg)

    # ---------- 5/6. 截面精修 + 平滑 ----------
    if args.no_refine:
        seam, valid = seam0, np.zeros(len(seam0), bool)
    else:
        refined, ok = refine_by_sections(raw, kdtree_raw, seam0, radial, cyl, plane,
                                         args.section_thk, args.section_window,
                                         args.line_thr, args.min_section_pts)
        seam, valid = smooth_seam(seam0, refined, ok, radial, cyl)
        dev = np.linalg.norm(seam - seam0, axis=1)
        print(f"[精修] 有效截面 {ok.sum()}/{len(ok)} ({100*ok.mean():.0f}%)  "
              f"相对解析交线平均偏移 {dev[valid].mean() if valid.any() else 0:.3f} mm, "
              f"最大 {dev.max():.3f} mm")

    # ---------- 7. 焊枪位姿 ----------
    P, rots, order = build_frames(seam, radial, plane, args.work_angle)
    valid = valid[order]
    Y_up = float(np.mean(rots[:, :, 1] @ plane.n))
    Z_up = float(np.mean(rots[:, :, 2] @ plane.n))
    Y_in = -float(np.mean(np.einsum("ij,ij->i", rots[:, :, 1], radial[order])))
    reversed_ = len(order) > 1 and order[1] < order[0]
    print(f"[位姿] {len(P)} 点  行走方向={'反向(已翻转以保证Y向上)' if reversed_ else '正向'}  "
          f"mean(Y·n)={Y_up:+.3f}  mean(Y·管内)={Y_in:+.3f}  mean(Z·n)={Z_up:+.3f}  (都应 >=0)")

    # ---------- 输出 ----------
    write_csv(args.out_csv, P, rots)
    merged = seam_lineset(P) + frames_lineset(P, rots, args.frame_every, args.axis_len)
    o3d.io.write_line_set(args.out_ply, merged)
    seam_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
    seam_pcd.normals = o3d.utility.Vector3dVector(rots[:, :, 2])
    o3d.io.write_point_cloud(args.out_ply.replace(".ply", "_points.ply"), seam_pcd)
    print(f"[输出] {args.out_csv}  {args.out_ply}  {args.out_ply.replace('.ply', '_points.ply')}")

    # ---------- 质量门限 ----------
    warn = []
    if plane_rms > 2 * args.plane_thr:
        warn.append("平面 RMS 偏大")
    if cyl_rms > 2 * args.cyl_thr:
        warn.append("圆柱 RMS 偏大")
    if not args.no_refine and valid.mean() < 0.4:
        warn.append("有效截面覆盖率 <40%，焊缝大部分来自模型外推")
    if Y_up < -1e-6 or Y_in <= 0 or Z_up <= 0:
        warn.append("坐标系朝向异常")
    print("[校验] " + ("OK" if not warn else "；".join(warn)))

    if args.screenshot or not args.no_vis:
        try:
            visualize(ds, plane_mask, cyl_mask, P, rots, args.frame_every, args.axis_len, args.screenshot)
        except Exception as e:  # 无显示环境
            print(f"[可视化] 跳过: {e}")


if __name__ == "__main__":
    main()
