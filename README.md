# 管-板角焊缝 3D 点云寻位

`weld_seam_pipe_plate.py`：输入圆管立在平板上的 PLY 点云（单位 mm），输出整圈焊缝轨迹和每点焊枪坐标系，并用 Open3D 可视化。

坐标系约定（右手系）：

| 轴 | 含义 |
| --- | --- |
| X | 焊缝切向（行走方向） |
| Y | 向上，沿管壁方向（保证 Y·板法向 > 0） |
| Z | 向上离开工件，相对板法向旋转 `--work-angle`（默认 45°，角焊缝标准工作角） |

## 安装

```bash
pip install -r requirements.txt
```

## 运行

```bash
# 弹窗可视化（红=焊缝，三色短线=坐标系 X红/Y绿/Z蓝，橙球=起点）
python weld_seam_pipe_plate.py scan.ply --radius 30

# 无显示器：离屏渲染截图
python weld_seam_pipe_plate.py scan.ply --radius 30 --no-vis --screenshot seam.png
```

输出：

- `seam_poses.csv`：`idx, x, y, z, 旋转矩阵9元素(列=X,Y,Z), 四元数(w,x,y,z)`
- `seam_result.ply`：焊缝线 + 坐标系线段（可在 MeshLab/CloudCompare 打开）
- `seam_result_points.ply`：焊缝点，法向字段为 Z 轴

## 主要参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--radius` / `--radius-tol` | 无 / 2.0 | 管外半径名义值与容差，强烈建议给出，防止拟到焊脚或倒角 |
| `--voxel` | 0.5 | 分割用下采样体素 |
| `--outlier` | radius | 去离群方式。`radius` 只删孤立飞点；`stat` 为统计滤波，会把掠射角下稀疏的管根点云整段删掉，慎用；`none` 不滤 |
| `--outlier-radius` / `--outlier-min-pts` | 2.0 / 3 | 半径滤波参数 |
| `--cluster-eps` | max(4·voxel, 2.5) | 管子聚类半径，根部稀疏断开时加大 |
| `--plane-thr` / `--cyl-thr` | 0.4 / 0.5 | 平面、圆柱 RANSAC 内点阈值 |
| `--step-deg` | 1.0 | 焊缝输出角度步长 |
| `--section-thk` / `--section-window` | 0.8 / 15 | 径向截面半厚与取点半径 |
| `--work-angle` | 45 | Z 相对板法向的工作角，0 表示 Z 垂直于板 |
| `--no-refine` | | 只用解析交线，不做截面精修 |
| `--roi xmin ymin zmin xmax ymax zmax` | | 先裁剪再处理 |

## 算法

1. 预处理：去离群、体素下采样、半径法向
2. RANSAC 平面 -> 平板，法向指向管子一侧
3. 圆柱：法向高斯球 PCA 求轴向，投影后 2D 圆 RANSAC，再非线性精修
4. 圆柱 ∩ 平面解析交线 -> 整圈焊缝初值（遮挡区也有）
5. 每个角度取过轴线的径向截面，板线 ∩ 管壁线 -> 真实焊缝点
6. 相对模型的偏移做环形中值滤波 + MAD 剔野，遮挡区回退模型
7. 生成焊枪坐标系，并按 Y 向上自动确定行走方向

终端会打印平面/圆柱 RMS、半径偏差、管轴倾角、有效截面覆盖率等校验量，超限时给出警告。
