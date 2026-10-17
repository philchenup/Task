# 曲面焊缝提取（LOBB + PCL）

从工件点云里提取曲面焊缝。流程对应 Zhang 等，*A Feature Extraction Approach Over Workpiece Point Clouds for Robotic Welding*，IEEE TASE，2025（DOI: [10.1109/TASE.2023.3345868](https://doi.org/10.1109/TASE.2023.3345868)）。

公开方法是三步，这里都做了：

1. **LOBB 特征**。对每个点取邻域，用协方差主轴建立局部有向包围盒。描述子包括盒的三边比例、点在盒内的位置、线性度 / 平面度 / 表面变化，以及盒内法向的分散程度。包围盒把点推到某一侧时是边界，两个切向都贴近盒面时是角点；法向分成两簇时是折痕。
2. **非线性激活**。每一维先按 5%–95% 分位归一化，再用 Otsu 阈值做 Sigmoid，把平面点和特征点拉开，避免直接聚类被噪声粘住。
3. **分层 K-means**。先分边界和其余点，再在其余点里分平面和折痕，最后在边界里取角点。

曲面焊缝不是工件外轮廓，而是折痕。折痕点的邻域被分成两张局部切平面，两平面的交线就是焊缝。交线点沿曲线排序、平滑、按弧长重采样，并给出焊枪姿态：

- `tangent`：沿焊缝的行进方向
- `approach`：从张开一侧指向焊缝，焊枪沿这个方向送进
- `binormal`：与两者构成右手系

## 编译

依赖 PCL（含 common、io、filters、kdtree、search）和 Eigen。

```bash
sudo apt-get install libpcl-dev build-essential
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j
```

## 运行

```bash
# 正弦交线的曲面角焊缝，以及圆管-平板的闭合焊缝。单位是毫米。
./build/lobb_weld --demo sine --output output/sine
./build/lobb_weld --demo pipe --output output/pipe

# 自己的点云。半径要能同时看到焊缝两侧，默认按点间距自动估计。
./build/lobb_weld --input workpiece.pcd --output output/workpiece
./build/lobb_weld --input workpiece.ply --voxel 1.0 --radius 8 --output output/workpiece
```

`--radius` 和 `--voxel` 与点云使用同一单位。点太密时先用 `--voxel` 降到大约一个焊缝宽度里有若干个点。

输出：

| 文件 | 内容 |
| --- | --- |
| `classes.pcd` | 灰平面、蓝边界、红折痕、橙角点 |
| `crease.pcd` / `boundary.pcd` / `corners.pcd` | 三类特征点 |
| `seam.csv` | 焊缝位姿，多条焊缝继续写 `seam_1.csv` … |
| `seam.pcd` | 焊缝轨迹点 |
| `preview.ppm` | 上：俯视 XY，下：侧视 XZ，绿色是焊缝 |

演示会和真值曲线比较。正弦角焊缝要求平均误差不超过 1 mm，圆管焊缝不超过 1.15 mm。
