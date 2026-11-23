# 流程编辑器（Qt 5.15）

用 Qt Widgets 搭的节点式流程画布，交互参照视觉步骤编排：左侧步骤库拖到网格上，再用端口连线。

只用 **Qt Widgets**（`Qt Core` / `Qt Gui` / `Qt Widgets`）。这些模块在 Qt 5.15 开源版里，协议是 LGPLv3 或 GPL，不需要 Qt 商业模块。

## 构建

Qt Creator：打开 `FlowEditor.pro`，Kit 选择 Qt 5.15，然后构建运行。

CMake：

```bash
cd flow-editor
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/5.15/gcc_64
cmake --build build
./build/flow-editor
```

本机已安装 Qt 5.15 开发包时：

```bash
cd flow-editor
qmake FlowEditor.pro -o build/Makefile && make -C build
./build/flow-editor
```

## 操作

- 从「步骤库」把步骤拖到画布，或双击步骤，加到视口中心。
- 按住输出端口（步骤下方的色块）拖到另一个步骤的输入端口（上方的色块）。类型必须一致，例如 `<Cloud(XYZ)>` 只能连到 `<Cloud(XYZ)>`。
- 拖动步骤本体可以挪位置，连线会跟着走。滚轮缩放，鼠标中键或 Alt+左键平移。
- `Delete` 删除选中的步骤和连线，右键也有删除。`Ctrl+0` 恢复 100% 缩放。
- 「运行」按依赖顺序走一遍拓扑；输入没接上的步骤会在「视觉」日志里跳过。
- 左侧工程列表会各自记住画布。打开时「算子库测试」里已经放好三条示例连线。
