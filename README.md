# Cosys-AirSim Manual Control and Trajectory Plotting

本仓库包含两个用于 Cosys-AirSim 无人机（Leader）轨迹可视化的脚本，支持实时绘制、渐变色路径、方向指示与结果导出，便于教学与研究参考。

## 文件说明

- `code_v5_for_leader_trajectory_plot.py`
  - 实时绘制 Leader 在 XY 平面的轨迹
  - 使用渐变色显示时间推进，支持右侧偏移的方向箭头
  - 可录制 GIF（基于 imageio）

- `code_v5_for_plot_3D.py`
  - 实时绘制 Leader 的 3D 轨迹（X-Y-Z），渐变色对应时间
  - Z 轴绘图采用 ENU（向上为正，通过对 NED 的 z 取负实现）
  - 自动缩放坐标轴，2D 投影图（XY、YZ、XZ）保持等比例显示
  - 导出 x/y/z 以及姿态角（roll/pitch/yaw）的随时间曲线 PNG
  - 导出 3D 轨迹 PNG 与 GIF，GIF 默认 3× 速度
  - 所有输出按运行序号保存在 `output/output_XX` 目录中

## 依赖环境

- Python 3.8+
- Cosys-AirSim Python API（`cosysairsim`）
- Matplotlib
- NumPy
- 可选：`imageio`（用于生成 GIF）

安装依赖示例：

```bash
pip install matplotlib numpy imageio
```

> 注：`cosysairsim` 请根据 Cosys-AirSim 的官方文档/发行包进行安装与配置。

## 使用方法

1. 确保 Cosys-AirSim 模拟器已启动且 Leader 处于可连接状态。
2. 运行脚本：
   - 2D XY 轨迹与箭头：
     ```bash
     python code_v5_for_leader_trajectory_plot.py
     ```
   - 3D 轨迹与导出：
     ```bash
     python code_v5_for_plot_3D.py
     ```
3. 运行时按 Ctrl+C 可停止实时绘制；脚本会在结束时自动保存图片/GIF 到 `output/output_XX`。

## 参数与自定义

- `DT`：采样与刷新周期（默认 0.05s，20Hz）
- `ARROW_INTERVAL_SEC`：XY 轨迹箭头间隔时间（1.0s）
- `ARROW_SCALE` / `ARROW_OFFSET`：箭头大小与右侧偏移量
- `GIF_SPEED_MULTIPLIER`：3D GIF 的速度倍率（默认 3）
- 颜色映射：默认 `cm.plasma`，可改为 `cm.viridis` 等

## 输出内容（3D脚本）

- `output/output_XX/trajectory_3d.png`：3D 轨迹图（Z 为 ENU）
- `output/output_XX/trajectory_3d.gif`：3D 动态 GIF（3× 速度）
- `output/output_XX/x_time.png`、`y_time.png`、`z_time.png`：位置随时间
- `output/output_XX/roll_time.png`、`pitch_time.png`、`yaw_time.png`：姿态角随时间（已解包裹，避免 ±180° 跳变）
- `output/output_XX/xy.png`、`yz.png`、`xz.png`：2D 投影图，比例相同

## 注意事项

- 坐标系：AirSim 使用 NED（Z 向下为正）。本项目绘图将 Z 转换为 ENU（向上为正）以更直观。
- 同时运行控制与绘图脚本时，请确保连接同一车辆名（`Leader`）。
- 姿态角绘图做了 `unwrap` 处理，避免角度跨越 ±180° 时的突变。
- GIF 生成依赖 `imageio`，未安装时仅保存 PNG 图片。

## 许可

- 本项目代码供学习与研究参考，版权归原作者所有。请在引用时保留作者与说明。
