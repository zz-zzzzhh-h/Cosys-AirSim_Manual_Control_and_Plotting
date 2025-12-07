# Cosys-AirSim Leader Drone Plotting & Control (V5)

本项目包含三个独立脚本，分别用于：
- `settings.json`：用于替换 AirSim 中的默认设置
- `code_v5_for_leader_mission_control.py`：控制 Leader 无人机按任务（示例为矩形）飞行。
- `code_v5_for_plot_3D.py`：实时记录并绘制 Leader 的 3D 轨迹（渐变色、坐标轴自适应），保存时间序列与投影图，以及 GIF（可加速）。

## 功能概览
- 实时 3D 轨迹（渐变色按时间），Z 轴按 ENU（向上为正）显示。
- 自动保存：
  - x/y/z 与姿态角（roll/pitch/yaw，度）关于时间的 PNG
  - xy、yz、xz 平面投影 PNG（等比例坐标）
  - 3D 轨迹 GIF（默认 3× 速度）
- 结果保存在脚本同目录的 `output/output_XX` 新建文件夹中（自动递增）。
- 手动控制脚本支持加减速与偏航速率控制，示例任务脚本演示按方形航线飞行。

## 环境要求
- 操作系统：Windows
- Python 3.11
- 依赖：
  - `cosysairsim`（Cosys-AirSim Python API）
  - `matplotlib`
  - `numpy`
  - `imageio`（可选，用于生成 GIF）

使用 pip 安装依赖：
```
pip install numpy matplotlib imageio
```

## 运行方式
建议在两个独立终端中分别运行控制与绘图。

0) 替换 `settings.json`
使用文件中的 `settings.json` 替换掉 `C:\Users\Administrator\Documents\AirSim` 中的同名文件 

1) 启动任务控制（示例矩形航线）
```
python code_v5_for_leader_mission_control.py
```
- 键位：W/S 前进/后退，A/D 左/右，U/I 下/上（NED 中 z 向下为正），J/L 左/右偏航，K 归零偏航速率，P 降落，ESC 退出。

2) 启动 3D 轨迹绘图与记录
```
python code_v5_for_plot_3D.py
```
- 实时窗口显示 3D 轨迹与颜色条（时间），结束后自动在 `output/output_XX` 目录生成所有 PNG 与 GIF 文件。
- 可在脚本顶部调整：
  - `DT`：采样与绘图周期
  - `GIF_SPEED_MULTIPLIER`：GIF 加速倍率（默认 3）
  - `RUNTIME_LIMIT_SEC`：运行时长限制（None 为手动 Ctrl+C 停止）

## 目录结构（输出示例）
```
output/
  output_01/
    trajectory_3d.png
    trajectory_3d.gif
    x_time.png
    y_time.png
    z_time.png
    roll_time.png
    pitch_time.png
    yaw_time.png
    xy.png
    yz.png
    xz.png
```

## 如果能帮到你，请不吝 Star一下！之后会持续更新基于 Cosys-AirSim 的相关代码，大家的支持是我最大的动力！THANKS :) ！！！
