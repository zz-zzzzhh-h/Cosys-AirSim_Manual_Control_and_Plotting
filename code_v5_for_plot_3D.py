"""
Author:      Zhu Zihua
Date:        2025-12-07
Version:     5.2
Description: 3D real-time plotting of leader drone trajectory with gradient color, autoscaled axes,
             and saving time-series plots (x, y, z, roll, pitch, yaw vs time), 2D projections (xy, yz, xz),
             and an overall GIF. Outputs are saved into an auto-incremented folder under ./output.
             Plot Z as ENU (up positive) by negating NED z for all figures and GIF.
"""

#-----Imports------------

import os
import time
import math
import glob
from datetime import datetime

import numpy as np
import cosysairsim as airsim
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.colors as mcolors

# 3D collections for gradient line
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed to enable 3D)
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Optional GIF writer
try:
    import imageio.v2 as imageio
    IMAGEIO_AVAILABLE = True
except ImportError:
    IMAGEIO_AVAILABLE = False


#-----Constants----------

LEADER_NAME = "Leader"
DT = 0.05  # 20Hz
RUNTIME_LIMIT_SEC = None  # set number (e.g., 120) to auto-stop; None for Ctrl+C
GIF_SPEED_MULTIPLIER = 3  # GIF speed relative to real-time

# -------------------- Utils --------------------

def quat_to_rpy(q: airsim.Quaternionr):
    # Convert quaternion to roll-pitch-yaw (radians)
    w = q.w_val
    x = q.x_val
    y = q.y_val
    z = q.z_val
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def ensure_output_dir(base_dir: str) -> str:
    out_root = os.path.join(base_dir, "output")
    os.makedirs(out_root, exist_ok=True)
    # Find next index
    existing = sorted(glob.glob(os.path.join(out_root, "output_*")))
    idx = 1
    if existing:
        # parse last index
        last = existing[-1]
        try:
            idx = int(os.path.basename(last).split("_")[-1]) + 1
        except Exception:
            idx = len(existing) + 1
    folder_name = f"output_{idx:02d}"
    out_dir = os.path.join(out_root, folder_name)
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def build_segments_3d(xs, ys, zs):
    if len(xs) < 2:
        return np.empty((0, 2, 3))
    pts = np.column_stack([xs, ys, zs])
    return np.concatenate([pts[:-1, None, :], pts[1:, None, :]], axis=1)


# -------------------- Main --------------------

def main():
    # Prepare output folder
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = ensure_output_dir(script_dir)
    print(f"Output folder: {out_dir}")

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, LEADER_NAME)
    client.armDisarm(True, LEADER_NAME)

    t0_global = time.time()

    t_list = []
    xs, ys, zs = [], [], []           # NED raw (z down positive)
    z_plot = []                       # ENU for plotting (z up positive) = -zs
    rolls, pitches, yaws = [], [], []

    # Realtime 3D plot
    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('Leader 3D Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m, ENU)')
    ax.grid(True)

    # Gradient line setup (3D)
    cmap = cm.plasma
    norm = mcolors.Normalize(vmin=0, vmax=1)
    lc3d = Line3DCollection([], cmap=cmap, norm=norm, linewidth=2)
    ax.add_collection(lc3d)
    cb = fig.colorbar(lc3d, ax=ax, pad=0.1)
    cb.set_label('Time (s)')

    frames = []

    print('Start plotting 3D trajectory... Press Ctrl+C to stop.')
    try:
        while True:
            now = time.time()
            if RUNTIME_LIMIT_SEC is not None and (now - t0_global) >= RUNTIME_LIMIT_SEC:
                break

            state = client.getMultirotorState(vehicle_name=LEADER_NAME)
            pos = state.kinematics_estimated.position
            orient = state.kinematics_estimated.orientation

            t_list.append(now - t0_global)
            xs.append(pos.x_val)
            ys.append(pos.y_val)
            zs.append(pos.z_val)        # NED raw
            z_plot.append(-pos.z_val)   # ENU for plotting
            r, p, y = quat_to_rpy(orient)
            rolls.append(r)
            pitches.append(p)
            yaws.append(y)

            # Update gradient 3D path (use ENU z for plotting)
            segments = build_segments_3d(xs, ys, z_plot)
            lc3d.set_segments(segments)
            if len(xs) > 1:
                seg_times = np.array(t_list[1:])
                lc3d.set_array(seg_times)
                lc3d.set_norm(mcolors.Normalize(vmin=0.0, vmax=max(1e-6, t_list[-1])))

            # Autoscale axes to fit data (with margin) using ENU z
            margin = 1.5
            ax.set_xlim3d(min(xs) - margin, max(xs) + margin)
            ax.set_ylim3d(min(ys) - margin, max(ys) + margin)
            ax.set_zlim3d(min(z_plot) - margin, max(z_plot) + margin)

            plt.draw()
            plt.pause(DT)

            # Save frame for GIF
            fig.canvas.draw()
            rgba = np.asarray(fig.canvas.buffer_rgba())
            image = rgba[:, :, :3].copy()
            frames.append(image)

    except KeyboardInterrupt:
        print('Stopped plotting.')
    finally:
        plt.ioff()
        # Save final 3D figure PNG
        fig_path = os.path.join(out_dir, 'trajectory_3d.png')
        try:
            fig.savefig(fig_path, dpi=150, bbox_inches='tight')
            print(f'Saved 3D figure: {fig_path}')
        except Exception as e:
            print('Failed to save 3D figure:', e)
        plt.close(fig)

        # ---------- Save time-series plots ----------
        if len(t_list) >= 2:
            # Positions over time
            def save_ts(title, ys_data, ylabel, fname):
                fig_ts, ax_ts = plt.subplots(figsize=(8, 4))
                ax_ts.set_title(title)
                ax_ts.set_xlabel('Time (s)')
                ax_ts.set_ylabel(ylabel)
                ax_ts.grid(True)
                ax_ts.plot(t_list, ys_data, 'b-')
                out_path = os.path.join(out_dir, fname)
                fig_ts.savefig(out_path, dpi=150, bbox_inches='tight')
                plt.close(fig_ts)
                print(f'Saved: {out_path}')

            save_ts('X vs Time', xs, 'X (m)', 'x_time.png')
            save_ts('Y vs Time', ys, 'Y (m)', 'y_time.png')
            # Z (ENU, up positive)
            save_ts('Z (ENU) vs Time', z_plot, 'Z (m, ENU)', 'z_time.png')

            # Attitude angles vs time (degrees)
            rolls_deg = np.degrees(np.unwrap(rolls))
            pitches_deg = np.degrees(np.unwrap(pitches))
            yaws_deg = np.degrees(np.unwrap(yaws))
            save_ts('Roll vs Time', rolls_deg, 'Roll (deg)', 'roll_time.png')
            save_ts('Pitch vs Time', pitches_deg, 'Pitch (deg)', 'pitch_time.png')
            save_ts('Yaw vs Time', yaws_deg, 'Yaw (deg)', 'yaw_time.png')

            # ---------- Save 2D projections (use ENU z) ----------
            def save_2d(title, x_data, y_data, xlabel, ylabel, fname, t_list_local):
                fig2d, ax2d = plt.subplots(figsize=(6, 6))
                ax2d.set_title(title)
                ax2d.set_xlabel(xlabel)
                ax2d.set_ylabel(ylabel)
                ax2d.grid(True)
                from matplotlib.collections import LineCollection
                if len(x_data) > 1:
                    pts2d = np.column_stack([x_data, y_data])
                    segments2d = np.concatenate([pts2d[:-1, None, :], pts2d[1:, None, :]], axis=1)
                    lc2d = LineCollection(segments2d, cmap=cm.plasma, linewidth=2)
                    seg_times2d = np.array(t_list_local[1:])
                    lc2d.set_array(seg_times2d)
                    lc2d.set_norm(mcolors.Normalize(vmin=0.0, vmax=max(1e-6, t_list_local[-1])))
                    ax2d.add_collection(lc2d)
                    cb2 = fig2d.colorbar(lc2d, ax=ax2d)
                    cb2.set_label('Time (s)')
                # autoscale with equal aspect
                if len(x_data) > 0:
                    margin = 1.5
                    xmin, xmax = min(x_data) - margin, max(x_data) + margin
                    ymin, ymax = min(y_data) - margin, max(y_data) + margin
                    ax2d.set_xlim(xmin, xmax)
                    ax2d.set_ylim(ymin, ymax)
                    # set equal aspect
                    ax2d.set_aspect('equal', adjustable='box')
                out_path = os.path.join(out_dir, fname)
                fig2d.savefig(out_path, dpi=150, bbox_inches='tight')
                plt.close(fig2d)
                print(f'Saved: {out_path}')

            save_2d('XY Plane', xs, ys, 'X (m)', 'Y (m)', 'xy.png', t_list)
            save_2d('YZ Plane', ys, z_plot, 'Y (m)', 'Z (m, ENU)', 'yz.png', t_list)
            save_2d('XZ Plane', xs, z_plot, 'X (m)', 'Z (m, ENU)', 'xz.png', t_list)
        else:
            print('Not enough samples to create time-series and projections.')

        # ---------- Save GIF ----------
        if IMAGEIO_AVAILABLE and len(frames) > 0:
            gif_path = os.path.join(out_dir, 'trajectory_3d.gif')
            try:
                base_fps = max(1, int(round(1/DT)))
                imageio.mimsave(gif_path, frames, fps=base_fps * GIF_SPEED_MULTIPLIER)  # 3x speed
                print(f'Saved GIF: {gif_path} (fps={base_fps * GIF_SPEED_MULTIPLIER})')
            except Exception as e:
                print('Failed to save GIF:', e)
        elif not IMAGEIO_AVAILABLE:
            print('imageio not installed. Install with: pip install imageio')
        else:
            print('No frames captured. GIF not saved.')

    # Optional: disable API control at end (commented to keep control outside)
    # client.armDisarm(False, vehicle_name=LEADER_NAME)
    # client.enableApiControl(False, vehicle_name=LEADER_NAME)


if __name__ == '__main__':
    main()
