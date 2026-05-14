"""
plots.py — standard figure helpers for CORGI experiments.

All functions save to `results_dir` and close the figure.
No ROS2 dependency.

Functions (inner EKF + VICON)
------------------------------
set_style()
plot_trajectory_xy(ekf, vi, results_dir, label)
plot_position_timeseries(ekf, vicon_i, results_dir, T_WALK_END, label)
plot_position_error(ekf, err_3d, results_dir, T_WALK_END, metrics)
plot_velocity_timeseries(ekf, vicon_i, results_dir, T_WALK_END, metrics, label)
plot_attitude(ekf, vicon_i, ekf_rpy, results_dir, T_WALK_END, label)
plot_bias(ba, bw, results_dir, T_WALK_END)
plot_foot_heights(vi, results_dir)
plot_contact_timeline(vi, gmo, results_dir, T_WALK_END, label)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch


# ──────────────────────────────────────────────────────────────────────────────
# Style
# ──────────────────────────────────────────────────────────────────────────────

C_EKF   = '#E53935'   # red
C_VICON = '#1E88E5'   # blue
C_ODOM  = '#43A047'   # green


def set_style(dpi: int = 120, fontsize: int = 9):
    plt.rcParams.update({'figure.dpi': dpi, 'font.size': fontsize})


# ──────────────────────────────────────────────────────────────────────────────
# Inner EKF vs VICON
# ──────────────────────────────────────────────────────────────────────────────

def plot_trajectory_xy(ekf: dict, vi, results_dir: str, label: str = 'EKF'):
    """fig01_trajectory_xy.png"""
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(ekf['px'], ekf['py'], label=label, lw=1.5, color=C_EKF)
    vm = ~np.isnan(vi.pos_m[:, 0])
    ax.plot(vi.pos_m[vm, 0], vi.pos_m[vm, 1],
            label='VICON', lw=1.5, color=C_VICON, ls='--')
    ax.set_aspect('equal')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title(f'XY Trajectory: {label} vs VICON')
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig01_trajectory_xy.png'))
    plt.close()


def plot_position_timeseries(ekf: dict, vicon_i: dict, results_dir: str,
                              T_WALK_END: float, label: str = 'EKF'):
    """fig02_position_timeseries.png"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    for ax, lbl, ek, vi, cv in zip(
            axes, ['X', 'Y', 'Z'],
            [ekf['px'], ekf['py'], ekf['pz']],
            [vicon_i['px'], vicon_i['py'], vicon_i['pz']],
            [np.sqrt(np.abs(ekf['cov_px'])),
             np.sqrt(np.abs(ekf['cov_py'])),
             np.sqrt(np.abs(ekf['cov_pz']))]):
        ax.plot(ekf['t'], ek, label=label, color=C_EKF, lw=1.2)
        ax.plot(ekf['t'], vi, label='VICON', color=C_VICON, lw=1.2, ls='--')
        ax.fill_between(ekf['t'], ek - 3*cv, ek + 3*cv, alpha=0.2, color=C_EKF)
        ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
        ax.set_ylabel(f'{lbl} [m]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time [s]')
    axes[0].set_title(f'Position: {label} vs VICON')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig02_position_timeseries.png'))
    plt.close()


def plot_position_error(ekf: dict, err_3d: np.ndarray, results_dir: str,
                         T_WALK_END: float, metrics: dict):
    """fig03_position_error.png"""
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.plot(ekf['t'], err_3d * 1000, lw=1.0, color='#9C27B0')
    ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('3D error [mm]')
    rmse_mm = metrics['pos_rmse_3d'] * 1000
    ax.set_title(f'3D Position Error — RMSE={rmse_mm:.1f} mm')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig03_position_error.png'))
    plt.close()


def plot_velocity_timeseries(ekf: dict, vicon_i: dict, results_dir: str,
                              T_WALK_END: float, metrics: dict,
                              label: str = 'EKF'):
    """fig04_velocity_timeseries.png"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    for ax, lbl, ev, vv, cv in zip(
            axes, ['Vx', 'Vy', 'Vz'],
            [ekf['vx'], ekf['vy'], ekf['vz']],
            [vicon_i['vx'], vicon_i['vy'], vicon_i['vz']],
            [np.sqrt(np.abs(ekf['cov_vx'])),
             np.sqrt(np.abs(ekf['cov_vy'])),
             np.sqrt(np.abs(ekf['cov_vz']))]):
        ax.plot(ekf['t'], ev, label=label, color=C_EKF, lw=1.2)
        ax.plot(ekf['t'], vv, label='VICON SG', color=C_VICON, lw=1.2, ls='--')
        ax.fill_between(ekf['t'], ev - 3*cv, ev + 3*cv, alpha=0.2, color=C_EKF)
        ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
        ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time [s]')
    vx_rmse_mm = metrics['vel_rmse_x'] * 1000
    axes[0].set_title(
        f'Body-Frame Velocity: {label} vs VICON  |  Vx RMSE={vx_rmse_mm:.1f}mm/s')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig04_velocity_timeseries.png'))
    plt.close()


def plot_attitude(ekf: dict, vicon_i: dict, ekf_rpy: np.ndarray,
                  results_dir: str, T_WALK_END: float, label: str = 'EKF'):
    """fig05_attitude_rpy.png"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    for i, (lbl, ea, va) in enumerate(zip(
            ['Roll', 'Pitch', 'Yaw'],
            [ekf_rpy[:, 0], ekf_rpy[:, 1], ekf_rpy[:, 2]],
            [vicon_i['roll'], vicon_i['pitch'], vicon_i['yaw']])):
        axes[i].plot(ekf['t'], np.degrees(ea), label=label, color=C_EKF, lw=1.2)
        axes[i].plot(ekf['t'], np.degrees(va), label='VICON', color=C_VICON,
                     lw=1.2, ls='--')
        axes[i].axvline(T_WALK_END, color='gray', ls=':', lw=1)
        axes[i].set_ylabel(f'{lbl} [°]')
        axes[i].legend(fontsize=8); axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time [s]')
    axes[0].set_title(f'Attitude (RPY): {label} vs VICON')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig05_attitude_rpy.png'))
    plt.close()


def plot_bias(ba: dict, bw: dict, results_dir: str, T_WALK_END: float):
    """fig06_accel_bias.png, fig07_gyro_bias.png"""
    for fname, data, ylabel, title in [
        ('fig06_accel_bias.png', ba, 'Accel bias [m/s²]', 'Accelerometer Bias ba'),
        ('fig07_gyro_bias.png',  bw, 'Gyro bias [rad/s]', 'Gyroscope Bias bw'),
    ]:
        fig, ax = plt.subplots(figsize=(12, 3))
        for axis, color in zip(['x', 'y', 'z'],
                                [C_EKF, C_VICON, C_ODOM]):
            ax.plot(data['t'], data[axis], label=axis, color=color, lw=1.0)
        ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
        ax.set_xlabel('Time [s]'); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(results_dir, fname))
        plt.close()


def plot_foot_heights(vi, results_dir: str):
    """fig08_foot_heights.png"""
    t = vi.t_traj
    thr_mm = vi.contact_threshold_m * 1000
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
    for ax, leg in zip(axes, ['LF', 'RF', 'RH', 'LH']):
        ax.plot(t, vi.foot_heights[leg] * 1000, lw=0.8, color=C_VICON)
        ax.axhline(thr_mm, color='red', ls='--', lw=1,
                   label=f'{thr_mm:.0f} mm threshold')
        ax.set_ylabel(f'{leg} Z [mm]'); ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('VICON Foot Height vs Contact Threshold')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig08_foot_heights.png'))
    plt.close()


def plot_contact_timeline(vi, gmo: dict, results_dir: str,
                           T_WALK_END: float, label: str = 'EKF'):
    """fig09_contact_timeline.png"""
    def _events(arr, t_arr):
        arr_f = arr.astype(float)
        edges = np.diff(arr_f, prepend=0, append=0)
        si = np.where(edges >  0.5)[0]; si = si[si < len(t_arr)]
        ei = np.where(edges < -0.5)[0]; ei = ei[ei < len(t_arr)]
        starts = t_arr[si]; ends = t_arr[ei]
        if len(ends) < len(starts):
            ends = np.append(ends, t_arr[-1])
        return list(zip(starts, ends))

    fig, axes = plt.subplots(4, 1, figsize=(14, 6), sharex=True)
    for ax, leg in zip(axes, ['LF', 'RF', 'RH', 'LH']):
        nnan = ~np.isnan(vi.foot_heights[leg])
        t_v  = vi.t_traj[nnan]
        for ts, te in _events(vi.contact[leg][nnan], t_v):
            ax.axvspan(ts, te, color='#2196F3', alpha=0.4)
        for ts, te in _events(gmo[leg].astype(bool), gmo['t']):
            ax.axvspan(ts, te, color='#FF5722', alpha=0.3, hatch='//')
        ax.set_ylabel(leg, rotation=0, labelpad=20)
        ax.set_ylim(-0.1, 1.1); ax.set_yticks([])
        ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)

    axes[0].legend(
        handles=[Patch(fc='#2196F3', alpha=0.5, label='VICON'),
                 Patch(fc='#FF5722', alpha=0.5, hatch='//', label='GMO')],
        loc='upper right', fontsize=8)
    axes[0].set_title(f'Contact Timeline: VICON vs GMO ({label})')
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'fig09_contact_timeline.png'))
    plt.close()
