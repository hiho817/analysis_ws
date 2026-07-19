#!/usr/bin/env python3
"""
CORGI Batch Analysis — 20260709
處理 RUGG Walk 與 Obstacle MPC，各分 NEW (ESEKF) 和 OLD (legacy)。

執行前需 source ROS2 workspace：
    source ~/corgi_ws/corgi_ros2_ws/install/setup.bash

用法：
    python3 analyze.py [--exp EXP_ID]  # 只跑特定實驗
    python3 analyze.py                  # 跑所有 21 筆有效實驗
"""

import os, sys, json, argparse, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_fusion_bag, load_legacy_bag

# ─── Experiment manifest ──────────────────────────────────────────────────────
# (exp_id, group, bag_name, vicon_csv, trigger_pair, flip_new, flip_old, exclude_stats)
#   trigger_pair  : int — which trigger ON/OFF pair to use (0=first, 1=second)
#   flip_new      : set of estimator signals to negate for NEW bags
#   flip_old      : set of estimator signals to negate for OLD bags
#   exclude_stats : bool — exclude this trial from group statistics
EXPERIMENTS = [
    ('RUGG_WLW_NEW_REAL_1', 'NEW_RUGG_WLW', 'odom_fusion20260719_211247', 'RUGG_WLW_NEW_REAL_1.csv', 0, set(), set(), False),
    ('RUGG_Walk_NEW_REAL_1', 'NEW_RUGG_WALK', 'odom_fusion20260709_161303', 'RUGG_Walk_NEW_REAL_1.csv', 0, set(), set(), False),
    ('RUGG_Walk_NEW_REAL_2', 'NEW_RUGG_WALK', 'odom_fusion20260709_161701', 'RUGG_Walk_NEW_REAL_2.csv', 0, set(), set(), False),
    ('RUGG_Walk_NEW_REAL_3', 'NEW_RUGG_WALK', 'odom_fusion20260709_162549', 'RUGG_Walk_NEW_REAL_3.csv', 0, set(), set(), False),
    ('RUGG_Walk_NEW_REAL_4', 'NEW_RUGG_WALK', 'odom_fusion20260709_162933', 'RUGG_Walk_NEW_REAL_4.csv', 0, set(), set(), True),
    ('RUGG_Walk_NEW_REAL_5', 'NEW_RUGG_WALK', 'odom_fusion20260709_163750', 'RUGG_Walk_NEW_REAL_5.csv', 0, set(), set(), False),
    ('RUGG_Walk_NEW_REAL_6', 'NEW_RUGG_WALK', 'odom_fusion20260709_164116', 'RUGG_Walk_NEW_REAL_6.csv', 0, set(), set(), True),
    ('RUGG_Walk_OLD_REAL_1', 'OLD_RUGG_WALK', 'legacy_odom20260709_165454', 'RUGG_Walk_OLD_REAL_1.csv', 0, set(), set(), False),
    ('RUGG_Walk_OLD_REAL_2', 'OLD_RUGG_WALK', 'legacy_odom20260709_165747', 'RUGG_Walk_OLD_REAL_2.csv', 0, set(), set(), False),
    ('RUGG_Walk_OLD_REAL_3', 'OLD_RUGG_WALK', 'legacy_odom20260709_170927', 'RUGG_WALK_OLD__REAL_3.csv', 0, set(), set(), False),
    ('RUGG_Walk_OLD_REAL_4', 'OLD_RUGG_WALK', 'legacy_odom20260709_171140', 'RUGG_WALK_OLD__REAL_4.csv', 0, set(), set(), False),
    ('RUGG_Walk_OLD_REAL_5', 'OLD_RUGG_WALK', 'legacy_odom20260709_171340', 'RUGG_WALK_OLD__REAL_5.csv', 0, set(), set(), False),
    ('OBS_MPC_NEW_REAL_1', 'NEW_OBS_MPC_GAIT', 'mpc_esekf_20260709_172919', 'OBS_MPC_NEW_REAL_1.csv', 0, set(), set(), True),
    ('OBS_MPC_NEW_REAL_2', 'NEW_OBS_MPC_GAIT', 'mpc_esekf_20260709_173128', 'OBS_MPC_NEW_REAL_2.csv', 0, set(), set(), True),
    ('OBS_MPC_NEW_REAL_3', 'NEW_OBS_MPC_GMO', 'mpc_esekf_20260709_173434', 'OBS_MPC_NEW_REAL_3.csv', 0, set(), set(), False),
    ('OBS_MPC_NEW_REAL_4', 'NEW_OBS_MPC_GMO', 'mpc_esekf_20260709_173921', 'OBS_MPC_NEW_REAL_4.csv', 0, set(), set(), False),
    ('OBS_MPC_NEW_REAL_5', 'NEW_OBS_MPC_GMO', 'mpc_esekf_20260709_174321', 'OBS_MPC_NEW_REAL_5.csv', 0, set(), set(), False),
    ('OBS_MPC_NEW_REAL_6', 'NEW_OBS_MPC_GMO', 'mpc_esekf_20260709_174531', 'OBS_MPC_NEW_REAL_6.csv', 0, set(), set(), False),
    ('OBS_MPC_NEW_REAL_7', 'NEW_OBS_MPC_GMO', 'mpc_esekf_20260709_174735', 'OBS_MPC_NEW_REAL_7.csv', 0, set(), set(), False),
    ('OBS_MPC_OLD_REAL_1', 'OLD_OBS_MPC', 'mpc_legacy_20260709_175349', 'OBS_MPC_OLD_REAL_1.csv', 0, set(), set(), False),
    ('OBS_MPC_OLD_REAL_2', 'OLD_OBS_MPC', 'mpc_legacy_20260709_175700', 'OBS_MPC_OLD_REAL_2.csv', 0, set(), set(), False),
    ('OBS_MPC_OLD_REAL_3', 'OLD_OBS_MPC', 'mpc_legacy_20260709_175934', 'OBS_MPC_OLD_REAL_3.csv', 0, set(), set(), False),
    ('OBS_MPC_OLD_REAL_4', 'OLD_OBS_MPC', 'mpc_legacy_20260709_180142', 'OBS_MPC_OLD_REAL_4.csv', 0, set(), set(), False),
    ('OBS_MPC_OLD_REAL_5', 'OLD_OBS_MPC', 'mpc_legacy_20260709_180331', 'OBS_MPC_OLD_REAL_5.csv', 0, set(), set(), False),
]

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
BAGS_DIR  = os.path.join(BASE, 'bags')
VICON_DIR = os.path.join(BASE, 'vicon')
OUT_DIR   = os.path.join(BASE, 'results')
os.makedirs(OUT_DIR, exist_ok=True)

CONTACT_THRESHOLD_M = 0.015
GROUND_MARKERS = ['G1', 'G2', 'G3', 'G4']

# ─── Helpers ──────────────────────────────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)


def quat_to_rpy_deg(qw, qx, qy, qz):
    r = Rotation.from_quat(np.column_stack([qx, qy, qz, qw]))
    return np.degrees(r.as_euler('ZYX')[:, ::-1])


def rmse(d):
    v = np.asarray(d, dtype=float)
    v = v[~np.isnan(v)]
    return float(np.sqrt(np.mean(v ** 2))) if len(v) > 0 else float('nan')


def align_vicon_orientation(rpy_vicon, rpy_ekf_deg):
    """Align VICON initial orientation to EKF frame."""
    valid = ~np.isnan(rpy_vicon).any(1)
    if not valid.any():
        return rpy_vicon
    idx0 = int(np.argmax(valid))
    R_vi0  = Rotation.from_euler('ZYX', rpy_vicon[idx0, ::-1]).as_matrix()
    R_ekf0 = Rotation.from_euler('ZYX', np.radians(rpy_ekf_deg[0, ::-1])).as_matrix()
    R_align = R_ekf0 @ R_vi0.T
    out = rpy_vicon.copy()
    for i in np.where(valid)[0]:
        Rv = Rotation.from_euler('ZYX', rpy_vicon[i, ::-1]).as_matrix()
        out[i] = Rotation.from_matrix(R_align @ Rv).as_euler('ZYX')[::-1]
    return out


# ═══════════════════════════════════════════════════════════════════════════════
# NEW bag analysis (odom_fusion / mpc_esekf)
# ═══════════════════════════════════════════════════════════════════════════════
def analyze_new(exp_id, group, bag_name, vicon_csv, out_dir,
                trigger_pair=0, flip=None):
    """Full ESEKF analysis: EKF + odom_mapping + lidar.

    Parameters
    ----------
    trigger_pair : int
        Which trigger ON/OFF pair to use from the bag (0=first, 1=second).
    flip : set, optional
        Set of signal names to negate in the estimator output.
        Supported: 'px','py','vx','vy','roll','pitch'
    """
    if flip is None:
        flip = set()

    print(f'\n{"="*60}\n[NEW] {exp_id}  ({bag_name})\n{"="*60}')
    os.makedirs(out_dir, exist_ok=True)

    bag_db = os.path.join(BAGS_DIR, bag_name, f'{bag_name}_0.db3')
    csv    = os.path.join(VICON_DIR, vicon_csv)

    vi  = load_vicon(csv, contact_threshold_m=CONTACT_THRESHOLD_M,
                     ground_markers=GROUND_MARKERS)
    bag = load_fusion_bag(bag_db, rate=1.0, trigger_pair=trigger_pair)

    ekf   = bag['ekf']; ba = bag['ba']; bw = bag['bw']
    odom = bag['odom']; fv = bag['fv']; lidar = bag['lidar']

    # Both ROS and VICON are already expressed relative to their trigger-ON event.
    # Do not infer an additional offset from recording duration or trigger-OFF.
    _bag_t_end = bag['t_trigger_end']
    _vi_t_end  = vi.t_trigger_end
    if _bag_t_end is None and _vi_t_end is None:
        _bag_t_end = float(ekf['t'][-1]) if len(ekf['t']) > 0 else 30.0
    T_END = min(x for x in [_vi_t_end, _bag_t_end] if x is not None)
    print(f'Analysis window: t ∈ [0, {T_END:.2f}] s')

    # ── Apply signal flips (coordinate frame correction) ──────────────────────
    if flip:
        print(f'  [flip] Negating estimator signals: {flip}')
        if 'px' in flip:
            ekf['px'] = -ekf['px']; odom['px'] = -odom['px']
            lidar['px'] = -lidar['px']
        if 'py' in flip:
            ekf['py'] = -ekf['py']; odom['py'] = -odom['py']
            lidar['py'] = -lidar['py']
        if 'vx' in flip:
            ekf['vx'] = -ekf['vx']; fv['x'] = -fv['x']
        if 'vy' in flip:
            ekf['vy'] = -ekf['vy']; fv['y'] = -fv['y']
        # Basis change S=Rz(pi): R' = S R S^-1, equivalent to qx,qy sign flip.
        if 'roll' in flip or 'pitch' in flip:
            ekf['qx'] = -ekf['qx']; ekf['qy'] = -ekf['qy']

    mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
    t_win       = vi.t_traj[mask_win_vi]
    pos_vicon   = vi.pos_m[mask_win_vi]
    v_body_vi   = vi.v_body[mask_win_vi]
    rpy_vicon   = vi.rpy[mask_win_vi]

    ekf_mask = (ekf['t'] >= 0.0) & (ekf['t'] <= T_END)
    et   = ekf['t'][ekf_mask]
    epx  = ekf['px'][ekf_mask]; epy = ekf['py'][ekf_mask]; epz = ekf['pz'][ekf_mask]
    evx  = ekf['vx'][ekf_mask]; evy = ekf['vy'][ekf_mask]; evz = ekf['vz'][ekf_mask]
    eqw  = ekf['qw'][ekf_mask]; eqx = ekf['qx'][ekf_mask]
    eqy  = ekf['qy'][ekf_mask]; eqz = ekf['qz'][ekf_mask]
    ecov_px = ekf['cov_px'][ekf_mask]
    ecov_py = ekf['cov_py'][ekf_mask]
    rpy_ekf_deg = quat_to_rpy_deg(eqw, eqx, eqy, eqz)

    rpy_vicon = align_vicon_orientation(rpy_vicon, rpy_ekf_deg)

    vi_valid = ~np.isnan(pos_vicon).any(1)
    vi_t_v   = t_win[vi_valid]
    rpy_vi_valid = ~np.isnan(rpy_vicon).any(1)
    rpy_vi_t_v   = t_win[rpy_vi_valid]

    def vi2ekf(src): return interp_to(vi_t_v, src[vi_valid], et)
    vi_px_e = vi2ekf(pos_vicon[:, 0]); vi_py_e = vi2ekf(pos_vicon[:, 1])
    vi_pz_e = vi2ekf(pos_vicon[:, 2])
    vi_vx_e = vi2ekf(v_body_vi[:, 0]); vi_vy_e = vi2ekf(v_body_vi[:, 1])
    vi_vz_e = vi2ekf(v_body_vi[:, 2])
    vi_roll_e  = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 0]), et)
    vi_pitch_e = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 1]), et)
    vi_yaw_e   = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 2]), et)

    # ── T_{odom←camera_init} (Procrustes) ─────────────────────────────────────
    lx = interp_to(lidar['t'], lidar['px'], odom['t'])
    ly = interp_to(lidar['t'], lidar['py'], odom['t'])
    lz = interp_to(lidar['t'], lidar['pz'], odom['t'])
    vl = ~np.isnan(lx)
    if vl.sum() >= 3:
        lpts = np.column_stack([lx[vl], ly[vl], lz[vl]])
        opts = np.column_stack([odom['px'][vl], odom['py'][vl], odom['pz'][vl]])
        lc = lpts.mean(0); oc = opts.mean(0)
        H  = (lpts - lc).T @ (opts - oc)
        U, S, Vt2 = np.linalg.svd(H)
        R_CO = Vt2.T @ U.T
        if np.linalg.det(R_CO) < 0:
            Vt2[-1] *= -1; R_CO = Vt2.T @ U.T
        t_CO = oc - R_CO @ lc
        rpy_CO = np.degrees(Rotation.from_matrix(R_CO).as_euler('ZYX')[::-1])
        p_chk = (R_CO @ lpts.T).T + t_CO
        resid = np.linalg.norm(p_chk - opts, axis=1)
        lidar_xyz_odom = (R_CO @ np.column_stack([lidar['px'], lidar['py'], lidar['pz']]).T).T + t_CO
        lidar['px_odom'] = lidar_xyz_odom[:, 0]
        lidar['py_odom'] = lidar_xyz_odom[:, 1]
        lidar['pz_odom'] = lidar_xyz_odom[:, 2]
        lidar['pz_odom'] = lidar_xyz_odom[:, 2]
        T_CO_rpy = rpy_CO.tolist(); T_CO_t = t_CO.tolist()
        resid_mean = float(resid.mean() * 100); resid_max = float(resid.max() * 100)
    else:
        lidar['px_odom'] = lidar['px']; lidar['py_odom'] = lidar['py']
        lidar['pz_odom'] = lidar['pz']
        T_CO_rpy = [0, 0, 0]; T_CO_t = [0, 0, 0]
        resid_mean = resid_max = float('nan')

    # Contact analysis intentionally omitted for the 20260709 batch.

    # ── STEP 2: Inner EKF ─────────────────────────────────────────────────────
    z_offset = epz[0] - vi_pz_e[0] if not np.isnan(vi_pz_e[0]) else 0.0
    err_px = epx - vi_px_e; err_py = epy - vi_py_e; err_pz = epz - vi_pz_e - z_offset
    err_3d = np.sqrt(err_px**2 + err_py**2 + err_pz**2)
    valid_p = ~np.isnan(err_3d)

    metrics_pos = {
        'RMSE_X_cm':  rmse(err_px[valid_p]) * 100,
        'RMSE_Y_cm':  rmse(err_py[valid_p]) * 100,
        'RMSE_Z_cm':  rmse(err_pz[valid_p]) * 100,
        'RMSE_3D_cm': rmse(err_3d[valid_p]) * 100,
        'MAX_3D_cm':  float(np.nanmax(err_3d[valid_p])) * 100 if valid_p.any() else float('nan'),
        'final_EKF_x': float(epx[-1]), 'final_EKF_y': float(epy[-1]),
        'final_VICON_x': float(vi_px_e[valid_p][-1]) if valid_p.any() else float('nan'),
        'final_VICON_y': float(vi_py_e[valid_p][-1]) if valid_p.any() else float('nan'),
    }
    print(f'  EKF Pos RMSE: X={metrics_pos["RMSE_X_cm"]:.2f}cm '
          f'Y={metrics_pos["RMSE_Y_cm"]:.2f}cm 3D={metrics_pos["RMSE_3D_cm"]:.2f}cm')

    t_vel_s = T_END * 0.35; t_vel_e = T_END * 0.75
    vmask = (et >= t_vel_s) & (et <= t_vel_e)
    metrics_vel = {
        'RMSE_vx': rmse((evx - vi_vx_e)[vmask & ~np.isnan(vi_vx_e)]),
        'RMSE_vy': rmse((evy - vi_vy_e)[vmask & ~np.isnan(vi_vy_e)]),
        'RMSE_vz': rmse((evz - vi_vz_e)[vmask & ~np.isnan(vi_vz_e)]),
        'peak_vx': float(np.nanmax(np.abs(evx[vmask]))) if vmask.any() else float('nan'),
    }
    vel_err = np.column_stack([evx - vi_vx_e, evy - vi_vy_e, evz - vi_vz_e])
    valid_vel_3d = vmask & np.isfinite(vel_err).all(axis=1)
    metrics_vel['RMSE_3D'] = rmse(np.linalg.norm(vel_err[valid_vel_3d], axis=1))
    metrics_vel['window_start'] = float(t_vel_s)
    metrics_vel['window_end'] = float(t_vel_e)
    print(f'  EKF Vel RMSE vx={metrics_vel["RMSE_vx"]:.3f} vy={metrics_vel["RMSE_vy"]:.3f} m/s')

    valid_roll  = ~np.isnan(vi_roll_e)
    valid_pitch = ~np.isnan(vi_pitch_e)
    valid_yaw   = ~np.isnan(vi_yaw_e)
    metrics_rpy = {
        'RMSE_roll_deg':  rmse((rpy_ekf_deg[:, 0] - vi_roll_e)[valid_roll]),
        'RMSE_pitch_deg': rmse((rpy_ekf_deg[:, 1] - vi_pitch_e)[valid_pitch]),
        'RMSE_yaw_deg':   rmse((rpy_ekf_deg[:, 2] - vi_yaw_e)[valid_yaw]),
    }

    # Bias metrics (may be empty for mpc_esekf)
    def _bias_stats(b):
        if len(b['x']) == 0:
            return {'x': [float('nan')] * 3, 'y': [float('nan')] * 3, 'z': [float('nan')] * 3}
        n = min(50, len(b['x'])); ss = min(200, len(b['x']))
        return {
            'x': [float(b['x'][:n].mean()), float(b['x'][-ss:].mean()), float(b['x'][-ss:].std())],
            'y': [float(b['y'][:n].mean()), float(b['y'][-ss:].mean()), float(b['y'][-ss:].std())],
            'z': [float(b['z'][:n].mean()), float(b['z'][-ss:].mean()), float(b['z'][-ss:].std())],
        }
    metrics_ba = _bias_stats(ba)
    metrics_bw = _bias_stats(bw)

    # EKF position plot
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axes[0].plot(et, epx, lw=1.0, label='EKF px'); axes[0].plot(vi_t_v, pos_vicon[vi_valid, 0], lw=1.0, label='VICON px')
    axes[0].fill_between(et, epx - 3*np.sqrt(np.abs(ecov_px)), epx + 3*np.sqrt(np.abs(ecov_px)), alpha=0.15)
    axes[1].plot(et, epy, lw=1.0, label='EKF py'); axes[1].plot(vi_t_v, pos_vicon[vi_valid, 1], lw=1.0, label='VICON py')
    axes[1].fill_between(et, epy - 3*np.sqrt(np.abs(ecov_py)), epy + 3*np.sqrt(np.abs(ecov_py)), alpha=0.15)
    axes[2].plot(et, err_3d * 100, lw=0.8, color='red', label='3D error [cm]')
    for ax in axes:
        ax.axvspan(0, T_END, color='gold', alpha=0.06)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    axes[0].set_ylabel('X [m]'); axes[1].set_ylabel('Y [m]'); axes[2].set_ylabel('Error [cm]')
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'Inner EKF Position — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_pos.pdf'), dpi=150); plt.close(fig)

    # EKF velocity plot
    fig, axes = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
    for ax, (ev, vv, lbl) in zip(axes, [(evx, vi_vx_e, 'vx'), (evy, vi_vy_e, 'vy'), (evz, vi_vz_e, 'vz')]):
        ax.plot(et, ev, lw=1.0, label=f'EKF {lbl}')
        ax.plot(et[~np.isnan(vv)], vv[~np.isnan(vv)], lw=1.0, label=f'VICON {lbl}')
        ax.axvspan(0, T_END, color='gold', alpha=0.06)
        ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'Inner EKF Velocity — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_vel.pdf'), dpi=150); plt.close(fig)

    # EKF RPY plot
    fig, axes = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
    for ax, (ekf_a, vi_a, lbl) in zip(axes, [
        (rpy_ekf_deg[:, 0], vi_roll_e,  'roll'),
        (rpy_ekf_deg[:, 1], vi_pitch_e, 'pitch'),
        (rpy_ekf_deg[:, 2], vi_yaw_e,   'yaw'),
    ]):
        ax.plot(et, ekf_a, lw=1.0, label=f'EKF {lbl}')
        valid_a = ~np.isnan(vi_a)
        ax.plot(et[valid_a], vi_a[valid_a], lw=1.0, label=f'VICON {lbl}')
        ax.axvspan(0, T_END, color='gold', alpha=0.06)
        ax.set_ylabel(f'{lbl} [°]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'Inner EKF Attitude — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_rpy.pdf'), dpi=150); plt.close(fig)

    # ── STEP 3: Outer Fusion (odom_mapping) ───────────────────────────────────
    odom_mask = (odom['t'] >= 0.0) & (odom['t'] <= T_END)
    ot = odom['t'][odom_mask]
    opx = odom['px'][odom_mask]; opy = odom['py'][odom_mask]

    vi_opx = interp_to(vi_t_v, pos_vicon[vi_valid, 0], ot)
    vi_opy = interp_to(vi_t_v, pos_vicon[vi_valid, 1], ot)
    err_ox = opx - vi_opx; err_oy = opy - vi_opy
    err_o2d = np.sqrt(err_ox**2 + err_oy**2)
    valid_o = ~np.isnan(err_o2d)

    metrics_odom = {
        'RMSE_X_cm':  rmse(err_ox[valid_o]) * 100,
        'RMSE_Y_cm':  rmse(err_oy[valid_o]) * 100,
        'RMSE_2D_cm': rmse(err_o2d[valid_o]) * 100,
        'MAX_2D_cm':  float(np.nanmax(err_o2d[valid_o])) * 100 if valid_o.any() else float('nan'),
    }
    print(f'  Odom RMSE: X={metrics_odom["RMSE_X_cm"]:.2f}cm '
          f'Y={metrics_odom["RMSE_Y_cm"]:.2f}cm 2D={metrics_odom["RMSE_2D_cm"]:.2f}cm')

    fv_mask = (fv['t'] >= 0.0) & (fv['t'] <= T_END)
    ft = fv['t'][fv_mask]
    fvx = fv['x'][fv_mask]; fvy = fv['y'][fv_mask]
    # /fusion/bv is published in odom/world frame.  Rotate VICON body velocity
    # back to the robot-centric world frame before comparison.
    vi_rot_valid = (~np.isnan(rpy_vicon).any(1) &
                    ~np.isnan(v_body_vi).any(1))
    vi_world_vel = np.full_like(v_body_vi, np.nan)
    vi_world_vel[vi_rot_valid] = Rotation.from_euler(
        'ZYX', rpy_vicon[vi_rot_valid, ::-1]).apply(v_body_vi[vi_rot_valid])
    vi_fvx = interp_to(t_win[vi_rot_valid], vi_world_vel[vi_rot_valid, 0], ft)
    vi_fvy = interp_to(t_win[vi_rot_valid], vi_world_vel[vi_rot_valid, 1], ft)
    vi_fvz = interp_to(t_win[vi_rot_valid], vi_world_vel[vi_rot_valid, 2], ft)
    fvz = fv['z'][fv_mask]
    fv_err = np.column_stack([fvx - vi_fvx, fvy - vi_fvy, fvz - vi_fvz])
    valid_fv_3d = np.isfinite(fv_err).all(axis=1)
    metrics_fv = {
        'RMSE_vx': rmse((fvx - vi_fvx)[~np.isnan(vi_fvx)]),
        'RMSE_vy': rmse((fvy - vi_fvy)[~np.isnan(vi_fvy)]),
        'RMSE_vz': rmse((fvz - vi_fvz)[~np.isnan(vi_fvz)]),
        'RMSE_3D': rmse(np.linalg.norm(fv_err[valid_fv_3d], axis=1)),
        'frame': 'odom',
    }

    # XZ trajectory plot  (VICON Z shifted by z_offset to align to estimator frame)
    vi_z_plot = pos_vicon[vi_valid, 2] + z_offset
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(pos_vicon[vi_valid, 0], vi_z_plot, lw=1.5, label='VICON', color='black')
    ax.plot(epx, epz, lw=1.0, label='EKF', color='steelblue', alpha=0.8)
    ax.plot(opx, odom['pz'][odom_mask], lw=1.0, label='odom_mapping', color='darkorange', alpha=0.8)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Z [m]')
    ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title(f'XZ Trajectory — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_xz.pdf'), dpi=150); plt.close(fig)

    # LiDAR XZ — transformed XYZ and trigger-gated experiment window only.
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(pos_vicon[vi_valid, 0], vi_z_plot, lw=1.5, label='VICON', color='black')
    lidar_mask = (lidar['t'] >= 0.0) & (lidar['t'] <= T_END)
    ax.plot(lidar['px_odom'][lidar_mask], lidar['pz_odom'][lidar_mask],
            lw=0.8, label='lidar (odom frame)', color='purple', alpha=0.7)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Z [m]')
    ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title(f'LiDAR XZ Trajectory — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_lidar_xz.pdf'), dpi=150); plt.close(fig)

    # ── STEP 4: LiDAR quality ─────────────────────────────────────────────────
    lt = lidar['t'][lidar_mask]
    metrics_lidar = {
        'n_msgs': int(lidar_mask.sum()),
        'rate_hz': float(lidar_mask.sum() / T_END) if T_END > 0 else float('nan'),
        'resid_mean_cm': resid_mean,
        'resid_max_cm':  resid_max,
    }

    # ── Save metrics ───────────────────────────────────────────────────────────
    metrics = {
        'exp_id': exp_id, 'group': group, 'bag': bag_name, 'T_END': T_END,
        'data_start': float(et[0]) if len(et) else float('nan'),
        'data_end': float(et[-1]) if len(et) else float('nan'),
        'position': metrics_pos,
        'velocity': metrics_vel,
        'attitude': metrics_rpy,
        'ba': metrics_ba, 'bw': metrics_bw,
        'odom_pos': metrics_odom,
        'fusion_bv': metrics_fv,
        'lidar': metrics_lidar,
        'T_CO': {'t_m': T_CO_t, 'RPY_deg': T_CO_rpy,
                 'resid_mean_cm': resid_mean, 'resid_max_cm': resid_max},
    }
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f'  → Saved metrics.json & figures to {out_dir}')
    return metrics


# ═══════════════════════════════════════════════════════════════════════════════
# OLD bag analysis (legacy_odom / mpc_legacy)
# ═══════════════════════════════════════════════════════════════════════════════
def analyze_old(exp_id, group, bag_name, vicon_csv, out_dir, flip=None):
    """Legacy odometry analysis: position/velocity vs VICON.

    Parameters
    ----------
    flip : set, optional
        Set of signal names to negate. Supported: 'px','py','vx','vy'
    """
    if flip is None:
        flip = set()
    print(f'\n{"="*60}\n[OLD] {exp_id}  ({bag_name})\n{"="*60}')
    os.makedirs(out_dir, exist_ok=True)

    bag_db = os.path.join(BAGS_DIR, bag_name, f'{bag_name}_0.db3')
    csv    = os.path.join(VICON_DIR, vicon_csv)

    vi  = load_vicon(csv, contact_threshold_m=CONTACT_THRESHOLD_M,
                     ground_markers=GROUND_MARKERS)
    bag = load_legacy_bag(bag_db, rate=1.0)

    pos = bag['pos']; vel = bag['vel']
    _bag_t_end = bag['t_trigger_end']
    _vi_t_end  = vi.t_trigger_end
    if _bag_t_end is None and _vi_t_end is None:
        _bag_t_end = float(pos['t'][-1]) if len(pos['t']) > 0 else 30.0
    T_END = min(x for x in [_vi_t_end, _bag_t_end] if x is not None)
    print(f'Analysis window: t ∈ [0, {T_END:.2f}] s')

    mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
    t_win     = vi.t_traj[mask_win_vi]
    pos_vicon = vi.pos_m[mask_win_vi]
    v_body_vi = vi.v_body[mask_win_vi]
    vi_valid  = ~np.isnan(pos_vicon).any(1)
    vi_t_v    = t_win[vi_valid]

    pos_mask = (pos['t'] >= 0.0) & (pos['t'] <= T_END)
    pt = pos['t'][pos_mask]
    px = pos['x'][pos_mask]; py = pos['y'][pos_mask]; pz = pos['z'][pos_mask]

    vel_mask = (vel['t'] >= 0.0) & (vel['t'] <= T_END)
    vt = vel['t'][vel_mask]
    vx = vel['x'][vel_mask]; vy = vel['y'][vel_mask]; vz = vel['z'][vel_mask]

    # ── Apply signal flips ────────────────────────────────────────────────────
    if flip:
        print(f'  [flip] Negating legacy signals: {flip}')
        if 'px' in flip: px = -px
        if 'py' in flip: py = -py
        if 'pz' in flip: pz = -pz
        if 'vx' in flip: vx = -vx
        if 'vy' in flip: vy = -vy
        if 'vz' in flip: vz = -vz

    vi_px = interp_to(vi_t_v, pos_vicon[vi_valid, 0], pt)
    vi_py = interp_to(vi_t_v, pos_vicon[vi_valid, 1], pt)
    vi_pz = interp_to(vi_t_v, pos_vicon[vi_valid, 2], pt)
    vi_vx = interp_to(vi_t_v, v_body_vi[vi_valid, 0], vt)
    vi_vy = interp_to(vi_t_v, v_body_vi[vi_valid, 1], vt)
    vi_vz = interp_to(vi_t_v, v_body_vi[vi_valid, 2], vt)

    # Align initial position
    valid_px = ~np.isnan(vi_px)
    if valid_px.any():
        offset_x = px[valid_px][0] - vi_px[valid_px][0]
        offset_y = py[valid_px][0] - vi_py[valid_px][0]
        offset_z = pz[valid_px][0] - vi_pz[valid_px][0]
    else:
        offset_x = offset_y = offset_z = 0.0

    err_px = px - vi_px - offset_x
    err_py = py - vi_py - offset_y
    err_pz = pz - vi_pz - offset_z
    pos_err = np.column_stack([err_px, err_py, err_pz])
    err_2d = np.linalg.norm(pos_err[:, :2], axis=1)
    err_3d = np.linalg.norm(pos_err, axis=1)
    valid_p = np.isfinite(pos_err).all(axis=1)

    metrics_pos = {
        'RMSE_X_cm':  rmse(err_px[valid_p]) * 100,
        'RMSE_Y_cm':  rmse(err_py[valid_p]) * 100,
        'RMSE_Z_cm':  rmse(err_pz[valid_p]) * 100,
        'RMSE_2D_cm': rmse(err_2d[valid_p]) * 100,
        'RMSE_3D_cm': rmse(err_3d[valid_p]) * 100,
        'MAX_2D_cm':  float(np.nanmax(err_2d[valid_p])) * 100 if valid_p.any() else float('nan'),
        'MAX_3D_cm':  float(np.nanmax(err_3d[valid_p])) * 100 if valid_p.any() else float('nan'),
        'final_leg_x': float(px[-1]) if len(px) > 0 else float('nan'),
        'final_leg_y': float(py[-1]) if len(py) > 0 else float('nan'),
        'final_VICON_x': float(vi_px[valid_p][-1]) if valid_p.any() else float('nan'),
        'final_VICON_y': float(vi_py[valid_p][-1]) if valid_p.any() else float('nan'),
    }
    print(f'  Legacy Pos RMSE: X={metrics_pos["RMSE_X_cm"]:.2f}cm '
          f'Y={metrics_pos["RMSE_Y_cm"]:.2f}cm 2D={metrics_pos["RMSE_2D_cm"]:.2f}cm')

    t_vel_s = T_END * 0.35; t_vel_e = T_END * 0.75
    vel_window = (vt >= t_vel_s) & (vt <= t_vel_e)
    vel_err = np.column_stack([vx - vi_vx, vy - vi_vy, vz - vi_vz])
    valid_vel = vel_window & np.isfinite(vel_err).all(axis=1)
    metrics_vel = {
        'RMSE_vx': rmse(vel_err[valid_vel, 0]),
        'RMSE_vy': rmse(vel_err[valid_vel, 1]),
        'RMSE_vz': rmse(vel_err[valid_vel, 2]),
        'RMSE_3D': rmse(np.linalg.norm(vel_err[valid_vel], axis=1)),
        'window_start': float(t_vel_s),
        'window_end': float(t_vel_e),
    }
    print(f'  Legacy Vel RMSE vx={metrics_vel["RMSE_vx"]:.3f} vy={metrics_vel["RMSE_vy"]:.3f} m/s')

    # XZ trajectory plot  (VICON Z shifted by offset_z to align to estimator frame)
    vi_z_plot_old = pos_vicon[vi_valid, 2] + offset_z
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(pos_vicon[vi_valid, 0], vi_z_plot_old, lw=1.5, label='VICON', color='black')
    # legacy pz is relative; X/Y aligned, Z raw (offset_z already applied to VICON)
    ax.plot(px - offset_x, pz, lw=1.0, label='Legacy Odom', color='darkorange', alpha=0.8)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Z [m]')
    ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title(f'XZ Trajectory — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_legacy_xz.pdf'), dpi=150); plt.close(fig)

    # Velocity plot
    fig, axes = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
    axes[0].plot(vt, vx, lw=1.0, label='Legacy vx')
    axes[0].plot(vt[~np.isnan(vi_vx)], vi_vx[~np.isnan(vi_vx)], lw=1.0, label='VICON vx')
    axes[1].plot(vt, vy, lw=1.0, label='Legacy vy')
    axes[1].plot(vt[~np.isnan(vi_vy)], vi_vy[~np.isnan(vi_vy)], lw=1.0, label='VICON vy')
    for ax in axes:
        ax.axvspan(0, T_END, color='gold', alpha=0.06)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    axes[0].set_ylabel('vx [m/s]'); axes[1].set_ylabel('vy [m/s]')
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'Legacy Odometry Velocity — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_legacy_vel.pdf'), dpi=150); plt.close(fig)

    metrics = {
        'exp_id': exp_id, 'group': group, 'bag': bag_name, 'T_END': T_END,
        'data_start': float(pt[0]) if len(pt) else float('nan'),
        'data_end': float(pt[-1]) if len(pt) else float('nan'),
        'exclude_stats': False,
        'position': metrics_pos,
        'velocity': metrics_vel,
    }
    with open(os.path.join(out_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f'  → Saved metrics.json & figures to {out_dir}')
    return metrics


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════
def write_20260709_report(all_metrics):
    """Write the 20260709 report using the 20260528 report structure."""
    included = [m for m in all_metrics if not m.get('exclude_stats', False)]

    group_defs = [
        ('NEW_RUGG_WALK', 'RUGG Walk（崎嶇地面步行）', 'ESEKF + fusion'),
        ('OLD_RUGG_WALK', 'RUGG Walk（崎嶇地面步行）', 'Legacy'),
        ('NEW_OBS_MPC_GMO', 'Obstacle MPC（障礙地形）', 'ESEKF + fusion'),
        ('OLD_OBS_MPC', 'Obstacle MPC（障礙地形）', 'Legacy'),
    ]

    def rows(group):
        return [m for m in included if m['group'] == group]

    def value(m, section, key):
        return float(m.get(section, {}).get(key, np.nan))

    def mean_std(values):
        values = np.asarray(values, dtype=float)
        values = values[np.isfinite(values)]
        if not len(values):
            return np.nan, np.nan
        return float(values.mean()), float(values.std(ddof=1) if len(values) > 1 else 0.0)

    def ms(group, section, key):
        return mean_std([value(m, section, key) for m in rows(group)])

    def fmt(v, digits=2):
        return 'N/A' if not np.isfinite(v) else f'{v:.{digits}f}'

    lines = [
        '# CORGI 實驗分析報告 — 20260709', '',
        '**日期：** 2026-07-09',
        '**實驗地形：** Rugged / obstacle terrain（崎嶇與障礙地形）',
        f'**有效實驗數：** {len(included)}',
        '**分析腳本：** `analyze.py`', '', '---', '',
        '## 實驗架構', '', '```',
        '/imu_raw, /motor/state ──► corgi_leg_odom ──► Inner EKF (/ekf)',
        '                                                      │',
        '/lidar_odom (FAST-LIO2) ──────────────► corgi_fusion_node ──► /odom_mapping',
        '                                                              /fusion/bv',
        '',
        'Legacy system: /odometry/legacy/position, /odometry/legacy/velocity',
        '```', '',
        '**實驗分組：**', '',
        '| 分組代碼 | 模式 | 里程計系統 | 試驗數 |',
        '|----------|------|-----------|--------|',
    ]
    for group, label, system in group_defs:
        lines.append(f'| {group} | {label} | {system} | {len(rows(group))} |')

    lines += ['', '---', '', '## 1. 每次試驗結果', '',
              '位置誤差單位為 cm；速度誤差單位為 m/s。位置使用 VICON 與估測器的有效重疊區間；速度使用 `35%–75% T_END` 穩態窗。',
              '',
              '| 實驗編號 | 分組 | 有效資料 (s) | 位置 X | 位置 Y | 位置 Z | 位置 3D | 速度 X | 速度 Y | 速度 Z | 速度 3D |',
              '|----------|------|--------------|--------|--------|--------|---------|--------|--------|--------|---------|']
    for m in included:
        p, v = m['position'], m['velocity']
        lines.append(
            f'| {m["exp_id"]} | {m["group"]} | {fmt(m.get("data_start", 0), 1)}–{fmt(m.get("data_end", m["T_END"]), 1)} '
            f'| {fmt(p.get("RMSE_X_cm", np.nan))} | {fmt(p.get("RMSE_Y_cm", np.nan))} '
            f'| {fmt(p.get("RMSE_Z_cm", np.nan))} | {fmt(p.get("RMSE_3D_cm", np.nan))} '
            f'| {fmt(v.get("RMSE_vx", np.nan), 3)} | {fmt(v.get("RMSE_vy", np.nan), 3)} '
            f'| {fmt(v.get("RMSE_vz", np.nan), 3)} | {fmt(v.get("RMSE_3D", np.nan), 3)} |'
        )

    lines += ['', '---', '', '## 2. 分組統計（平均 ± 樣本標準差）', '',
              '| 模式 | 系統 | n | Pos X (cm) | Pos Y (cm) | Pos Z (cm) | Pos 3D (cm) | Vel X (m/s) | Vel Y (m/s) | Vel Z (m/s) | Vel 3D (m/s) |',
              '|------|------|---|------------|------------|------------|-------------|-------------|-------------|-------------|--------------|']
    for group, label, system in group_defs:
        vals = [ms(group, 'position', k) for k in ('RMSE_X_cm','RMSE_Y_cm','RMSE_Z_cm','RMSE_3D_cm')]
        vals += [ms(group, 'velocity', k) for k in ('RMSE_vx','RMSE_vy','RMSE_vz','RMSE_3D')]
        cells = [f'{fmt(a, 3)} ± {fmt(b, 3)}' for a, b in vals]
        lines.append(f'| {label} | {system} | {len(rows(group))} | ' + ' | '.join(cells) + ' |')

    lines += ['', '---', '', '## 3. NEW vs OLD 比較（位置與速度 3D RMSE）', '',
              '### 3.1 位置 3D RMSE', '',
              '| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |',
              '|------|------------|-------------|----------|']
    pairs = [
        ('RUGG Walk', 'NEW_RUGG_WALK', 'OLD_RUGG_WALK'),
        ('Obstacle MPC', 'NEW_OBS_MPC_GMO', 'OLD_OBS_MPC'),
    ]
    for label, ng, og in pairs:
        nm, ns = ms(ng, 'position', 'RMSE_3D_cm'); om, osd = ms(og, 'position', 'RMSE_3D_cm')
        imp = (om - nm) / om * 100
        lines.append(f'| {label} | {nm:.2f} ± {ns:.2f} | {om:.2f} ± {osd:.2f} | {imp:+.1f}% |')
    lines += ['', '### 3.2 速度 3D RMSE', '',
              '| 模式 | ESEKF (m/s) | Legacy (m/s) | 改善幅度 |',
              '|------|-------------|--------------|----------|']
    for label, ng, og in pairs:
        nm, ns = ms(ng, 'velocity', 'RMSE_3D'); om, osd = ms(og, 'velocity', 'RMSE_3D')
        imp = (om - nm) / om * 100
        lines.append(f'| {label} | {nm:.3f} ± {ns:.3f} | {om:.3f} ± {osd:.3f} | {imp:+.1f}% |')
    lines += ['', '> 改善幅度為 `(Legacy − ESEKF) / Legacy × 100%`。', '', '---', '',
              '## 4. ESEKF 系統詳細指標', '', '### 4.1 姿態估計（Inner EKF RPY RMSE）', '',
              '| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |',
              '|----------|----------|-----------|---------|']
    for m in included:
        if 'attitude' in m:
            a = m['attitude']
            lines.append(f'| {m["exp_id"]} | {fmt(a["RMSE_roll_deg"],3)} | {fmt(a["RMSE_pitch_deg"],3)} | {fmt(a["RMSE_yaw_deg"],3)} |')

    lines += ['', '### 4.2 odom_mapping 位置 RMSE', '',
              '| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |',
              '|----------|-------------|-------------|--------------|']
    for m in included:
        if 'odom_pos' in m:
            p = m['odom_pos']
            lines.append(f'| {m["exp_id"]} | {fmt(p["RMSE_X_cm"])} | {fmt(p["RMSE_Y_cm"])} | {fmt(p["RMSE_2D_cm"])} |')

    lines += ['', '### 4.3 LiDAR 輸入品質', '',
              '| 實驗編號 | Rate (Hz) | 配準 residual mean (cm) | residual max (cm) |',
              '|----------|-----------|-------------------------|-------------------|']
    for m in included:
        if 'lidar' in m:
            l = m['lidar']
            lines.append(f'| {m["exp_id"]} | {fmt(l["rate_hz"],2)} | {fmt(l["resid_mean_cm"],2)} | {fmt(l["resid_max_cm"],2)} |')

    mpc = rows('NEW_OBS_MPC_GMO') + rows('OLD_OBS_MPC')
    lines += ['', '---', '', '## 5. MPC 終點 X 位置分析（目標：3.0 m）', '',
              'MPC 控制器以 X = 3 m 為目標停止。估測器 final X 是控制器的停止依據，VICON final X 是實際停止位置。',
              '',
              '| 實驗編號 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | 停止誤差 VICON (cm) |',
              '|----------|-------------------|-------------------|--------------|--------------------|']
    endpoint = {}
    for m in mpc:
        p = m['position']
        est = p.get('final_EKF_x', p.get('final_leg_x', np.nan))
        vic = p.get('final_VICON_x', np.nan)
        endpoint[m['exp_id']] = (est, vic)
        lines.append(f'| {m["exp_id"]} | {fmt(est,3)} | {fmt(vic,3)} | {(est-3)*100:+.1f} | {(vic-3)*100:+.1f} |')

    lines += ['', '**統計摘要（目標 X = 3.0 m）**', '',
              '| 系統 | n | 估測器 final X | 實際 VICON final X | VICON 停止誤差 (abs mean) |',
              '|------|---|------------------|---------------------|-----------------------------|']
    for group, system in [('NEW_OBS_MPC_GMO','ESEKF (NEW)'), ('OLD_OBS_MPC','Legacy (OLD)')]:
        es = [endpoint[m['exp_id']][0] for m in rows(group)]
        vs = [endpoint[m['exp_id']][1] for m in rows(group)]
        em, esd = mean_std(es); vm, vsd = mean_std(vs)
        stop = np.mean(np.abs(np.asarray(vs)-3))*100
        lines.append(f'| {system} | {len(es)} | {em:.3f} ± {esd:.3f} m | {vm:.3f} ± {vsd:.3f} m | {stop:.1f} cm |')

    lines += ['', '---', '', '## 7. Closed-Loop（MPC）vs Open-Loop（Walk）比較', '',
              '本節比較 ESEKF 的 Closed-Loop Obstacle MPC 與 Open-Loop RUGG Walk。姿態數值是 EKF 相對 VICON 的估測 RMSE，不是機體相對水平面的實際震盪量。',
              '',
              '| 指標 | Closed-Loop（MPC） | Open-Loop（RUGG Walk） |',
              '|------|-------------------|------------------------|']
    closed, opened = 'NEW_OBS_MPC_GMO', 'NEW_RUGG_WALK'
    comparisons = [
        ('n（試驗數）', str(len(rows(closed))), str(len(rows(opened)))),
        ('Position 3D RMSE (cm)', *[f'{ms(g,"position","RMSE_3D_cm")[0]:.2f} ± {ms(g,"position","RMSE_3D_cm")[1]:.2f}' for g in (closed,opened)]),
        ('Velocity 3D RMSE (m/s)', *[f'{ms(g,"velocity","RMSE_3D")[0]:.3f} ± {ms(g,"velocity","RMSE_3D")[1]:.3f}' for g in (closed,opened)]),
        ('Roll estimation RMSE (°)', *[f'{ms(g,"attitude","RMSE_roll_deg")[0]:.2f} ± {ms(g,"attitude","RMSE_roll_deg")[1]:.2f}' for g in (closed,opened)]),
        ('Pitch estimation RMSE (°)', *[f'{ms(g,"attitude","RMSE_pitch_deg")[0]:.2f} ± {ms(g,"attitude","RMSE_pitch_deg")[1]:.2f}' for g in (closed,opened)]),
        ('Yaw estimation RMSE (°)', *[f'{ms(g,"attitude","RMSE_yaw_deg")[0]:.2f} ± {ms(g,"attitude","RMSE_yaw_deg")[1]:.2f}' for g in (closed,opened)]),
        ('peak vx EKF（35–75% T_END，m/s）', *[f'{ms(g,"velocity","peak_vx")[0]:.3f} ± {ms(g,"velocity","peak_vx")[1]:.3f}' for g in (closed,opened)]),
    ]
    for label, cv, ov in comparisons:
        lines.append(f'| {label} | {cv} | {ov} |')

    rugg_new = ms('NEW_RUGG_WALK','position','RMSE_3D_cm')[0]
    rugg_old = ms('OLD_RUGG_WALK','position','RMSE_3D_cm')[0]
    mpc_new = ms('NEW_OBS_MPC_GMO','position','RMSE_3D_cm')[0]
    mpc_old = ms('OLD_OBS_MPC','position','RMSE_3D_cm')[0]
    lines += ['', '### 7.1 分析', '',
              f'- Closed-Loop MPC 的位置 3D RMSE 為 {mpc_new:.2f} cm；Open-Loop RUGG Walk 為 {rugg_new:.2f} cm。',
              '- 兩組地形與任務條件不同，因此本比較用於描述系統行為，不應解讀為單一控制器因素的因果效果。',
              '- peak vx 可反映步態中的瞬時速度振盪；姿態 RMSE 則反映估測器追蹤 VICON 的一致性。', '',
              '---', '', '## 8. 觀察與結論', '',
              '### 崎嶇地面步行（RUGG Walk）', '',
              f'- ESEKF 位置 3D RMSE 為 {rugg_new:.2f} cm；Legacy 為 {rugg_old:.2f} cm。',
              '- 修正後 LiDAR XZ 高度與 VICON 地形起伏一致，先前的大幅 Z 漂移來自未轉換的 LiDAR Z 軸與錯誤時間窗。', '',
              '### 障礙地形 MPC', '',
              f'- ESEKF 位置 3D RMSE 為 {mpc_new:.2f} cm；Legacy 為 {mpc_old:.2f} cm。',
              '- MPC 終點表直接呈現估測器停止依據與 VICON 實際停止位置，可用來判斷里程計累積誤差是否導致提前停止。', '',
              '### 整體結論', '',
              '- 20260709 有效資料中，ESEKF + LiDAR fusion 的位置誤差低於 Legacy。',
              '- 所有統計均來自 position、velocity、attitude、outer fusion 與 LiDAR 品質。', '',
              '---', '', '*報告由 `analyze.py --report-only` 從各試驗 metrics 重建。更新日期：2026-07-12*', '']

    report_path = os.path.join(OUT_DIR, 'analysis_report.md')
    with open(report_path, 'w') as fp:
        fp.write('\n'.join(lines))
    print(f'[Summary] → {report_path}')

def write_20260709_comparison(all_metrics):
    included = [m for m in all_metrics if not m.get('exclude_stats', False)]
    groups = list(dict.fromkeys(m['group'] for m in included))
    pos_means = []
    vel_means = []
    for group in groups:
        rows = [m for m in included if m['group'] == group]
        pos_means.append(np.nanmean([
            m['position'].get('RMSE_3D_cm', m['position'].get('RMSE_2D_cm'))
            for m in rows]))
        vel_means.append(np.nanmean([m['velocity']['RMSE_3D'] for m in rows]))
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    x = np.arange(len(groups))
    axes[0].bar(x, pos_means, color='steelblue')
    axes[1].bar(x, vel_means, color='darkorange')
    for ax, labels, title in [
        (axes[0], pos_means, 'Position RMSE [cm]'),
        (axes[1], vel_means, 'Velocity RMSE [m/s]'),
    ]:
        ax.set_xticks(x); ax.set_xticklabels(groups, rotation=25, ha='right')
        ax.set_title(title); ax.grid(True, axis='y', alpha=0.3)
        for i, value in enumerate(labels):
            ax.text(i, value, f'{value:.2f}' if ax is axes[0] else f'{value:.3f}',
                    ha='center', va='bottom', fontsize=8)
    fig.suptitle('20260709 CORGI experiments')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, 'fig_comparison.pdf'), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp', default=None,
                        help='Run only this experiment ID (e.g. FLAT_Walk_NEW_REAL_1)')
    parser.add_argument('--report-only', action='store_true',
                        help='Rebuild report/comparison from existing metrics only')
    args = parser.parse_args()

    if args.report_only:
        all_metrics = []
        for exp_id, _, _, _, _, _, _, excl in EXPERIMENTS:
            metrics_path = os.path.join(OUT_DIR, exp_id, 'metrics.json')
            with open(metrics_path) as fp:
                metrics = json.load(fp)
            metrics['exclude_stats'] = excl
            with open(metrics_path, 'w') as fp:
                json.dump(metrics, fp, indent=2)
            all_metrics.append(metrics)
        write_20260709_report(all_metrics)
        write_20260709_comparison(all_metrics)
        return

    all_metrics = []
    failed = []

    for exp_id, group, bag_name, vicon_csv, trigger_pair, flip_new, flip_old, excl in EXPERIMENTS:
        if args.exp and exp_id != args.exp:
            continue
        out_dir = os.path.join(OUT_DIR, exp_id)
        try:
            if 'NEW' in group:
                m = analyze_new(exp_id, group, bag_name, vicon_csv, out_dir,
                                trigger_pair=trigger_pair, flip=flip_new)
            else:
                m = analyze_old(exp_id, group, bag_name, vicon_csv, out_dir,
                                flip=flip_old)
            m['exclude_stats'] = excl
            # Persist the manifest exclusion flag in the per-trial artifact.
            with open(os.path.join(out_dir, 'metrics.json'), 'w') as fp:
                json.dump(m, fp, indent=2)
            all_metrics.append(m)
        except Exception as e:
            import traceback
            print(f'\n[ERROR] {exp_id}: {e}')
            traceback.print_exc()
            failed.append((exp_id, str(e)))

    if all_metrics and not args.exp:
        write_20260709_report(all_metrics)
        write_20260709_comparison(all_metrics)

    if failed:
        print(f'\n{"="*60}\nFailed experiments:')
        for eid, err in failed:
            print(f'  {eid}: {err}')
    print(f'\nDone. {len(all_metrics)} experiments processed, {len(failed)} failed.')


if __name__ == '__main__':
    main()
