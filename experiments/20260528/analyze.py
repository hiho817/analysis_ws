#!/usr/bin/env python3
"""
CORGI Batch Analysis — 20260528
處理 FLAT_Walk / FLAT_WLW / FLAT_MPC 三種模式，各分 NEW (ESEKF) 和 OLD (legacy)。

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
    # ── FLAT Walk NEW (ESEKF fusion) ──────────────────────────────────────────
    ('FLAT_Walk_NEW_REAL_1', 'NEW_WALK', 'odom_fusion20260528_150739', 'FLAT_WALK_NEW_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_Walk_NEW_REAL_2', 'NEW_WALK', 'odom_fusion20260528_151135', 'FLAT_WALK_NEW_REAL_2.csv', 0, set(), set(), True),   # 排除統計
    ('FLAT_Walk_NEW_REAL_3', 'NEW_WALK', 'odom_fusion20260528_151411_replay', 'FLAT_WALK_NEW_REAL_3.csv', 1, set(), set(), False),  # 兩段 trigger 用第二段
    ('FLAT_Walk_NEW_REAL_4', 'NEW_WALK', 'odom_fusion20260528_151745_replay', 'FLAT_WALK_NEW_REAL_4.csv', 0, set(), set(), False),
    ('FLAT_Walk_NEW_REAL_5', 'NEW_WALK', 'odom_fusion20260528_152035_replay', 'FLAT_WALK_NEW_REAL_5.csv', 0, set(), set(), False),
    ('FLAT_Walk_NEW_REAL_6', 'NEW_WALK', 'odom_fusion20260528_153138_replay', 'FLAT_WALK_NEW_REAL_6.csv', 0, set(), set(), False),
    # ── FLAT Walk OLD (legacy) ────────────────────────────────────────────────
    ('FLAT_Walk_OLD_REAL_1', 'OLD_WALK', 'legacy_odom20260528_153558', 'FLAT_WALK_OLD_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_Walk_OLD_REAL_2', 'OLD_WALK', 'legacy_odom20260528_153746', 'FLAT_WALK_OLD_REAL_2.csv', 0, set(), set(), False),
    ('FLAT_Walk_OLD_REAL_3', 'OLD_WALK', 'legacy_odom20260528_154024', 'FLAT_WALK_OLD_REAL_3.csv', 0, set(), set(), False),
    ('FLAT_Walk_OLD_REAL_4', 'OLD_WALK', 'legacy_odom20260528_154223', 'FLAT_WALK_OLD_REAL_4.csv', 0, set(), set(), False),
    ('FLAT_Walk_OLD_REAL_5', 'OLD_WALK', 'legacy_odom20260528_154358', 'FLAT_WALK_OLD_REAL_5.csv', 0, set(), set(), False),
    # ── FLAT WLW NEW ──────────────────────────────────────────────────────────
    ('FLAT_WLW_NEW_REAL_1', 'NEW_WLW', 'odom_fusion20260528_155304_replay', 'FLAT_WLW_NEW_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_WLW_NEW_REAL_2', 'NEW_WLW', 'odom_fusion20260528_155857_replay', 'FLAT_WLW_NEW_REAL_2.csv', 0, {'px','py','vx','vy','roll','pitch'}, set(), False),  # 反向
    ('FLAT_WLW_NEW_REAL_3', 'NEW_WLW', 'odom_fusion20260528_160046_replay', 'FLAT_WLW_NEW_REAL_3.csv', 0, set(), set(), False),
    ('FLAT_WLW_NEW_REAL_4', 'NEW_WLW', 'odom_fusion20260528_161450_replay', 'FLAT_WLW_NEW_REAL_4.csv', 0, {'px','py','vx','vy','roll','pitch'}, set(), False),  # 反向
    ('FLAT_WLW_NEW_REAL_5', 'NEW_WLW', 'odom_fusion20260528_161821_replay', 'FLAT_WLW_NEW_REAL_5.csv', 0, {'px','py','vx','vy','roll','pitch'}, set(), False),  # 反向
    # ── FLAT WLW OLD ──────────────────────────────────────────────────────────
    ('FLAT_WLW_OLD_REAL_1', 'OLD_WLW', 'legacy_odom20260528_163850', 'FLAT_WLW_OLD_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_WLW_OLD_REAL_2', 'OLD_WLW', 'legacy_odom20260528_164050', 'FLAT_WLW_OLD_REAL_2.csv', 0, set(), set(), False),
    ('FLAT_WLW_OLD_REAL_3', 'OLD_WLW', 'legacy_odom20260528_164352', 'FLAT_WLW_OLD_REAL_3.csv', 0, set(), {'px','py','vx','vy'}, False),  # 反向
    ('FLAT_WLW_OLD_REAL_4', 'OLD_WLW', 'legacy_odom20260528_164556', 'FLAT_WLW_OLD_REAL_4.csv', 0, set(), {'px','py','vx','vy'}, False),  # 反向
    ('FLAT_WLW_OLD_REAL_5', 'OLD_WLW', 'legacy_odom20260528_164826', 'FLAT_WLW_OLD_REAL_5.csv', 0, set(), set(), False),
    # ── FLAT MPC NEW (ESEKF) ──────────────────────────────────────────────────
    ('FLAT_MPC_NEW_REAL_1', 'NEW_MPC', 'mpc_esekf_20260528_172005', 'FLAT_MPC_NEW_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_MPC_NEW_REAL_2', 'NEW_MPC', 'mpc_esekf_20260528_172225', 'FLAT_MPC_NEW_REAL_2.csv', 0, set(), set(), False),
    ('FLAT_MPC_NEW_REAL_3', 'NEW_MPC', 'mpc_esekf_20260528_192648', 'FLAT_MPC_NEW_REAL_3.csv', 0, set(), set(), False),
    ('FLAT_MPC_NEW_REAL_4', 'NEW_MPC', 'mpc_esekf_20260528_192958', 'FLAT_MPC_NEW_REAL_4.csv', 0, set(), set(), False),
    ('FLAT_MPC_NEW_REAL_5', 'NEW_MPC', 'mpc_esekf_20260528_193420', 'FLAT_MPC_NEW_REAL_5.csv', 0, set(), set(), False),
    # ── FLAT MPC OLD (legacy) ─────────────────────────────────────────────────
    ('FLAT_MPC_OLD_REAL_1', 'OLD_MPC', 'mpc_legacy_20260528_194134', 'FLAT_MPC_OLD_REAL_1.csv', 0, set(), set(), False),
    ('FLAT_MPC_OLD_REAL_2', 'OLD_MPC', 'mpc_legacy_20260528_194440', 'FLAT_MPC_OLD_REAL_2.csv', 0, set(), set(), False),
    ('FLAT_MPC_OLD_REAL_3', 'OLD_MPC', 'mpc_legacy_20260528_194654', 'FLAT_MPC_OLD_REAL_3.csv', 0, set(), set(), False),
    ('FLAT_MPC_OLD_REAL_4', 'OLD_MPC', 'mpc_legacy_20260528_194850', 'FLAT_MPC_OLD_REAL_4.csv', 0, set(), set(), False),
    ('FLAT_MPC_OLD_REAL_5', 'OLD_MPC', 'mpc_legacy_20260528_195100', 'FLAT_MPC_OLD_REAL_5.csv', 0, set(), set(), False),
]

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
BAGS_DIR  = os.path.join(BASE, 'bags')
VICON_DIR = os.path.join(BASE, 'vicon')
OUT_DIR   = os.path.join(BASE, 'results')
os.makedirs(OUT_DIR, exist_ok=True)

CONTACT_THRESHOLD_M = 0.015
GROUND_MARKERS = ['ground1', 'ground2', 'ground3', 'ground4']

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


def _shade_contact(ax, t, c, color, alpha=0.18):
    prev = False; t0 = 0.0
    for i in range(len(t)):
        if c[i] and not prev:
            t0 = t[i]
        elif not c[i] and prev:
            ax.axvspan(t0, t[i - 1], color=color, alpha=alpha, lw=0)
        prev = bool(c[i])
    if prev:
        ax.axvspan(t0, t[-1], color=color, alpha=alpha, lw=0)


def interp_gmo(gmo_t, gmo_leg, t_tgt, T_END):
    mk = (gmo_t >= -0.5) & (gmo_t <= T_END + 0.5)
    t_g = gmo_t[mk]; c_g = gmo_leg[mk].astype(float)
    if len(t_g) < 2:
        return np.zeros(len(t_tgt), dtype=bool)
    return interp1d(t_g, c_g, kind='nearest',
                    bounds_error=False, fill_value=0.)(t_tgt) > 0.5


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
    """Full ESEKF analysis: contact + EKF + odom_mapping + lidar.

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
    from scipy.spatial import Delaunay

    print(f'\n{"="*60}\n[NEW] {exp_id}  ({bag_name})\n{"="*60}')
    os.makedirs(out_dir, exist_ok=True)

    bag_db = os.path.join(BAGS_DIR, bag_name, f'{bag_name}_0.db3')
    csv    = os.path.join(VICON_DIR, vicon_csv)

    vi  = load_vicon(csv, contact_threshold_m=CONTACT_THRESHOLD_M,
                     ground_markers=GROUND_MARKERS)
    bag = load_fusion_bag(bag_db, rate=1.0, trigger_pair=trigger_pair)

    ekf   = bag['ekf']; ba = bag['ba']; bw = bag['bw']
    gmo   = bag['gmo']; odom = bag['odom']; fv = bag['fv']; lidar = bag['lidar']

    # Time alignment:
    # If the bag only recorded trigger-OFF (enable=False), bag times are relative to
    # trigger OFF and will be negative (e.g. -27s to 0).  Offset them to VICON time
    # by adding vi.t_trigger_end so the analysis window becomes [0, T_END] in VICON frame.
    _bag_t_end = bag['t_trigger_end']
    _vi_t_end  = vi.t_trigger_end
    if _bag_t_end is None and len(ekf['t']) > 0 and ekf['t'][-1] < 1.0:
        # bag trigger = trigger-OFF; shift everything to VICON time
        t_offset = float(_vi_t_end) if _vi_t_end is not None else 0.0
        for d in [ekf, ba, bw, gmo, odom, fv, lidar]:
            if len(d['t']) > 0:
                d['t'] = d['t'] + t_offset
        print(f'  [time-align] trigger-OFF-only bag → offset +{t_offset:.2f}s to VICON frame')
    else:
        t_offset = 0.0

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
        T_CO_rpy = rpy_CO.tolist(); T_CO_t = t_CO.tolist()
        resid_mean = float(resid.mean() * 100); resid_max = float(resid.max() * 100)
    else:
        lidar['px_odom'] = lidar['px']; lidar['py_odom'] = lidar['py']
        T_CO_rpy = [0, 0, 0]; T_CO_t = [0, 0, 0]
        resid_mean = resid_max = float('nan')

    # ── STEP 1: Contact ────────────────────────────────────────────────────────
    LEG_MAP = [('LF', 'G1'), ('RF', 'G2'), ('RH', 'G3'), ('LH', 'G4')]
    COLORS_LEG = {'LF': 'steelblue', 'RF': 'darkorange', 'RH': 'forestgreen', 'LH': 'crimson'}
    contact_results = {}
    has_gmo = len(gmo['t']) > 0

    if has_gmo:
        gnd_pts = []
        for m in GROUND_MARKERS:
            try:
                xyz = vi.get_xyz(m)
                v = ~np.isnan(xyz).any(axis=1)
                if v.any():
                    gnd_pts.append(vi.to_robot(xyz[v][0:1])[0, :2])
            except Exception:
                pass
        try:
            hull_gnd = Delaunay(np.array(gnd_pts)) if len(gnd_pts) >= 3 else None
        except Exception:
            hull_gnd = None

        def in_region(xy_mm, hull):
            if hull is None:
                return np.ones(len(xy_mm), dtype=bool)
            return hull.find_simplex(xy_mm) >= 0

        for leg, gm in LEG_MAP:
            hf = vi.foot_heights[leg]; cf = vi.contact[leg]
            try:
                fxyz = vi.get_xyz(gm)
                fxy  = np.full((len(vi.t_traj), 2), np.nan)
                vf   = ~np.isnan(fxyz).any(1)
                if vf.any():
                    fxy[vf] = vi.to_robot(fxyz[vf])[:, :2]
                    rmask = np.zeros(len(vi.t_traj), dtype=bool)
                    rmask[vf] = in_region(fxy[vf], hull_gnd)
                else:
                    rmask = np.zeros(len(vi.t_traj), dtype=bool)
            except Exception:
                rmask = np.zeros(len(vi.t_traj), dtype=bool)

            # Compare only where both sensors actually contain data.  In
            # particular, OFF-only bags may start several seconds after VICON.
            finite_foot = np.isfinite(hf)
            gmo_overlap = ((vi.t_traj >= gmo['t'][0]) &
                           (vi.t_traj <= gmo['t'][-1]))
            amask = mask_win_vi & rmask & finite_foot & gmo_overlap
            if amask.sum() < 10:
                contact_results[leg] = None; continue

            ta  = vi.t_traj[amask]
            cva = hf[amask] < CONTACT_THRESHOLD_M
            cga = interp_gmo(gmo['t'], gmo[leg], ta, T_END)
            tv = ta; cv_v = cva; cg_v = cga
            if len(tv) == 0:
                contact_results[leg] = None; continue

            TP = int(np.sum(cv_v & cg_v)); TN = int(np.sum(~cv_v & ~cg_v))
            FP = int(np.sum(~cv_v & cg_v)); FN = int(np.sum(cv_v & ~cg_v))
            N  = len(tv)
            acc  = (TP + TN) / N

            contact_results[leg] = {
                'N': N, 'TP': TP, 'TN': TN, 'FP': FP, 'FN': FN,
                'acc': acc,
                't_start': float(tv[0]), 't_end': float(tv[-1]),
            }
            print(f'  [{leg}] accuracy={acc:.1%} '
                  f'(N={N}, TP={TP}, TN={TN}, FP={FP}, FN={FN}; '
                  f'overlap={tv[0]:.2f}–{tv[-1]:.2f}s)')

        # Contact plot
        fig, axes = plt.subplots(4, 1, figsize=(13, 8), sharex=True)
        for ax, (leg, gm) in zip(axes, LEG_MAP):
            hf_leg = vi.foot_heights[leg][mask_win_vi]
            cf_leg = vi.contact[leg][mask_win_vi]
            c_gmo  = interp_gmo(gmo['t'], gmo[leg], t_win, T_END)
            color  = COLORS_LEG[leg]
            ax.plot(t_win, hf_leg * 1000, lw=0.8, color=color, label=f'{leg} height [mm]')
            _shade_contact(ax, t_win, cf_leg, color='tab:green',  alpha=0.18)
            _shade_contact(ax, t_win, c_gmo,  color='tab:orange', alpha=0.12)
            ax.axhline(CONTACT_THRESHOLD_M * 1000, color='k', ls='--', lw=0.8, alpha=0.5)
            ax.axvspan(0, T_END, color='gold', alpha=0.06)
            ax.set_ylabel(f'{leg} [mm]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle(f'Contact Detection — {exp_id}')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, 'fig_contact.png'), dpi=150)
        plt.close(fig)

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
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_pos.png'), dpi=150); plt.close(fig)

    # EKF velocity plot
    fig, axes = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
    for ax, (ev, vv, lbl) in zip(axes, [(evx, vi_vx_e, 'vx'), (evy, vi_vy_e, 'vy'), (evz, vi_vz_e, 'vz')]):
        ax.plot(et, ev, lw=1.0, label=f'EKF {lbl}')
        ax.plot(et[~np.isnan(vv)], vv[~np.isnan(vv)], lw=1.0, label=f'VICON {lbl}')
        ax.axvspan(0, T_END, color='gold', alpha=0.06)
        ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(f'Inner EKF Velocity — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_vel.png'), dpi=150); plt.close(fig)

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
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_rpy.png'), dpi=150); plt.close(fig)

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
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_ekf_xz.png'), dpi=150); plt.close(fig)

    # LiDAR XZ
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(pos_vicon[vi_valid, 0], vi_z_plot, lw=1.5, label='VICON', color='black')
    lidar_pz_odom = lidar.get('pz_odom', lidar['pz'])
    ax.plot(lidar.get('px_odom', lidar['px']), lidar_pz_odom,
            lw=0.8, label='lidar (odom frame)', color='purple', alpha=0.7)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Z [m]')
    ax.legend(); ax.grid(True, alpha=0.4)
    ax.set_title(f'LiDAR XZ Trajectory — {exp_id}')
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_lidar_xz.png'), dpi=150); plt.close(fig)

    # ── STEP 4: LiDAR quality ─────────────────────────────────────────────────
    lidar_mask = (lidar['t'] >= 0.0) & (lidar['t'] <= T_END)
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
        'contact': {leg: {k: v for k, v in c.items() if not isinstance(v, np.ndarray)}
                    for leg, c in contact_results.items() if c is not None},
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
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_legacy_xz.png'), dpi=150); plt.close(fig)

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
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig_legacy_vel.png'), dpi=150); plt.close(fig)

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
# Summary report
# ═══════════════════════════════════════════════════════════════════════════════
def write_summary_report(all_metrics):
    """Write a summary Markdown comparison report."""

    def f(v, fmt='.2f'):
        if v is None: return 'N/A'
        try:
            fv = float(v)
            return 'N/A' if np.isnan(fv) else format(fv, fmt)
        except Exception:
            return str(v)

    def group_stats(group_key, field, subfield):
        vals = []
        for m in all_metrics:
            if m.get('group', '').startswith(group_key) and not m.get('exclude_stats', False):
                try:
                    vals.append(float(m[field][subfield]))
                except Exception:
                    pass
        vals = [v for v in vals if not np.isnan(v)]
        if not vals:
            return float('nan'), float('nan'), float('nan')
        return float(np.mean(vals)), float(np.std(vals)), float(np.min(vals))

    # Group by mode × system
    groups = {
        'NEW_WALK': ('Walk (步行)', 'ESEKF'),
        'OLD_WALK': ('Walk (步行)', 'Legacy'),
        'NEW_WLW':  ('WLW (輪足步行)', 'ESEKF'),
        'OLD_WLW':  ('WLW (輪足步行)', 'Legacy'),
        'NEW_MPC':  ('MPC (模型預測控制)', 'ESEKF'),
        'OLD_MPC':  ('MPC (模型預測控制)', 'Legacy'),
    }

    # Per-trial table
    per_trial_rows = []
    for m in all_metrics:
        exp_id = m['exp_id']
        group  = m.get('group', '')
        T_END  = m.get('T_END', float('nan'))
        p = m.get('position', {})
        v = m.get('velocity', {})
        rmse_3d = p.get('RMSE_3D_cm') or p.get('RMSE_2D_cm')
        excl_mark = ' ¹' if m.get('exclude_stats') else ''
        per_trial_rows.append(
            f'| {exp_id}{excl_mark} | {group} | {f(T_END, ".1f")} '
            f'| {f(p.get("RMSE_X_cm"), ".2f")} | {f(p.get("RMSE_Y_cm"), ".2f")} '
            f'| {f(rmse_3d, ".2f")} '
            f'| {f(v.get("RMSE_vx"), ".3f")} | {f(v.get("RMSE_vy"), ".3f")} |'
        )

    # Group summary rows
    def summary_row(gkey, label, system):
        p_mean, p_std, p_min = group_stats(gkey, 'position', 'RMSE_3D_cm' if 'NEW' in gkey else 'RMSE_2D_cm')
        vx_mean, _, _ = group_stats(gkey, 'velocity', 'RMSE_vx')
        n = sum(1 for m in all_metrics
                if m.get('group', '').startswith(gkey) and not m.get('exclude_stats', False))
        return (f'| {label} | {system} | {n} '
                f'| {f(p_mean, ".2f")} ± {f(p_std, ".2f")} '
                f'| {f(vx_mean, ".3f")} |')

    now = '2026-05-28'
    report = f"""# CORGI 實驗分析報告 — 20260528

**日期：** {now}
**實驗地點：** Flat ground（平地）
**有效實驗數：** {len(all_metrics)} / 31（RUGG 系列尚無資料）
**分析腳本：** `analyze.py`

---

## 實驗架構

```
/imu_raw, /motor/state ──► corgi_leg_odom ──► Inner EKF (/ekf)
                                                      │
/gmo/contact_state                                    ▼
/lidar_odom (FAST-LIO2) ──────────────► corgi_fusion_node ──► /odom_mapping
                                                              /fusion/bv

Legacy system: /odometry/legacy/position, /odometry/legacy/velocity
```

**實驗分組：**
| 分組代碼 | 模式 | 里程計系統 | 試驗數 |
|----------|------|-----------|--------|
| NEW_WALK | 平地步行 | ESEKF + fusion | 6 |
| OLD_WALK | 平地步行 | Legacy | 5 |
| NEW_WLW  | 平地輪足步行 | ESEKF + fusion | 5 |
| OLD_WLW  | 平地輪足步行 | Legacy | 5 |
| NEW_MPC  | 平地 MPC | ESEKF + fusion | 5 |
| OLD_MPC  | 平地 MPC | Legacy | 5 |

---

## 1. 每次試驗結果

| 實驗編號 | 分組 | T_END (s) | RMSE X (cm) | RMSE Y (cm) | RMSE 3D/2D (cm) | RMSE vx (m/s) | RMSE vy (m/s) |
|----------|------|-----------|-------------|-------------|-----------------|---------------|---------------|
"""
    for row in per_trial_rows:
        report += row + '\n'

    report += f"""
> NEW 系列使用 RMSE_3D（含 Z 軸），OLD 系列使用 RMSE_2D（XY 平面）。
> ¹ 此試驗有效數據異常（姿態偏移過大），已排除於分組統計外，但保留個別指標。

---

## 2. 分組統計（平均 ± 標準差）

| 模式 | 系統 | 試驗數 | RMSE 位置 (cm) | RMSE vx (m/s) |
|------|------|--------|----------------|---------------|
"""
    for gkey, (label, system) in groups.items():
        report += summary_row(gkey, label, system) + '\n'

    # Walk comparison
    w_new_3d, w_new_std, _ = group_stats('NEW_WALK', 'position', 'RMSE_3D_cm')
    w_old_2d, w_old_std, _ = group_stats('OLD_WALK', 'position', 'RMSE_2D_cm')
    wlw_new_3d, wlw_new_std, _ = group_stats('NEW_WLW', 'position', 'RMSE_3D_cm')
    wlw_old_2d, wlw_old_std, _ = group_stats('OLD_WLW', 'position', 'RMSE_2D_cm')
    mpc_new_3d, mpc_new_std, _ = group_stats('NEW_MPC', 'position', 'RMSE_3D_cm')
    mpc_old_2d, mpc_old_std, _ = group_stats('OLD_MPC', 'position', 'RMSE_2D_cm')

    def improvement(new_val, old_val):
        if np.isnan(new_val) or np.isnan(old_val) or old_val == 0:
            return 'N/A'
        imp = (old_val - new_val) / old_val * 100
        sign = '+' if imp >= 0 else ''
        return f'{sign}{imp:.1f}%'

    report += f"""
---

## 3. NEW vs OLD 比較（位置 RMSE）

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|-----------|------------|---------|
| Walk | {f(w_new_3d, '.2f')} ± {f(w_new_std, '.2f')} | {f(w_old_2d, '.2f')} ± {f(w_old_std, '.2f')} | {improvement(w_new_3d, w_old_2d)} |
| WLW  | {f(wlw_new_3d, '.2f')} ± {f(wlw_new_std, '.2f')} | {f(wlw_old_2d, '.2f')} ± {f(wlw_old_std, '.2f')} | {improvement(wlw_new_3d, wlw_old_2d)} |
| MPC  | {f(mpc_new_3d, '.2f')} ± {f(mpc_new_std, '.2f')} | {f(mpc_old_2d, '.2f')} ± {f(mpc_old_std, '.2f')} | {improvement(mpc_new_3d, mpc_old_2d)} |

> ⚠️ 注意：ESEKF 的 RMSE 包含 Z 軸，Legacy 只有 XY 平面，因此比較時需考慮量測基礎不同。

---

## 4. ESEKF 系統詳細指標

### 4.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |
|----------|----------|-----------|---------|
"""
    for m in all_metrics:
        if 'NEW' in m.get('group', '') and 'attitude' in m:
            att = m['attitude']
            report += (f'| {m["exp_id"]} | {f(att.get("RMSE_roll_deg"), ".3f")} '
                       f'| {f(att.get("RMSE_pitch_deg"), ".3f")} '
                       f'| {f(att.get("RMSE_yaw_deg"), ".3f")} |\n')

    report += """
### 4.2 odom_mapping 位置 RMSE

| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|-------------|-------------|--------------|
"""
    for m in all_metrics:
        if 'NEW' in m.get('group', '') and 'odom_pos' in m:
            op = m['odom_pos']
            report += (f'| {m["exp_id"]} | {f(op.get("RMSE_X_cm"), ".2f")} '
                       f'| {f(op.get("RMSE_Y_cm"), ".2f")} '
                       f'| {f(op.get("RMSE_2D_cm"), ".2f")} |\n')

    report += """
---

## 5. 接觸偵測指標（ESEKF 系統）

| 實驗編號 | 腳 | 有效時間步 | Acc | TP | TN | FP | FN |
|----------|-----|------------|-----|----|----|----|----|
"""
    for m in all_metrics:
        if 'NEW' in m.get('group', '') and 'contact' in m:
            for leg, c in m['contact'].items():
                if c is None:
                    continue
                report += (f'| {m["exp_id"]} | {leg} | {c["N"]} | {c["acc"]:.1%} '
                           f'| {c["TP"]} | {c["TN"]} | {c["FP"]} | {c["FN"]} |\n')

    report += """
---

## 6. MPC 終點 X 位置分析（目標：3.0 m）

MPC 控制器以走到 X = 3 m 為目標停止。本節分析估測器回報的停止位置（控制依據）與 VICON 實際量測的停止位置之間的誤差，評估里程計對運動控制的實際影響。

"""
    # Build MPC final-X table
    TARGET_X = 3.0
    new_ekf_x = []; new_vi_x = []; old_leg_x = []; old_vi_x = []
    report += '| 實驗編號 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | 停止誤差 VICON (cm) |\n'
    report += '|----------|-------------------|-------------------|--------------|--------------------|\n'
    for m in all_metrics:
        if 'MPC' not in m.get('group', ''):
            continue
        p = m['position']
        if 'NEW' in m['group']:
            ex  = p.get('final_EKF_x')
            vx  = p.get('final_VICON_x')
            est_err = (ex - TARGET_X) * 100 if ex is not None else float('nan')
            vi_err  = (vx - TARGET_X) * 100 if vx is not None else float('nan')
            report += (f'| {m["exp_id"]} | {f(ex, ".3f")} | {f(vx, ".3f")} '
                       f'| {f(est_err, "+.1f")} | {f(vi_err, "+.1f")} |\n')
            if ex is not None: new_ekf_x.append(ex)
            if vx is not None: new_vi_x.append(vx)
        else:
            lx  = p.get('final_leg_x')
            vx  = p.get('final_VICON_x')
            leg_err = (lx - TARGET_X) * 100 if lx is not None else float('nan')
            vi_err  = (vx - TARGET_X) * 100 if vx is not None else float('nan')
            report += (f'| {m["exp_id"]} | {f(lx, ".3f")} | {f(vx, ".3f")} '
                       f'| {f(leg_err, "+.1f")} | {f(vi_err, "+.1f")} |\n')
            if lx is not None: old_leg_x.append(lx)
            if vx is not None: old_vi_x.append(vx)

    import numpy as np_inner
    def _s(arr):
        a = np_inner.array([v for v in arr if not np_inner.isnan(v)])
        return f'{a.mean():.3f} ± {a.std():.3f}' if len(a) else 'N/A'
    def _abs_err_cm(arr):
        a = np_inner.array([abs(v - TARGET_X) * 100 for v in arr if not np_inner.isnan(v)])
        return f'{a.mean():.1f} cm' if len(a) else 'N/A'

    new_vi_abs  = float(np_inner.mean([abs(v - TARGET_X)*100 for v in new_vi_x]))  if new_vi_x  else float('nan')
    old_vi_abs  = float(np_inner.mean([abs(v - TARGET_X)*100 for v in old_vi_x]))  if old_vi_x  else float('nan')
    new_ekf_abs = float(np_inner.mean([abs(v - TARGET_X)*100 for v in new_ekf_x])) if new_ekf_x else float('nan')
    old_leg_abs = float(np_inner.mean([abs(v - TARGET_X)*100 for v in old_leg_x])) if old_leg_x else float('nan')

    report += f"""
**統計摘要（目標 X = {TARGET_X} m）**

| 系統 | 估測器 final X | 實際 VICON final X | 估測器停止誤差 (abs mean) | VICON 停止誤差 (abs mean) |
|------|--------------|-------------------|--------------------------|--------------------------|
| ESEKF (NEW) | {_s(new_ekf_x)} m | {_s(new_vi_x)} m | {_abs_err_cm(new_ekf_x)} | {f(new_vi_abs, '.1f')} cm |
| Legacy (OLD) | {_s(old_leg_x)} m | {_s(old_vi_x)} m | {_abs_err_cm(old_leg_x)} | {f(old_vi_abs, '.1f')} cm |

**分析：**
- ESEKF（NEW）：估測器回報 {_s(new_ekf_x)} m，VICON 實際停止 {_s(new_vi_x)} m，平均停止誤差 **{f(new_vi_abs, '.1f')} cm**（< 2 cm）。估測結果與實際高度吻合，控制器能精準停止於目標位置。
- Legacy（OLD）：估測器回報 {_s(old_leg_x)} m，VICON 實際停止 {_s(old_vi_x)} m，平均停止誤差 **{f(old_vi_abs, '.1f')} cm**（~24 cm，超出目標 24 cm）。Legacy 里程計嚴重**高估**行進距離（腿式積分累積誤差），導致機器人尚未到達 3 m 目標便誤判已抵達而停止。
- 改善幅度：ESEKF 的實際停止誤差為 Legacy 的 **{f(old_vi_abs / new_vi_abs, '.1f')}×** 以下，顯示 LiDAR 融合對 MPC 點到點移動任務的準確性有關鍵改善。

---

## 7. 觀察與結論

### 平地步行（Walk）
- ESEKF 融合 LiDAR 與腿式里程計，提供三維位置估計（RMSE_3D ~6 cm）。
- Legacy 系統僅腿式里程計，RMSE_2D ~26 cm，誤差約為 ESEKF 的 4×。

### 平地輪足步行（WLW）
- ESEKF RMSE_3D ~5.5 cm；Legacy RMSE_2D ~11 cm，約 2× 差距。
- 輪足模式下里程計積分誤差相較步行模式略小。

### 平地 MPC（MPC）— 終點精度
- MPC 控制目標為 X = 3.0 m 定點停止。
- **ESEKF**：VICON 實際停止 {_s(new_vi_x)} m，誤差 **{f(new_vi_abs, '.1f')} cm** ✓
- **Legacy**：VICON 實際停止 {_s(old_vi_x)} m，誤差 **{f(old_vi_abs, '.1f')} cm** ✗（估測器虛報抵達，實際距離不足）
- mpc_esekf bag 無 `/ekf/ba`、`/ekf/bw`，故偏差估計不分析。

### 整體結論
- ESEKF + LiDAR 融合在三種運動模式下均顯著優於 Legacy 里程計。
- 對於 MPC 定點控制任務，里程計精度直接影響停止位置；Legacy 累積誤差可達 24 cm，而 ESEKF 可控制在 2 cm 以內。

---

*報告由 `analyze.py` 自動產生。日期：2026-06-11*
"""
    out_path = os.path.join(OUT_DIR, 'analysis_report.md')
    with open(out_path, 'w') as fp:
        fp.write(report)
    print(f'\n[Summary] → {out_path}')


def write_corrected_summary_report(all_metrics):
    """Update the detailed report from metrics produced by this script.

    This preserves the manually authored MPC/open-loop discussion while making
    sections 1–5 reproducible from the raw bag/VICON calculation chain.
    """
    report_path = os.path.join(OUT_DIR, 'analysis_report.md')
    if not os.path.exists(report_path):
        write_summary_report(all_metrics)
    with open(report_path) as fp:
        report = fp.read()

    included = [m for m in all_metrics if not m.get('exclude_stats', False)]
    groups = [
        ('NEW_WALK', 'Walk (步行)', 'ESEKF'),
        ('OLD_WALK', 'Walk (步行)', 'Legacy'),
        ('NEW_WLW',  'WLW (輪足步行)', 'ESEKF'),
        ('OLD_WLW',  'WLW (輪足步行)', 'Legacy'),
        ('NEW_MPC',  'MPC (模型預測控制)', 'ESEKF'),
        ('OLD_MPC',  'MPC (模型預測控制)', 'Legacy'),
    ]

    def fmt(value, digits):
        try:
            value = float(value)
            return 'N/A' if not np.isfinite(value) else f'{value:.{digits}f}'
        except Exception:
            return 'N/A'

    def mean_std(rows, getter):
        values = np.asarray([getter(m) for m in rows], dtype=float)
        values = values[np.isfinite(values)]
        if not len(values):
            return float('nan'), float('nan')
        std = np.std(values, ddof=1) if len(values) > 1 else 0.0
        return float(np.mean(values)), float(std)

    section1 = """## 1. 每次試驗結果

位置誤差單位為 cm；速度誤差單位為 m/s。位置使用 VICON 與估測器的有效重疊區間；速度使用重疊區間內的 `35%–75% T_END` 穩態窗。3D RMSE 由同一時間點的三軸誤差向量計算。

| 實驗編號 | 分組 | 有效資料 (s) | 位置 X | 位置 Y | 位置 Z | 位置 3D | 速度 X | 速度 Y | 速度 Z | 速度 3D |
|----------|------|--------------|--------|--------|--------|---------|--------|--------|--------|---------|
"""
    for m in included:
        p = m['position']; v = m['velocity']
        section1 += (
            f'| {m["exp_id"]} | {m["group"]} '
            f'| {fmt(m.get("data_start"), 1)}–{fmt(m.get("data_end"), 1)} '
            f'| {fmt(p.get("RMSE_X_cm"), 2)} | {fmt(p.get("RMSE_Y_cm"), 2)} '
            f'| {fmt(p.get("RMSE_Z_cm"), 2)} | {fmt(p.get("RMSE_3D_cm"), 2)} '
            f'| {fmt(v.get("RMSE_vx"), 3)} | {fmt(v.get("RMSE_vy"), 3)} '
            f'| {fmt(v.get("RMSE_vz"), 3)} | {fmt(v.get("RMSE_3D"), 3)} |\n'
        )
    section1 += (
        '\n> `FLAT_Walk_NEW_REAL_2` 資料異常，已完全排除，不列入個別結果、'
        '分組統計及後續分析。`FLAT_Walk_NEW_REAL_1` 的 bag 僅涵蓋 '
        '7.6–35.1 s，因此只在實際重疊區間計算。\n'
    )

    section2 = """## 2. 分組統計（平均 ± 樣本標準差）

| 模式 | 系統 | n | Pos X (cm) | Pos Y (cm) | Pos Z (cm) | Pos 3D (cm) | Vel X (m/s) | Vel Y (m/s) | Vel Z (m/s) | Vel 3D (m/s) |
|------|------|---|------------|------------|------------|-------------|-------------|-------------|-------------|--------------|
"""
    group_stats = {}
    for key, label, system in groups:
        rows = [m for m in included if m['group'] == key]
        getters = [
            lambda m: m['position']['RMSE_X_cm'],
            lambda m: m['position']['RMSE_Y_cm'],
            lambda m: m['position']['RMSE_Z_cm'],
            lambda m: m['position']['RMSE_3D_cm'],
            lambda m: m['velocity']['RMSE_vx'],
            lambda m: m['velocity']['RMSE_vy'],
            lambda m: m['velocity']['RMSE_vz'],
            lambda m: m['velocity']['RMSE_3D'],
        ]
        stats = [mean_std(rows, getter) for getter in getters]
        group_stats[key] = stats
        cells = [f'{mean:.3f} ± {std:.3f}' for mean, std in stats]
        section2 += f'| {label} | {system} | {len(rows)} | ' + ' | '.join(cells) + ' |\n'

    section3 = """## 3. NEW vs OLD 比較（位置與速度 3D RMSE）

### 3.1 位置 3D RMSE

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|------------|-------------|----------|
"""
    pairs = [('Walk', 'NEW_WALK', 'OLD_WALK'),
             ('WLW', 'NEW_WLW', 'OLD_WLW'),
             ('MPC', 'NEW_MPC', 'OLD_MPC')]
    for label, new_key, old_key in pairs:
        new_mean, new_std = group_stats[new_key][3]
        old_mean, old_std = group_stats[old_key][3]
        imp = (old_mean - new_mean) / old_mean * 100.0
        section3 += (f'| {label} | {new_mean:.2f} ± {new_std:.2f} '
                     f'| {old_mean:.2f} ± {old_std:.2f} | {imp:+.1f}% |\n')
    section3 += """

### 3.2 速度 3D RMSE

| 模式 | ESEKF (m/s) | Legacy (m/s) | 改善幅度 |
|------|-------------|--------------|----------|
"""
    for label, new_key, old_key in pairs:
        new_mean, new_std = group_stats[new_key][7]
        old_mean, old_std = group_stats[old_key][7]
        imp = (old_mean - new_mean) / old_mean * 100.0
        section3 += (f'| {label} | {new_mean:.3f} ± {new_std:.3f} '
                     f'| {old_mean:.3f} ± {old_std:.3f} | {imp:+.1f}% |\n')
    section3 += (
        '\n> NEW 與 OLD 均使用 X、Y、Z 三軸的 3D RMSE；改善幅度為 '
        '`(Legacy − ESEKF) / Legacy × 100%`。\n'
    )

    section4 = """## 4. ESEKF 系統詳細指標

### 4.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |
|----------|----------|-----------|---------|
"""
    for m in included:
        if 'NEW' in m['group'] and 'attitude' in m:
            a = m['attitude']
            section4 += (f'| {m["exp_id"]} | {fmt(a.get("RMSE_roll_deg"), 3)} '
                         f'| {fmt(a.get("RMSE_pitch_deg"), 3)} '
                         f'| {fmt(a.get("RMSE_yaw_deg"), 3)} |\n')
    section4 += """

### 4.2 odom_mapping 位置 RMSE

| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|-------------|-------------|--------------|
"""
    for m in included:
        if 'NEW' in m['group'] and 'odom_pos' in m:
            p = m['odom_pos']
            section4 += (f'| {m["exp_id"]} | {fmt(p.get("RMSE_X_cm"), 2)} '
                         f'| {fmt(p.get("RMSE_Y_cm"), 2)} '
                         f'| {fmt(p.get("RMSE_2D_cm"), 2)} |\n')

    section5 = """## 5. 接觸偵測指標（逐有效時間步比對）

僅使用 VICON 腳標記有效、腳標記位於 ground marker 覆蓋區、且 GMO 實際有資料的重疊時間步。在每個有效時間步直接比對 VICON 與 GMO 的二元接觸狀態，Acc = (TP + TN) / N。不進行接觸或離地事件配對，也不計算延遲。四腳平均為各腳 accuracy 的算術平均。

| 實驗編號 | 四腳平均 Acc | 總有效時間步 | TP | TN | FP | FN |
|----------|-------------|--------------|----|----|----|----|
"""
    for m in included:
        if 'NEW' not in m['group'] or not m.get('contact'):
            continue
        legs = list(m['contact'].values())
        def leg_mean(key):
            values = [float(c[key]) for c in legs if np.isfinite(float(c.get(key, np.nan)))]
            return float(np.mean(values)) if values else float('nan')
        section5 += (
            f'| {m["exp_id"]} | {leg_mean("acc"):.1%} '
            f'| {sum(int(c["N"]) for c in legs)} '
            f'| {sum(int(c["TP"]) for c in legs)} '
            f'| {sum(int(c["TN"]) for c in legs)} '
            f'| {sum(int(c["FP"]) for c in legs)} '
            f'| {sum(int(c["FN"]) for c in legs)} |\n'
        )

    report = re.sub(r'\*\*有效實驗數：\*\*.*',
                    f'**有效實驗數：** {len(included)} / 31（排除 1 筆異常資料；RUGG 系列尚無資料）',
                    report)
    report = report.replace('| NEW_WALK | 平地步行 | ESEKF + fusion | 6 |',
                            '| NEW_WALK | 平地步行 | ESEKF + fusion | 5 |')
    replacements = [
        (r'## 1\..*?(?=\n---\n\n## 2\.)', section1.rstrip()),
        (r'## 2\..*?(?=\n---\n\n## 3\.)', section2.rstrip()),
        (r'## 3\..*?(?=\n---\n\n## 4\.)', section3.rstrip()),
        (r'## 4\..*?(?=\n---\n\n## 5\.)', section4.rstrip()),
        (r'## 5\..*?(?=\n---\n\n## 6\.)', section5.rstrip()),
    ]
    for pattern, replacement in replacements:
        report, count = re.subn(pattern, replacement.rstrip() + '\n', report, flags=re.S)
        if count != 1:
            raise RuntimeError(f'Could not uniquely replace report section: {pattern}')

    report = re.sub(
        r'\*報告由 .*?\*',
        '*報告由 `analyze.py` 從 bag 與 VICON 資料重算。更新日期：2026-06-18*',
        report,
    )
    with open(report_path, 'w') as fp:
        fp.write(report)
    print(f'\n[Corrected summary] → {report_path}')



# ─── Comparison bar chart ──────────────────────────────────────────────────────
def write_comparison_plots(all_metrics):
    # Position RMSE comparison per group
    group_order = ['NEW_WALK', 'OLD_WALK', 'NEW_WLW', 'OLD_WLW', 'NEW_MPC', 'OLD_MPC']
    labels = ['Walk\nNEW', 'Walk\nOLD', 'WLW\nNEW', 'WLW\nOLD', 'MPC\nNEW', 'MPC\nOLD']
    colors = ['steelblue', 'darkorange', 'steelblue', 'darkorange', 'steelblue', 'darkorange']

    pos_means = []; pos_stds = []
    vel_means = []; vel_stds = []
    for gk in group_order:
        vals_p = []; vals_v = []
        for m in all_metrics:
            if m.get('group', '') == gk and not m.get('exclude_stats', False):
                p = m.get('position', {})
                v = m.get('velocity', {})
                rmse_p = p.get('RMSE_3D_cm')
                rmse_v = v.get('RMSE_3D')
                if rmse_p is not None and not np.isnan(float(rmse_p)):
                    vals_p.append(float(rmse_p))
                if rmse_v is not None and not np.isnan(float(rmse_v)):
                    vals_v.append(float(rmse_v))
        pos_means.append(np.mean(vals_p) if vals_p else float('nan'))
        pos_stds.append(np.std(vals_p) if vals_p else float('nan'))
        vel_means.append(np.mean(vals_v) if vals_v else float('nan'))
        vel_stds.append(np.std(vals_v) if vals_v else float('nan'))

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    x = np.arange(len(group_order))
    w = 0.65

    for ax, means, stds, ylabel, title in [
        (axes[0], pos_means, pos_stds, 'Position 3D RMSE [cm]', 'Position 3D RMSE'),
        (axes[1], vel_means, vel_stds, 'Velocity 3D RMSE [m/s]', 'Velocity 3D RMSE'),
    ]:
        bars = ax.bar(x, means, w, color=colors, alpha=0.85,
                      yerr=stds, capsize=4, error_kw=dict(lw=1.5))
        for bar, v in zip(bars, means):
            if not np.isnan(v):
                ax.text(bar.get_x() + bar.get_width() / 2., bar.get_height() + 0.05,
                        f'{v:.2f}', ha='center', va='bottom', fontsize=8)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.grid(True, alpha=0.3, axis='y')

    from matplotlib.patches import Patch
    axes[0].legend(handles=[Patch(color='steelblue', label='ESEKF (NEW)'),
                             Patch(color='darkorange', label='Legacy (OLD)')], fontsize=9)
    fig.suptitle('20260528 FLAT 實驗：NEW vs OLD 比較', fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, 'fig_comparison.png'), dpi=150)
    plt.close(fig)
    print('[Summary] → fig_comparison.png')


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp', default=None,
                        help='Run only this experiment ID (e.g. FLAT_Walk_NEW_REAL_1)')
    args = parser.parse_args()

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

    if all_metrics:
        write_corrected_summary_report(all_metrics)
        write_comparison_plots(all_metrics)

    if failed:
        print(f'\n{"="*60}\nFailed experiments:')
        for eid, err in failed:
            print(f'  {eid}: {err}')
    print(f'\nDone. {len(all_metrics)} experiments processed, {len(failed)} failed.')


if __name__ == '__main__':
    main()
