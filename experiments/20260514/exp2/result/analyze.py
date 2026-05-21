#!/usr/bin/env python3
"""
CORGI Experiment Full Analysis — 20260514 exp1
walk_2m_01_plain_odometry (ESEKF)

Follows corgi-data-analysis skill Steps 1-4:
  Step 1: Contact Detection (all 4 legs, single ground region)
  Step 2: Inner EKF (position, velocity, attitude, ba, bw)
  Step 3: Outer Fusion Node (odom_mapping, fusion/bv)
  Step 4: LiDAR Input Quality (/lidar_odom)

NOTE: source ~/corgi_ws/corgi_ros2_ws/install/setup.bash before running.
"""

import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial import Delaunay
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_fusion_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
RESULTS   = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'odom_fusion20260514_220252',
                         'odom_fusion20260514_220252_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'EXP_02.csv')
TRIAL     = 'walk_2m_01_plain_odometry'
DATE      = '20260514'
EXP_ID    = 'exp2'
CONTACT_THRESHOLD_M = 0.015   # 15 mm

# ─── Helpers ──────────────────────────────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)

def quat_to_rpy_deg(qw, qx, qy, qz):
    r = Rotation.from_quat(np.column_stack([qx, qy, qz, qw]))
    return np.degrees(r.as_euler('ZYX')[:, ::-1])   # [roll, pitch, yaw]

def rmse(d):
    v = np.asarray(d)
    return float(np.sqrt(np.nanmean(v ** 2)))

def _shade_window(ax, t_end, label=True):
    if t_end is not None:
        ax.axvspan(0, t_end, color='gold', alpha=0.08,
                   label='trigger window' if label else None)

def _shade_contact(ax, t, c, color, alpha=0.18):
    prev = False; t0 = 0.0
    for i in range(len(t)):
        if c[i] and not prev:
            t0 = t[i]
        elif not c[i] and prev:
            ax.axvspan(t0, t[i-1], color=color, alpha=alpha, lw=0)
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

# ─── Load ─────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
print('='*60)
vi = load_vicon(VICON_CSV,
                contact_threshold_m=CONTACT_THRESHOLD_M,
                ground_markers=['ground1', 'ground2', 'ground3', 'ground4'])
bag = load_fusion_bag(BAG_DB, rate=1.0)
ekf = bag['ekf']; ba = bag['ba']; bw = bag['bw']
gmo = bag['gmo']; odom = bag['odom']; fv = bag['fv']; lidar = bag['lidar']

T_END = min(vi.t_trigger_end, bag['t_trigger_end'])
print(f'\nAnalysis window: t ∈ [0, {T_END:.2f}] s')

# VICON window
mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_win       = vi.t_traj[mask_win_vi]
pos_vicon   = vi.pos_m[mask_win_vi]
v_body_vi   = vi.v_body[mask_win_vi]
rpy_vicon   = vi.rpy[mask_win_vi]

# EKF window
ekf_mask = (ekf['t'] >= 0.0) & (ekf['t'] <= T_END)
et    = ekf['t'][ekf_mask]
epx   = ekf['px'][ekf_mask];  epy = ekf['py'][ekf_mask];  epz = ekf['pz'][ekf_mask]
evx   = ekf['vx'][ekf_mask];  evy = ekf['vy'][ekf_mask];  evz = ekf['vz'][ekf_mask]
eqw   = ekf['qw'][ekf_mask];  eqx = ekf['qx'][ekf_mask]
eqy   = ekf['qy'][ekf_mask];  eqz = ekf['qz'][ekf_mask]
ecov_px = ekf['cov_px'][ekf_mask]
ecov_py = ekf['cov_py'][ekf_mask]
ecov_pz = ekf['cov_pz'][ekf_mask]
rpy_ekf_deg = quat_to_rpy_deg(eqw, eqx, eqy, eqz)

# ─── Align VICON initial orientation to EKF at t=0 ───────────────────────────
# R_align = R_EKF(t=0) @ R_VICON(t=0)^T  applied to every VICON body frame,
# so that at t=0 the VICON RPY matches the EKF RPY exactly.
_vi_rot_ok = ~np.isnan(rpy_vicon).any(1)
_idx_vi0   = int(np.argmax(_vi_rot_ok))
_R_vi0     = Rotation.from_euler('ZYX', rpy_vicon[_idx_vi0, ::-1]).as_matrix()
_R_ekf0    = Rotation.from_quat([eqx[0], eqy[0], eqz[0], eqw[0]]).as_matrix()
_R_align   = _R_ekf0 @ _R_vi0.T
_rpy_vicon_aligned = rpy_vicon.copy()
for _i in np.where(_vi_rot_ok)[0]:
    _Rv = Rotation.from_euler('ZYX', rpy_vicon[_i, ::-1]).as_matrix()
    _rpy_vicon_aligned[_i] = Rotation.from_matrix(_R_align @ _Rv).as_euler('ZYX')[::-1]
rpy_vicon = _rpy_vicon_aligned
print(f'[VICON→EKF align] before={np.degrees(rpy_vicon[_idx_vi0])}, '
      f'EKF_t0={rpy_ekf_deg[0]}')
# ─────────────────────────────────────────────────────────────────────────────

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

# ─── T_{odom←camera_init} ─────────────────────────────────────────────────────
print('\n=== T_{odom←camera_init} via Procrustes ===')
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
    p_chk  = (R_CO @ lpts.T).T + t_CO
    resid  = np.linalg.norm(p_chk - opts, axis=1)
    print(f'  t_CO={t_CO}  RPY={rpy_CO}°')
    print(f'  Residual: mean={resid.mean()*100:.1f}cm max={resid.max()*100:.1f}cm')
    lidar_xyz_odom = (R_CO @ np.column_stack([lidar['px'], lidar['py'], lidar['pz']]).T).T + t_CO
    lidar['px_odom'] = lidar_xyz_odom[:, 0]
    lidar['py_odom'] = lidar_xyz_odom[:, 1]
    lidar['pz_odom'] = lidar_xyz_odom[:, 2]
    T_CO_rpy   = rpy_CO
    T_CO_t     = t_CO
    resid_mean = resid.mean() * 100
    resid_max  = resid.max() * 100
else:
    print('  [WARN] not enough LiDAR points for Procrustes')
    lidar['px_odom'] = lidar['px']; lidar['py_odom'] = lidar['py']
    lidar['pz_odom'] = lidar['pz']
    T_CO_rpy = [0,0,0]; T_CO_t = [0,0,0]
    resid_mean = resid_max = float('nan')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Contact Detection
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 1: Contact Detection')

# Build single convex hull from ground1-4 in robot-centric XY
def get_ground_pts(vi):
    pts = []
    for m in ['ground1', 'ground2', 'ground3', 'ground4']:
        xyz = vi.get_xyz(m)
        v = ~np.isnan(xyz).any(axis=1)
        if v.any():
            pts.append(vi.to_robot(xyz[v][0:1])[0, :2])
    return np.array(pts)

gnd_pts = get_ground_pts(vi)
try:
    hull_gnd = Delaunay(gnd_pts)
    print(f'  Ground hull built from {len(gnd_pts)} points')
except Exception as e:
    hull_gnd = None
    print(f'  [WARN] hull build failed: {e}')

def in_region(xy_mm, hull):
    if hull is None:
        return np.ones(len(xy_mm), dtype=bool)
    return hull.find_simplex(xy_mm) >= 0

LEG_MAP = [('LF', 'G1'), ('RF', 'G2'), ('RH', 'G3'), ('LH', 'G4')]
contact_results = {}

for leg, gm in LEG_MAP:
    hf = vi.foot_heights[leg]
    cf = vi.contact[leg]
    fxyz = vi.get_xyz(gm)
    fxy  = np.full((len(vi.t_traj), 2), np.nan)
    vf   = ~np.isnan(fxyz).any(1)
    if vf.any():
        fxy[vf] = vi.to_robot(fxyz[vf])[:, :2]
    rmask = np.zeros(len(vi.t_traj), dtype=bool)
    if vf.any():
        rmask[vf] = in_region(fxy[vf], hull_gnd)
    amask = mask_win_vi & rmask

    if amask.sum() < 10:
        print(f'  [{leg}] too few region-filtered frames ({amask.sum()}), skip metrics')
        contact_results[leg] = None
        continue

    ta  = vi.t_traj[amask]
    cva = cf[amask]
    cga = interp_gmo(gmo['t'], gmo[leg], ta, T_END)

    valid_v = ~np.isnan(cva)
    tv = ta[valid_v]; cv_v = cva[valid_v]; cg_v = cga[valid_v]
    if len(tv) == 0:
        contact_results[leg] = None
        continue

    TP = int(np.sum(cv_v & cg_v))
    TN = int(np.sum(~cv_v & ~cg_v))
    FP = int(np.sum(~cv_v & cg_v))
    FN = int(np.sum(cv_v & ~cg_v))
    N  = len(tv)
    acc  = (TP + TN) / N if N > 0 else float('nan')
    prec = TP / (TP + FP) if (TP + FP) > 0 else float('nan')
    rec  = TP / (TP + FN) if (TP + FN) > 0 else float('nan')
    f1   = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else float('nan')

    # Mean latency: time from VICON onset to first GMO onset after it
    latencies = []
    dt = np.diff(cv_v.astype(int), prepend=0)
    dt_g = np.diff(cg_v.astype(int), prepend=0)
    for i in np.where(dt == 1)[0]:
        future = np.where((dt_g == 1) & (np.arange(len(tv)) >= i))[0]
        if future.size > 0:
            latencies.append((tv[future[0]] - tv[i]) * 1000)
    mean_lat = float(np.mean(latencies)) if latencies else float('nan')

    contact_results[leg] = {
        'N': N, 'TP': TP, 'TN': TN, 'FP': FP, 'FN': FN,
        'acc': acc, 'prec': prec, 'rec': rec, 'f1': f1, 'mean_lat_ms': mean_lat,
        't': tv, 'cv': cv_v, 'cg': cg_v,
    }
    print(f'  [{leg}] N={N} TP={TP} TN={TN} FP={FP} FN={FN} '
          f'Acc={acc:.1%} Prec={prec:.3f} Rec={rec:.3f} F1={f1:.4f} '
          f'Lat={mean_lat:.1f}ms')

# ─── Contact plot ──────────────────────────────────────────────────────────────
COLORS_LEG = {'LF': 'steelblue', 'RF': 'darkorange', 'RH': 'forestgreen', 'LH': 'crimson'}
fig, axes = plt.subplots(4, 1, figsize=(13, 8), sharex=True)
for ax, (leg, gm) in zip(axes, LEG_MAP):
    hf = vi.foot_heights[leg][mask_win_vi]
    cf = vi.contact[leg][mask_win_vi]
    c_gmo = interp_gmo(gmo['t'], gmo[leg], t_win, T_END)
    color = COLORS_LEG[leg]
    ax.plot(t_win, hf * 1000, lw=0.8, color=color, label=f'{leg} height [mm]')
    _shade_contact(ax, t_win, cf,   color='tab:green', alpha=0.18)
    _shade_contact(ax, t_win, c_gmo, color='tab:orange', alpha=0.12)
    ax.axhline(CONTACT_THRESHOLD_M * 1000, color='k', ls='--', lw=0.8, alpha=0.5)
    _shade_window(ax, T_END, label=False)
    ax.set_ylabel(f'{leg} [mm]')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'Contact Detection — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_contact.png'), dpi=150)
plt.close(fig)
print('Saved fig_contact.png')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Inner EKF Analysis
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 2: Inner EKF')

# 2.1 Position
err_px = epx - vi_px_e; err_py = epy - vi_py_e; err_pz = epz - vi_pz_e
err_3d = np.sqrt(err_px**2 + err_py**2 + err_pz**2)
valid_p = ~np.isnan(err_3d)

metrics_pos = {
    'RMSE_X_cm':  rmse(err_px[valid_p]) * 100,
    'RMSE_Y_cm':  rmse(err_py[valid_p]) * 100,
    'RMSE_Z_cm':  rmse(err_pz[valid_p]) * 100,
    'RMSE_3D_cm': rmse(err_3d[valid_p]) * 100,
    'MAX_3D_cm':  float(np.max(err_3d[valid_p])) * 100,
    'final_EKF_x':   float(epx[-1]),
    'final_EKF_y':   float(epy[-1]),
    'final_VICON_x': float(vi_px_e[valid_p][-1]) if valid_p.any() else float('nan'),
    'final_VICON_y': float(vi_py_e[valid_p][-1]) if valid_p.any() else float('nan'),
}
print(f'  Pos RMSE: X={metrics_pos["RMSE_X_cm"]:.2f}cm '
      f'Y={metrics_pos["RMSE_Y_cm"]:.2f}cm '
      f'3D={metrics_pos["RMSE_3D_cm"]:.2f}cm')

# 2.2 Velocity - stable window (40-70% of T_END)
t_vel_s = T_END * 0.40; t_vel_e = T_END * 0.70
vmask = (et >= t_vel_s) & (et <= t_vel_e)
metrics_vel = {
    'RMSE_vx': rmse((evx - vi_vx_e)[vmask & ~np.isnan(vi_vx_e)]),
    'RMSE_vy': rmse((evy - vi_vy_e)[vmask & ~np.isnan(vi_vy_e)]),
    'RMSE_vz': rmse((evz - vi_vz_e)[vmask & ~np.isnan(vi_vz_e)]),
    'peak_vx': float(np.nanmax(np.abs(evx[vmask]))),
    't_vel_s': t_vel_s, 't_vel_e': t_vel_e,
}
print(f'  Vel RMSE (t={t_vel_s:.1f}-{t_vel_e:.1f}s): '
      f'vx={metrics_vel["RMSE_vx"]:.3f} vy={metrics_vel["RMSE_vy"]:.3f} m/s')

# 2.3 Attitude
valid_roll  = ~np.isnan(vi_roll_e)
valid_pitch = ~np.isnan(vi_pitch_e)
valid_yaw   = ~np.isnan(vi_yaw_e)
metrics_rpy = {
    'RMSE_roll_deg':  rmse((rpy_ekf_deg[:,0] - vi_roll_e)[valid_roll]),
    'RMSE_pitch_deg': rmse((rpy_ekf_deg[:,1] - vi_pitch_e)[valid_pitch]),
    'RMSE_yaw_deg':   rmse((rpy_ekf_deg[:,2] - vi_yaw_e)[valid_yaw]),
    'final_yaw_EKF_deg':   float(rpy_ekf_deg[-1, 2]),
    'final_yaw_VICON_deg': float(vi_yaw_e[valid_yaw][-1]) if valid_yaw.any() else float('nan'),
}
print(f'  RPY RMSE: roll={metrics_rpy["RMSE_roll_deg"]:.2f}° '
      f'pitch={metrics_rpy["RMSE_pitch_deg"]:.2f}° '
      f'yaw={metrics_rpy["RMSE_yaw_deg"]:.2f}°')

# 2.4 & 2.5 Bias
ba_mask = (ba['t'] >= 0) & (ba['t'] <= T_END)
bw_mask = (bw['t'] >= 0) & (bw['t'] <= T_END)
ss_start = T_END * 0.6
ba_ss = ba_mask & (ba['t'] >= ss_start)
bw_ss = bw_mask & (bw['t'] >= ss_start)
metrics_ba = {ax: {'init': float(ba[ax][ba_mask][0]) if ba_mask.any() else float('nan'),
                   'ss':   float(np.mean(ba[ax][ba_ss])) if ba_ss.any() else float('nan'),
                   'std':  float(np.std(ba[ax][ba_ss]))  if ba_ss.any() else float('nan')}
              for ax in ['x', 'y', 'z']}
metrics_bw = {ax: {'init': float(bw[ax][bw_mask][0]) if bw_mask.any() else float('nan'),
                   'ss':   float(np.mean(bw[ax][bw_ss])) if bw_ss.any() else float('nan'),
                   'std':  float(np.std(bw[ax][bw_ss]))  if bw_ss.any() else float('nan')}
              for ax in ['x', 'y', 'z']}

# ─── EKF plots ────────────────────────────────────────────────────────────────
# XY trajectory
fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(vi_px_e[valid_p], vi_py_e[valid_p], 'k-', lw=1, label='VICON', zorder=4)
sc = ax.scatter(epx, epy, c=et, cmap='viridis', s=2, lw=0)
ax.plot(epx[0], epy[0], 'go', ms=8, label='EKF start', zorder=5)
ax.plot(epx[-1], epy[-1], 'r^', ms=8, label='EKF end',   zorder=5)
plt.colorbar(sc, ax=ax, label='Time [s]')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'EKF XY Trajectory — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_xy.png'), dpi=150)
plt.close(fig)

# Position time series
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
for ax, key, lbl, vi_val in zip(axes, ['px','py','pz'],
                                 ['X','Y','Z'], [vi_px_e, vi_py_e, vi_pz_e]):
    cov = ekf[f'cov_p{lbl.lower()}'][ekf_mask]
    sigma = np.sqrt(np.abs(cov))
    ax.fill_between(et, ekf[key][ekf_mask] - 3*sigma,
                    ekf[key][ekf_mask] + 3*sigma, alpha=0.2, label='3σ')
    ax.plot(et, ekf[key][ekf_mask], lw=0.8, label='EKF')
    ax.plot(t_win[vi_valid], pos_vicon[vi_valid, 'XYZ'.index(lbl)],
            'k--', lw=1, alpha=0.7, label='VICON')
    _shade_window(ax, T_END, label=(lbl == 'X'))
    ax.set_ylabel(f'{lbl} [m]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Position — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_pos.png'), dpi=150)
plt.close(fig)

# Velocity
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
for ax, ek_val, vi_val, lbl in zip(axes,
                                    [evx, evy, evz],
                                    [vi_vx_e, vi_vy_e, vi_vz_e],
                                    ['vx', 'vy', 'vz']):
    ax.plot(et, ek_val, lw=0.8, label='EKF')
    valid_vi = ~np.isnan(vi_val)
    ax.plot(et[valid_vi], vi_val[valid_vi], 'k--', lw=1, alpha=0.7, label='VICON')
    ax.axvspan(t_vel_s, t_vel_e, color='skyblue', alpha=0.12, label='vel window')
    _shade_window(ax, T_END, label=False)
    ax.axhline(0, color='k', lw=0.5, ls='--', alpha=0.3)
    ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Velocity (body frame) — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_vel.png'), dpi=150)
plt.close(fig)

# RPY
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
vi_rpy_list = [vi_roll_e, vi_pitch_e, vi_yaw_e]
for ax, ek_val, vi_val, lbl in zip(axes,
                                    [rpy_ekf_deg[:,0], rpy_ekf_deg[:,1], rpy_ekf_deg[:,2]],
                                    vi_rpy_list, ['Roll', 'Pitch', 'Yaw']):
    ax.plot(et, ek_val, lw=0.8, label='EKF')
    valid_vi = ~np.isnan(vi_val)
    ax.plot(et[valid_vi], vi_val[valid_vi], 'k--', lw=1, alpha=0.7, label='VICON')
    _shade_window(ax, T_END, label=False)
    ax.axhline(0, color='k', lw=0.5, ls='--', alpha=0.3)
    ax.set_ylabel(f'{lbl} [°]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Attitude (RPY) — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_rpy.png'), dpi=150)
plt.close(fig)

# Bias
fig, axes = plt.subplots(2, 3, figsize=(14, 7))
for col, ax_col in enumerate(axes[0]):
    ax = ax_col
    ax_key = 'xyz'[col]
    ax.plot(ba['t'][ba_mask], ba[ax_key][ba_mask], lw=0.8)
    ax.axhline(0, color='k', lw=0.5, ls='--', alpha=0.4)
    _shade_window(ax, T_END)
    ax.set_title(f'Accel bias {ax_key}'); ax.set_ylabel(f'ba.{ax_key} [m/s²]')
    ax.grid(True, alpha=0.4)
for col, ax_col in enumerate(axes[1]):
    ax = ax_col
    ax_key = 'xyz'[col]
    ax.plot(bw['t'][bw_mask], bw[ax_key][bw_mask], lw=0.8)
    ax.axhline(0, color='k', lw=0.5, ls='--', alpha=0.4)
    _shade_window(ax, T_END)
    ax.set_title(f'Gyro bias {ax_key}'); ax.set_ylabel(f'bw.{ax_key} [rad/s]')
    ax.grid(True, alpha=0.4)
fig.suptitle(f'Inner EKF Bias — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_bias.png'), dpi=150)
plt.close(fig)
print('Saved EKF figures')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Outer Fusion Node
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 3: Outer Fusion Node')

odom_mask = (odom['t'] >= 0.0) & (odom['t'] <= T_END)
ot = odom['t'][odom_mask]
opx = odom['px'][odom_mask]; opy = odom['py'][odom_mask]
oqw = odom['qw'][odom_mask]; oqx = odom['qx'][odom_mask]
oqy = odom['qy'][odom_mask]; oqz = odom['qz'][odom_mask]
rpy_odom_deg = quat_to_rpy_deg(oqw, oqx, oqy, oqz)

# 3.1 Position
vi_px_o = interp_to(vi_t_v, pos_vicon[vi_valid, 0], ot)
vi_py_o = interp_to(vi_t_v, pos_vicon[vi_valid, 1], ot)
err_ox = opx - vi_px_o; err_oy = opy - vi_py_o
err_o2d = np.sqrt(err_ox**2 + err_oy**2)
valid_o = ~np.isnan(err_o2d)

# EKF on odom timestamps for comparison
ekf_px_o = interp_to(et, epx, ot)
ekf_py_o = interp_to(et, epy, ot)
err_oe2d = np.sqrt((opx - ekf_px_o)**2 + (opy - ekf_py_o)**2)
valid_oe = ~np.isnan(err_oe2d)

metrics_odom_pos = {
    'RMSE_2D_vs_VICON_cm': rmse(err_o2d[valid_o]) * 100,
    'MAX_2D_vs_VICON_cm':  float(np.max(err_o2d[valid_o])) * 100 if valid_o.any() else float('nan'),
    'RMSE_2D_vs_EKF_cm':   rmse(err_oe2d[valid_oe]) * 100,
    'final_odom_x': float(opx[-1]),
    'final_odom_y': float(opy[-1]),
}
print(f'  odom_mapping RMSE 2D vs VICON: {metrics_odom_pos["RMSE_2D_vs_VICON_cm"]:.2f}cm')

# 3.2 Yaw
vi_yaw_o = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 2]), ot)
ekf_yaw_o = interp_to(et, rpy_ekf_deg[:, 2], ot)
valid_oy  = ~np.isnan(vi_yaw_o)
valid_oye = ~np.isnan(ekf_yaw_o)
metrics_odom_yaw = {
    'RMSE_yaw_vs_VICON_deg': rmse((rpy_odom_deg[valid_oy, 2] - vi_yaw_o[valid_oy])),
    'RMSE_yaw_vs_EKF_deg':   rmse((rpy_odom_deg[valid_oye, 2] - ekf_yaw_o[valid_oye])),
    'final_yaw_odom_deg':  float(rpy_odom_deg[-1, 2]),
    'final_yaw_vicon_deg': float(vi_yaw_o[valid_oy][-1]) if valid_oy.any() else float('nan'),
}
print(f'  odom yaw RMSE vs VICON: {metrics_odom_yaw["RMSE_yaw_vs_VICON_deg"]:.2f}°')

# 3.3 fusion/bv
fv_mask = (fv['t'] >= 0.0) & (fv['t'] <= T_END)
fv_t = fv['t'][fv_mask]; fv_x = fv['x'][fv_mask]
fv_y = fv['y'][fv_mask]; fv_z = fv['z'][fv_mask]
fv_mag = np.sqrt(fv_x**2 + fv_y**2)
metrics_fv = {
    'mean_bv_x':    float(np.nanmean(fv_x)),
    'mean_bv_y':    float(np.nanmean(fv_y)),
    'mean_bv_mag':  float(np.nanmean(fv_mag)),
    'max_bv_mag':   float(np.nanmax(fv_mag)),
}
print(f'  fusion/bv mean mag: {metrics_fv["mean_bv_mag"]:.4f} m/s')

# odom velocity vs VICON
om_vx = odom['vx'][odom_mask]; om_vy = odom['vy'][odom_mask]
vi_vx_o = interp_to(vi_t_v, v_body_vi[vi_valid, 0], ot)
vi_vy_o = interp_to(vi_t_v, v_body_vi[vi_valid, 1], ot)
vm_vel = ~np.isnan(vi_vx_o)
metrics_odom_vel = {
    'RMSE_vx_vs_VICON': rmse((om_vx - vi_vx_o)[vm_vel]),
    'RMSE_vy_vs_VICON': rmse((om_vy - vi_vy_o)[vm_vel]),
}

# Plots
fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(vi_t_v, pos_vicon[vi_valid, 0], 'b-', lw=0.8, label='VICON X')
ax.plot(vi_t_v, pos_vicon[vi_valid, 1], 'r-', lw=0.8, label='VICON Y')
ax.plot(et, epx, 'b--', lw=0.8, label='EKF X'); ax.plot(et, epy, 'r--', lw=0.8, label='EKF Y')
ax.plot(ot, opx, 'b-.', lw=1, label='odom X'); ax.plot(ot, opy, 'r-.', lw=1, label='odom Y')
ax.set_xlabel('Time [s]'); ax.set_ylabel('Position [m]')
ax.set_title(f'Position Comparison — {DATE} {EXP_ID}')
ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_fusion_pos.png'), dpi=150)
plt.close(fig)

fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(pos_vicon[vi_valid, 0], pos_vicon[vi_valid, 1], 'k-', lw=1, label='VICON')
ax.scatter(epx, epy, c=et, cmap='Blues', s=1, label='EKF')
ax.scatter(opx, opy, c=ot, cmap='Oranges', s=1, label='odom_mapping')
ax.plot(vi_px_e[valid_p], vi_py_e[valid_p], 'k--', lw=1, alpha=0.6)
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'Fusion XY Trajectory — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_fusion_xy.png'), dpi=150)
plt.close(fig)

fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
axes[0].plot(fv_t, fv_x, lw=0.8, label='bv_x')
axes[0].plot(fv_t, fv_y, lw=0.8, label='bv_y')
axes[0].axhline(0, color='k', lw=0.5, ls='--', alpha=0.3)
axes[0].set_ylabel('fusion/bv [m/s]'); axes[0].legend(fontsize=7)
axes[0].set_title(f'Fusion/bv (velocity bias correction) — {DATE} {EXP_ID}')
axes[1].plot(et, evx, lw=0.8, label='EKF vx')
axes[1].plot(et, evy, lw=0.8, label='EKF vy')
vi_v_valid = ~np.isnan(vi_vx_e)
axes[1].plot(et[vi_v_valid], vi_vx_e[vi_v_valid], 'k--', lw=1, alpha=0.7, label='VICON vx')
axes[1].set_ylabel('Velocity [m/s]'); axes[1].set_xlabel('Time [s]')
axes[1].legend(fontsize=7); axes[1].grid(True, alpha=0.4)
for ax in axes: ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_fusion_bv.png'), dpi=150)
plt.close(fig)
print('Saved Fusion figures')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: LiDAR Input Quality
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 4: LiDAR Input Quality')

lidar_mask = (lidar['t'] >= 0.0) & (lidar['t'] <= T_END)
lt = lidar['t'][lidar_mask]
lpx_o = lidar['px_odom'][lidar_mask]
lpy_o = lidar['py_odom'][lidar_mask]

# Rate
if len(lt) > 1:
    lidar_rate = len(lt) / (lt[-1] - lt[0]) if (lt[-1] - lt[0]) > 0 else float('nan')
else:
    lidar_rate = float('nan')

# Jump detection
if len(lpx_o) > 1:
    dp = np.sqrt(np.diff(lpx_o)**2 + np.diff(lpy_o)**2)
    jumps = np.sum(dp > 0.1)
else:
    dp = np.array([]); jumps = 0

metrics_lidar = {
    'n_msgs': int(lidar_mask.sum()),
    'rate_hz': float(lidar_rate),
    'n_jumps_gt10cm': int(jumps),
}
print(f'  LiDAR: {metrics_lidar["n_msgs"]} msgs, {lidar_rate:.1f} Hz, {jumps} jumps>10cm')

fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(pos_vicon[vi_valid, 0], pos_vicon[vi_valid, 1], 'k-', lw=1, label='VICON', alpha=0.7)
ax.scatter(epx, epy, c=et, cmap='Blues', s=1, label='EKF')
ax.scatter(lpx_o, lpy_o, c=lt, cmap='Reds', s=1, label='LiDAR (odom frame)')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'LiDAR XY (odom frame) — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_lidar_xy.png'), dpi=150)
plt.close(fig)
print('Saved LiDAR figure')

# ═══════════════════════════════════════════════════════════════════════════════
# Save metrics JSON
# ═══════════════════════════════════════════════════════════════════════════════
contact_json = {}
for leg, res in contact_results.items():
    if res is None:
        contact_json[leg] = None
        continue
    contact_json[leg] = {k: v for k, v in res.items()
                         if k not in ('t', 'cv', 'cg')}

all_metrics = {
    'exp': EXP_ID, 'trial': TRIAL, 'date': DATE, 'T_END': T_END,
    'contact': contact_json,
    'position': metrics_pos,
    'velocity': metrics_vel,
    'attitude': metrics_rpy,
    'ba': metrics_ba, 'bw': metrics_bw,
    'odom_pos': metrics_odom_pos,
    'odom_yaw': metrics_odom_yaw,
    'fusion_bv': metrics_fv,
    'odom_vel': metrics_odom_vel,
    'lidar': metrics_lidar,
    'T_CO': {'RPY_deg': T_CO_rpy.tolist() if hasattr(T_CO_rpy,'tolist') else list(T_CO_rpy),
             't_m': T_CO_t.tolist() if hasattr(T_CO_t,'tolist') else list(T_CO_t),
             'resid_mean_cm': resid_mean, 'resid_max_cm': resid_max},
}

with open(os.path.join(RESULTS, 'metrics.json'), 'w') as f:
    json.dump(all_metrics, f, indent=2, default=str)
print('\nSaved metrics.json')
print(f'\n{"="*60}')
print(f'Analysis complete — {EXP_ID} ({TRIAL})')
print(f'  Pos 3D RMSE:  {metrics_pos["RMSE_3D_cm"]:.2f} cm')
print(f'  EKF yaw RMSE: {metrics_rpy["RMSE_yaw_deg"]:.2f}°')
print(f'  odom 2D RMSE: {metrics_odom_pos["RMSE_2D_vs_VICON_cm"]:.2f} cm')
print(f'  T_END:        {T_END:.2f} s')
