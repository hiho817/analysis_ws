#!/usr/bin/env python3
"""
CORGI Experiment Analysis — 20260514 exp3
walk_2m_01_obs_odometry_legacy (Information Filter)

Analyses position and velocity vs VICON ground truth.
Topics: /odometry/legacy/position, /odometry/legacy/velocity

NOTE: source ~/corgi_ws/corgi_ros2_ws/install/setup.bash before running.
"""

import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_legacy_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
RESULTS   = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'legacy_odom20260514_232823',
                         'legacy_odom20260514_232823_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'EXP_06.csv')
TRIAL     = 'walk_2m_01_obs_odometry_legacy'
DATE      = '20260514'
EXP_ID    = 'exp6'
CONTACT_THRESHOLD_M = 0.015   # 15 mm

# ─── Helpers ──────────────────────────────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)

def rmse(d):
    v = np.asarray(d)
    return float(np.sqrt(np.nanmean(v ** 2)))

def _shade_window(ax, t_end, label=True):
    if t_end is not None:
        ax.axvspan(0, t_end, color='gold', alpha=0.08,
                   label='trigger window' if label else None)

# ─── Load ─────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
print('='*60)
vi  = load_vicon(VICON_CSV,
                 contact_threshold_m=CONTACT_THRESHOLD_M,
                 ground_markers=['ground1', 'ground2', 'ground3', 'ground4'])
bag = load_legacy_bag(BAG_DB, rate=1.0)
pos = bag['pos']; vel = bag['vel']

T_END = min(vi.t_trigger_end, bag['t_trigger_end'])
print(f'\nAnalysis window: t ∈ [0, {T_END:.2f}] s')

# VICON window
mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_win       = vi.t_traj[mask_win_vi]
pos_vicon   = vi.pos_m[mask_win_vi]
v_body_vi   = vi.v_body[mask_win_vi]

# Legacy pos/vel window
pos_mask = (pos['t'] >= 0.0) & (pos['t'] <= T_END)
vel_mask = (vel['t'] >= 0.0) & (vel['t'] <= T_END)
pt = pos['t'][pos_mask]; px = pos['x'][pos_mask]
py = pos['y'][pos_mask]; pz = pos['z'][pos_mask]
vt = vel['t'][vel_mask]; vx = vel['x'][vel_mask]
vy = vel['y'][vel_mask]; vz = vel['z'][vel_mask]

# VICON valid
vi_valid = ~np.isnan(pos_vicon).any(1)
vi_t_v   = t_win[vi_valid]

# Flip VICON Y: Legacy +Y = right (world integration frame), VICON +Y = left → negate position only
pos_vicon[:, 1] = -pos_vicon[:, 1]

# Interpolate VICON to legacy timestamps
vi_px_p = interp_to(vi_t_v, pos_vicon[vi_valid, 0], pt)
vi_py_p = interp_to(vi_t_v, pos_vicon[vi_valid, 1], pt)
vi_pz_p = interp_to(vi_t_v, pos_vicon[vi_valid, 2], pt)
vi_vx_v = interp_to(vi_t_v, v_body_vi[vi_valid, 0], vt)
vi_vy_v = interp_to(vi_t_v, v_body_vi[vi_valid, 1], vt)
vi_vz_v = interp_to(vi_t_v, v_body_vi[vi_valid, 2], vt)

# ═══════════════════════════════════════════════════════════════════════════════
# Position Metrics
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nPosition Analysis')
err_px = px - vi_px_p; err_py = py - vi_py_p; err_pz = pz - vi_pz_p
err_3d = np.sqrt(err_px**2 + err_py**2 + err_pz**2)
valid_p = ~np.isnan(err_3d)

metrics_pos = {
    'RMSE_X_cm':  rmse(err_px[valid_p]) * 100,
    'RMSE_Y_cm':  rmse(err_py[valid_p]) * 100,
    'RMSE_Z_cm':  rmse(err_pz[valid_p]) * 100,
    'RMSE_3D_cm': rmse(err_3d[valid_p]) * 100,
    'MAX_3D_cm':  float(np.max(err_3d[valid_p])) * 100 if valid_p.any() else float('nan'),
    'final_pos_x':   float(px[-1]) if len(px) > 0 else float('nan'),
    'final_pos_y':   float(py[-1]) if len(py) > 0 else float('nan'),
    'final_VICON_x': float(vi_px_p[valid_p][-1]) if valid_p.any() else float('nan'),
    'final_VICON_y': float(vi_py_p[valid_p][-1]) if valid_p.any() else float('nan'),
}
print(f'  Pos RMSE: X={metrics_pos["RMSE_X_cm"]:.2f}cm '
      f'Y={metrics_pos["RMSE_Y_cm"]:.2f}cm '
      f'3D={metrics_pos["RMSE_3D_cm"]:.2f}cm')

# ═══════════════════════════════════════════════════════════════════════════════
# Velocity Metrics
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nVelocity Analysis')
t_vel_s = T_END * 0.40; t_vel_e = T_END * 0.70
vmask = (vt >= t_vel_s) & (vt <= t_vel_e)
valid_vx = ~np.isnan(vi_vx_v)
valid_vy = ~np.isnan(vi_vy_v)
metrics_vel = {
    'RMSE_vx': rmse((vx - vi_vx_v)[vmask & valid_vx]),
    'RMSE_vy': rmse((vy - vi_vy_v)[vmask & valid_vy]),
    'RMSE_vz': rmse((vz - vi_vz_v)[vmask & ~np.isnan(vi_vz_v)]),
    'peak_vx': float(np.nanmax(np.abs(vx[vmask]))) if vmask.any() else float('nan'),
    't_vel_s': t_vel_s, 't_vel_e': t_vel_e,
}
print(f'  Vel RMSE (t={t_vel_s:.1f}-{t_vel_e:.1f}s): '
      f'vx={metrics_vel["RMSE_vx"]:.3f} vy={metrics_vel["RMSE_vy"]:.3f} m/s')

# ═══════════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════════
# XY Trajectory
fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(vi_t_v, pos_vicon[vi_valid, 0],
        vi_t_v, pos_vicon[vi_valid, 1], 'k-', lw=1, label='VICON', alpha=0.6)
sc = ax.scatter(px, py, c=pt, cmap='viridis', s=3, lw=0, label='Legacy')
ax.plot(px[0], py[0], 'go', ms=8, label='start', zorder=5)
ax.plot(px[-1], py[-1], 'r^', ms=8, label='end',   zorder=5)
# VICON as line
ax.plot(vi_t_v, pos_vicon[vi_valid, 0], 'k--', lw=1, alpha=0.5)
plt.colorbar(sc, ax=ax, label='Time [s]')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'Legacy Odometry XY — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_pos_xy.png'), dpi=150)
plt.close(fig)

# -- Better XY plot using actual X vs Y
fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(pos_vicon[vi_valid, 0], pos_vicon[vi_valid, 1],
        'k-', lw=1.5, label='VICON', zorder=4)
sc = ax.scatter(px, py, c=pt, cmap='viridis', s=3, lw=0, label='Legacy')
ax.plot(px[0], py[0], 'go', ms=8, label='start', zorder=5)
ax.plot(px[-1], py[-1], 'r^', ms=8, label='end',   zorder=5)
plt.colorbar(sc, ax=ax, label='Time [s]')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'Legacy XY Trajectory — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_traj_xy.png'), dpi=150)
plt.close(fig)

# Position time series
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
vi_vals = [pos_vicon[vi_valid, 0], pos_vicon[vi_valid, 1], pos_vicon[vi_valid, 2]]
leg_vals = [px, py, pz]
labels = ['X', 'Y', 'Z']
for ax, lbl, lv, vi_v in zip(axes, labels, leg_vals, vi_vals):
    ax.plot(pt, lv, lw=0.8, label='Legacy')
    ax.plot(vi_t_v, vi_v, 'k--', lw=1, alpha=0.7, label='VICON')
    _shade_window(ax, T_END, label=(lbl == 'X'))
    ax.set_ylabel(f'{lbl} [m]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'Legacy Position — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_pos_time.png'), dpi=150)
plt.close(fig)

# Velocity time series
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
vi_vvals = [vi_vx_v, vi_vy_v, vi_vz_v]
leg_vvals = [vx, vy, vz]
for ax, lbl, lv, vi_v in zip(axes, ['vx', 'vy', 'vz'], leg_vvals, vi_vvals):
    ax.plot(vt, lv, lw=0.8, label='Legacy')
    valid_vi = ~np.isnan(vi_v)
    ax.plot(vt[valid_vi], vi_v[valid_vi], 'k--', lw=1, alpha=0.7, label='VICON')
    ax.axvspan(t_vel_s, t_vel_e, color='skyblue', alpha=0.12, label='vel window')
    _shade_window(ax, T_END, label=False)
    ax.axhline(0, color='k', lw=0.5, ls='--', alpha=0.3)
    ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'Legacy Velocity — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_vel_time.png'), dpi=150)
plt.close(fig)
print('Saved figures')

# ═══════════════════════════════════════════════════════════════════════════════
# Save metrics
# ═══════════════════════════════════════════════════════════════════════════════
all_metrics = {
    'exp': EXP_ID, 'trial': TRIAL, 'date': DATE, 'T_END': T_END,
    'position': metrics_pos,
    'velocity': metrics_vel,
}
with open(os.path.join(RESULTS, 'metrics.json'), 'w') as f:
    json.dump(all_metrics, f, indent=2, default=str)
print('\nSaved metrics.json')
print(f'\n{"="*60}')
print(f'Analysis complete — {EXP_ID} ({TRIAL})')
print(f'  Pos 3D RMSE:  {metrics_pos["RMSE_3D_cm"]:.2f} cm')
print(f'  Vel RMSE vx:  {metrics_vel["RMSE_vx"]:.3f} m/s')
print(f'  T_END:        {T_END:.2f} s')
