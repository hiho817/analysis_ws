#!/usr/bin/env python3
"""
plot_comparison.py — 四路對比圖 (walk_2m_01_01)
=============================
位置 X/Y/Z  +  速度 X/Y/Z，四條線：
  - VICON (ground truth)
  - Inner EKF (full fusion bag)
  - odom_mapping (outer fusion)
  - Ablation EKF (leg-only, no LiDAR)

以及 fusion/bv 補正量說明圖。

NOTE: source ~/corgi_ws/corgi_ros2_ws/install/setup.bash before running.
"""

import os, sys
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
from corgi_analysis.bag_loader   import load_fusion_bag, load_inner_ekf_bag, load_legacy_bag

# ─── Paths ─────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
BAG_FUSION = os.path.join(BASE, '..', 'bags',
                           'odom_fusion20260512_222613_trimmed',
                           'odom_fusion20260512_222613_trimmed_0.db3')
BAG_ABLAT  = os.path.join(BASE, '..', 'bags',
                           'ablation_leg_only_20260514_020024',
                           'ablation_leg_only_20260514_020024_0.db3')
BAG_LEGACY = os.path.join(BASE, '..', 'bags',
                           'legacy_20260514_172734',
                           'legacy_20260514_172734_0.db3')
VICON_CSV  = os.path.join(BASE, '..', 'vicon', 'walk_2m_01_01.csv')
OUT        = BASE

# ─── Colours ──────────────────────────────────────────────────────────────────
C_VI   = '#444444'       # VICON — dark grey
C_EKF  = '#1E88E5'       # Inner EKF (full) — blue
C_ODOM = '#43A047'       # odom_mapping — green
C_ABL  = '#E53935'       # Ablation EKF — red
C_LEG  = '#8E24AA'       # Legacy odometry — purple
C_BV   = '#FB8C00'       # fusion/bv — orange

LW = 1.4

# ─── Helpers ──────────────────────────────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)

# ─── Load data ─────────────────────────────────────────────────────────────────
print('Loading VICON ...')
vi = load_vicon(VICON_CSV, contact_threshold_m=0.012,
                ground_markers=['groundB1','groundB2','groundA3','groundA4'])

print('Loading full-fusion bag ...')
fbag = load_fusion_bag(BAG_FUSION, rate=1.0)
fekf = fbag['ekf']; fodom = fbag['odom']; ffv = fbag['fv']

print('Loading ablation bag ...')
abag = load_inner_ekf_bag(BAG_ABLAT, rate=2.0)
aekf = abag['ekf']

print('Loading legacy bag ...')
lbag = load_legacy_bag(BAG_LEGACY, rate=2.0)
lpos = lbag['pos']; lvel = lbag['vel']

# ─── Common time window ────────────────────────────────────────────────────────
T_END = min(vi.t_trigger_end,
            fbag['t_trigger_end'],
            abag['t_trigger_end'],
            lpos['t'][-1] if len(lpos['t']) else 9999.)
print(f'Common window: t in [0, {T_END:.2f}] s')

# ─── VICON arrays on common window ────────────────────────────────────────────
mask_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_vi   = vi.t_traj[mask_vi]
pos_vi = vi.pos_m[mask_vi]       # (N,3) X Y Z in metres
vel_vi = vi.v_body[mask_vi]      # (N,3) vx vy vz in m/s

vi_ok  = ~np.isnan(pos_vi).any(1)

# ─── Inner EKF arrays (full fusion bag) ────────────────────────────────────────
fe_mask = (fekf['t'] >= 0.0) & (fekf['t'] <= T_END)
fe_t    = fekf['t'][fe_mask]
fe_px   = fekf['px'][fe_mask]; fe_py = fekf['py'][fe_mask]; fe_pz = fekf['pz'][fe_mask]
fe_vx   = fekf['vx'][fe_mask]; fe_vy = fekf['vy'][fe_mask]; fe_vz = fekf['vz'][fe_mask]

# ─── odom_mapping ─────────────────────────────────────────────────────────────
om_mask = (fodom['t'] >= 0.0) & (fodom['t'] <= T_END)
om_t    = fodom['t'][om_mask]
om_px   = fodom['px'][om_mask]; om_py = fodom['py'][om_mask]; om_pz = fodom['pz'][om_mask]
# odom_mapping twist is all-zero — compute velocity from central finite differences
def _central_diff(t, x):
    """Central finite difference; edges use forward/backward diff."""
    v = np.empty_like(x)
    v[1:-1]  = (x[2:] - x[:-2]) / (t[2:] - t[:-2])
    v[0]     = (x[1] - x[0]) / (t[1] - t[0]) if len(t) > 1 else 0.0
    v[-1]    = (x[-1] - x[-2]) / (t[-1] - t[-2]) if len(t) > 1 else 0.0
    return v
om_vx = _central_diff(om_t, om_px)
om_vy = _central_diff(om_t, om_py)
om_vz = _central_diff(om_t, om_pz)

# ─── Ablation EKF arrays ──────────────────────────────────────────────────────
ae_mask = (aekf['t'] >= 0.0) & (aekf['t'] <= T_END)
ae_t    = aekf['t'][ae_mask]
ae_px   = aekf['px'][ae_mask]; ae_py = aekf['py'][ae_mask]; ae_pz = aekf['pz'][ae_mask]
ae_vx   = aekf['vx'][ae_mask]; ae_vy = aekf['vy'][ae_mask]; ae_vz = aekf['vz'][ae_mask]

# ─── Legacy odometry arrays ───────────────────────────────────────────────────
le_mask = (lpos['t'] >= 0.0) & (lpos['t'] <= T_END)
le_t    = lpos['t'][le_mask]
le_px   = lpos['x'][le_mask]; le_py = lpos['y'][le_mask]; le_pz = lpos['z'][le_mask]
lv_mask = (lvel['t'] >= 0.0) & (lvel['t'] <= T_END)
lv_t    = lvel['t'][lv_mask]
le_vx   = lvel['x'][lv_mask]; le_vy = lvel['y'][lv_mask]; le_vz = lvel['z'][lv_mask]

# ─── fusion/bv ────────────────────────────────────────────────────────────────
fv_mask = (ffv['t'] >= 0.0) & (ffv['t'] <= T_END)
fv_t    = ffv['t'][fv_mask]
fv_x    = ffv['x'][fv_mask]; fv_y = ffv['y'][fv_mask]; fv_z = ffv['z'][fv_mask]
fv_mag  = np.sqrt(fv_x**2 + fv_y**2 + fv_z**2)

# ─── VICON interpolated onto each time grid ────────────────────────────────────
def vi_on(tgt_t, axis):
    """VICON position on target time grid, 0=X 1=Y 2=Z"""
    src = pos_vi[:, axis]
    return interp_to(t_vi[vi_ok], src[vi_ok], tgt_t)

def viv_on(tgt_t, axis):
    """VICON velocity on target time grid"""
    src = vel_vi[:, axis]
    vok = ~np.isnan(src)
    return interp_to(t_vi[vok], src[vok], tgt_t)

# ─── RMSE helper (used by figures and summary) ───────────────────────────────
def rmse(a, b):
    d = np.asarray(a) - np.asarray(b)
    v = d[~np.isnan(d)]
    return float(np.sqrt(np.mean(v**2))) if len(v) else float('nan')

# ══════════════════════════════════════════════════════════════════════════════
# Figure 1: Position X / Y / Z
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
fig.suptitle('Position Comparison  (VICON / Inner EKF / odom_mapping / Ablation EKF)',
             fontsize=12, fontweight='bold')

labels = ['X  [m]', 'Y  [m]', 'Z  [m]']
for ai, ax in enumerate(axes):
    ax.plot(t_vi[vi_ok],           pos_vi[vi_ok, ai],  lw=LW+0.4, color=C_VI,   label='VICON (GT)',       zorder=5)
    ax.plot(fe_t,                  [fe_px, fe_py, fe_pz][ai],  lw=LW, color=C_EKF,  label='Inner EKF (fusion)',  zorder=4)
    ax.plot(om_t,                  [om_px, om_py, om_pz][ai],  lw=LW, color=C_ODOM, label='odom_mapping',   zorder=3, ls='--')
    ax.plot(ae_t,                  [ae_px, ae_py, ae_pz][ai],  lw=LW, color=C_ABL,  label='Ablation EKF',  zorder=2, ls=':')
    ax.plot(le_t,                  [le_px, le_py, le_pz][ai],  lw=LW, color=C_LEG,  label='Legacy odom',   zorder=1, ls=(0,(4,2)))
    ax.set_ylabel(labels[ai]); ax.grid(True, alpha=0.35)
    if ai == 0: ax.legend(loc='upper left', fontsize=8, ncol=2)

axes[-1].set_xlabel('Time  [s]')
plt.tight_layout()
out_path = os.path.join(OUT, 'fig_comparison_position.png')
plt.savefig(out_path, dpi=150)
print(f'Saved: {out_path}')
plt.close()

# ══════════════════════════════════════════════════════════════════════════════
# Figure 2: Velocity X / Y / Z
# ══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
fig.suptitle('Velocity Comparison  (VICON / Inner EKF / odom_mapping / Ablation EKF)',
             fontsize=12, fontweight='bold')

vel_labels = ['vx  [m/s]', 'vy  [m/s]', 'vz  [m/s]']
for ai, ax in enumerate(axes):
    vi_vok = ~np.isnan(vel_vi[:, ai])
    ax.plot(t_vi[vi_vok], vel_vi[vi_vok, ai], lw=LW+0.4, color=C_VI,   label='VICON (GT)',       zorder=5)
    ax.plot(fe_t,  [fe_vx, fe_vy, fe_vz][ai], lw=LW,     color=C_EKF,  label='Inner EKF (fusion)',  zorder=4)
    ax.plot(om_t,  [om_vx, om_vy, om_vz][ai], lw=LW,     color=C_ODOM, label='odom_mapping',     zorder=3, ls='--')
    ax.plot(ae_t,  [ae_vx, ae_vy, ae_vz][ai], lw=LW,     color=C_ABL,  label='Ablation EKF',     zorder=2, ls=':')
    ax.plot(lv_t,  [le_vx, le_vy, le_vz][ai], lw=LW,     color=C_LEG,  label='Legacy odom',      zorder=1, ls=(0,(4,2)))
    ax.axvspan(12, 17, color='gold', alpha=0.18, label='RMSE window 12-17 s')
    ax.set_ylabel(vel_labels[ai]); ax.grid(True, alpha=0.35)
    if ai == 0: ax.legend(loc='upper left', fontsize=8, ncol=2)
    # ── velocity RMSE text box ──
    _ekf_r = rmse([fe_vx, fe_vy, fe_vz][ai], viv_on(fe_t,  ai))
    _abl_r = rmse([ae_vx, ae_vy, ae_vz][ai], viv_on(ae_t,  ai))
    _leg_r = rmse([le_vx, le_vy, le_vz][ai], viv_on(lv_t, ai))
    _txt = (f'RMSE (full range)\n'
            f'EKF     : {_ekf_r*100:.1f} cm/s\n'
            f'Ablation: {_abl_r*100:.1f} cm/s\n'
            f'Legacy  : {_leg_r*100:.1f} cm/s')
    ax.text(0.985, 0.97, _txt, transform=ax.transAxes,
            ha='right', va='top', fontsize=7.5, family='monospace',
            bbox=dict(boxstyle='round,pad=0.35', fc='white', ec='#aaaaaa', alpha=0.88))

axes[-1].set_xlabel('Time  [s]')
plt.tight_layout()
out_path = os.path.join(OUT, 'fig_comparison_velocity.png')
plt.savefig(out_path, dpi=150)
print(f'Saved: {out_path}')
plt.close()

# ══════════════════════════════════════════════════════════════════════════════
# Figure 3: fusion/bv correction explanation
# ══════════════════════════════════════════════════════════════════════════════
# Ablation position error vs VICON (interpolated to ablation time grid)
abl_err_x = ae_px - vi_on(ae_t, 0)
abl_err_y = ae_py - vi_on(ae_t, 1)
abl_err_z = ae_pz - vi_on(ae_t, 2)
abl_err_3d = np.sqrt(np.where(np.isnan(abl_err_x), np.nan,
                     abl_err_x**2 + abl_err_y**2 + abl_err_z**2))

# Full EKF position error vs VICON
fek_err_x = fe_px - vi_on(fe_t, 0)
fek_err_y = fe_py - vi_on(fe_t, 1)
fek_err_z = fe_pz - vi_on(fe_t, 2)
fek_err_3d = np.sqrt(np.where(np.isnan(fek_err_x), np.nan,
                     fek_err_x**2 + fek_err_y**2 + fek_err_z**2))

# Cumulative |bv| integral  ~= cumulative position correction applied
if len(fv_t) > 1:
    dt_fv = np.diff(fv_t, prepend=fv_t[0])
    cum_bv_correction = np.cumsum(fv_mag * dt_fv)   # [m]
else:
    cum_bv_correction = np.zeros_like(fv_t)

fv_mean_mag = fv_mag.mean()

fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
fig.suptitle('Why does tiny fusion/bv still correct large position drift?\n'
             'Small bv integrates over time  ->  large cumulative position correction', fontsize=11)

# Panel 1: bv components
ax = axes[0]
ax.plot(fv_t, fv_x*1e3, color=C_BV,   lw=LW, label='bv_x')
ax.plot(fv_t, fv_y*1e3, color='#8E24AA', lw=LW, label='bv_y')
ax.plot(fv_t, fv_z*1e3, color='#00897B', lw=LW, label='bv_z')
ax.plot(fv_t, fv_mag*1e3, color='k', lw=1.2, ls='--', label='|bv|')
ax.axhline(0, color='grey', lw=0.6)
ax.set_ylabel('fusion/bv  [mm/s]')
ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.35)
ax.set_title(f'fusion/bv velocity bias correction  (mean |bv| = {fv_mean_mag*1e3:.1f} mm/s)', fontsize=9)

# Panel 2: cumulative bv correction vs actual position error reduction
ax = axes[1]
ax.fill_between(ae_t, 0, abl_err_3d*100, alpha=0.25, color=C_ABL, label='Ablation 3D error [cm]')
ax.fill_between(fe_t, 0, fek_err_3d*100, alpha=0.25, color=C_EKF, label='Full fusion 3D error [cm]')
ax.plot(ae_t, abl_err_3d*100, color=C_ABL, lw=LW, ls=':')
ax.plot(fe_t, fek_err_3d*100, color=C_EKF, lw=LW)
ax2 = ax.twinx()
ax2.plot(fv_t, cum_bv_correction*100, color=C_BV, lw=1.5, ls='--', label='Cumulative |bv|·dt [cm]')
ax2.set_ylabel('Cumulative |bv|·dt  [cm]', color=C_BV)
ax2.tick_params(axis='y', labelcolor=C_BV)
ax.set_ylabel('3D position error  [cm]')
ax.grid(True, alpha=0.35)
lines1, labs1 = ax.get_legend_handles_labels()
lines2, labs2 = ax2.get_legend_handles_labels()
ax.legend(lines1+lines2, labs1+labs2, loc='upper left', fontsize=8)
ax.set_title('3D Position Error  vs  Cumulative bv Correction', fontsize=9)

# Panel 3: Y-axis breakdown (main drift axis)
ax = axes[2]
ax.plot(ae_t, abl_err_y*100, color=C_ABL, lw=LW, ls=':', label='Ablation Y error [cm]')
ax.plot(fe_t, fek_err_y*100, color=C_EKF, lw=LW,        label='Full fusion Y error [cm]')
if len(fv_t) > 1:
    dt_fv2 = np.diff(fv_t, prepend=fv_t[0])
    cum_bvy = np.cumsum(fv_y * dt_fv2)
else:
    cum_bvy = np.zeros_like(fv_t)
ax3 = ax.twinx()
ax3.plot(fv_t, cum_bvy*100, color=C_BV, lw=1.5, ls='--', label='Cumulative bv_y·dt [cm]')
ax3.set_ylabel('Cumulative bv_y·dt  [cm]', color=C_BV)
ax3.tick_params(axis='y', labelcolor=C_BV)
ax.axhline(0, color='grey', lw=0.6)
ax.set_ylabel('Y position error  [cm]'); ax.set_xlabel('Time  [s]')
ax.grid(True, alpha=0.35)
lines1, labs1 = ax.get_legend_handles_labels()
lines2, labs2 = ax3.get_legend_handles_labels()
ax.legend(lines1+lines2, labs1+labs2, loc='upper left', fontsize=8)
ax.set_title('Y-axis breakdown  (dominant lateral drift direction)', fontsize=9)

plt.tight_layout()
out_path = os.path.join(OUT, 'fig_bv_correction_explain.png')
plt.savefig(out_path, dpi=150)
print(f'Saved: {out_path}')
plt.close()

# ══════════════════════════════════════════════════════════════════════════════
# Print RMSE summary
# ══════════════════════════════════════════════════════════════════════════════
vw = (fe_t >= 12.) & (fe_t <= 17.)
aw = (ae_t >= 12.) & (ae_t <= 17.)
lw = (lv_t >= 12.) & (lv_t <= 17.)
print('\n' + '='*60)
print('RMSE Summary  (walk_2m_01_01)')
print('='*60)
print(f"  EKF pos X: {rmse(fe_px, vi_on(fe_t,0))*100:.2f} cm")
print(f"  EKF pos Y: {rmse(fe_py, vi_on(fe_t,1))*100:.2f} cm")
print(f"  EKF pos Z: {rmse(fe_pz, vi_on(fe_t,2))*100:.2f} cm")
print(f"  EKF vel vx (12-17s): {rmse(fe_vx[vw], viv_on(fe_t,0)[vw])*100:.1f} cm/s")
print(f"  Ablation pos X: {rmse(ae_px, vi_on(ae_t,0))*100:.2f} cm")
print(f"  Ablation pos Y: {rmse(ae_py, vi_on(ae_t,1))*100:.2f} cm")
print(f"  Ablation pos Z: {rmse(ae_pz, vi_on(ae_t,2))*100:.2f} cm")
print(f"  Ablation vel vx (12-17s): {rmse(ae_vx[aw], viv_on(ae_t,0)[aw])*100:.1f} cm/s")
print(f"  Legacy pos X:  {rmse(le_px, vi_on(le_t,0))*100:.2f} cm")
print(f"  Legacy pos Y:  {rmse(le_py, vi_on(le_t,1))*100:.2f} cm")
print(f"  Legacy pos Z:  {rmse(le_pz, vi_on(le_t,2))*100:.2f} cm")
print(f"  Legacy vel vx (12-17s): {rmse(le_vx[lw], viv_on(lv_t,0)[lw])*100:.1f} cm/s")
print(f"\n  fusion/bv mean_mag: {fv_mag.mean()*1e3:.2f} mm/s")
if len(cum_bv_correction):
    print(f"  Cumulative |bv|·dt (total): {cum_bv_correction[-1]*100:.2f} cm")
print('='*60)
