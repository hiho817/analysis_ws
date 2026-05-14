#!/usr/bin/env python3
"""
Ablation Analysis — walk_2m_01 (Inner EKF only, NO LiDAR/fusion)

Loads the ablation bag (leg_odom only) and computes Steps 1-2:
  Step 1: Contact Detection
  Step 2: Inner EKF (position, velocity, attitude, ba, bw)

Saves to ablation_metrics.json for comparison with full-fusion metrics.json.
"""

import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial import ConvexHull, Delaunay
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_inner_ekf_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
RESULTS   = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'ablation_leg_only_20260513_192159',
                         'ablation_leg_only_20260513_192159_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'walk_2m_01.csv')
TRIAL     = 'walk_2m_01_ablation_leg_only'
DATE      = '20260513'
CONTACT_THRESHOLD_M = 0.012

# ─── Helpers (same as main analyze.py) ────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)

def quat_to_rpy_deg(qw, qx, qy, qz):
    r = Rotation.from_quat(np.column_stack([qx, qy, qz, qw]))
    return np.degrees(r.as_euler('ZYX')[:, ::-1])

def rmse(d):
    v = np.asarray(d)
    v = v[~np.isnan(v)]
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else float('nan')

def _shade(ax, t, c, color, alpha, hatch=None):
    prev = False; t0 = 0
    for i in range(len(t)):
        if c[i] and not prev: t0 = t[i]
        elif not c[i] and prev:
            kw = dict(color=color, alpha=alpha, lw=0)
            if hatch: kw['hatch'] = hatch
            ax.axvspan(t0, t[i-1], **kw)
        prev = bool(c[i])
    if prev:
        kw = dict(color=color, alpha=alpha, lw=0)
        if hatch: kw['hatch'] = hatch
        ax.axvspan(t0, t[-1], **kw)

def in_region(xy_mm, hA, hB):
    return (hA.find_simplex(xy_mm) >= 0) | (hB.find_simplex(xy_mm) >= 0)

def interp_gmo(gmo_t, gmo_leg, t_tgt, T_END):
    mk = (gmo_t >= -0.5) & (gmo_t <= T_END + 0.5)
    t_g = gmo_t[mk]; c_g = gmo_leg[mk].astype(float)
    if len(t_g) < 2: return np.zeros(len(t_tgt), dtype=bool)
    return interp1d(t_g, c_g, kind='nearest', bounds_error=False, fill_value=0.)(t_tgt) > 0.5

# ─── Load ─────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
print('='*60)
vi  = load_vicon(VICON_CSV, contact_threshold_m=CONTACT_THRESHOLD_M,
                 ground_markers=['groundB1','groundB2','groundA3','groundA4'])
bag = load_inner_ekf_bag(BAG_DB, rate=2.0)
ekf = bag['ekf']; ba = bag['ba']; bw = bag['bw']; gmo = bag['gmo']

T_END = min(vi.t_trigger_end, bag['t_trigger_end'])
print(f'\nAblation window: t ∈ [0, {T_END:.2f}] s')

mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_win = vi.t_traj[mask_win_vi]
pos_vicon  = vi.pos_m[mask_win_vi]
v_body_vi  = vi.v_body[mask_win_vi]
rpy_vicon  = vi.rpy[mask_win_vi]

ekf_mask = (ekf['t'] >= 0.0) & (ekf['t'] <= T_END)
et  = ekf['t'][ekf_mask]
epx = ekf['px'][ekf_mask]; epy = ekf['py'][ekf_mask]; epz = ekf['pz'][ekf_mask]
evx = ekf['vx'][ekf_mask]; evy = ekf['vy'][ekf_mask]; evz = ekf['vz'][ekf_mask]
eqw = ekf['qw'][ekf_mask]; eqx = ekf['qx'][ekf_mask]
eqy = ekf['qy'][ekf_mask]; eqz = ekf['qz'][ekf_mask]
ecov_px = ekf['cov_px'][ekf_mask]; ecov_py = ekf['cov_py'][ekf_mask]; ecov_pz = ekf['cov_pz'][ekf_mask]
rpy_ekf_deg = quat_to_rpy_deg(eqw, eqx, eqy, eqz)

vi_valid = ~np.isnan(pos_vicon).any(1)
vi_t_v = t_win[vi_valid]
rpy_vi_valid = ~np.isnan(rpy_vicon).any(1)
rpy_vi_t_v = t_win[rpy_vi_valid]

def vi2ekf(src): return interp_to(vi_t_v, src[vi_valid], et)

vi_px_e = vi2ekf(pos_vicon[:,0]); vi_py_e = vi2ekf(pos_vicon[:,1]); vi_pz_e = vi2ekf(pos_vicon[:,2])
vi_vx_e = vi2ekf(v_body_vi[:,0]); vi_vy_e = vi2ekf(v_body_vi[:,1]); vi_vz_e = vi2ekf(v_body_vi[:,2])
vi_roll_e  = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 0]), et)
vi_pitch_e = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 1]), et)
vi_yaw_e   = interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid, 2]), et)

# ─── Build ground region hulls ─────────────────────────────────────────────────
def build_region_hulls(vi):
    def xy(m):
        xyz = vi.get_xyz(m)
        v = ~np.isnan(xyz).any(axis=1)
        return vi.to_robot(xyz[v][0:1])[0, :2]
    pts_A = np.array([xy(f'groundA{i}') for i in range(1,5)])
    pts_B = np.array([xy(f'groundB{i}') for i in range(1,5)])
    return Delaunay(pts_A), Delaunay(pts_B), pts_A, pts_B

hA, hB, pts_A, pts_B = build_region_hulls(vi)

C_VI='#1E88E5'; C_GMO='#FF5722'; C_REG='#4CAF50'

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Contact Detection
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 1: Contact Detection')
contact_results = {}
for leg, gm in [('RF','G2'), ('RH','G3')]:
    hf = vi.foot_heights[leg]; cf = vi.contact[leg]
    fxyz = vi.get_xyz(gm)
    fxy = np.full((len(vi.t_traj), 2), np.nan)
    vf = ~np.isnan(fxyz).any(1)
    if vf.any(): fxy[vf] = vi.to_robot(fxyz[vf])[:, :2]
    rmask = np.zeros(len(vi.t_traj), dtype=bool)
    if vf.any(): rmask[vf] = in_region(fxy[vf], hA, hB)
    amask = mask_win_vi & rmask
    ta = vi.t_traj[amask]; cva = cf[amask]; ha = hf[amask]
    cga = interp_gmo(gmo['t'], gmo[leg], ta, T_END)
    nm = ~np.isnan(ha); cvv = cva[nm]; cgv = cga[nm]
    TP = int(np.sum(cvv & cgv)); TN = int(np.sum(~cvv & ~cgv))
    FP = int(np.sum(~cvv & cgv)); FN = int(np.sum(cvv & ~cgv)); N = TP+TN+FP+FN
    acc  = (TP+TN)/N if N else float('nan')
    prec = TP/(TP+FP) if (TP+FP) else float('nan')
    rec  = TP/(TP+FN) if (TP+FN) else float('nan')
    f1   = 2*TP/(2*TP+FP+FN) if (2*TP+FP+FN) else float('nan')
    vi_on  = ta[np.diff(cva.astype(int), prepend=0)==1]
    gmo_on = ta[np.diff(cga.astype(int), prepend=0)==1]
    lats = []
    for vt in vi_on:
        d = gmo_on - vt; near = d[np.abs(d) < 0.1]
        if len(near): lats.append(near[np.argmin(np.abs(near))])
    lat_ms = float(np.mean(lats)*1000) if lats else float('nan')
    contact_results[leg] = dict(TP=TP,TN=TN,FP=FP,FN=FN,N=N,
        accuracy=acc, precision=prec, recall=rec, f1=f1, mean_latency_ms=lat_ms,
        t_analysis=ta, c_vi_analysis=cva, c_gmo_analysis=cga, h_analysis=ha,
        region_mask_full=rmask, foot_xy_robot=fxy, n_in_region=int(np.sum(amask)))
    print(f'  {leg}({gm}): N={N} TP={TP} TN={TN} FP={FP} FN={FN}  '
          f'Acc={acc*100:.1f}% Prec={prec*100:.1f}% Rec={rec*100:.1f}% '
          f'F1={f1:.4f} Lat={lat_ms:.1f}ms')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Inner EKF Analysis
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 2: Inner EKF Analysis (ablation — leg only)')
err_x = epx-vi_px_e; err_y = epy-vi_py_e; err_z = epz-vi_pz_e
err_3d = np.sqrt(err_x**2+err_y**2+err_z**2); vp = ~np.isnan(err_3d)
m21 = {'RMSE_X':rmse(err_x[vp]),'RMSE_Y':rmse(err_y[vp]),'RMSE_Z':rmse(err_z[vp]),
       'RMSE_3D':rmse(err_3d[vp]),'MAX_3D':float(np.max(err_3d[vp])),
       'final_EKF':(float(epx[-1]),float(epy[-1])),
       'final_VICON':(float(vi_px_e[vp][-1]),float(vi_py_e[vp][-1]))}

# Velocity RMSE restricted to t=12-17s
_vw = (et >= 12.) & (et <= 17.)
m22 = {'RMSE_vx':rmse(evx[_vw]-vi_vx_e[_vw]), 'RMSE_vy':rmse(evy[_vw]-vi_vy_e[_vw]),
       'RMSE_vz':rmse(evz[_vw]-vi_vz_e[_vw]), 'peak_fwd':float(np.nanmax(evx)),
       'vel_window':'12-17s'}

m23 = {'RMSE_roll_deg':rmse(rpy_ekf_deg[:,0]-vi_roll_e),
       'RMSE_pitch_deg':rmse(rpy_ekf_deg[:,1]-vi_pitch_e),
       'RMSE_yaw_deg':rmse(rpy_ekf_deg[:,2]-vi_yaw_e),
       'final_yaw_EKF':float(rpy_ekf_deg[-1,2]),
       'final_yaw_VICON':float(vi_yaw_e[vp][-1]) if vp.any() else float('nan')}

ba_mask = (ba['t'] >= 0.) & (ba['t'] <= T_END)
bw_mask = (bw['t'] >= 0.) & (bw['t'] <= T_END)
m24 = {}; m25 = {}
for bdict, bmask, mout in [(ba,ba_mask,m24),(bw,bw_mask,m25)]:
    for ax_name in ['x','y','z']:
        v = bdict[ax_name][bmask]
        if len(v) == 0: continue
        mout[ax_name] = {'init':float(v[0]),'ss':float(v[-len(v)//4:].mean()),
                         'std':float(v[-len(v)//4:].std())}

print(f'  2.1 Pos RMSE — X={m21["RMSE_X"]*100:.1f}cm Y={m21["RMSE_Y"]*100:.1f}cm '
      f'Z={m21["RMSE_Z"]*100:.1f}cm 3D={m21["RMSE_3D"]*100:.1f}cm MAX={m21["MAX_3D"]*100:.1f}cm')
print(f'  2.2 Vel RMSE (12-17s) — vx={m22["RMSE_vx"]:.3f} vy={m22["RMSE_vy"]:.3f} vz={m22["RMSE_vz"]:.3f} m/s')
print(f'  2.3 Att RMSE — roll={m23["RMSE_roll_deg"]:.2f}° pitch={m23["RMSE_pitch_deg"]:.2f}° yaw={m23["RMSE_yaw_deg"]:.2f}°')
print(f'  2.4 ba ss — x={m24["x"]["ss"]:.5f} y={m24["y"]["ss"]:.5f} z={m24["z"]["ss"]:.5f} m/s²')
print(f'  2.5 bw ss — x={m25["x"]["ss"]:.6f} y={m25["y"]["ss"]:.6f} z={m25["z"]["ss"]:.6f} rad/s')

# ─── Load fusion metrics for overlay ──────────────────────────────────────────
fusion_metrics_path = os.path.join(BASE, 'metrics.json')
fusion_m = json.load(open(fusion_metrics_path)) if os.path.exists(fusion_metrics_path) else {}

# ─── Plots ────────────────────────────────────────────────────────────────────
# Plot A1: XY comparison (ablation leg vs VICON, overlay fusion if available)
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
ax = axes[0]
ax.plot(vi.pos_m[vi.valid_hip & mask_win_vi, 0],
        vi.pos_m[vi.valid_hip & mask_win_vi, 1],
        lw=2, label='VICON', color='#1E88E5')
ax.plot(epx, epy, lw=1.5, label='EKF (leg only)', color='#E53935', alpha=0.9)
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.3)
ax.set_title(f'{TRIAL} — 2.1 XY Trajectory (Ablation: No LiDAR)')
ax = axes[1]
ax.plot(et, epz*100, label='EKF Z (leg only)', lw=1., color='#E53935')
ax.plot(vi_t_v, pos_vicon[vi_valid, 2]*100, label='VICON Z', lw=1., color='#1E88E5', alpha=0.7)
ax.set_xlabel('Time [s]'); ax.set_ylabel('Z [cm]'); ax.legend(); ax.grid(True, alpha=0.3)
ax.set_title('Z vs Time')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig_ablation_ekf_xy.png'), dpi=150); plt.close()
print('Saved: fig_ablation_ekf_xy.png')

# Plot A2: Position time series
fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
for ax, lbl, ev, vv, cov in [(axes[0], 'X[m]', epx, vi_px_e, ecov_px),
                               (axes[1], 'Y[m]', epy, vi_py_e, ecov_py),
                               (axes[2], 'Z[m]', epz, vi_pz_e, ecov_pz)]:
    ax.plot(et, ev, lw=1., label='EKF (leg only)', color='#E53935')
    ax.plot(et, vv, lw=1., label='VICON', color='#1E88E5', alpha=0.8)
    sig = np.sqrt(np.abs(cov))
    ax.fill_between(et, ev-3*sig, ev+3*sig, alpha=0.15, color='#E53935', label='3σ')
    ax.set_ylabel(lbl); ax.legend(loc='upper right', fontsize=7); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.1 Position vs VICON (Ablation: No LiDAR)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig_ablation_ekf_pos.png'), dpi=150); plt.close()
print('Saved: fig_ablation_ekf_pos.png')

# Plot A3: Velocity
fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
for ax, lbl, ev, vv in [(axes[0], 'vx[m/s]', evx, vi_vx_e),
                         (axes[1], 'vy[m/s]', evy, vi_vy_e),
                         (axes[2], 'vz[m/s]', evz, vi_vz_e)]:
    ax.plot(et, ev, lw=0.8, label='EKF (leg only)', color='#E53935')
    ax.plot(et, vv, lw=0.8, label='VICON SG', color='#1E88E5', alpha=0.8)
    ax.axvspan(12, 17, color='yellow', alpha=0.15, label='RMSE window 12-17s')
    ax.set_ylabel(lbl); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.2 Velocity (Ablation: No LiDAR)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig_ablation_ekf_vel.png'), dpi=150); plt.close()
print('Saved: fig_ablation_ekf_vel.png')

# Plot A4: RPY
fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
for ax, lbl, ev, vv in [(axes[0], 'Roll[°]', rpy_ekf_deg[:,0], vi_roll_e),
                         (axes[1], 'Pitch[°]', rpy_ekf_deg[:,1], vi_pitch_e),
                         (axes[2], 'Yaw[°]', rpy_ekf_deg[:,2], vi_yaw_e)]:
    ax.plot(et, ev, lw=0.8, label='EKF (leg only)', color='#E53935')
    ax.plot(et, vv, lw=0.8, label='VICON', color='#1E88E5', alpha=0.8)
    ax.set_ylabel(lbl); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.3 Attitude (Ablation: No LiDAR)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig_ablation_ekf_rpy.png'), dpi=150); plt.close()
print('Saved: fig_ablation_ekf_rpy.png')

# Plot A5: Bias
fig, axes = plt.subplots(2, 3, figsize=(14, 6))
for row, (bd, bmask, bname, unit) in enumerate([(ba, ba_mask, 'ba(accel)', 'm/s²'),
                                                  (bw, bw_mask, 'bw(gyro)', 'rad/s')]):
    for col, ax_name in enumerate(['x', 'y', 'z']):
        ax = axes[row][col]
        ax.plot(bd['t'][bmask], bd[ax_name][bmask], lw=0.7)
        ax.set_title(f'{bname} {ax_name}'); ax.set_ylabel(unit); ax.grid(True, alpha=0.3)
        if row == 1: ax.set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.4/2.5 IMU Bias (Ablation: No LiDAR)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig_ablation_ekf_bias.png'), dpi=150); plt.close()
print('Saved: fig_ablation_ekf_bias.png')

# Plot A6: Ablation comparison bar chart (vs fusion)
if fusion_m:
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    labels = ['Pos 3D RMSE [cm]', 'Vel vx RMSE\n(12-17s) [m/s]', 'Yaw RMSE [°]']
    ablation_vals = [m21['RMSE_3D']*100, m22['RMSE_vx'], m23['RMSE_yaw_deg']]
    fusion_pos_rmse = np.sqrt(fusion_m['ekf_pos']['RMSE_X']**2 +
                               fusion_m['ekf_pos']['RMSE_Y']**2 +
                               fusion_m['ekf_pos']['RMSE_Z']**2)
    fusion_vals = [fusion_pos_rmse*100,
                   fusion_m['ekf_vel']['RMSE_vx'],
                   fusion_m['ekf_att']['RMSE_yaw_deg']]
    for ax, lbl, av, fv_ in zip(axes, labels, ablation_vals, fusion_vals):
        bars = ax.bar(['Leg only\n(ablation)', 'Leg+LiDAR\n(fusion)'],
                      [av, fv_], color=['#E53935','#43A047'], alpha=0.8, width=0.5)
        ax.bar_label(bars, fmt='%.3f', padding=2, fontsize=9)
        ax.set_title(lbl, fontsize=9); ax.set_ylim(0, max(av, fv_)*1.3)
        ax.grid(True, alpha=0.3, axis='y')
    fig.suptitle(f'walk_2m_01 — Inner EKF Ablation: Leg Only vs Leg+LiDAR Fusion')
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS, 'fig_ablation_comparison.png'), dpi=150); plt.close()
    print('Saved: fig_ablation_comparison.png')

    # Plot A7: XY overlay comparison
    # Load fusion EKF trajectory from full metrics
    # We need the actual full analyze.py to have saved the data — instead just overlay in one figure
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(vi.pos_m[vi.valid_hip & mask_win_vi, 0],
            vi.pos_m[vi.valid_hip & mask_win_vi, 1],
            lw=2, label='VICON', color='#1E88E5')
    ax.plot(epx, epy, lw=1.5, label=f'Leg only (RMSE 3D={m21["RMSE_3D"]*100:.1f}cm)',
            color='#E53935', alpha=0.9)
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    ax.set_title('Ablation XY: Leg Only vs VICON\n(compare with fig_ekf_xy.png for fusion)')
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS, 'fig_ablation_xy_overlay.png'), dpi=150); plt.close()
    print('Saved: fig_ablation_xy_overlay.png')

# ─── Save ablation metrics ────────────────────────────────────────────────────
abl_metrics = {
    'trial': TRIAL, 'date': DATE, 'T_END': T_END,
    'contact_threshold_m': CONTACT_THRESHOLD_M,
    'contact': {leg: {k: v for k, v in contact_results[leg].items()
                      if not isinstance(v, np.ndarray)} for leg in ['RF','RH']},
    'ekf_pos': m21, 'ekf_vel': m22, 'ekf_att': m23,
    'ekf_ba': {ax: m24[ax] for ax in ['x','y','z']},
    'ekf_bw': {ax: m25[ax] for ax in ['x','y','z']},
    'note': 'Inner EKF only — no LiDAR, no fusion node',
}
with open(os.path.join(RESULTS, 'ablation_metrics.json'), 'w') as f:
    json.dump(abl_metrics, f, indent=2)
print('Saved: ablation_metrics.json')

print('\n'+'='*60)
print(f'ABLATION SUMMARY — {TRIAL} ({DATE})')
print(f'  Window: [0,{T_END:.2f}]s  (leg only, no LiDAR)')
print(f'  Pos  RMSE 3D  = {m21["RMSE_3D"]*100:.2f} cm')
print(f'  Vel  RMSE vx  = {m22["RMSE_vx"]:.4f} m/s (12-17s)')
print(f'  Att  RMSE yaw = {m23["RMSE_yaw_deg"]:.3f} °')
if fusion_m:
    fpos_rmse = np.sqrt(fusion_m['ekf_pos']['RMSE_X']**2 +
                         fusion_m['ekf_pos']['RMSE_Y']**2 +
                         fusion_m['ekf_pos']['RMSE_Z']**2)
    print(f'\n  [Fusion comparison]')
    print(f'  Pos 3D RMSE: ablation={m21["RMSE_3D"]*100:.2f}cm vs fusion={fpos_rmse*100:.2f}cm')
    print(f'  Vel vx RMSE: ablation={m22["RMSE_vx"]:.4f} vs fusion={fusion_m["ekf_vel"]["RMSE_vx"]:.4f} m/s')
    print(f'  Yaw  RMSE:   ablation={m23["RMSE_yaw_deg"]:.3f}° vs fusion={fusion_m["ekf_att"]["RMSE_yaw_deg"]:.3f}°')
print('Done.')
