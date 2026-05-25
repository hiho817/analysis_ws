#!/usr/bin/env python3
"""
CORGI Experiment Analysis — 20260522 exp3
mpc_legacy_gait (Legacy Information Filter + fixed gait contact)

Steps: Contact Detection (VICON vs fixed gait), Position, Velocity
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
import sqlite3

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_legacy_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
RESULTS   = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'mpc_legacy_20260522_193533',
                         'mpc_legacy_20260522_193533_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'EXP3.csv')
TRIAL     = 'mpc_legacy_gait'
DATE      = '20260522'
EXP_ID    = 'exp3'
CONTACT_THRESHOLD_M = 0.015

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

# ─── Load legacy contact topic (/odometry/legacy/contact) ─────────────────────
def load_legacy_contact(bag_db, trg_ts0, rate=1.0):
    """Parse /odometry/legacy/contact using storage timestamps (header stamp is zero)."""
    from rclpy.serialization import deserialize_message
    from corgi_msgs.msg import ContactStateStamped

    conn = sqlite3.connect(bag_db)
    cur  = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    tid  = tmap.get('/odometry/legacy/contact')
    if tid is None:
        conn.close()
        return {'t': np.array([]), 'LF': np.array([]), 'RF': np.array([]),
                'RH': np.array([]), 'LH': np.array([])}
    cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp")
    rows = cur.fetchall()
    conn.close()

    d = {'t': [], 'LF': [], 'RF': [], 'RH': [], 'LH': []}
    for ts, data in rows:
        msg = deserialize_message(data, ContactStateStamped)
        t = (ts - trg_ts0) / 1e9 * rate    # storage-based time, same as pos/vel
        d['t'].append(t)
        d['LF'].append(msg.module_a.contact)
        d['RF'].append(msg.module_b.contact)
        d['RH'].append(msg.module_c.contact)
        d['LH'].append(msg.module_d.contact)
    for k in d:
        d[k] = np.array(d[k])
    return d

# ─── Load ─────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
print('='*60)
vi  = load_vicon(VICON_CSV,
                 contact_threshold_m=CONTACT_THRESHOLD_M,
                 ground_markers=['ground1', 'ground2', 'ground3', 'ground4'])
bag = load_legacy_bag(BAG_DB, rate=1.0)
pos = bag['pos']; vel = bag['vel']

# Get trigger storage timestamp for contact time alignment
def _get_trg_ts0(db_path):
    import sqlite3 as _sq3
    conn = _sq3.connect(db_path)
    cur  = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    tid  = tmap.get('/trigger')
    cur.execute(f"SELECT timestamp FROM messages WHERE topic_id={tid} ORDER BY timestamp LIMIT 1")
    ts0 = cur.fetchone()[0]
    conn.close()
    return ts0

contact_bag = load_legacy_contact(BAG_DB, _get_trg_ts0(BAG_DB), rate=1.0)
print(f'[contact] loaded {len(contact_bag["t"])} contact msgs')

T_END = min(vi.t_trigger_end, bag['t_trigger_end'])
print(f'\nAnalysis window: t ∈ [0, {T_END:.2f}] s')

mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_win       = vi.t_traj[mask_win_vi]
pos_vicon   = vi.pos_m[mask_win_vi]
v_body_vi   = vi.v_body[mask_win_vi]

pos_mask = (pos['t'] >= 0.0) & (pos['t'] <= T_END)
vel_mask = (vel['t'] >= 0.0) & (vel['t'] <= T_END)
pt = pos['t'][pos_mask]; px = pos['x'][pos_mask]
py = pos['y'][pos_mask]; pz = pos['z'][pos_mask]
vt = vel['t'][vel_mask]; vx = vel['x'][vel_mask]
vy = vel['y'][vel_mask]; vz = vel['z'][vel_mask]

vi_valid = ~np.isnan(pos_vicon).any(1)
vi_t_v   = t_win[vi_valid]

# Both Legacy and VICON use the same Y convention — no flip needed
pos_vicon_cmp = pos_vicon.copy()

vi_px_p = interp_to(vi_t_v, pos_vicon_cmp[vi_valid, 0], pt)
vi_py_p = interp_to(vi_t_v, pos_vicon_cmp[vi_valid, 1], pt)
vi_pz_p = interp_to(vi_t_v, pos_vicon_cmp[vi_valid, 2], pt)
vi_vx_v = interp_to(vi_t_v, v_body_vi[vi_valid, 0], vt)
vi_vy_v = interp_to(vi_t_v, v_body_vi[vi_valid, 1], vt)
vi_vz_v = interp_to(vi_t_v, v_body_vi[vi_valid, 2], vt)

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Contact Detection (VICON vs Legacy fixed-gait)
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 1: Contact Detection')

def get_ground_pts(vi_obj):
    pts = []
    for m in ['ground1', 'ground2', 'ground3', 'ground4']:
        xyz = vi_obj.get_xyz(m)
        v = ~np.isnan(xyz).any(axis=1)
        if v.any():
            pts.append(vi_obj.to_robot(xyz[v][0:1])[0, :2])
    return np.array(pts)

gnd_pts = get_ground_pts(vi)
try:
    hull_gnd = Delaunay(gnd_pts)
except Exception:
    hull_gnd = None

def in_region(xy_mm, hull):
    if hull is None:
        return np.ones(len(xy_mm), dtype=bool)
    return hull.find_simplex(xy_mm) >= 0

LEG_MAP = [('LF', 'G1'), ('RF', 'G2'), ('RH', 'G3'), ('LH', 'G4')]
COLORS_LEG = {'LF': 'steelblue', 'RF': 'darkorange', 'RH': 'forestgreen', 'LH': 'crimson'}

def interp_contact(ctt, ctl, t_tgt, T_END):
    mk = (ctt >= -0.5) & (ctt <= T_END + 0.5)
    t_g = ctt[mk]; c_g = ctl[mk].astype(float)
    if len(t_g) < 2:
        return np.zeros(len(t_tgt), dtype=bool)
    return interp1d(t_g, c_g, kind='nearest',
                    bounds_error=False, fill_value=0.)(t_tgt) > 0.5

contact_results = {}
for leg, gm in LEG_MAP:
    hf = vi.foot_heights[leg]; cf = vi.contact[leg]
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
        contact_results[leg] = None; continue

    ta  = vi.t_traj[amask]; cva = cf[amask]
    if len(contact_bag['t']) > 1:
        cga = interp_contact(contact_bag['t'], contact_bag[leg], ta, T_END)
    else:
        cga = np.zeros(len(ta), dtype=bool)

    valid_v = ~np.isnan(cva)
    tv = ta[valid_v]; cv_v = cva[valid_v]; cg_v = cga[valid_v]
    if len(tv) == 0:
        contact_results[leg] = None; continue

    TP = int(np.sum(cv_v & cg_v)); TN = int(np.sum(~cv_v & ~cg_v))
    FP = int(np.sum(~cv_v & cg_v)); FN = int(np.sum(cv_v & ~cg_v))
    N  = len(tv)
    prec = TP / (TP + FP) if (TP + FP) > 0 else float('nan')
    rec  = TP / (TP + FN) if (TP + FN) > 0 else float('nan')
    f1   = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else float('nan')
    acc  = (TP + TN) / N

    latencies = []
    dt   = np.diff(cv_v.astype(int), prepend=0)
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
    print(f'  [{leg}] Acc={acc:.1%} Prec={prec:.3f} Rec={rec:.3f} F1={f1:.4f} Lat={mean_lat:.1f}ms')

fig, axes = plt.subplots(4, 1, figsize=(13, 8), sharex=True)
for ax, (leg, gm) in zip(axes, LEG_MAP):
    hf = vi.foot_heights[leg][mask_win_vi]
    cf = vi.contact[leg][mask_win_vi]
    color = COLORS_LEG[leg]
    ax.plot(t_win, hf * 1000, lw=0.8, color=color, label=f'{leg} height [mm]')
    _shade_contact(ax, t_win, cf, color='tab:green', alpha=0.18)
    if len(contact_bag['t']) > 1:
        c_leg = interp_contact(contact_bag['t'], contact_bag[leg], t_win, T_END)
        _shade_contact(ax, t_win, c_leg, color='tab:red', alpha=0.12)
    ax.axhline(CONTACT_THRESHOLD_M * 1000, color='k', ls='--', lw=0.8, alpha=0.5)
    _shade_window(ax, T_END, label=False)
    ax.set_ylabel(f'{leg} [mm]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'Contact Detection (green=VICON, red=Legacy fixed-gait) — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_contact.png'), dpi=150)
plt.close(fig)
print('Saved fig_contact.png')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Position Analysis
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 2: Position Analysis')

z_offset_z = pz[0] - vi_pz_p[0]   # ~0.20 m body height vs VICON origin
err_px = px - vi_px_p; err_py = py - vi_py_p; err_pz = pz - vi_pz_p - z_offset_z
err_3d = np.sqrt(err_px**2 + err_py**2 + err_pz**2)
valid_p = ~np.isnan(err_3d)

metrics_pos = {
    'RMSE_X_cm':  rmse(err_px[valid_p]) * 100,
    'RMSE_Y_cm':  rmse(err_py[valid_p]) * 100,
    'RMSE_Z_cm':  rmse(err_pz[valid_p]) * 100,
    'RMSE_3D_cm': rmse(err_3d[valid_p]) * 100,
    'MAX_3D_cm':  float(np.max(err_3d[valid_p])) * 100 if valid_p.any() else float('nan'),
    'final_pos_x': float(px[-1]) if len(px) > 0 else float('nan'),
    'final_pos_y': float(py[-1]) if len(py) > 0 else float('nan'),
    'final_VICON_x': float(vi_px_p[valid_p][-1]) if valid_p.any() else float('nan'),
    'final_VICON_y': float(vi_py_p[valid_p][-1]) if valid_p.any() else float('nan'),
}
print(f'  Pos RMSE: X={metrics_pos["RMSE_X_cm"]:.2f}cm '
      f'Y={metrics_pos["RMSE_Y_cm"]:.2f}cm 3D={metrics_pos["RMSE_3D_cm"]:.2f}cm')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Velocity Analysis
# ═══════════════════════════════════════════════════════════════════════════════
print('\n' + '─'*60 + '\nSTEP 3: Velocity Analysis')

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
print(f'  Vel RMSE: vx={metrics_vel["RMSE_vx"]:.3f} vy={metrics_vel["RMSE_vy"]:.3f} m/s')

# ═══════════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════════
# XY Trajectory
fig, ax = plt.subplots(figsize=(7, 5))
ax.plot(pos_vicon_cmp[vi_valid, 0], pos_vicon_cmp[vi_valid, 1], 'k-', lw=1.5, label='VICON', zorder=4)
sc = ax.scatter(px, py, c=pt, cmap='viridis', s=3, lw=0, label='Legacy')
ax.plot(px[0], py[0], 'go', ms=8, label='start')
ax.plot(px[-1], py[-1], 'r^', ms=8, label='end')
plt.colorbar(sc, ax=ax, label='Time [s]')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'Legacy XY Trajectory — {DATE} {EXP_ID}')
ax.set_aspect('equal'); ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_traj_xy.png'), dpi=150); plt.close(fig)

# Position time series
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
vi_vals = [pos_vicon_cmp[vi_valid, 0], pos_vicon_cmp[vi_valid, 1], pos_vicon_cmp[vi_valid, 2]]
leg_vals = [px, py, pz - z_offset_z]
for ax, lbl, lv, vi_v in zip(axes, ['X','Y','Z'], leg_vals, vi_vals):
    ax.plot(pt, lv, lw=0.8, label='Legacy')
    ax.plot(vi_t_v, vi_v, 'k--', lw=1, alpha=0.7, label='VICON')
    _shade_window(ax, T_END, label=(lbl == 'X'))
    ax.set_ylabel(f'{lbl} [m]'); ax.legend(fontsize=7); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'Legacy Position — {DATE} {EXP_ID}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_pos_time.png'), dpi=150); plt.close(fig)

# Velocity time series
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
vi_vvals = [vi_vx_v, vi_vy_v, vi_vz_v]
for ax, lbl, lv, vi_v in zip(axes, ['vx','vy','vz'], [vx, vy, vz], vi_vvals):
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
fig.savefig(os.path.join(RESULTS, 'fig_vel_time.png'), dpi=150); plt.close(fig)
print('Saved figures')

# ═══════════════════════════════════════════════════════════════════════════════
# Save metrics
# ═══════════════════════════════════════════════════════════════════════════════
contact_json = {}
for leg, res in contact_results.items():
    if res is None:
        contact_json[leg] = None; continue
    contact_json[leg] = {k: v for k, v in res.items() if k not in ('t','cv','cg')}

all_metrics = {
    'exp': EXP_ID, 'trial': TRIAL, 'date': DATE, 'T_END': T_END,
    'contact': contact_json,
    'position': metrics_pos,
    'velocity': metrics_vel,
}
with open(os.path.join(RESULTS, 'metrics.json'), 'w') as f:
    json.dump(all_metrics, f, indent=2, default=str)
print('\nSaved metrics.json')
print('='*60)
print(f'Analysis complete — {EXP_ID} ({TRIAL})')
print(f'  Pos 3D RMSE:  {metrics_pos["RMSE_3D_cm"]:.2f} cm')
print(f'  Vel RMSE vx:  {metrics_vel["RMSE_vx"]:.3f} m/s')
print(f'  T_END:        {T_END:.2f} s')
