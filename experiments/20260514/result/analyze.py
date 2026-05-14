#!/usr/bin/env python3
"""
CORGI Experiment Analysis — 20260514 leg_odom
No VICON ground truth, no LiDAR input.

Plots:
  fig_ekf_xy.png        — EKF XY trajectory
  fig_ekf_pos.png       — EKF position X/Y/Z time series
  fig_ekf_vel.png       — EKF velocity vx/vy/vz time series
  fig_ekf_rpy.png       — EKF attitude Roll/Pitch/Yaw
  fig_ekf_ba_bw.png     — Accel + gyro bias convergence
  fig_gmo_contact.png   — GMO contact state (LF/RF/RH/LH)
  fig_gmo_force.png     — GMO rm_force per leg

NOTE: source ~/corgi_ws/corgi_ros2_ws/install/setup.bash before running.
"""

import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.bag_loader import load_inner_ekf_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE    = os.path.dirname(os.path.abspath(__file__))
RESULTS = BASE
BAG_DB  = os.path.join(BASE, '..', 'bags',
                        'leg_odom20260514_151737',
                        'leg_odom20260514_151737_0.db3')
DATE  = '20260514'
TRIAL = 'leg_odom'

# ─── Load data ────────────────────────────────────────────────────────────────
print(f'Loading bag: {BAG_DB}')
data = load_inner_ekf_bag(BAG_DB, rate=1.0)

ekf = data['ekf']
ba  = data['ba']
bw  = data['bw']
gmo = data['gmo']
t_end = data.get('t_trigger_end')

print(f'EKF: t=[{ekf["t"][0]:.2f}, {ekf["t"][-1]:.2f}] s')
if t_end:
    print(f'Trigger window: 0 → {t_end:.2f} s')

# ─── Derived: RPY from quaternion ─────────────────────────────────────────────
rpy = Rotation.from_quat(
    np.column_stack([ekf['qx'], ekf['qy'], ekf['qz'], ekf['qw']])
).as_euler('ZYX')[:, ::-1]   # → [roll, pitch, yaw]  radians
roll_deg  = np.degrees(rpy[:, 0])
pitch_deg = np.degrees(rpy[:, 1])
yaw_deg   = np.degrees(rpy[:, 2])

# ─── Helper: shade trigger window ─────────────────────────────────────────────
def shade_trigger(ax, t_end, color='gold', alpha=0.08, label=True):
    if t_end is not None:
        ax.axvspan(0, t_end, color=color, alpha=alpha,
                   label='trigger window' if label else None)

# ─── Plot 1: EKF XY trajectory ────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(6, 6))
sc = ax.scatter(ekf['px'], ekf['py'],
                c=ekf['t'], cmap='viridis', s=2, lw=0)
ax.plot(ekf['px'][0],  ekf['py'][0],  'go', ms=8, label='start', zorder=5)
ax.plot(ekf['px'][-1], ekf['py'][-1], 'r^', ms=8, label='end',   zorder=5)
plt.colorbar(sc, ax=ax, label='Time [s]')
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title(f'EKF XY Trajectory — {DATE} {TRIAL}')
ax.set_aspect('equal'); ax.legend(); ax.grid(True, alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_xy.png'), dpi=150)
plt.close(fig)
print('Saved fig_ekf_xy.png')

# ─── Plot 2: EKF position time series ─────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for ax, key, lbl in zip(axes, ['px', 'py', 'pz'], ['X', 'Y', 'Z']):
    ax.plot(ekf['t'], ekf[key], lw=0.8)
    sigma = np.sqrt(np.abs(ekf[f'cov_p{lbl.lower()}']))
    ax.fill_between(ekf['t'],
                    ekf[key] - 3*sigma,
                    ekf[key] + 3*sigma,
                    alpha=0.25, label='3σ')
    shade_trigger(ax, t_end, label=(lbl == 'X'))
    ax.set_ylabel(f'{lbl} [m]')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Position — {DATE} {TRIAL}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_pos.png'), dpi=150)
plt.close(fig)
print('Saved fig_ekf_pos.png')

# ─── Plot 3: EKF velocity time series ─────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for ax, key, lbl in zip(axes, ['vx', 'vy', 'vz'], ['vx', 'vy', 'vz']):
    ax.plot(ekf['t'], ekf[key], lw=0.8)
    sigma = np.sqrt(np.abs(ekf[f'cov_{lbl}']))
    ax.fill_between(ekf['t'],
                    ekf[key] - 3*sigma,
                    ekf[key] + 3*sigma,
                    alpha=0.25, label='3σ')
    shade_trigger(ax, t_end, label=(lbl == 'vx'))
    ax.set_ylabel(f'{lbl} [m/s]')
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Velocity (body frame) — {DATE} {TRIAL}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_vel.png'), dpi=150)
plt.close(fig)
print('Saved fig_ekf_vel.png')

# ─── Plot 4: EKF attitude RPY ─────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for ax, vals, lbl in zip(axes,
                         [roll_deg, pitch_deg, yaw_deg],
                         ['Roll', 'Pitch', 'Yaw']):
    ax.plot(ekf['t'], vals, lw=0.8)
    shade_trigger(ax, t_end, label=(lbl == 'Roll'))
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    ax.set_ylabel(f'{lbl} [°]')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'EKF Attitude (RPY) — {DATE} {TRIAL}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_rpy.png'), dpi=150)
plt.close(fig)
print('Saved fig_ekf_rpy.png')

# ─── Plot 5: Bias convergence ─────────────────────────────────────────────────
fig, axes = plt.subplots(2, 3, figsize=(14, 6), sharex=True)
labels = ['x', 'y', 'z']
for col, lbl in enumerate(labels):
    ax = axes[0, col]
    ax.plot(ba['t'], ba[lbl], lw=0.8)
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    shade_trigger(ax, t_end, label=(col == 0))
    ax.set_ylabel(f'ba.{lbl} [m/s²]')
    ax.set_title(f'Accel bias {lbl}')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    ax.set_xlabel('Time [s]')

    ax = axes[1, col]
    ax.plot(bw['t'], bw[lbl], lw=0.8)
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    shade_trigger(ax, t_end, label=(col == 0))
    ax.set_ylabel(f'bw.{lbl} [rad/s]')
    ax.set_title(f'Gyro bias {lbl}')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.4)
    ax.set_xlabel('Time [s]')

fig.suptitle(f'Inner EKF Bias Convergence — {DATE} {TRIAL}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_ekf_ba_bw.png'), dpi=150)
plt.close(fig)
print('Saved fig_ekf_ba_bw.png')

# ─── Helper: shade contact spans ──────────────────────────────────────────────
def shade_contact(ax, t, contact, color, alpha=0.35):
    prev = False; t0 = 0.0
    for i in range(len(t)):
        c = bool(contact[i])
        if c and not prev:
            t0 = t[i]
        elif not c and prev:
            ax.axvspan(t0, t[i], color=color, alpha=alpha, lw=0)
        prev = c
    if prev:
        ax.axvspan(t0, t[-1], color=color, alpha=alpha, lw=0)

# ─── Plot 6: GMO contact state ────────────────────────────────────────────────
LEG_COLORS = {'LF': 'C0', 'RF': 'C1', 'RH': 'C2', 'LH': 'C3'}
LEG_ORDER  = ['LF', 'RF', 'RH', 'LH']

fig, ax = plt.subplots(figsize=(12, 3))
offsets = {'LF': 3, 'RF': 2, 'RH': 1, 'LH': 0}
for leg in LEG_ORDER:
    off = offsets[leg]
    c = gmo[leg].astype(float)
    ax.plot(gmo['t'], c + off, lw=0.6, color=LEG_COLORS[leg], label=leg)
    shade_contact(ax, gmo['t'], gmo[leg], color=LEG_COLORS[leg], alpha=0.25)

shade_trigger(ax, t_end, alpha=0.08)
ax.set_yticks([0, 1, 2, 3]); ax.set_yticklabels(['LH', 'RH', 'RF', 'LF'])
ax.set_xlabel('Time [s]'); ax.set_ylabel('Leg')
ax.set_title(f'GMO Contact State — {DATE} {TRIAL}')
ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_gmo_contact.png'), dpi=150)
plt.close(fig)
print('Saved fig_gmo_contact.png')

# ─── Plot 7: GMO rm_force ─────────────────────────────────────────────────────
# Reload raw rm_force from bag
import sqlite3
from rclpy.serialization import deserialize_message
from corgi_msgs.msg import GMOContactStateStamped

conn = sqlite3.connect(BAG_DB)
cur  = conn.cursor()
cur.execute("SELECT id FROM topics WHERE name='/trigger'")
trg_tid = cur.fetchone()[0]
cur.execute(f"SELECT timestamp FROM messages WHERE topic_id={trg_tid} LIMIT 1")
trg_ts0 = cur.fetchone()[0]
cur.execute("SELECT id FROM topics WHERE name='/gmo/contact_state'")
gmo_tid = cur.fetchone()[0]
cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={gmo_tid} ORDER BY timestamp")
rows = cur.fetchall()
conn.close()

gt, lf_f, rf_f, rh_f, lh_f = [], [], [], [], []
seen = set()
for ts, raw in rows:
    if ts in seen: continue
    seen.add(ts)
    msg = deserialize_message(raw, GMOContactStateStamped)
    gt.append((ts - trg_ts0) / 1e9)
    lf_f.append(msg.module_a.rm_force)
    rf_f.append(msg.module_b.rm_force)
    rh_f.append(msg.module_c.rm_force)
    lh_f.append(msg.module_d.rm_force)
gt = np.array(gt); lf_f = np.array(lf_f); rf_f = np.array(rf_f)
rh_f = np.array(rh_f); lh_f = np.array(lh_f)

fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
for ax, vals, leg in zip(axes,
                         [lf_f, rf_f, rh_f, lh_f],
                         ['LF', 'RF', 'RH', 'LH']):
    ax.plot(gt, vals, lw=0.6, color=LEG_COLORS[leg])
    ax.axhline(0, color='gray', ls='--', lw=0.5)
    shade_contact(ax, gt,
                  gmo[leg][np.searchsorted(gmo['t'], gt, side='right').clip(0, len(gmo['t'])-1)],
                  color=LEG_COLORS[leg], alpha=0.2)
    shade_trigger(ax, t_end, alpha=0.08, label=False)
    ax.set_ylabel(f'{leg} rm_force')
    ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'GMO rm_force — {DATE} {TRIAL}')
fig.tight_layout()
fig.savefig(os.path.join(RESULTS, 'fig_gmo_force.png'), dpi=150)
plt.close(fig)
print('Saved fig_gmo_force.png')

print('\nAll plots saved to:', RESULTS)
