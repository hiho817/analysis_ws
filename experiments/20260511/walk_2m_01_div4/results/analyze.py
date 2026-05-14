#!/usr/bin/env python3
"""
CORGI Experiment Analysis — 20260511 walk_2m_01_div4
Algorithm change: theta_d/beta_d divided by 4 instead of 2
Replayed original bag (leg_odom20260507_161231), compared with VICON

Thin wrapper — shared analysis logic lives in tools/corgi_analysis/.
"""

# ── THIS FILE IS A THIN WRAPPER — see tools/corgi_analysis/ for shared logic ─
# STOP HERE: the rest of this file is the old monolithic implementation.
# It will be replaced by the new thin wrapper below this block.
# ---------------------------------------------------------------------------
# TEMPORARY SHIM: import new wrapper and run it, so the old code below is dead.

import os as _os, sys as _sys
_TOOLS = _os.path.normpath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                           '..', '..', '..', '..', 'tools'))
_sys.path.insert(0, _TOOLS)

# ── THIN WRAPPER START ────────────────────────────────────────────────────────
import numpy as np
from scipy.spatial.transform import Rotation
import os, sys

from corgi_analysis.bag_loader   import load_inner_ekf_bag
from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.metrics      import (bias_stats, contact_metrics_all,
                                          interp_vicon_to_ekf, ekf_metrics)
import corgi_analysis.plots as cplt

BASE    = os.path.dirname(os.path.abspath(__file__))
RESULTS = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'replay_div4_20260511_233348',
                         'replay_div4_20260511_233348_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'walk_2m_01.csv')
T_WALK_END          = 14.0
REPLAY_RATE         = 2.0
CONTACT_THRESHOLD_M = 0.005   # 5 mm
EKF_LABEL           = 'EKF (div/4)'

print(f'Loading bag: {BAG_DB}')
bag = load_inner_ekf_bag(BAG_DB, rate=REPLAY_RATE)
ekf = bag['ekf']; ba = bag['ba']; bw = bag['bw']; gmo = bag['gmo']
print(f'EKF: {len(ekf["t"])} msgs, t=[{ekf["t"][0]:.2f}, {ekf["t"][-1]:.2f}] s')

print(f'Loading VICON: {VICON_CSV}')
vi = load_vicon(VICON_CSV, contact_threshold_m=CONTACT_THRESHOLD_M)

arr_q   = np.column_stack([ekf['qx'], ekf['qy'], ekf['qz'], ekf['qw']])
ekf_rpy = Rotation.from_quat(arr_q).as_euler('ZYX')[:, ::-1]
vicon_i = interp_vicon_to_ekf(vi, ekf['t'])
mask_walk = ekf['t'] < T_WALK_END

cm      = contact_metrics_all(vi, gmo, T_WALK_END)
metrics = ekf_metrics(ekf, ekf_rpy, vicon_i, mask_walk)
err_3d  = metrics.pop('err_3d')

print('\n=== Contact Metrics ===')
for leg, m in cm.items():
    print(f'  {leg}: P={m["precision"]:.2f} R={m["recall"]:.2f} '
          f'lat={m["mean_latency_ms"]:.1f}ms stance={m["stance_ratio"]:.1f}%')
print('\n=== EKF Metrics ===')
print(f"  Position RMSE: X={metrics['pos_rmse_x']*1000:.1f}mm  "
      f"Y={metrics['pos_rmse_y']*1000:.1f}mm  "
      f"Z={metrics['pos_rmse_z']*1000:.1f}mm  3D={metrics['pos_rmse_3d']*1000:.1f}mm")
print(f"  Velocity RMSE: Vx={metrics['vel_rmse_x']*1000:.1f}mm/s  "
      f"Vy={metrics['vel_rmse_y']*1000:.1f}mm/s")
print(f"  Attitude RMSE: Roll={np.degrees(metrics['att_rmse_roll']):.2f}°  "
      f"Yaw={np.degrees(metrics['att_rmse_yaw']):.2f}°")

last_ekf_x = float(ekf['px'][-1]); last_ekf_y = float(ekf['py'][-1])
vi_rows = np.where(vi.valid_hip)[0]; last_vi = vi_rows[-1]
last_vx = float(vi.pos_m[last_vi, 0]); last_vy = float(vi.pos_m[last_vi, 1])
vicon_yaw_end_arr = vicon_i['yaw'][mask_walk & ~np.isnan(vicon_i['yaw'])]
vicon_yaw_end = float(np.degrees(vicon_yaw_end_arr[-1])) if len(vicon_yaw_end_arr) else float('nan')
ekf_yaw_end   = float(np.degrees(ekf_rpy[:, 2][mask_walk][-1]))
t_valid_min = np.nanmin(vi.t_traj[vi.valid_hip])
t_valid_max = np.nanmax(vi.t_traj[vi.valid_hip])
t_common_min = max(float(ekf['t'][0]), t_valid_min)
t_common_max = min(float(ekf['t'][-1]), t_valid_max)

cplt.set_style()
cplt.plot_trajectory_xy(ekf, vi, RESULTS, label=EKF_LABEL)
cplt.plot_position_timeseries(ekf, vicon_i, RESULTS, T_WALK_END, label=EKF_LABEL)
cplt.plot_position_error(ekf, err_3d, RESULTS, T_WALK_END, metrics)
cplt.plot_velocity_timeseries(ekf, vicon_i, RESULTS, T_WALK_END, metrics, label=EKF_LABEL)
cplt.plot_attitude(ekf, vicon_i, ekf_rpy, RESULTS, T_WALK_END, label=EKF_LABEL)
cplt.plot_bias(ba, bw, RESULTS, T_WALK_END)
cplt.plot_foot_heights(vi, RESULTS)
cplt.plot_contact_timeline(vi, gmo, RESULTS, T_WALK_END, label=EKF_LABEL)
print('\nAll plots saved.')

ba_stats_all = [bias_stats(ba, ax) for ax in ['x', 'y', 'z']]
bw_stats_all = [bias_stats(bw, ax) for ax in ['x', 'y', 'z']]
cm_mean_recall = float(np.mean([cm[l]['recall'] for l in ['LF', 'RF', 'RH', 'LH']]))

report = f"""# CORGI Experiment Analysis Report

**Date:** 2026-05-11
**Experiment:** `walk_2m_01_div4`
**Bag (replay output):** `replay_div4_20260511_233348`
**Bag (replay input):** `leg_odom20260507_161231`
**VICON CSV:** `walk_2m_01.csv`
**步行階段:** t = 0 – {T_WALK_END:.0f} s
**Analysis script:** `analyze.py`
**演算法修改:** `theta_d` / `beta_d` 除數：2 → 4（對照原始 bag replay）
**接觸閾值 (CONTACT_THRESHOLD_M):** {CONTACT_THRESHOLD_M*1000:.0f} mm

---

## System Architecture

```
/imu_raw ─┐
/motor/state ──┤──► corgi_leg_odom (div/4) ──► /ekf
/trigger ──────┘
/gmo/contact_state
```

---

## 1. 觸地偵測（Contact Detection）

觸地閾值：**{CONTACT_THRESHOLD_M*1000:.0f} mm**。

![Contact Timeline](fig09_contact_timeline.png)
![Foot Height](fig08_foot_heights.png)

| 腿 | Stance ratio | 平均 stance [ms] | Precision | Recall | Latency [ms] |
|-----|-------------|-----------------|-----------|--------|--------------|
| LF (G1) | {cm['LF']['stance_ratio']:.1f}% | {cm['LF']['mean_stance_ms']:.0f} | {cm['LF']['precision']:.2f} | {cm['LF']['recall']:.2f} | {cm['LF']['mean_latency_ms']:.1f} |
| RF (G2) | {cm['RF']['stance_ratio']:.1f}% | {cm['RF']['mean_stance_ms']:.0f} | {cm['RF']['precision']:.2f} | {cm['RF']['recall']:.2f} | {cm['RF']['mean_latency_ms']:.1f} |
| RH (G3) | {cm['RH']['stance_ratio']:.1f}% | {cm['RH']['mean_stance_ms']:.0f} | {cm['RH']['precision']:.2f} | {cm['RH']['recall']:.2f} | {cm['RH']['mean_latency_ms']:.1f} |
| LH (G4) | {cm['LH']['stance_ratio']:.1f}% | {cm['LH']['mean_stance_ms']:.0f} | {cm['LH']['precision']:.2f} | {cm['LH']['recall']:.2f} | {cm['LH']['mean_latency_ms']:.1f} |

GMO Recall 平均 {cm_mean_recall:.2f}。

---

## 2. Inner EKF 分析

### 2.1 位置

![EKF Position XY](fig01_trajectory_xy.png)
![EKF Position Time](fig02_position_timeseries.png)
![3D Position Error](fig03_position_error.png)

| Metric | Value |
|--------|-------|
| RMSE X | {metrics['pos_rmse_x']:.3f} m |
| RMSE Y | {metrics['pos_rmse_y']:.3f} m |
| RMSE Z | {metrics['pos_rmse_z']:.3f} m |
| RMSE 3D | {metrics['pos_rmse_3d']:.3f} m |
| Max 3D error | {metrics['pos_max_3d']:.3f} m |
| Final EKF | ({last_ekf_x:.3f}, {last_ekf_y:.3f}) m |
| Final VICON | ({last_vx:.3f}, {last_vy:.3f}) m |

### 2.2 速度

![EKF Velocity](fig04_velocity_timeseries.png)

| Metric | Value |
|--------|-------|
| RMSE vx | {metrics['vel_rmse_x']:.4f} m/s |
| RMSE vy | {metrics['vel_rmse_y']:.4f} m/s |
| RMSE vz | {metrics['vel_rmse_z']:.4f} m/s |
| Peak Vx (VICON) | {metrics['vel_peak_vx']:.2f} m/s |

### 2.3 姿態（RPY）

![EKF Attitude](fig05_attitude_rpy.png)

| Metric | Value |
|--------|-------|
| RMSE roll | {np.degrees(metrics['att_rmse_roll']):.2f}° |
| RMSE pitch | {np.degrees(metrics['att_rmse_pitch']):.2f}° |
| RMSE yaw | {np.degrees(metrics['att_rmse_yaw']):.2f}° |
| Final yaw EKF | {ekf_yaw_end:.1f}° |
| Final yaw VICON | {vicon_yaw_end:.1f}° |

### 2.4 加速度計偏差（ba）

![Accel Bias](fig06_accel_bias.png)

| Axis | Initial [m/s²] | SS [m/s²] | Std [m/s²] |
|------|---------------|-----------|------------|
| x | {ba_stats_all[0][0]:.4f} | {ba_stats_all[0][1]:.4f} | {ba_stats_all[0][2]:.5f} |
| y | {ba_stats_all[1][0]:.4f} | {ba_stats_all[1][1]:.4f} | {ba_stats_all[1][2]:.5f} |
| z | {ba_stats_all[2][0]:.4f} | {ba_stats_all[2][1]:.4f} | {ba_stats_all[2][2]:.5f} |

### 2.5 陀螺儀偏差（bw）

![Gyro Bias](fig07_gyro_bias.png)

| Axis | Initial [rad/s] | SS [rad/s] | Std [rad/s] |
|------|----------------|------------|-------------|
| x | {bw_stats_all[0][0]:.5f} | {bw_stats_all[0][1]:.5f} | {bw_stats_all[0][2]:.6f} |
| y | {bw_stats_all[1][0]:.5f} | {bw_stats_all[1][1]:.5f} | {bw_stats_all[1][2]:.6f} |
| z | {bw_stats_all[2][0]:.5f} | {bw_stats_all[2][1]:.5f} | {bw_stats_all[2][2]:.6f} |

---

## 3. 總結

| Component | Value |
|-----------|-------|
| Inner EKF pos 3D RMSE | {metrics['pos_rmse_3d']:.3f} m |
| Inner EKF Vx RMSE | {metrics['vel_rmse_x']:.4f} m/s |
| Inner EKF yaw RMSE | {np.degrees(metrics['att_rmse_yaw']):.2f}° |
| Contact recall（平均） | {cm_mean_recall:.2f} |

*由 analyze.py 自動產生（2026-05-11）。分析時間軌跡窗口：{t_common_min:.2f} – {t_common_max:.2f} s，EKF {len(ekf['t'])} msgs。*
"""

with open(os.path.join(RESULTS, 'analysis_report.md'), 'w', encoding='utf-8') as f:
    f.write(report)
print('Report saved.')
raise SystemExit(0)   # skip dead code below
# ── THIN WRAPPER END — dead code from old implementation follows ───────────────

import sqlite3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import pandas as pd
import os

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion
from corgi_msgs.msg import TriggerStamped, GMOContactStateStamped

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE    = os.path.dirname(os.path.abspath(__file__))
RESULTS = BASE

BAG_DIR = os.path.join(BASE, '..', 'bags', 'replay_div4_20260511_233348')
BAG_DB  = os.path.join(BAG_DIR, 'replay_div4_20260511_233348_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'walk_2m_01.csv')

T_WALK_END = 14.0   # walking phase end (s, EKF-relative)

# ── Load EKF data ─────────────────────────────────────────────────────────────
print(f'Loading bag: {BAG_DB}')
conn = sqlite3.connect(BAG_DB)
cur  = conn.cursor()
cur.execute("SELECT name, id FROM topics")
tmap = {r[0]: r[1] for r in cur.fetchall()}
print('Topics:', list(tmap.keys()))

def fetch(topic):
    cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tmap[topic]} ORDER BY timestamp")
    return cur.fetchall()

rows_ekf = fetch('/ekf')
rows_ori = fetch('/ekf/orientation')
rows_ba  = fetch('/ekf/ba')
rows_bw  = fetch('/ekf/bw')
rows_trg = fetch('/trigger')
rows_gmo = fetch('/gmo/contact_state')
conn.close()

trg_msg = deserialize_message(rows_trg[0][1], TriggerStamped)
t_ros_trigger = trg_msg.header.stamp.sec + trg_msg.header.stamp.nanosec * 1e-9
print(f't_ros_trigger (header): {t_ros_trigger:.3f} s')

# ── t=0 is the trigger header stamp (real experiment time) ───────────────────
# Use header stamps for EKF time axis — replay at --rate 2.0 means storage
# timestamps are 2x compressed relative to real experiment time.
def t_hdr(msg_stamp): return (msg_stamp.sec + msg_stamp.nanosec * 1e-9) - t_ros_trigger

# Parse /ekf
ekf = {k: [] for k in ['t','px','py','pz','vx','vy','vz',
                        'qw','qx','qy','qz',
                        'cov_px','cov_py','cov_pz',
                        'cov_vx','cov_vy','cov_vz']}
for ts, data in rows_ekf:
    msg = deserialize_message(data, Odometry)
    ekf['t'].append(t_hdr(msg.header.stamp))
    ekf['px'].append(msg.pose.pose.position.x)
    ekf['py'].append(msg.pose.pose.position.y)
    ekf['pz'].append(msg.pose.pose.position.z)
    ekf['vx'].append(msg.twist.twist.linear.x)
    ekf['vy'].append(msg.twist.twist.linear.y)
    ekf['vz'].append(msg.twist.twist.linear.z)
    ekf['qw'].append(msg.pose.pose.orientation.w)
    ekf['qx'].append(msg.pose.pose.orientation.x)
    ekf['qy'].append(msg.pose.pose.orientation.y)
    ekf['qz'].append(msg.pose.pose.orientation.z)
    c = msg.pose.covariance
    ekf['cov_px'].append(c[0]); ekf['cov_py'].append(c[7]); ekf['cov_pz'].append(c[14])
    c2 = msg.twist.covariance
    ekf['cov_vx'].append(c2[0]); ekf['cov_vy'].append(c2[7]); ekf['cov_vz'].append(c2[14])
for k in ekf: ekf[k] = np.array(ekf[k])

# Parse bias
# bias topics share the same header stamp cadence as /ekf
# but they are Vector3 (no header) — use storage ts ratio to get header time
# storage → header: multiply by (header_span / storage_span) = 2.0
_storage_span = (rows_ekf[-1][0] - rows_ekf[0][0])
_header_span_ns = (rows_ekf[-1][0] - rows_ekf[0][0])  # placeholder; compute from EKF
# Correct approach: parse ba/bw storage ts relative to ekf storage ts, scale by 2x
_ekf_t0_sto = rows_ekf[0][0]  # first EKF storage timestamp
_rate = 2.0  # replay rate

def t_bias(ts_ns):
    """Convert bias topic storage ts to header-equivalent time (relative to trigger)."""
    return (ts_ns - rows_trg[0][0]) / 1e9 * _rate + (ekf['t'][0] if len(ekf['t']) else 0.0)

ba = {k: [] for k in ['t','x','y','z']}
for ts, data in rows_ba:
    msg = deserialize_message(data, Vector3)
    ba['t'].append((ts - rows_trg[0][0]) / 1e9 * _rate)
    ba['x'].append(msg.x); ba['y'].append(msg.y); ba['z'].append(msg.z)
for k in ba: ba[k] = np.array(ba[k])

bw = {k: [] for k in ['t','x','y','z']}
for ts, data in rows_bw:
    msg = deserialize_message(data, Vector3)
    bw['t'].append((ts - rows_trg[0][0]) / 1e9 * _rate)
    bw['x'].append(msg.x); bw['y'].append(msg.y); bw['z'].append(msg.z)
for k in bw: bw[k] = np.array(bw[k])

print(f'EKF: {len(ekf["t"])} msgs, t=[{ekf["t"][0]:.2f}, {ekf["t"][-1]:.2f}] s')

# Parse GMO
gmo = {'t': [], 'LF': [], 'RF': [], 'RH': [], 'LH': []}
seen_ts = set()
for ts, data in rows_gmo:
    if ts in seen_ts: continue
    seen_ts.add(ts)
    msg = deserialize_message(data, GMOContactStateStamped)
    gmo['t'].append((ts - rows_trg[0][0]) / 1e9 * _rate)
    gmo['LF'].append(msg.module_a.contact)
    gmo['RF'].append(msg.module_b.contact)
    gmo['RH'].append(msg.module_c.contact)
    gmo['LH'].append(msg.module_d.contact)
for k in gmo: gmo[k] = np.array(gmo[k])

# ── Parse VICON CSV ───────────────────────────────────────────────────────────
print('Loading VICON CSV ...')

def find_section_row(filepath, section_name):
    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            if line.strip().startswith(section_name):
                return i
    raise ValueError(f"Section '{section_name}' not found")

def build_marker_col_map(csv_path, traj_section_row):
    raw = pd.read_csv(csv_path, skiprows=traj_section_row + 2,
                      nrows=1, header=None, sep='\t').iloc[0].tolist()
    marker_map = {}
    col = 2
    while col < len(raw):
        name = str(raw[col]).strip()
        if pd.notna(raw[col]) and name and name != 'nan':
            short = name.split(":")[-1]
            marker_map[short] = [col, col+1, col+2]
        col += 3
    return marker_map

traj_row = find_section_row(VICON_CSV, 'Trajectories')
traj_df  = pd.read_csv(VICON_CSV, skiprows=traj_row + 5, header=None, sep='\t')
marker_col_map = build_marker_col_map(VICON_CSV, traj_row)
print('Markers:', list(marker_col_map.keys()))
TRAJ_FS = 500.0

def get_xyz(marker_name):
    cols = marker_col_map[marker_name]
    return traj_df[cols].values.astype(float)

# Ground plane fit
ground_pts = []
for m in ['Ground1', 'Ground2', 'Ground3', 'Ground4']:
    xyz = get_xyz(m)
    valid = ~np.isnan(xyz).any(axis=1)
    if valid.any():
        ground_pts.append(xyz[valid][0])
ground_pts = np.array(ground_pts)
centroid_ground = ground_pts.mean(axis=0)
_, _, Vt = np.linalg.svd(ground_pts - centroid_ground)
normal_ground = Vt[-1]
if normal_ground[2] < 0: normal_ground = -normal_ground

def rotation_to_align_z(n):
    n = n / np.linalg.norm(n)
    z = np.array([0., 0., 1.])
    v = np.cross(n, z); s = np.linalg.norm(v); c = np.dot(n, z)
    if s < 1e-10: return np.eye(3)
    Vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + Vx + Vx @ Vx * ((1-c)/s**2)

R_ground = rotation_to_align_z(normal_ground)

def to_world(p):
    return (R_ground @ (p - centroid_ground).T).T

# ── Detect trigger frame (Tigger marker) ────────────────────────────────────
# Must be done BEFORE robot-centric alignment so trigger frame is used as origin
_tigger_raw = traj_df[marker_col_map['Tigger']].values.astype(float)
_tigger_valid = ~np.isnan(_tigger_raw).any(axis=1)
frame_trig = int(np.where(_tigger_valid)[0][0])
t_vicon_trigger = frame_trig / TRAJ_FS
print(f'Trigger: VICON frame {frame_trig} = {t_vicon_trigger:.3f} s')

# Robot-centric alignment — origin = robot position AT TRIGGER TIME
O1 = to_world(get_xyz('O1')); O2 = to_world(get_xyz('O2'))
O3 = to_world(get_xyz('O3')); O4 = to_world(get_xyz('O4'))
valid_hip = ~(np.isnan(O1).any(axis=1) | np.isnan(O2).any(axis=1) |
              np.isnan(O3).any(axis=1) | np.isnan(O4).any(axis=1))
# Use trigger frame as spatial origin (EKF origin = robot pos at trigger)
ref_frame = frame_trig if valid_hip[frame_trig] else int(np.where(valid_hip)[0][0])
print(f'Robot-centric origin: VICON frame {ref_frame}')
centroid_robot_t0 = np.array([O1[ref_frame], O2[ref_frame],
                               O3[ref_frame], O4[ref_frame]]).mean(axis=0)
heading = O1[ref_frame] - O4[ref_frame]
heading[2] = 0.; heading = heading / np.linalg.norm(heading)
angle = np.arctan2(heading[1], heading[0])
c, s = np.cos(-angle), np.sin(-angle)
R_heading = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])

def to_robot_world(p_vicon):
    p_g = to_world(p_vicon)
    return (R_heading @ (p_g - centroid_robot_t0).T).T

# Kinematics
N = len(traj_df)
O1r = to_robot_world(get_xyz('O1')); O2r = to_robot_world(get_xyz('O2'))
O3r = to_robot_world(get_xyz('O3')); O4r = to_robot_world(get_xyz('O4'))

centroid_vicon = np.full((N, 3), np.nan)
centroid_vicon[valid_hip] = (O1r[valid_hip]+O2r[valid_hip]+O3r[valid_hip]+O4r[valid_hip])/4.0
pos_vicon = centroid_vicon / 1000.0  # mm → m

R_body = np.full((N, 3, 3), np.nan)
for i in np.where(valid_hip)[0]:
    pts = np.stack([O1r[i], O2r[i], O3r[i], O4r[i]])
    cent = pts.mean(axis=0)
    _, _, Vt2 = np.linalg.svd(pts - cent)
    Zb = Vt2[-1]
    if Zb[2] < 0: Zb = -Zb
    x_raw = O1r[i] - O4r[i]
    x_raw -= np.dot(x_raw, Zb) * Zb
    Xb = x_raw / np.linalg.norm(x_raw)
    Yb = np.cross(Zb, Xb); Yb /= np.linalg.norm(Yb)
    R_body[i] = np.column_stack([Xb, Yb, Zb])

rpy_vicon = np.full((N, 3), np.nan)
valid_rot = ~np.isnan(R_body).any(axis=(1,2))
for i in np.where(valid_rot)[0]:
    rpy_vicon[i] = Rotation.from_matrix(R_body[i]).as_euler('ZYX')[::-1]

def sg_velocity(pos, fs=500.0, window=11, poly=3):
    vel = np.full_like(pos, np.nan)
    valid = ~np.isnan(pos).any(axis=1)
    changes = np.diff(valid.astype(int), prepend=0, append=0)
    for st, en in zip(np.where(changes==1)[0], np.where(changes==-1)[0]):
        seg = pos[st:en]
        if len(seg) >= window:
            for ax in range(3):
                vel[st:en, ax] = savgol_filter(seg[:,ax], window, poly, deriv=1, delta=1./fs)
    return vel

v_world = sg_velocity(pos_vicon, fs=500.0)
v_body_vicon = np.full_like(v_world, np.nan)
ok = valid_rot & ~np.isnan(v_world).any(axis=1)
v_body_vicon[ok] = np.einsum('nij,nj->ni', R_body[ok].transpose(0,2,1), v_world[ok])

# ── Time sync (trigger = t=0 for both axes) ─────────────────────────────────
# VICON: trigger frame → t=0;  t_traj_ekf = (frame - frame_trig) / 500
# EKF:   trigger storage ts → t=0  (t0_ns = rows_trg[0][0], see above)
t_traj_ekf = np.arange(N) / TRAJ_FS - t_vicon_trigger  # trigger frame = t=0
print(f'Trigger: VICON frame {frame_trig} = {t_vicon_trigger:.3f}s, '
      f't_ros_trigger (header) = {t_ros_trigger:.3f}s')
print(f'EKF t=0 offset from trigger: {ekf["t"][0]:.4f}s (header-based, should be ~+0.004s)')
print(f'VICON aligned: [{t_traj_ekf[0]:.2f}, {t_traj_ekf[-1]:.2f}] s')
print(f'EKF range:     [{ekf["t"][0]:.2f}, {ekf["t"][-1]:.2f}] s')
t_common_min = max(ekf['t'][0], np.nanmin(t_traj_ekf[valid_hip]))
t_common_max = min(ekf['t'][-1], np.nanmax(t_traj_ekf[valid_hip]))
print(f'Common window: {t_common_min:.2f} – {t_common_max:.2f} s ({t_common_max-t_common_min:.1f} s)')

# ── Interpolate VICON onto EKF time ──────────────────────────────────────────
def interp_vicon(arr):
    vm = ~np.isnan(arr)
    if vm.sum() < 2: return np.full_like(ekf['t'], np.nan)
    return interp1d(t_traj_ekf[vm], arr[vm], bounds_error=False, fill_value=np.nan)(ekf['t'])

vicon_px = interp_vicon(pos_vicon[:,0]); vicon_py = interp_vicon(pos_vicon[:,1])
vicon_pz = interp_vicon(pos_vicon[:,2])
vicon_vx = interp_vicon(v_body_vicon[:,0]); vicon_vy = interp_vicon(v_body_vicon[:,1])
vicon_vz = interp_vicon(v_body_vicon[:,2])
vicon_roll  = interp_vicon(rpy_vicon[:,0])
vicon_pitch = interp_vicon(rpy_vicon[:,1])
vicon_yaw   = interp_vicon(rpy_vicon[:,2])

# EKF RPY
def quat_to_rpy(qw, qx, qy, qz):
    arr = np.column_stack([qx, qy, qz, qw])
    return Rotation.from_quat(arr).as_euler('ZYX')[:, ::-1]
ekf_rpy = quat_to_rpy(ekf['qw'], ekf['qx'], ekf['qy'], ekf['qz'])

# Foot heights
CONTACT_THRESHOLD_M = 0.025
foot_heights = {}
contact_vicon = {}
for leg, marker in [('LF','G1'),('RF','G2'),('RH','G3'),('LH','G4')]:
    h_mm = to_world(get_xyz(marker))[:,2]  # absolute height above ground plane (Z-up, ground=0)
    foot_heights[leg] = h_mm / 1000.0
    contact_vicon[leg] = foot_heights[leg] < CONTACT_THRESHOLD_M

# ── Contact metrics ─────────────────────────────────────────────────────────────────
mask_walk = ekf['t'] < T_WALK_END

t_common_walk = np.linspace(0.0, T_WALK_END, int(T_WALK_END * TRAJ_FS))

def compute_contact_metrics(leg):
    h = foot_heights[leg]
    nnan = ~np.isnan(h)
    t_vi = t_traj_ekf[nnan]
    c_vi = contact_vicon[leg][nnan].astype(float)
    c_gm = np.interp(t_common_walk, gmo['t'], gmo[leg].astype(float))
    c_vi_on_walk = np.interp(t_common_walk, t_vi, c_vi) > 0.5
    c_gm_on_walk = c_gm > 0.5
    tp = np.sum(c_vi_on_walk & c_gm_on_walk)
    fp = np.sum(~c_vi_on_walk & c_gm_on_walk)
    fn = np.sum(c_vi_on_walk & ~c_gm_on_walk)
    tn = np.sum(~c_vi_on_walk & ~c_gm_on_walk)
    prec = tp / (tp + fp + 1e-9)
    rec  = tp / (tp + fn + 1e-9)
    stance_ratio = float(np.mean(c_vi_on_walk)) * 100.0
    # Mean stance duration
    edges = np.diff(c_vi_on_walk.astype(int), prepend=0, append=0)
    starts = np.where(edges == 1)[0]; ends = np.where(edges == -1)[0]
    durations = [(ends[i] - starts[i]) / TRAJ_FS * 1000 for i in range(min(len(starts), len(ends)))]
    mean_dur = float(np.mean(durations)) if durations else 0.0
    # Mean detection latency (rising edge delay)
    v_edges = np.diff(c_vi_on_walk.astype(int), prepend=0)
    g_edges = np.diff(c_gm_on_walk.astype(int), prepend=0)
    lats = []
    for vs in np.where(v_edges == 1)[0]:
        window = g_edges[max(0,vs-25):vs+25]
        rel = np.where(window == 1)[0]
        if len(rel): lats.append((rel[0] - min(25,vs)) / TRAJ_FS * 1000)
    mean_lat = float(np.mean(lats)) if lats else float('nan')
    return {'precision': prec, 'recall': rec, 'stance_ratio': stance_ratio,
            'mean_stance_ms': mean_dur, 'mean_latency_ms': mean_lat}

cm = {leg: compute_contact_metrics(leg) for leg in ['LF','RF','RH','LH']}
print('\n=== Contact Metrics ===')
for leg, m in cm.items():
    print(f'  {leg}: P={m["precision"]:.2f} R={m["recall"]:.2f} lat={m["mean_latency_ms"]:.1f}ms stance={m["stance_ratio"]:.1f}%')

# ── Bias convergence values ────────────────────────────────────────────────────────────
def bias_stats(data, axis, n_init=50, n_ss=200):
    v = data[axis]
    init = float(v[:n_init].mean()) if len(v) >= n_init else float('nan')
    ss   = float(v[-n_ss:].mean()) if len(v) >= n_ss else float('nan')
    std  = float(v[-n_ss:].std())  if len(v) >= n_ss else float('nan')
    return init, ss, std


def rmse(a, b, mask=None):
    d = a - b
    if mask is not None: d = d[mask]
    v = ~np.isnan(d)
    return float(np.sqrt(np.mean(d[v]**2))) if v.any() else float('nan')

err_x = ekf['px'] - vicon_px; err_y = ekf['py'] - vicon_py; err_z = ekf['pz'] - vicon_pz
err_3d = np.sqrt(np.where(~np.isnan(err_x)&~np.isnan(err_y)&~np.isnan(err_z),
                           err_x**2+err_y**2+err_z**2, np.nan))
valid_pos = ~np.isnan(err_3d) & mask_walk

metrics = {
    'pos_rmse_x':  rmse(ekf['px'], vicon_px, valid_pos),
    'pos_rmse_y':  rmse(ekf['py'], vicon_py, valid_pos),
    'pos_rmse_z':  rmse(ekf['pz'], vicon_pz, valid_pos),
    'pos_rmse_3d': float(np.sqrt(np.nanmean(err_3d[valid_pos]**2))) if valid_pos.any() else float('nan'),
    'pos_max_3d':  float(np.nanmax(err_3d[valid_pos])) if valid_pos.any() else float('nan'),
    'vel_rmse_x':  rmse(ekf['vx'], vicon_vx, mask_walk & ~np.isnan(vicon_vx)),
    'vel_rmse_y':  rmse(ekf['vy'], vicon_vy, mask_walk & ~np.isnan(vicon_vy)),
    'vel_rmse_z':  rmse(ekf['vz'], vicon_vz, mask_walk & ~np.isnan(vicon_vz)),
    'att_rmse_roll':  rmse(ekf_rpy[:,0], vicon_roll,  mask_walk & ~np.isnan(vicon_roll)),
    'att_rmse_pitch': rmse(ekf_rpy[:,1], vicon_pitch, mask_walk & ~np.isnan(vicon_pitch)),
    'att_rmse_yaw':   rmse(ekf_rpy[:,2], vicon_yaw,   mask_walk & ~np.isnan(vicon_yaw)),
    'vel_peak_vx': float(np.nanmax(np.abs(vicon_vx[mask_walk & ~np.isnan(vicon_vx)]))),
}
print('\n=== Metrics (div/4 algorithm) ===')
print(f"  Position RMSE: X={metrics['pos_rmse_x']*1000:.1f}mm  Y={metrics['pos_rmse_y']*1000:.1f}mm  Z={metrics['pos_rmse_z']*1000:.1f}mm  3D={metrics['pos_rmse_3d']*1000:.1f}mm")
print(f"  Velocity RMSE: Vx={metrics['vel_rmse_x']*1000:.1f}mm/s  Vy={metrics['vel_rmse_y']*1000:.1f}mm/s  Vz={metrics['vel_rmse_z']*1000:.1f}mm/s")
print(f"  Attitude RMSE: Roll={np.degrees(metrics['att_rmse_roll']):.2f}°  Pitch={np.degrees(metrics['att_rmse_pitch']):.2f}°  Yaw={np.degrees(metrics['att_rmse_yaw']):.2f}°")
print(f"  Vx RMSE = {metrics['vel_rmse_x']:.4f} m/s  (target: ~0.027 m/s)")

last_ekf_x = ekf['px'][-1]; last_ekf_y = ekf['py'][-1]
last_vi = np.where(valid_hip)[0][-1]
last_vx = pos_vicon[last_vi,0]; last_vy = pos_vicon[last_vi,1]
print(f"  EKF final: ({last_ekf_x:.3f}, {last_ekf_y:.3f}) m  dist={np.hypot(last_ekf_x,last_ekf_y):.3f}m")
print(f"  VICON final: ({last_vx:.3f}, {last_vy:.3f}) m  dist={np.hypot(last_vx,last_vy):.3f}m")

# Final yaw values
vicon_yaw_final = vicon_yaw[mask_walk & ~np.isnan(vicon_yaw)]
vicon_yaw_end = float(np.degrees(vicon_yaw_final[-1])) if len(vicon_yaw_final) else float('nan')
ekf_yaw_end   = float(np.degrees(ekf_rpy[:,2][mask_walk][-1]))

# ── Plots ─────────────────────────────────────────────────────────────────────
plt.rcParams.update({'figure.dpi': 120, 'font.size': 9})

# Fig 1: XY trajectory
fig, ax = plt.subplots(figsize=(7,5))
ax.plot(ekf['px'], ekf['py'], label='EKF (div/4)', lw=1.5, color='#E53935')
vi = ~np.isnan(pos_vicon[:,0])
ax.plot(pos_vicon[vi,0], pos_vicon[vi,1], label='VICON', lw=1.5, color='#1E88E5', ls='--')
ax.set_aspect('equal'); ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_title('XY Trajectory: EKF(div/4) vs VICON')
ax.legend(); ax.grid(True, alpha=0.3); plt.tight_layout()
plt.savefig(os.path.join(RESULTS, 'fig01_trajectory_xy.png')); plt.close()

# Fig 2: Position time series
fig, axes = plt.subplots(3, 1, figsize=(12,7), sharex=True)
for ax, lbl, ep, vp, cv in zip(axes, ['X','Y','Z'],
    [ekf['px'],ekf['py'],ekf['pz']], [vicon_px,vicon_py,vicon_pz],
    [np.sqrt(np.abs(ekf['cov_px'])),np.sqrt(np.abs(ekf['cov_py'])),np.sqrt(np.abs(ekf['cov_pz']))]):
    ax.plot(ekf['t'], ep, label='EKF', color='#E53935', lw=1.2)
    ax.plot(ekf['t'], vp, label='VICON', color='#1E88E5', lw=1.2, ls='--')
    ax.fill_between(ekf['t'], ep-3*cv, ep+3*cv, alpha=0.2, color='#E53935')
    ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
    ax.set_ylabel(f'{lbl} [m]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
axes[0].set_title('Position: EKF(div/4) vs VICON')
plt.tight_layout(); plt.savefig(os.path.join(RESULTS,'fig02_position_timeseries.png')); plt.close()

# Fig 3: 3D position error
fig, ax = plt.subplots(figsize=(12,3))
ax.plot(ekf['t'], err_3d*1000, lw=1.0, color='#9C27B0')
ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
ax.set_xlabel('Time [s]'); ax.set_ylabel('3D error [mm]')
ax.set_title(f'3D Position Error — RMSE={metrics["pos_rmse_3d"]*1000:.1f}mm')
ax.grid(True, alpha=0.3); plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig03_position_error.png')); plt.close()

# Fig 4: Velocity
fig, axes = plt.subplots(3, 1, figsize=(12,7), sharex=True)
for ax, lbl, ev, vv, cv in zip(axes, ['Vx','Vy','Vz'],
    [ekf['vx'],ekf['vy'],ekf['vz']], [vicon_vx,vicon_vy,vicon_vz],
    [np.sqrt(np.abs(ekf['cov_vx'])),np.sqrt(np.abs(ekf['cov_vy'])),np.sqrt(np.abs(ekf['cov_vz']))]):
    ax.plot(ekf['t'], ev, label='EKF', color='#E53935', lw=1.2)
    ax.plot(ekf['t'], vv, label='VICON SG', color='#1E88E5', lw=1.2, ls='--')
    ax.fill_between(ekf['t'], ev-3*cv, ev+3*cv, alpha=0.2, color='#E53935')
    ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
    ax.set_ylabel(f'{lbl} [m/s]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
axes[0].set_title(f'Body-Frame Velocity: EKF(div/4) vs VICON  |  Vx RMSE={metrics["vel_rmse_x"]*1000:.1f}mm/s')
plt.tight_layout(); plt.savefig(os.path.join(RESULTS,'fig04_velocity_timeseries.png')); plt.close()

# Fig 5: Attitude
fig, axes = plt.subplots(3, 1, figsize=(12,7), sharex=True)
for i, (lbl, ea, va) in enumerate(zip(['Roll','Pitch','Yaw'],
        [ekf_rpy[:,0],ekf_rpy[:,1],ekf_rpy[:,2]],
        [vicon_roll,vicon_pitch,vicon_yaw])):
    axes[i].plot(ekf['t'], np.degrees(ea), label='EKF', color='#E53935', lw=1.2)
    axes[i].plot(ekf['t'], np.degrees(va), label='VICON', color='#1E88E5', lw=1.2, ls='--')
    axes[i].axvline(T_WALK_END, color='gray', ls=':', lw=1)
    axes[i].set_ylabel(f'{lbl} [°]'); axes[i].legend(fontsize=8); axes[i].grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
axes[0].set_title('Attitude (RPY): EKF(div/4) vs VICON')
plt.tight_layout(); plt.savefig(os.path.join(RESULTS,'fig05_attitude_rpy.png')); plt.close()

# Fig 6: Bias
for fname, data, ylabel, title in [
    ('fig06_accel_bias.png', ba, 'Accel bias [m/s²]', 'Accelerometer Bias ba'),
    ('fig07_gyro_bias.png',  bw, 'Gyro bias [rad/s]', 'Gyroscope Bias bw')]:
    fig, ax = plt.subplots(figsize=(12,3))
    for axis, color in zip(['x','y','z'], ['#E53935','#1E88E5','#43A047']):
        ax.plot(data['t'], data[axis], label=f'{axis}', color=color, lw=1.0)
    ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
    ax.set_xlabel('Time [s]'); ax.set_ylabel(ylabel); ax.set_title(title)
    ax.legend(); ax.grid(True, alpha=0.3); plt.tight_layout()
    plt.savefig(os.path.join(RESULTS, fname)); plt.close()

# Fig 8: Foot heights
fig, axes = plt.subplots(4,1,figsize=(12,8),sharex=True)
for ax, leg in zip(axes, ['LF','RF','RH','LH']):
    ax.plot(t_traj_ekf, foot_heights[leg]*1000, lw=0.8, color='#1E88E5')
    ax.axhline(CONTACT_THRESHOLD_M*1000, color='red', ls='--', lw=1)
    ax.set_ylabel(f'{leg} Z [mm]'); ax.grid(True, alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle('VICON Foot Height vs Contact Threshold (div/4)')
plt.tight_layout(); plt.savefig(os.path.join(RESULTS,'fig08_foot_heights.png')); plt.close()

# Fig 9: Contact timeline
def contact_events(arr, t_arr):
    arr_f = arr.astype(float)
    edges = np.diff(arr_f, prepend=0, append=0)
    si = np.where(edges > 0.5)[0]; ei = np.where(edges < -0.5)[0]
    si = si[si < len(t_arr)]; ei = ei[ei < len(t_arr)]
    starts = t_arr[si]; ends = t_arr[ei]
    if len(ends) < len(starts): ends = np.append(ends, t_arr[-1])
    return list(zip(starts, ends))

fig, axes = plt.subplots(4,1,figsize=(14,6),sharex=True)
for ax, leg in zip(axes, ['LF','RF','RH','LH']):
    nnan = ~np.isnan(foot_heights[leg])
    for t_s, t_e in contact_events(contact_vicon[leg][nnan], t_traj_ekf[nnan]):
        ax.axvspan(t_s, t_e, color='#2196F3', alpha=0.4)
    for t_s, t_e in contact_events(gmo[leg].astype(bool), gmo['t']):
        ax.axvspan(t_s, t_e, color='#FF5722', alpha=0.3, hatch='//')
    ax.set_ylabel(leg, rotation=0, labelpad=20)
    ax.set_ylim(-0.1,1.1); ax.set_yticks([]); ax.axvline(T_WALK_END, color='gray', ls=':', lw=1)
from matplotlib.patches import Patch
axes[0].legend(handles=[Patch(fc='#2196F3',alpha=0.5,label='VICON'),
                         Patch(fc='#FF5722',alpha=0.5,hatch='//',label='GMO')],
               loc='upper right', fontsize=8)
axes[0].set_title('Contact Timeline: VICON vs GMO (div/4)')
axes[-1].set_xlabel('Time [s]')
plt.tight_layout(); plt.savefig(os.path.join(RESULTS,'fig09_contact_timeline.png')); plt.close()

print('\nAll plots saved.')

# ── Markdown report (full template format) ────────────────────────────────────
# Bias tables
ba_stats_all = [bias_stats(ba, ax) for ax in ['x','y','z']]
bw_stats_all = [bias_stats(bw, ax) for ax in ['x','y','z']]

# Contact summary
cm_mean_recall = float(np.mean([cm[l]['recall'] for l in ['LF','RF','RH','LH']]))

report = f"""# CORGI Experiment Analysis Report

**Date:** 2026-05-11
**Experiment:** `walk_2m_01_div4`
**Bag (replay output):** `replay_div4_20260511_233348`
**Bag (replay input):** `leg_odom20260507_161231`
**VICON CSV:** `walk_2m_01.csv`
**步行階段:** t = 0 – {T_WALK_END:.0f} s
**Analysis script:** `analyze.py`
**演算法修改:** `theta_d` / `beta_d` 除數：2 → 4（對照原始 bag replay）

---

## System Architecture

```
/imu_raw ─┐
/motor/state ──┤──► corgi_leg_odom (div/4) ──► /ekf ──────────────── (odom→base_link)
/trigger ──────┘
/gmo/contact_state
```

> 輸入 bag 已確認為原始未修改的 `leg_odom20260507_161231`，只播放 `/imu_raw /motor/state /trigger /gmo/contact_state`。

---

## 1. 觸地偵測（Contact Detection）

### 1.1 VICON 地面真度觸地判斷（G1–G4）

觸地閾值：**{CONTACT_THRESHOLD_M*1000:.0f} mm**。

![Contact Timeline](fig09_contact_timeline.png)

| 腿 | Stance ratio（步行階段） | 平均 stance 持續時間 [ms] |
|-----|--------------------------|---------------------------|
| LF (G1) | {cm['LF']['stance_ratio']:.1f}% | {cm['LF']['mean_stance_ms']:.0f} |
| RF (G2) | {cm['RF']['stance_ratio']:.1f}% | {cm['RF']['mean_stance_ms']:.0f} |
| RH (G3) | {cm['RH']['stance_ratio']:.1f}% | {cm['RH']['mean_stance_ms']:.0f} |
| LH (G4) | {cm['LH']['stance_ratio']:.1f}% | {cm['LH']['mean_stance_ms']:.0f} |

### 1.2 GMO 與 VICON 觸地比較

![Foot Height](fig08_foot_heights.png)

| 腿 | Precision | Recall | Mean Latency [ms] |
|-----|-----------|--------|-------------------|
| LF  | {cm['LF']['precision']:.2f}      | {cm['LF']['recall']:.2f}   | {cm['LF']['mean_latency_ms']:.1f}              |
| RF  | {cm['RF']['precision']:.2f}      | {cm['RF']['recall']:.2f}   | {cm['RF']['mean_latency_ms']:.1f}              |
| RH  | {cm['RH']['precision']:.2f}      | {cm['RH']['recall']:.2f}   | {cm['RH']['mean_latency_ms']:.1f}              |
| LH  | {cm['LH']['precision']:.2f}      | {cm['LH']['recall']:.2f}   | {cm['LH']['mean_latency_ms']:.1f}              |

**觀察：**
- GMO 對四腿均能偵測到主要的觸地事件，Recall 平均 {cm_mean_recall:.2f}。
- div/4 修改對腿部接觸估計演算法未有直接影響（GMO 使用獨立的資料流）。

---

## 2. Inner EKF 分析

### 2.1 位置

![EKF Position XY](fig01_trajectory_xy.png)
![EKF Position Time](fig02_position_timeseries.png)

| Metric | Value |
|--------|-------|
| RMSE X (vs VICON) | {metrics['pos_rmse_x']:.3f} m |
| RMSE Y (vs VICON) | {metrics['pos_rmse_y']:.3f} m |
| RMSE Z (vs VICON) | {metrics['pos_rmse_z']:.3f} m |
| RMSE 3D (vs VICON) | {metrics['pos_rmse_3d']:.3f} m |
| Max 3D error | {metrics['pos_max_3d']:.3f} m |
| Final position (EKF) | ({last_ekf_x:.3f}, {last_ekf_y:.3f}) m |
| Final position (VICON) | ({last_vx:.3f}, {last_vy:.3f}) m |

**觀察：**
- X 軸偏差最大（{metrics['pos_rmse_x']*1000:.0f} mm），EKF 位移 {np.hypot(last_ekf_x,last_ekf_y):.3f} m 超過 VICON {np.hypot(last_vx,last_vy):.3f} m，前進速度被高估。
- 原因：div/4 使腿部里程計的車輪速度觀測縮小一半，IMU 積分主導導致漂移。

### 2.2 速度

![EKF Velocity](fig04_velocity_timeseries.png)

| Metric | Value |
|--------|-------|
| RMSE vx (vs VICON) | {metrics['vel_rmse_x']:.3f} m/s |
| RMSE vy (vs VICON) | {metrics['vel_rmse_y']:.3f} m/s |
| RMSE vz (vs VICON) | {metrics['vel_rmse_z']:.3f} m/s |
| Peak forward speed (VICON) | {metrics['vel_peak_vx']:.2f} m/s |

**觀察：**
- Vx RMSE = {metrics['vel_rmse_x']:.4f} m/s，遠高於目標 ~0.027 m/s（div/2 的結果）。
- EKF 對前進速度持續高估，表明腿部里程計的速度觀測量不正確。

### 2.3 姿態（RPY）

![EKF Attitude](fig05_attitude_rpy.png)

| Metric | Value |
|--------|-------|
| RMSE roll (vs VICON) | {np.degrees(metrics['att_rmse_roll']):.2f}° |
| RMSE pitch (vs VICON) | {np.degrees(metrics['att_rmse_pitch']):.2f}° |
| RMSE yaw (vs VICON) | {np.degrees(metrics['att_rmse_yaw']):.2f}° |
| Final yaw (EKF) | {ekf_yaw_end:.1f}° |
| Final yaw (VICON) | {vicon_yaw_end:.1f}° |

**觀察：**
- Roll/Pitch RMSE < 1°，表示姿態估計演算法對此修改影響小。
- Yaw 漂移較小，主要誤差集中在前進速度。

### 2.4 加速度計偏差（ba）

![Accel Bias](fig06_accel_bias.png)

| Axis | Initial [m/s²] | Steady-state [m/s²] | Std [m/s²] |
|------|---------------|---------------------|------------|
| x    | {ba_stats_all[0][0]:.4f}        | {ba_stats_all[0][1]:.4f}              | {ba_stats_all[0][2]:.5f}    |
| y    | {ba_stats_all[1][0]:.4f}        | {ba_stats_all[1][1]:.4f}              | {ba_stats_all[1][2]:.5f}    |
| z    | {ba_stats_all[2][0]:.4f}        | {ba_stats_all[2][1]:.4f}              | {ba_stats_all[2][2]:.5f}    |

### 2.5 陀螺儀偏差（bw）

![Gyro Bias](fig07_gyro_bias.png)

| Axis | Initial [rad/s] | Steady-state [rad/s] | Std [rad/s] |
|------|----------------|----------------------|-------------|
| x    | {bw_stats_all[0][0]:.5f}        | {bw_stats_all[0][1]:.5f}              | {bw_stats_all[0][2]:.6f}    |
| y    | {bw_stats_all[1][0]:.5f}        | {bw_stats_all[1][1]:.5f}              | {bw_stats_all[1][2]:.6f}    |
| z    | {bw_stats_all[2][0]:.5f}        | {bw_stats_all[2][1]:.5f}              | {bw_stats_all[2][2]:.6f}    |

**觀察：**
- 加速度計偏差在步行期間持續收斂，z 軸偏差流程與重力即時偏差有關。
- 陀螺儀偏差較小，表示專式 IMU 結構。

---

## 3. Outer Fusion Node

> 本實驗未包含 LiDAR 輸入，第 3、4 章不適用。

---

## 5. 總結

### Key Metrics

| Component | RMSE (vs VICON) | Note |
|-----------|----------------|------|
| Inner EKF position 3D | {metrics['pos_rmse_3d']:.3f} m | 步行階段 |
| Inner EKF velocity Vx | {metrics['vel_rmse_x']:.3f} m/s | 步行階段 |
| Inner EKF yaw | {np.degrees(metrics['att_rmse_yaw']):.2f}° | |
| Contact recall（平均） | {cm_mean_recall:.2f} | |

### Key Findings

1. **觸地偵測**：GMO 對四腿均能正常運作，平均 Recall {cm_mean_recall:.2f}，div/4 修改未影響觸圖。
2. **Inner EKF**：div/4 將腿部里程計的車輪速度觀測縮小一半，Vx RMSE = {metrics['vel_rmse_x']:.3f} m/s（高於目標 0.027 m/s），位置漂移明顯（{metrics['pos_rmse_3d']*1000:.0f} mm）。
3. **結論**：div/2 才是正確的除數，已復原並重新編譯。

### Recommendations

- [x] 已復原 DataProcessor.cpp 為 /2.0 並重新編譯
- [ ] 若需進一步驗證，可用相同 input bag replay div/2 並比較 Vx RMSE

---

*由 analyze.py 自動產生（2026-05-11）。分析時間軌跡窗口：{t_common_min:.2f} – {t_common_max:.2f} s，EKF {len(ekf['t'])} msgs。*
"""
with open(os.path.join(RESULTS,'analysis_report.md'), 'w', encoding='utf-8') as f:
    f.write(report)
print('Report saved.')
