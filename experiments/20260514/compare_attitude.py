"""
compare_attitude.py
比較 ESEKF 姿態估測 vs CX5 AHRS IMU 姿態 vs VICON ground truth

比較對象：
  exp2/exp4 (ESEKF)  → /ekf/orientation (geometry_msgs/Quaternion, headerless)
  exp3/exp6 (Legacy) → /imu orientation (corgi_msgs/ImuStamped, CX5 AHRS filter)

NOTE: /imu_raw 不包含姿態估測，不分析。

VICON: EXP_02/03/04/06_z_corrected.csv

Output: ablation_result/fig_compare_attitude.png
        ablation_result/fig_compare_yaw.png
        ablation_result/attitude_rmse.txt
"""

import sys, os
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../tools'))

# ── ROS2 message types ────────────────────────────────────────────────────────
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Quaternion as QuaternionMsg
from nav_msgs.msg import Odometry
from corgi_msgs.msg import ImuStamped, TriggerStamped

# ── VICON loader ──────────────────────────────────────────────────────────────
from corgi_analysis.vicon_loader import load_vicon

# ─────────────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(__file__)
OUT  = os.path.join(BASE, 'ablation_result')
os.makedirs(OUT, exist_ok=True)

GROUND_MARKERS = ['ground1', 'ground2', 'ground3', 'ground4']

# exp2/exp4: ESEKF — /ekf/orientation (headerless) + /ekf (for timestamps) + /imu_raw (NOT used for attitude)
# exp3/exp6: Legacy — /imu (CX5 AHRS, has header stamp + orientation)
EXPS = {
    'exp2': {
        'bag':        os.path.join(BASE, 'exp2/bags/odom_fusion20260514_220252'),
        'vicon':      os.path.join(BASE, 'exp2/vicon/EXP_02.csv'),
        'source':     'esekf',
        'label':      'exp2  ESEKF (plain)',
        'color':      '#2196F3',
        # ESEKF outputs in ROS2 ENU: X=fwd, Y=left, Z=up → matches VICON directly
        'pitch_sign': +1,
        'yaw_sign':   +1,
    },
    'exp4': {
        'bag':        os.path.join(BASE, 'exp4/bags/odom_fusion20260514_225104'),
        'vicon':      os.path.join(BASE, 'exp4/vicon/EXP_04.csv'),
        'source':     'esekf',
        'label':      'exp4  ESEKF (obs)',
        'color':      '#1565C0',
        'pitch_sign': +1,
        'yaw_sign':   +1,
    },
    'exp3': {
        'bag':        os.path.join(BASE, 'exp3/bags/legacy_odom20260514_222433'),
        'vicon':      os.path.join(BASE, 'exp3/vicon/EXP_03.csv'),
        'source':     'legacy',
        'label':      'exp3  Legacy CX5 (plain)',
        'color':      '#F44336',
        # CX5 AHRS body frame: X=fwd, Y=right, Z=down
        # → pitch and yaw are sign-flipped vs VICON (ROS2 ENU: Y=left, Z=up)
        'pitch_sign': -1,
        'yaw_sign':   -1,
    },
    'exp6': {
        'bag':        os.path.join(BASE, 'exp6/bags/legacy_odom20260514_232823'),
        'vicon':      os.path.join(BASE, 'exp6/vicon/EXP_06_z_corrected.csv'),
        'source':     'legacy',
        'label':      'exp6  Legacy CX5 (obs)',
        'color':      '#B71C1C',
        'pitch_sign': -1,
        'yaw_sign':   -1,
    },
}


# ─────────────────────────────────────────────────────────────────────────────
def find_db3(bag_dir):
    for f in os.listdir(bag_dir):
        if f.endswith('.db3'):
            return os.path.join(bag_dir, f)
    raise FileNotFoundError(f"No .db3 in {bag_dir}")


def open_bag(db3):
    conn = sqlite3.connect(db3)
    cur  = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    return conn, cur, tmap


def fetch(cur, tmap, topic):
    tid = tmap.get(topic)
    if tid is None:
        return []
    cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp")
    return cur.fetchall()


def parse_trigger(rows):
    for ts, data in rows:
        msg = deserialize_message(data, TriggerStamped)
        if msg.enable:
            t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            return t_sec, ts
    raise RuntimeError("No trigger ON found")


def quat_rows_to_rpy(rows):
    """geometry_msgs/Quaternion rows (headerless) → arrays of roll/pitch/yaw [deg]."""
    roll_list, pitch_list, yaw_list = [], [], []
    for _, data in rows:
        q = deserialize_message(data, QuaternionMsg)
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        rpy = r.as_euler('xyz', degrees=True)
        roll_list.append(rpy[0])
        pitch_list.append(rpy[1])
        yaw_list.append(rpy[2])
    return np.array(roll_list), np.array(pitch_list), np.array(yaw_list)


def imu_rows_to_rpy(rows, t_ros_trigger):
    """corgi_msgs/ImuStamped rows → (t, roll, pitch, yaw [deg]) using header stamp."""
    t_list, roll_list, pitch_list, yaw_list = [], [], [], []
    for _, data in rows:
        msg = deserialize_message(data, ImuStamped)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t_ros_trigger
        q = msg.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        rpy = r.as_euler('xyz', degrees=True)
        t_list.append(t)
        roll_list.append(rpy[0])
        pitch_list.append(rpy[1])
        yaw_list.append(rpy[2])
    return np.array(t_list), np.array(roll_list), np.array(pitch_list), np.array(yaw_list)


def load_exp_data(exp_key, cfg):
    db3 = find_db3(cfg['bag'])
    conn, cur, tmap = open_bag(db3)
    rows_trg    = fetch(cur, tmap, '/trigger')
    rows_ekf    = fetch(cur, tmap, '/ekf')
    rows_orient = fetch(cur, tmap, '/ekf/orientation')
    rows_imu    = fetch(cur, tmap, '/imu')
    conn.close()

    t_ros_trigger, _ = parse_trigger(rows_trg)

    if cfg['source'] == 'esekf':
        # Time axis from /ekf (has header); orientation from /ekf/orientation (headerless, same count)
        t_list = []
        for _, data in rows_ekf:
            msg = deserialize_message(data, Odometry)
            t_list.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t_ros_trigger)
        n = min(len(rows_orient), len(t_list))
        roll_e, pitch_e, yaw_e = quat_rows_to_rpy(rows_orient[:n])
        att = {'t': np.array(t_list[:n]),
               'roll':  roll_e,
               'pitch': pitch_e * cfg['pitch_sign'],
               'yaw':   yaw_e   * cfg['yaw_sign']}

    else:  # legacy: CX5 AHRS from /imu
        t_i, roll_i, pitch_i, yaw_i = imu_rows_to_rpy(rows_imu, t_ros_trigger)
        att = {'t': t_i,
               'roll':  roll_i,
               'pitch': pitch_i * cfg['pitch_sign'],
               'yaw':   yaw_i   * cfg['yaw_sign']}

    # VICON
    vi = load_vicon(cfg['vicon'], ground_markers=GROUND_MARKERS)
    t_vi   = vi.t_traj
    rpy_vi = np.degrees(vi.rpy)
    t_end  = vi.t_trigger_end if vi.t_trigger_end > 0 else t_vi[-1]
    mask   = (t_vi >= 0) & (t_vi <= t_end)

    return {
        'att':   att,
        'vicon': {'t': t_vi[mask], 'roll': rpy_vi[mask, 0],
                  'pitch': rpy_vi[mask, 1], 'yaw': rpy_vi[mask, 2]},
        'label':  cfg['label'],
        'color':  cfg['color'],
        'source': cfg['source'],
    }


def interp_to(t_ref, t_src, vals):
    mask = (t_ref >= t_src[0]) & (t_ref <= t_src[-1])
    f = interp1d(t_src, vals, kind='linear')
    out = np.full(len(t_ref), np.nan)
    out[mask] = f(t_ref[mask])
    return out, mask


def rmse_masked(pred, gt):
    diff = pred - gt
    valid = ~np.isnan(diff)
    return np.sqrt(np.mean(diff[valid] ** 2))


def initial_offset(t_vi, att_t, att_val):
    """Compute estimator_value(t=0) - vicon_value(t=0) for offset alignment."""
    t0 = t_vi[0]
    # VICON value at t=0 (first sample)
    vi_t0 = float(interp1d(t_vi, np.arange(len(t_vi)), kind='nearest')(t0))
    vicon_v0 = float(interp1d(t_vi, np.zeros(len(t_vi)) + t_vi[0]*0)(t0))  # dummy
    # Use first VICON sample directly
    idx = np.argmin(np.abs(t_vi - t0))
    vi_v0 = 0.0  # we want both to be aligned to VICON at t0; compute est offset below

    # Estimator interpolated at t=0
    if t0 < att_t[0] or t0 > att_t[-1]:
        # Use nearest value
        nearest_idx = np.argmin(np.abs(att_t - t0))
        est_v0 = att_val[nearest_idx]
    else:
        est_v0 = float(interp1d(att_t, att_val, kind='linear')(t0))

    # VICON value at t=0
    vicon_v0 = float(interp1d(t_vi, np.array([d['vicon'] for d in []]))(t0)) \
        if False else 0.0  # placeholder — filled per-angle below
    return est_v0  # return just the estimator value at t=0; caller does alignment


# ─────────────────────────────────────────────────────────────────────────────
print("Loading experiment data...")
data = {}
for key, cfg in EXPS.items():
    print(f"  {key}...")
    data[key] = load_exp_data(key, cfg)

# ── Initial-offset alignment: align estimator to VICON at t=0 ────────────────
# For each exp and each angle: subtract (est(t=0) - vicon(t=0)) from the estimator.
# This removes absolute heading offset; remaining error = pure drift.
for key, d in data.items():
    t_vi  = d['vicon']['t']
    att   = d['att']
    t0    = t_vi[0]
    for a_key in ['roll', 'pitch', 'yaw']:
        # VICON value at t=0
        vi_v0 = float(interp1d(t_vi, d['vicon'][a_key], kind='linear')(t0))
        # Estimator value at t=0
        t_s = att['t']
        if t0 < t_s[0]:
            est_v0 = att[a_key][0]
        elif t0 > t_s[-1]:
            est_v0 = att[a_key][-1]
        else:
            est_v0 = float(interp1d(t_s, att[a_key], kind='linear')(t0))
        # Remove offset from estimator
        att[a_key] = att[a_key] - (est_v0 - vi_v0)

# ─────────────────────────────────────────────────────────────────────────────
# RMSE table
# ─────────────────────────────────────────────────────────────────────────────
print(f"\n{'Exp':<6} {'Source':<22} {'Roll':>8} {'Pitch':>8} {'Yaw':>8}  (RMSE vs VICON [deg])")
print("-" * 60)
lines = ["Attitude RMSE vs VICON [degrees]\n",
         f"{'Exp':<6} {'Source':<22} {'Roll [°]':>10} {'Pitch [°]':>10} {'Yaw [°]':>10}\n",
         "-" * 62 + "\n"]

for key in ['exp2', 'exp4', 'exp3', 'exp6']:
    d = data[key]
    t_vi = d['vicon']['t']
    att  = d['att']
    src_label = 'ESEKF /ekf/orientation' if d['source'] == 'esekf' else 'CX5 AHRS /imu'

    r_i, _ = interp_to(t_vi, att['t'], att['roll'])
    p_i, _ = interp_to(t_vi, att['t'], att['pitch'])
    y_i, _ = interp_to(t_vi, att['t'], att['yaw'])

    r_rmse = rmse_masked(r_i, d['vicon']['roll'])
    p_rmse = rmse_masked(p_i, d['vicon']['pitch'])
    y_rmse = rmse_masked(y_i, d['vicon']['yaw'])

    row = f"{key:<6} {src_label:<22} {r_rmse:>8.3f}° {p_rmse:>8.3f}° {y_rmse:>8.3f}°"
    print(row)
    lines.append(row + "\n")

    # cache for plotting
    data[key]['rmse'] = {'roll': r_rmse, 'pitch': p_rmse, 'yaw': y_rmse}

with open(os.path.join(OUT, 'attitude_rmse.txt'), 'w') as f:
    f.writelines(lines)
print(f"\nSaved attitude_rmse.txt")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 1: Roll & Pitch time series  (4 rows × 2 cols)
# ─────────────────────────────────────────────────────────────────────────────
exp_order  = ['exp2', 'exp4', 'exp3', 'exp6']
angle_keys = ['roll', 'pitch']
angle_lbls = ['Roll [°]', 'Pitch [°]']

fig, axes = plt.subplots(4, 2, figsize=(14, 13))
fig.suptitle(
    'Attitude: ESEKF /ekf/orientation  vs  CX5 AHRS /imu  vs  VICON GT\n'
    'Blue = ESEKF (ROS2 ENU, no correction)   Red = CX5 AHRS (pitch & yaw sign-corrected: Y=right→left, Z=down→up)\n'
    'MPC frame: X=fwd  Y=left  Z=up  |  roll(+)=右傾  pitch(+)=前仰  yaw(+)=左轉',
    fontsize=10, fontweight='bold')

for ri, key in enumerate(exp_order):
    d    = data[key]
    t_vi = d['vicon']['t']
    att  = d['att']

    # trim estimator to visible window
    t_mask = (att['t'] >= t_vi[0] - 0.5) & (att['t'] <= t_vi[-1] + 0.5)

    for ci, (a_key, a_lbl) in enumerate(zip(angle_keys, angle_lbls)):
        ax = axes[ri][ci]

        ax.plot(t_vi, d['vicon'][a_key], 'k-', lw=1.5, label='VICON GT', zorder=5)
        ax.plot(att['t'][t_mask], att[a_key][t_mask],
                color=d['color'], lw=1.0, alpha=0.85,
                label='ESEKF' if d['source'] == 'esekf' else 'CX5 AHRS')

        rmse_val = data[key]['rmse'][a_key]
        src_str  = 'ESEKF' if d['source'] == 'esekf' else 'CX5'
        ax.text(0.02, 0.97, f"{src_str} RMSE: {rmse_val:.3f}°",
                transform=ax.transAxes, fontsize=8, va='top',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

        if ci == 0:
            ax.set_ylabel(f"{d['label']}\n{a_lbl}", fontsize=8)
        else:
            ax.set_ylabel(a_lbl, fontsize=8)
        if ri == 0:
            ax.set_title(a_lbl, fontsize=10, fontweight='bold')
        if ri == len(exp_order) - 1:
            ax.set_xlabel('Time [s]', fontsize=8)
        if ri == 0 and ci == 0:
            ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(t_vi[0], t_vi[-1])

plt.tight_layout()
out1 = os.path.join(OUT, 'fig_compare_attitude.png')
plt.savefig(out1, dpi=150, bbox_inches='tight')
print(f"Saved {out1}")
plt.close()

# ─────────────────────────────────────────────────────────────────────────────
# Figure 2: Yaw drift (relative)
# ─────────────────────────────────────────────────────────────────────────────
fig2, axes2 = plt.subplots(2, 2, figsize=(13, 8))
fig2.suptitle('Yaw Drift vs VICON Ground Truth (relative to t=0)',
              fontsize=12, fontweight='bold')

for i, key in enumerate(exp_order):
    ax  = axes2[i // 2][i % 2]
    d   = data[key]
    t_vi = d['vicon']['t']
    att  = d['att']

    yaw_vi_rel = d['vicon']['yaw'] - d['vicon']['yaw'][0]
    ax.plot(t_vi, yaw_vi_rel, 'k-', lw=1.5, label='VICON GT', zorder=5)

    t_mask = (att['t'] >= t_vi[0] - 0.5) & (att['t'] <= t_vi[-1] + 0.5)
    t_plot = att['t'][t_mask]
    if len(t_plot) > 0:
        yaw_rel = att['yaw'][t_mask] - att['yaw'][t_mask][0]
        ax.plot(t_plot, yaw_rel, color=d['color'], lw=1.0, alpha=0.85,
                label='ESEKF' if d['source'] == 'esekf' else 'CX5 AHRS')

    ax.set_title(d['label'], fontsize=10)
    rmse_val = data[key]['rmse']['yaw']
    ax.text(0.02, 0.97, f"Yaw RMSE: {rmse_val:.3f}°",
            transform=ax.transAxes, fontsize=8, va='top',
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))
    ax.set_xlabel('Time [s]', fontsize=8)
    ax.set_ylabel('Yaw (relative) [°]', fontsize=8)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(t_vi[0], t_vi[-1])

plt.tight_layout()
out2 = os.path.join(OUT, 'fig_compare_yaw.png')
plt.savefig(out2, dpi=150, bbox_inches='tight')
print(f"Saved {out2}")
plt.close()

print("\nDone.")
