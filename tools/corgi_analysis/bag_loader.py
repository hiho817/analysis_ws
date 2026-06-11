"""
bag_loader.py — load ROS2 SQLite bag files for CORGI experiments.

Functions
---------
load_inner_ekf_bag(bag_db, rate=2.0)
    Inner EKF bag: /ekf, /ekf/ba, /ekf/bw, /trigger, /gmo/contact_state

load_fusion_bag(bag_db)
    Outer-fusion bag: /ekf, /ekf/ba, /ekf/bw, /trigger,
                      /gmo/contact_state, /odom_mapping, /fusion/bv

Time convention
---------------
t = 0 is the trigger message **header stamp** (real experiment time).
For topics with no header (ba, bw, gmo) the storage timestamp is used
and scaled by the replay `rate` to recover header-equivalent time.

Requires: rclpy (source ROS2 workspace before calling)
"""

import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from corgi_msgs.msg import TriggerStamped, GMOContactStateStamped


# ──────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ──────────────────────────────────────────────────────────────────────────────

def _open_bag(bag_db: str):
    conn = sqlite3.connect(bag_db)
    cur  = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    return conn, cur, tmap


def _fetch(cur, tmap, topic):
    tid = tmap.get(topic)
    if tid is None:
        return []
    cur.execute(
        f"SELECT timestamp, data FROM messages WHERE topic_id={tid} ORDER BY timestamp"
    )
    return cur.fetchall()


def _parse_ekf(rows, t_ros_trigger):
    """Parse /ekf (nav_msgs/Odometry) rows → dict of numpy arrays."""
    ekf = {k: [] for k in [
        't', 'px', 'py', 'pz', 'vx', 'vy', 'vz',
        'qw', 'qx', 'qy', 'qz',
        'cov_px', 'cov_py', 'cov_pz',
        'cov_vx', 'cov_vy', 'cov_vz',
    ]}
    for _, data in rows:
        msg = deserialize_message(data, Odometry)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t_ros_trigger
        ekf['t'].append(t)
        p = msg.pose.pose.position
        ekf['px'].append(p.x); ekf['py'].append(p.y); ekf['pz'].append(p.z)
        v = msg.twist.twist.linear
        ekf['vx'].append(v.x); ekf['vy'].append(v.y); ekf['vz'].append(v.z)
        q = msg.pose.pose.orientation
        ekf['qw'].append(q.w); ekf['qx'].append(q.x)
        ekf['qy'].append(q.y); ekf['qz'].append(q.z)
        cp = msg.pose.covariance
        ekf['cov_px'].append(cp[0]); ekf['cov_py'].append(cp[7]); ekf['cov_pz'].append(cp[14])
        cv = msg.twist.covariance
        ekf['cov_vx'].append(cv[0]); ekf['cov_vy'].append(cv[7]); ekf['cov_vz'].append(cv[14])
    for k in ekf:
        ekf[k] = np.array(ekf[k])
    return ekf


def _parse_vector3_bias(rows, trg_ts0, rate):
    """Parse Vector3 bias topic rows → dict with storage-based time axis."""
    d = {'t': [], 'x': [], 'y': [], 'z': []}
    for ts, data in rows:
        msg = deserialize_message(data, Vector3)
        d['t'].append((ts - trg_ts0) / 1e9 * rate)
        d['x'].append(msg.x); d['y'].append(msg.y); d['z'].append(msg.z)
    for k in d:
        d[k] = np.array(d[k])
    return d


def _parse_gmo(rows, trg_ts0, rate):
    """Parse /gmo/contact_state rows → dict with storage-based time axis."""
    gmo = {'t': [], 'LF': [], 'RF': [], 'RH': [], 'LH': []}
    seen = set()
    for ts, data in rows:
        if ts in seen:
            continue
        seen.add(ts)
        msg = deserialize_message(data, GMOContactStateStamped)
        gmo['t'].append((ts - trg_ts0) / 1e9 * rate)
        gmo['LF'].append(msg.module_a.contact)
        gmo['RF'].append(msg.module_b.contact)
        gmo['RH'].append(msg.module_c.contact)
        gmo['LH'].append(msg.module_d.contact)
    for k in gmo:
        gmo[k] = np.array(gmo[k])
    return gmo


# ──────────────────────────────────────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────────────────────────────────────

def load_inner_ekf_bag(bag_db: str, rate: float = 2.0) -> dict:
    """
    Load inner-EKF bag topics.

    Parameters
    ----------
    bag_db : str
        Path to the .db3 SQLite bag file.
    rate : float
        ros2 bag play --rate value used during recording.
        Used to convert storage timestamps of headerless topics to real time.

    Returns
    -------
    dict with keys:
        ekf            : dict(t, px, py, pz, vx, vy, vz, qw, qx, qy, qz,
                              cov_px, cov_py, cov_pz, cov_vx, cov_vy, cov_vz)
        ba             : dict(t, x, y, z)  — accelerometer bias
        bw             : dict(t, x, y, z)  — gyroscope bias
        gmo            : dict(t, LF, RF, RH, LH) — contact state
        t_ros_trigger  : float  — trigger header stamp (seconds, absolute)
    """
    conn, cur, tmap = _open_bag(bag_db)
    rows_trg = _fetch(cur, tmap, '/trigger')
    rows_ekf = _fetch(cur, tmap, '/ekf')
    rows_ba  = _fetch(cur, tmap, '/ekf/ba')
    rows_bw  = _fetch(cur, tmap, '/ekf/bw')
    rows_gmo = _fetch(cur, tmap, '/gmo/contact_state')
    conn.close()

    trg_msg = deserialize_message(rows_trg[0][1], TriggerStamped)
    t_ros_trigger = trg_msg.header.stamp.sec + trg_msg.header.stamp.nanosec * 1e-9
    trg_ts0 = rows_trg[0][0]
    # Trigger OFF: second message (enable=False) if present
    t_ros_trigger_off = None
    if len(rows_trg) >= 2:
        trg_off_msg = deserialize_message(rows_trg[1][1], TriggerStamped)
        if not trg_off_msg.enable:
            t_ros_trigger_off = (trg_off_msg.header.stamp.sec
                                 + trg_off_msg.header.stamp.nanosec * 1e-9)

    ekf = _parse_ekf(rows_ekf, t_ros_trigger)
    ba  = _parse_vector3_bias(rows_ba,  trg_ts0, rate)
    bw  = _parse_vector3_bias(rows_bw,  trg_ts0, rate)
    gmo = _parse_gmo(rows_gmo, trg_ts0, rate)

    t_bag_trigger_end = (
        t_ros_trigger_off - t_ros_trigger if t_ros_trigger_off is not None else None
    )
    print(f'[bag_loader] EKF: {len(ekf["t"])} msgs, '
          f't=[{ekf["t"][0]:.2f}, {ekf["t"][-1]:.2f}] s')
    print(f'[bag_loader] t_ros_trigger = {t_ros_trigger:.3f} s, '
          f't_trigger_end = {t_bag_trigger_end}')

    return {
        'ekf': ekf, 'ba': ba, 'bw': bw, 'gmo': gmo,
        't_ros_trigger': t_ros_trigger,
        't_ros_trigger_off': t_ros_trigger_off,
        't_trigger_end': t_bag_trigger_end,
    }


def load_fusion_bag(bag_db: str, rate: float = 1.0, trigger_pair: int = 0) -> dict:
    """
    Load outer-fusion bag topics (inner EKF + odom_mapping + fusion/bv).

    Parameters
    ----------
    trigger_pair : int
        0-based index of the trigger ON/OFF pair to use.  Default 0 (first pair).
        Use trigger_pair=1 when the first trigger window was aborted and the
        experiment was restarted within the same bag.

    Returns
    -------
    Same as load_inner_ekf_bag plus:
        odom   : dict(t, px, py, pz, vx, vy, vz, qw, qx, qy, qz) — /odom_mapping
        fv     : dict(t, x, y, z) — /fusion/bv body velocity
    """
    from geometry_msgs.msg import Vector3Stamped  # only needed for fusion

    conn, cur, tmap = _open_bag(bag_db)
    rows_trg   = _fetch(cur, tmap, '/trigger')
    rows_ekf   = _fetch(cur, tmap, '/ekf')
    rows_ba    = _fetch(cur, tmap, '/ekf/ba')
    rows_bw    = _fetch(cur, tmap, '/ekf/bw')
    rows_gmo   = _fetch(cur, tmap, '/gmo/contact_state')
    rows_odom  = _fetch(cur, tmap, '/odom_mapping')
    rows_fv    = _fetch(cur, tmap, '/fusion/bv')
    rows_lidar = _fetch(cur, tmap, '/lidar_odom')
    conn.close()

    if not rows_trg:
        raise RuntimeError("No /trigger messages found in bag")

    # ── Parse all trigger ON/OFF pairs ────────────────────────────────────────
    _pairs = []   # list of (storage_ts_on, ros_t_on, storage_ts_off, ros_t_off)
    _pending = None
    for ts, data in rows_trg:
        msg = deserialize_message(data, TriggerStamped)
        ros_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.enable:
            _pending = (ts, ros_t)
        elif _pending is not None:
            _pairs.append((*_pending, ts, ros_t))
            _pending = None
    if _pending is not None:
        # Bag ended before trigger-OFF
        _pairs.append((*_pending, None, None))
    if not _pairs:
        # Bag only has trigger-OFF (legacy recording mode)
        ts0, data0 = rows_trg[0]
        msg0 = deserialize_message(data0, TriggerStamped)
        ros_t0 = msg0.header.stamp.sec + msg0.header.stamp.nanosec * 1e-9
        _pairs.append((ts0, ros_t0, None, None))

    if trigger_pair >= len(_pairs):
        raise ValueError(f'trigger_pair={trigger_pair} requested but only '
                         f'{len(_pairs)} pair(s) found in {bag_db}')
    trg_ts0, t_ros_trigger, trg_ts_off, t_ros_trigger_off = _pairs[trigger_pair]
    print(f'[bag_loader] Using trigger pair {trigger_pair}: '
          f'ON={t_ros_trigger:.3f}s, OFF={t_ros_trigger_off}')

    ekf  = _parse_ekf(rows_ekf,  t_ros_trigger)
    ba   = _parse_vector3_bias(rows_ba,  trg_ts0, rate)
    bw   = _parse_vector3_bias(rows_bw,  trg_ts0, rate)
    gmo  = _parse_gmo(rows_gmo, trg_ts0, rate)
    odom = _parse_ekf(rows_odom, t_ros_trigger)  # same Odometry type
    lidar = _parse_ekf(rows_lidar, t_ros_trigger)  # /lidar_odom in camera_init frame

    fv = {'t': [], 'x': [], 'y': [], 'z': []}
    for ts, data in rows_fv:
        msg = deserialize_message(data, Vector3Stamped)
        fv['t'].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t_ros_trigger)
        fv['x'].append(msg.vector.x)
        fv['y'].append(msg.vector.y)
        fv['z'].append(msg.vector.z)
    for k in fv:
        fv[k] = np.array(fv[k])

    t_bag_trigger_end = (
        t_ros_trigger_off - t_ros_trigger if t_ros_trigger_off is not None else None
    )
    print(f'[bag_loader] EKF: {len(ekf["t"])} msgs, '
          f'odom: {len(odom["t"])} msgs, fv: {len(fv["t"])} msgs')
    print(f'[bag_loader] t_ros_trigger = {t_ros_trigger:.3f} s, '
          f't_trigger_end = {t_bag_trigger_end}')

    return {
        'ekf': ekf, 'ba': ba, 'bw': bw, 'gmo': gmo,
        'odom': odom, 'fv': fv, 'lidar': lidar,
        't_ros_trigger': t_ros_trigger,
        't_ros_trigger_off': t_ros_trigger_off,
        't_trigger_end': t_bag_trigger_end,
    }


def load_legacy_bag(bag_db: str, rate: float = 2.0) -> dict:
    """
    Load corgi_odometry_legacy bag topics.

    /odometry/legacy/position and /odometry/legacy/velocity are plain
    geometry_msgs/Vector3 (no header stamp) — storage timestamps are
    converted to sim-time via (ts - trg_ts0) / 1e9 * rate.

    Returns
    -------
    dict with keys:
        pos  : dict(t, x, y, z)  — integrated position (world frame)
        vel  : dict(t, x, y, z)  — estimated body velocity
        t_ros_trigger  : float
        t_trigger_end  : float or None
    """
    conn, cur, tmap = _open_bag(bag_db)
    rows_trg = _fetch(cur, tmap, '/trigger')
    rows_pos = _fetch(cur, tmap, '/odometry/legacy/position')
    rows_vel = _fetch(cur, tmap, '/odometry/legacy/velocity')
    conn.close()

    if not rows_trg:
        raise RuntimeError("No /trigger messages found in legacy bag")

    trg_msg = deserialize_message(rows_trg[0][1], TriggerStamped)
    t_ros_trigger = trg_msg.header.stamp.sec + trg_msg.header.stamp.nanosec * 1e-9
    trg_ts0 = rows_trg[0][0]

    t_ros_trigger_off = None
    if len(rows_trg) >= 2:
        trg_off_msg = deserialize_message(rows_trg[1][1], TriggerStamped)
        if not trg_off_msg.enable:
            t_ros_trigger_off = (trg_off_msg.header.stamp.sec
                                 + trg_off_msg.header.stamp.nanosec * 1e-9)

    def _parse_vec3(rows):
        d = {'t': [], 'x': [], 'y': [], 'z': []}
        for ts, data in rows:
            msg = deserialize_message(data, Vector3)
            d['t'].append((ts - trg_ts0) / 1e9 * rate)
            d['x'].append(msg.x); d['y'].append(msg.y); d['z'].append(msg.z)
        for k in d:
            d[k] = np.array(d[k])
        return d

    pos = _parse_vec3(rows_pos)
    vel = _parse_vec3(rows_vel)

    t_bag_trigger_end = (
        t_ros_trigger_off - t_ros_trigger if t_ros_trigger_off is not None else None
    )

    print(f'[bag_loader] legacy pos: {len(pos["t"])} msgs, '
          f't=[{pos["t"][0]:.2f}, {pos["t"][-1]:.2f}] s')

    return {
        'pos': pos, 'vel': vel,
        't_ros_trigger': t_ros_trigger,
        't_ros_trigger_off': t_ros_trigger_off,
        't_trigger_end': t_bag_trigger_end,
    }
