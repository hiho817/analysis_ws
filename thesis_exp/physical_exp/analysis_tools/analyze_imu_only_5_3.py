#!/usr/bin/env python3
"""Analyze Section 5.3 IMU-only ablation and update the thesis report."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
from statistics import mean, stdev
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from corgi_msgs.msg import TriggerStamped
from corgi_msgs.msg import ImuStamped


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp")
RESULT = ROOT / "results" / "5.3_flat_experiment"
BAGS = RESULT / "imu_only_bags"
sys.path.insert(0, str(ROOT / "common"))
from corgi_analysis.vicon_loader import load_vicon  # noqa: E402


IMU_SELECTED = {
    "NEW_WALK": [
        "FLAT_Walk_NEW_REAL_3", "FLAT_Walk_NEW_REAL_5",
        "FLAT_Walk_NEW_REAL_6"],
    "NEW_WLW": [
        "FLAT_WLW_NEW_REAL_2", "FLAT_WLW_NEW_REAL_4",
        "FLAT_WLW_NEW_REAL_5"],
}
FLIP = {
    "FLAT_WLW_NEW_REAL_2", "FLAT_WLW_NEW_REAL_4",
    "FLAT_WLW_NEW_REAL_5",
}
INVALID = {}


def rows(db: Path, topic: str):
    connection = sqlite3.connect(db)
    cursor = connection.cursor()
    found = cursor.execute(
        "SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
    data = [] if found is None else cursor.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (found[0],)).fetchall()
    connection.close()
    return data


def trigger_on_time(db: Path, fallback_end: float | None = None):
    parsed = []
    for storage, data in rows(db, "/trigger"):
        msg = deserialize_message(data, TriggerStamped)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        parsed.append((storage, stamp, bool(msg.enable)))
    for _, stamp, enabled in parsed:
        if enabled and stamp > 0:
            return stamp
    if parsed and fallback_end is not None:
        # REAL_1 contains only OFF. Infer ON from VICON's measured trigger span.
        return parsed[-1][1] - fallback_end
    return None


def trigger_pair_times(db: Path, pair_index: int):
    pairs, pending = [], None
    for _, data in rows(db, "/trigger"):
        msg = deserialize_message(data, TriggerStamped)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.enable:
            pending = stamp
        elif pending is not None:
            pairs.append((pending, stamp))
            pending = None
    if pair_index >= len(pairs):
        raise RuntimeError(f"trigger pair {pair_index} unavailable in {db}")
    return pairs[pair_index]


def trigger_off_time(db: Path):
    """Return the unique/last trigger-OFF header timestamp."""
    off = []
    for _, data in rows(db, "/trigger"):
        msg = deserialize_message(data, TriggerStamped)
        if not msg.enable:
            off.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    return off[-1] if off else None


def load_odom(db: Path, topic: str, t0: float):
    out = {key: [] for key in (
        "t", "px", "py", "pz", "vx", "vy", "vz",
        "roll", "pitch", "yaw")}
    for _, data in rows(db, topic):
        msg = deserialize_message(data, Odometry)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
        values = (stamp - t0, p.x, p.y, p.z, v.x, v.y, v.z, *rpy)
        for key, value in zip(out, values):
            out[key].append(value)
    return {key: np.asarray(value, dtype=float) for key, value in out.items()}


def integrate_real2_raw_imu(db: Path, off_time: float, vicon_duration: float):
    """Integrate REAL_2 raw IMU using a post-experiment static calibration."""
    samples = []
    for _, data in rows(db, "/imu_raw"):
        msg = deserialize_message(data, ImuStamped)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        samples.append((
            stamp,
            [msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z],
            [msg.angular_velocity.x, msg.angular_velocity.y,
             msg.angular_velocity.z],
        ))
    stamp = np.asarray([s[0] for s in samples])
    accel = np.asarray([s[1] for s in samples], dtype=float)
    gyro = np.asarray([s[2] for s in samples], dtype=float)

    # Find the quietest 200-sample window after trigger OFF.  This calibrates
    # fixed biases without feeding any external position or velocity update.
    candidates = []
    n = 200
    for end in range(n, len(stamp), 20):
        if stamp[end - 1] <= off_time:
            continue
        duration = stamp[end - 1] - stamp[end - n]
        if not 0.15 <= duration <= 0.30:
            continue
        wm, am = gyro[end - n:end], accel[end - n:end]
        mean_norm = np.linalg.norm(wm.mean(axis=0))
        gyro_rms = np.sqrt(np.mean(np.sum((wm - wm.mean(axis=0)) ** 2, axis=1)))
        accel_rms = np.sqrt(np.mean(np.sum((am - am.mean(axis=0)) ** 2, axis=1)))
        score = mean_norm + 3.0 * gyro_rms + 0.2 * accel_rms
        candidates.append((score, end, mean_norm, gyro_rms, accel_rms))
    if not candidates:
        raise RuntimeError("REAL_2 has no usable post-OFF static IMU window")
    score, end, mean_norm, gyro_rms, accel_rms = min(candidates)
    a_mean = accel[end - n:end].mean(axis=0)
    w_mean = gyro[end - n:end].mean(axis=0)

    g_world = np.array([0.0, 0.0, -9.81])
    g_body = -a_mean / np.linalg.norm(a_mean)
    # Rotation.align_vectors returns body-to-world R with R*g_body=-Z.
    q = Rotation.align_vectors(
        np.array([[0.0, 0.0, -1.0]]), g_body.reshape(1, 3))[0]
    ba = a_mean + q.as_matrix().T @ g_world
    bw = w_mean

    aligned_t = stamp - off_time + vicon_duration
    valid = (aligned_t >= 0.0) & (aligned_t <= vicon_duration)
    indices = np.where(valid)[0]
    out = {key: [] for key in (
        "t", "px", "py", "pz", "vx", "vy", "vz",
        "roll", "pitch", "yaw")}
    p = np.zeros(3)
    v = np.zeros(3)  # no VICON/leg velocity injected at partial-data start
    previous = None
    for index in indices:
        if previous is not None:
            dt = stamp[index] - stamp[previous]
            if not 0.0005 <= dt <= 0.004:
                previous = index
                continue
            a_m = 0.5 * (accel[previous] + accel[index])
            w_m = 0.5 * (gyro[previous] + gyro[index])
            rotation = q.as_matrix()
            delta = Rotation.from_rotvec((w_m - bw) * dt)
            p = p + rotation @ v * dt
            v = delta.as_matrix().T @ v + ((a_m - ba) + rotation.T @ g_world) * dt
            q = q * delta
        rpy = q.as_euler("xyz")
        values = (aligned_t[index], *p, *v, *rpy)
        for key, value in zip(out, values):
            out[key].append(value)
        previous = index
    calibration = {
        "method": "quietest 200-sample post-trigger-OFF window",
        "window_start_aligned_s": float(aligned_t[end - n]),
        "window_end_aligned_s": float(aligned_t[end - 1]),
        "gyro_mean_norm_rad_s": float(mean_norm),
        "gyro_rms_rad_s": float(gyro_rms),
        "accel_rms_m_s2": float(accel_rms),
        "ba_m_s2": ba.tolist(),
        "bw_rad_s": bw.tolist(),
    }
    return ({key: np.asarray(value, dtype=float) for key, value in out.items()},
            calibration)


def rms(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values * values))) if len(values) else float("nan")


def interp_columns(t_ref, matrix, t):
    valid = np.isfinite(matrix).all(axis=1)
    return np.column_stack([
        interp1d(t_ref[valid], matrix[valid, axis], bounds_error=False,
                 fill_value=np.nan)(t) for axis in range(matrix.shape[1])])


def source_bag(exp_id: str):
    if exp_id == "FLAT_Walk_NEW_REAL_3":
        return Path("/home/hiho817/analysis_ws/experiments/20260528/bags/odom_fusion20260528_151411/odom_fusion20260528_151411_0.db3")
    candidates = list((ROOT / "experiments" / exp_id / "bags").glob("*/*.db3"))
    candidates = [p for p in candidates if "imu_only" not in p.parent.name]
    return max(candidates, key=lambda p: p.stat().st_size)


def analyze(exp_id: str):
    gait = "WLW" if "_WLW_" in exp_id else "WALK"
    exp = ROOT / "experiments" / exp_id
    csv = next((exp / "vicon").glob("*.csv"))
    baseline_json = json.loads((exp / "results" / exp_id / "metrics.json").read_text())
    vi = load_vicon(str(csv), contact_threshold_m=0.020 if gait == "WLW" else 0.015,
                    ground_markers=["ground1", "ground2", "ground3", "ground4"])
    baseline_db = source_bag(exp_id)
    if baseline_json is None:
        baseline = None
    else:
        # Walk REAL_3 has an aborted first trigger pair.  Its baseline and
        # IMU-only streams must both use the second effective pair.
        if exp_id == "FLAT_Walk_NEW_REAL_3":
            base_t0, _ = trigger_pair_times(baseline_db, 1)
        else:
            base_t0 = trigger_on_time(baseline_db, vi.t_trigger_end)
        baseline = load_odom(baseline_db, "/ekf", base_t0)

    item = {
        "exp_id": exp_id, "group": f"NEW_{gait}",
        "baseline": None if baseline_json is None else {
            "position_rmse_3d_m": baseline_json["position"]["RMSE_3D_cm"] / 100.0,
            "velocity_rmse_3d": baseline_json["velocity"]["RMSE_3D"],
            "attitude": baseline_json.get("attitude")},
        "valid_imu_only": exp_id not in INVALID,
        "invalid_reason": INVALID.get(exp_id),
    }
    if exp_id in INVALID:
        return item, vi, baseline, None

    if exp_id == "FLAT_Walk_NEW_REAL_3":
        imu_db = next((BAGS / exp_id).glob("*.db3"))
        on_time, off_time = trigger_pair_times(imu_db, 1)
        imu = load_odom(imu_db, "/imu_only/ekf", on_time)
        item["time_alignment"] = {
            "method": "second trigger ON/OFF pair",
            "trigger_pair_index": 1,
            "trigger_on_s": float(on_time),
            "trigger_off_s": float(off_time),
            "data_start_s": float(imu["t"][imu["t"] >= 0][0]),
            "data_end_s": float(min(vi.t_trigger_end, imu["t"][-1])),
        }
    else:
        imu_db = next((BAGS / exp_id).glob("*.db3"))
        imu_t0 = trigger_on_time(imu_db)
        imu = load_odom(imu_db, "/imu_only/ekf", imu_t0)
    if exp_id in FLIP:
        for key in ("px", "py", "vx", "vy", "roll", "pitch"):
            imu[key] *= -1
        if baseline is not None:
            for key in ("px", "py", "vx", "vy", "roll", "pitch"):
                baseline[key] *= -1

    t_end = min(float(vi.t_trigger_end), float(imu["t"][-1]))
    mask = (imu["t"] >= 0) & (imu["t"] <= t_end)
    t = imu["t"][mask]
    pos = np.column_stack([imu[key][mask] for key in ("px", "py", "pz")])
    vel = np.column_stack([imu[key][mask] for key in ("vx", "vy", "vz")])
    rpy = np.column_stack([imu[key][mask] for key in ("roll", "pitch", "yaw")])
    pos_ref = interp_columns(vi.t_traj, vi.pos_m, t)
    vel_ref = interp_columns(vi.t_traj, vi.v_body, t)
    rpy_ref = interp_columns(vi.t_traj, vi.rpy, t)

    # Align estimator and VICON origins at the first common valid sample.
    first = np.where(np.isfinite(pos_ref).all(axis=1))[0][0]
    offset = pos[first] - pos_ref[first]
    pos_error = pos - pos_ref - offset
    pvalid = np.isfinite(pos_error).all(axis=1)
    if baseline_json is None:
        velocity_start, velocity_end = t_end * 0.35, t_end * 0.75
    else:
        velocity_start = baseline_json["velocity"]["window_start"]
        velocity_end = baseline_json["velocity"]["window_end"]
    vmask = ((t >= velocity_start)
             & (t <= velocity_end)
             & np.isfinite(vel_ref).all(axis=1))
    vel_error = vel - vel_ref
    angle_error = np.arctan2(np.sin(rpy - rpy_ref), np.cos(rpy - rpy_ref))
    avalid = np.isfinite(angle_error).all(axis=1)

    final = np.where(pvalid)[0][-1]
    horizontal = np.linalg.norm(pos_error[:, :2], axis=1)
    checkpoints = {}
    for second in (5.0, 10.0, 15.0):
        if t[0] <= second <= t[-1]:
            index = int(np.argmin(np.abs(t - second)))
            checkpoints[f"{int(second)}s_horizontal_drift_m"] = (
                float(horizontal[index]) if pvalid[index] else None)
        else:
            checkpoints[f"{int(second)}s_horizontal_drift_m"] = None
    item["imu_only"] = {
        "analysis_end_s": float(t[final]),
        "position_rmse_x_m": rms(pos_error[pvalid, 0]),
        "position_rmse_y_m": rms(pos_error[pvalid, 1]),
        "position_rmse_z_m": rms(pos_error[pvalid, 2]),
        "position_rmse_3d_m": rms(np.linalg.norm(pos_error[pvalid], axis=1)),
        "velocity_rmse_vx": rms(vel_error[vmask, 0]),
        "velocity_rmse_vy": rms(vel_error[vmask, 1]),
        "velocity_rmse_vz": rms(vel_error[vmask, 2]),
        "velocity_rmse_3d": rms(np.linalg.norm(vel_error[vmask], axis=1)),
        "attitude_rmse_roll_deg": np.degrees(rms(angle_error[avalid, 0])),
        "attitude_rmse_pitch_deg": np.degrees(rms(angle_error[avalid, 1])),
        "attitude_rmse_yaw_deg": np.degrees(rms(angle_error[avalid, 2])),
        "final_horizontal_drift_m": float(horizontal[final]),
        **checkpoints,
    }
    imu["plot_t"] = t
    imu["plot_pos"] = pos - offset
    return item, vi, baseline, imu


def summarize(records, gait):
    valid = [r for r in records if r["group"] == f"NEW_{gait}" and r["valid_imu_only"]]
    fields = (
        "position_rmse_x_m", "position_rmse_y_m", "position_rmse_z_m",
        "position_rmse_3d_m",
        "velocity_rmse_vx", "velocity_rmse_vy", "velocity_rmse_vz",
        "velocity_rmse_3d",
        "attitude_rmse_roll_deg", "attitude_rmse_pitch_deg",
        "attitude_rmse_yaw_deg", "final_horizontal_drift_m")
    result = {"n": len(valid), "experiment_ids": [r["exp_id"] for r in valid]}
    for field in fields:
        values = [r["imu_only"][field] for r in valid]
        result[field] = {
            "values": values, "mean": mean(values),
            "sample_std": stdev(values) if len(values) > 1 else 0.0}
    return result


def fmt(stat, digits=2):
    return f"{stat['mean']:.{digits}f} ± {stat['sample_std']:.{digits}f}"


def make_xy_plot(plot_data):
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for ax, (item, vi, baseline, imu) in zip(axes.flat, plot_data):
        end = vi.t_trigger_end
        vm = (vi.t_traj >= 0) & (vi.t_traj <= end) & np.isfinite(vi.pos_m).all(axis=1)
        ax.plot(vi.pos_m[vm, 0], vi.pos_m[vm, 1], "k", lw=2, label="VICON")
        if baseline is not None:
            bm = ((baseline["t"] >= 0) & (baseline["t"] <= end)
                  & np.isfinite(baseline["px"]) & np.isfinite(baseline["py"]))
            if bm.any():
                bx, by = baseline["px"][bm], baseline["py"][bm]
                ax.plot(bx - bx[0], by - by[0], color="#0072B2", label="NEW")
        if imu is not None:
            ax.plot(imu["plot_pos"][:, 0], imu["plot_pos"][:, 1],
                    color="#D55E00", label="IMU-only")
        else:
            ax.text(0.5, 0.08, "IMU-only unavailable", transform=ax.transAxes,
                    ha="center", color="#D55E00")
        ax.set_title(item["exp_id"].replace("FLAT_", ""), fontsize=10)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.25)
    # Use a valid trial for the legend because REAL_1 has no IMU-only line.
    axes.flat[1].legend(fontsize=8)
    fig.savefig(RESULT / "fig_imu_only_xy.png", dpi=180)
    plt.close(fig)


def make_summary_plot(records):
    valid = [r for r in records if r["valid_imu_only"]]
    labels = [r["exp_id"].replace("FLAT_", "").replace("_REAL_", "_") for r in valid]
    x = np.arange(len(valid))
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), constrained_layout=True)
    axes[0].bar(x - .18, [np.nan if r["baseline"] is None else r["baseline"]["position_rmse_3d_m"] for r in valid],
                .36, label="NEW")
    axes[0].bar(x + .18, [r["imu_only"]["position_rmse_3d_m"] for r in valid],
                .36, label="IMU-only")
    axes[0].set_ylabel("Position RMSE 3D [m]")
    axes[1].bar(x - .18, [np.nan if r["baseline"] is None else r["baseline"]["velocity_rmse_3d"] for r in valid], .36)
    axes[1].bar(x + .18, [r["imu_only"]["velocity_rmse_3d"] for r in valid], .36)
    axes[1].set_ylabel("Velocity RMSE 3D [m/s]")
    for ax in axes:
        ax.set_xticks(x, labels, rotation=25, ha="right", fontsize=8)
        ax.set_yscale("log")
        ax.grid(axis="y", alpha=.25)
    axes[0].legend()
    fig.savefig(RESULT / "fig_imu_only_rmse.png", dpi=180)
    plt.close(fig)


def update_report(records, stats):
    report_path = RESULT / "5.3_平地實驗.md"
    report = report_path.read_text(encoding="utf-8")
    report = report.replace(
        "本節主比較重新彙整既有 `metrics.json` 中的狀態估測結果，另以 ROS 2 bag replay 執行 NEW 的純 IMU 積分消融；不重新分析觸地狀態。",
        "本節比較 NEW（ES-EKF）、OLD（Legacy）與純 IMU 積分三種方法；不重新分析觸地狀態。",
    )
    if "| 純 IMU Walk |" not in report:
        report = report.replace(
            "| OLD WLW | FLAT_WLW_OLD_REAL_1, FLAT_WLW_OLD_REAL_2, FLAT_WLW_OLD_REAL_3 | FLAT_WLW_OLD_REAL_4, FLAT_WLW_OLD_REAL_5 |",
            "| OLD WLW | FLAT_WLW_OLD_REAL_1, FLAT_WLW_OLD_REAL_2, FLAT_WLW_OLD_REAL_3 | FLAT_WLW_OLD_REAL_4, FLAT_WLW_OLD_REAL_5 |\n"
            "| 純 IMU Walk | FLAT_Walk_NEW_REAL_3, FLAT_Walk_NEW_REAL_5, FLAT_Walk_NEW_REAL_6 | FLAT_Walk_NEW_REAL_1、FLAT_Walk_NEW_REAL_2（保持無值） |\n"
            "| 純 IMU WLW | FLAT_WLW_NEW_REAL_2, FLAT_WLW_NEW_REAL_4, FLAT_WLW_NEW_REAL_5 | — |",
        )
    # Keep the existing 5.3.1–5.3.3 baseline material, but rebuild everything
    # after the attitude introduction so reruns never restore an ablation section.
    if "## 5.3.4 純 IMU 積分消融實驗" in report:
        report = report.split("## 5.3.4 純 IMU 積分消融實驗")[0].rstrip()
    elif "## 5.3.4 小結" in report:
        report = report.split("## 5.3.4 小結")[0].rstrip()

    # Add pure-IMU rows directly to the main 5.3.2 comparison table.
    for gait, label in (("WALK", "Walk"), ("WLW", "WLW")):
        s = stats[gait]
        row = (
            f"| {label} | 純 IMU 積分 | {s['n']} | "
            f"{fmt(s['position_rmse_x_m'], 3)} / {fmt(s['position_rmse_y_m'], 3)} / "
            f"{fmt(s['position_rmse_z_m'], 3)} | {fmt(s['position_rmse_3d_m'], 3)} | "
            f"{fmt(s['velocity_rmse_vx'], 3)} / {fmt(s['velocity_rmse_vy'], 3)} / "
            f"{fmt(s['velocity_rmse_vz'], 3)} | {fmt(s['velocity_rmse_3d'], 3)} |"
        )
        anchor = (f"| {label} | OLD（Legacy）")
        lines_now = report.splitlines()
        insert_at = next(i for i, line in enumerate(lines_now) if line.startswith(anchor)) + 1
        if row not in lines_now:
            lines_now.insert(insert_at, row)
        report = "\n".join(lines_now)

    imu_lines = [
        "", "### 純 IMU 積分資料與個別結果", "",
        "純 IMU 積分為本節第三種直接比較方法。其 propagation 使用動態時間步長與梯形平均，但不使用腿部速度更新、ZUPT、GMO/contact、`/fusion/bv`、LiDAR 或 VICON 狀態校正。Walk 使用 REAL_3、REAL_5、REAL_6；WLW 使用 REAL_2、REAL_4、REAL_5。",
        "",
        "`FLAT_Walk_NEW_REAL_1` 與 `FLAT_Walk_NEW_REAL_2` 保持無純 IMU值。Walk REAL_3 使用第二組 trigger ON/OFF；第一組僅 2.343 s，視為中止段，不納入分析。",
        "",
        "| Trial | 方法 | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] | 最終水平漂移 [m] |",
        "|---|---|---:|---:|---:|",
        "| FLAT_Walk_NEW_REAL_1 | 純 IMU 積分 | — | — | — |",
        "| FLAT_Walk_NEW_REAL_2 | 純 IMU 積分 | — | — | — |",
    ]
    for r in records:
        if r["valid_imu_only"]:
            im = r["imu_only"]
            imu_lines.append(
                f"| {r['exp_id']} | 純 IMU 積分 | {im['position_rmse_3d_m']:.3f} | "
                f"{im['velocity_rmse_3d']:.3f} | {im['final_horizontal_drift_m']:.2f} |")

    # Insert the pure-IMU details immediately before 5.3.3.
    before_attitude, attitude = report.split("## 5.3.3 姿態估測結果", 1)
    if "### 純 IMU 積分資料與個別結果" in before_attitude:
        before_attitude = before_attitude.split("### 純 IMU 積分資料與個別結果")[0].rstrip()
    report = before_attitude.rstrip() + "\n" + "\n".join(imu_lines) + "\n\n## 5.3.3 姿態估測結果" + attitude

    # Pure IMU is also a direct attitude comparator; OLD remains unavailable.
    attitude_rows = []
    for gait, label in (("WALK", "Walk"), ("WLW", "WLW")):
        s = stats[gait]
        attitude_rows.append(
            f"| {label} | 純 IMU 積分 | {s['n']} | "
            f"{fmt(s['attitude_rmse_roll_deg'])} | {fmt(s['attitude_rmse_pitch_deg'])} | "
            f"{fmt(s['attitude_rmse_yaw_deg'])} |")
    if attitude_rows[0] not in report:
        report = report.replace(
            "| Walk | OLD（Legacy） | 3 | — | — | — |",
            "| Walk | OLD（Legacy） | 3 | — | — | — |\n" + attitude_rows[0])
    if attitude_rows[1] not in report:
        report = report.replace(
            "| WLW | OLD（Legacy） | 3 | — | — | — |",
            "| WLW | OLD（Legacy） | 3 | — | — | — |\n" + attitude_rows[1])
    report = report.replace(
        "姿態 RMSE 僅分析 NEW（ES-EKF），因 OLD（Legacy）未估測姿態，故無 Roll、Pitch 與 Yaw 可供比較。",
        "姿態 RMSE 比較 NEW（ES-EKF）與純 IMU 積分；OLD（Legacy）未估測姿態，故無 Roll、Pitch 與 Yaw 可供比較。",
    )
    report += (
        "\n\n## 5.3.4 小結\n\n"
        "平地實驗直接比較 NEW（ES-EKF）、OLD（Legacy）與純 IMU 積分。NEW 在 Walk 與 WLW 的位置與速度 RMSE 均低於 OLD；純 IMU 積分因缺少速度與位置觀測約束，誤差再明顯增加。純 IMU Walk 使用 REAL_3、5、6，WLW 使用 REAL_2、4、5；Walk REAL_1、REAL_2 保持無值。\n"
    )
    report_path.write_text(report.strip() + "\n", encoding="utf-8")


def main():
    records = [{
        "exp_id": "FLAT_Walk_NEW_REAL_1", "group": "NEW_WALK",
        "baseline": None, "valid_imu_only": False,
        "invalid_reason": "依使用者指定保持無純 IMU值。",
    }, {
        "exp_id": "FLAT_Walk_NEW_REAL_2", "group": "NEW_WALK",
        "baseline": None, "valid_imu_only": False,
        "invalid_reason": "IMU 數值不合理，依使用者指定保持無值。",
    }]
    plot_data = []
    for group in ("NEW_WALK", "NEW_WLW"):
        for exp_id in IMU_SELECTED[group]:
            item, vi, baseline, imu = analyze(exp_id)
            records.append(item)
            plot_data.append((item, vi, baseline, imu))
            print(exp_id, "valid" if item["valid_imu_only"] else "invalid",
                  item.get("imu_only", {}).get("position_rmse_3d_m"))
    stats = {gait: summarize(records, gait) for gait in ("WALK", "WLW")}
    payload = {
        "method": "prediction-only IMU integration; no leg update, ZUPT, GMO, or fusion feedback",
        "selected_imu_only_experiments": IMU_SELECTED,
        "no_value_experiments": ["FLAT_Walk_NEW_REAL_1", "FLAT_Walk_NEW_REAL_2"],
        "records": records,
        "group_statistics": stats,
    }
    (RESULT / "imu_only_metrics.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    # Figures are intentionally not generated or embedded until their desired
    # presentation is specified.
    update_report(records, stats)
    print("Wrote", RESULT / "imu_only_metrics.json")
    print("Updated", RESULT / "5.3_平地實驗.md")


if __name__ == "__main__":
    main()
