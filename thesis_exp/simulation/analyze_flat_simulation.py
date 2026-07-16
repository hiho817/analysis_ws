#!/usr/bin/env python3
"""Summarize WALK/WLW simulation estimators and prediction-only IMU integration."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from corgi_msgs.msg import TriggerStamped


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp")
SIM = ROOT / "simulation"
REPORT_DIR = ROOT / "physical_exp/results/5.3_flat_experiment"
FIG = REPORT_DIR / "figures"
REPORT = REPORT_DIR / "5.3_平地實驗_模擬.md"
SUMMARY_JSON = REPORT_DIR / "5.3_flat_simulation_metrics.json"

CASES = {
    "Walk": {
        "source": SIM / "FLAT_WALK_NEW_SIM/walk_openloop.db3",
        "imu": SIM / "FLAT_WALK_NEW_SIM/results/imu_only_bag/flat_walk_imu_only_0.db3",
        "existing": SIM / "FLAT_WALK_NEW_SIM/results/metrics.json",
        "empty": SIM / "FLAT_WALK_NEW_SIM/FLAT_WALK_NEW_SIM_0.db3",
        "slug": "walk",
    },
    "WLW": {
        "source": SIM / "FLAT_WLW_NEW_SIM/FLAT_WLW_NEW_SIM_0.db3",
        "imu": SIM / "FLAT_WLW_NEW_SIM/results/imu_only_bag/flat_wlw_imu_only_0.db3",
        "existing": SIM / "FLAT_WLW_NEW_SIM/results/metrics.json",
        "slug": "wlw",
    },
}

COLORS = {"gt": "#222222", "imu": "#D55E00", "proposed": "#0072B2"}


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def rows(db: Path, topic: str):
    with sqlite3.connect(db) as con:
        found = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if found is None:
            raise RuntimeError(f"missing {topic} in {db}")
        return con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (found[0],)).fetchall()


def trigger_on(db: Path):
    for _, raw in rows(db, "/trigger"):
        msg = deserialize_message(raw, TriggerStamped)
        if msg.enable:
            return stamp(msg.header.stamp)
    raise RuntimeError(f"no trigger ON in {db}")


def load_vec(db: Path, topic: str, t0: float):
    t, xyz = [], []
    for storage, raw in rows(db, topic):
        msg = deserialize_message(raw, Vector3)
        t.append(storage * 1e-9 - t0)
        xyz.append([msg.x, msg.y, msg.z])
    return {"t": np.asarray(t), "x": np.asarray(xyz)}


def load_odom(db: Path, topic: str, t0: float):
    t, p, v, q = [], [], [], []
    for _, raw in rows(db, topic):
        msg = deserialize_message(raw, Odometry)
        t.append(stamp(msg.header.stamp) - t0)
        pp, vv, qq = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
        p.append([pp.x, pp.y, pp.z])
        v.append([vv.x, vv.y, vv.z])
        q.append([qq.x, qq.y, qq.z, qq.w])
    return {"t": np.asarray(t), "p": np.asarray(p), "v": np.asarray(v), "q": np.asarray(q)}


def load_tf(db: Path, t0: float, ekf):
    """Load simulator GT TF, excluding the estimator's duplicate TF output.

    Both the simulator and corgi_leg_odom publish odom->base_link on /tf.  At
    every /ekf timestamp the bag therefore contains two transforms with the
    same frame names.  The estimator transform is identified by its exact
    timestamp and quaternion match against /ekf; the remaining transform is
    the simulator ground truth.
    """
    candidates = {}
    for _, raw in rows(db, "/tf"):
        msg = deserialize_message(raw, TFMessage)
        for tr in msg.transforms:
            if tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                qq = tr.transform.rotation
                time = stamp(tr.header.stamp) - t0
                candidates.setdefault(round(time, 9), []).append(
                    np.asarray([qq.x, qq.y, qq.z, qq.w]))
    ekf_by_time = {round(time, 9): quat for time, quat in zip(ekf["t"], ekf["q"])}
    t, q = [], []
    for key in sorted(candidates):
        quaternions = candidates[key]
        if len(quaternions) > 1 and key in ekf_by_time:
            estimator = ekf_by_time[key]
            # Quaternion signs q and -q represent the same rotation.
            distance = [min(np.linalg.norm(x - estimator), np.linalg.norm(x + estimator))
                        for x in quaternions]
            keep = int(np.argmax(distance))
        else:
            keep = 0
        t.append(key)
        q.append(quaternions[keep])
    return {"t": np.asarray(t), "q": np.asarray(q)}


def interp(t, x, tq):
    return interp1d(t, x, axis=0, bounds_error=False, fill_value=np.nan)(tq)


def rpy_deg(q):
    return np.degrees(Rotation.from_quat(q).as_euler("xyz"))


def angle_error_deg(est, truth):
    return np.degrees(np.arctan2(
        np.sin(np.radians(est - truth)), np.cos(np.radians(est - truth))))


def rms(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x * x)))


def align_position(est, gt_t, gt_p, n=500):
    truth = interp(gt_t, gt_p, est["t"])
    valid = np.flatnonzero(np.isfinite(truth).all(axis=1))[:n]
    offset = np.mean(est["p"][valid] - truth[valid], axis=0)
    return truth + offset, offset


def crop(data, start, end):
    mask = (data["t"] >= start) & (data["t"] <= end)
    return {key: value[mask] for key, value in data.items()}


def estimator_metrics(est, gt_pos, gt_bv, gt_tf):
    truth_p, offset = align_position(est, gt_pos["t"], gt_pos["x"])
    truth_v = interp(gt_bv["t"], gt_bv["x"], est["t"])
    truth_rpy = rpy_deg(interp(gt_tf["t"], gt_tf["q"], est["t"]))
    est_rpy = rpy_deg(est["q"])
    valid = np.isfinite(truth_rpy).all(axis=1)
    idx = np.flatnonzero(valid)[:500]
    angle_offset = np.mean(angle_error_deg(est_rpy[idx], truth_rpy[idx]), axis=0)
    p_err = est["p"] - truth_p
    v_err = est["v"] - truth_v
    a_err = angle_error_deg(est_rpy - angle_offset, truth_rpy)
    return {
        "position_rmse_xyz_m": [rms(p_err[:, i]) for i in range(3)],
        "position_rmse_3d_m": rms(np.linalg.norm(p_err, axis=1)),
        "position_max_3d_m": float(np.nanmax(np.linalg.norm(p_err, axis=1))),
        "velocity_rmse_xyz_mps": [rms(v_err[:, i]) for i in range(3)],
        "velocity_rmse_3d_mps": rms(np.linalg.norm(v_err, axis=1)),
        "rpy_rmse_deg": [rms(a_err[:, i]) for i in range(3)],
        "position_offset_m": offset.tolist(),
        "attitude_offset_deg": angle_offset.tolist(),
        "final_position_error_xyz_m": p_err[-1].tolist(),
        "final_horizontal_error_m": float(np.linalg.norm(p_err[-1, :2])),
        "final_velocity_error_xyz_mps": v_err[-1].tolist(),
        "series": {
            "truth_p": truth_p, "truth_v": truth_v, "truth_rpy": truth_rpy,
            "est_rpy": est_rpy - angle_offset, "p_err": p_err,
            "v_err": v_err, "a_err": a_err,
        },
    }


def imu_diagnostics(imu, metrics):
    dt = np.diff(imu["t"])
    positive = dt[dt > 0]
    fit = (imu["t"] >= 5.0) & np.isfinite(metrics["series"]["v_err"]).all(axis=1)
    slopes = [float(np.polyfit(imu["t"][fit], metrics["series"]["v_err"][fit, i], 1)[0])
              for i in range(3)]
    horizontal = float(np.linalg.norm(slopes[:2]))
    checkpoints = {}
    pnorm = np.linalg.norm(metrics["series"]["p_err"][:, :2], axis=1)
    for second in (5, 10, 15, 20, 25, 30):
        if imu["t"][0] <= second <= imu["t"][-1]:
            checkpoints[f"horizontal_error_{second}s_m"] = float(
                pnorm[np.argmin(np.abs(imu["t"] - second))])
    return {
        "messages": int(len(imu["t"])),
        "duplicate_timestamps": int(np.count_nonzero(dt == 0)),
        "median_positive_dt_s": float(np.median(positive)),
        "effective_median_rate_hz": float(1.0 / np.median(positive)),
        "dt_outside_1_to_4_ms": int(np.count_nonzero((positive < .001) | (positive > .004))),
        "velocity_error_slope_xyz_mps2": slopes,
        "horizontal_effective_acceleration_mps2": horizontal,
        "equivalent_gravity_tilt_deg": float(np.degrees(np.arcsin(min(horizontal / 9.81, 1.0)))),
        **checkpoints,
    }


def padded_reference_limits(*series, padding=.06):
    """Return finite limits from Ground Truth and Proposed Method only."""
    values = np.concatenate([np.asarray(value).ravel() for value in series])
    values = values[np.isfinite(values)]
    if values.size == 0:
        return -1.0, 1.0
    lower, upper = float(values.min()), float(values.max())
    span = upper - lower
    if span <= np.finfo(float).eps:
        span = max(abs(lower), 1.0) * .1
    margin = span * padding
    return lower - margin, upper + margin


def nice_ceiling(value):
    """Round a positive limit upward to 1, 2, 5, or 10 times a power of ten."""
    if not np.isfinite(value) or value <= 0:
        return 1.0
    exponent = np.floor(np.log10(value))
    scaled = value / 10.0 ** exponent
    for step in (1.0, 2.0, 5.0, 10.0):
        if scaled <= step:
            return step * 10.0 ** exponent
    raise AssertionError("unreachable")


def position_reference_limits(truth, proposed):
    """Use common zero-centred Y/Z limits and an integer-multiple X span."""
    yz_values = np.concatenate((truth[:, 1:3].ravel(), proposed[:, 1:3].ravel()))
    yz_values = yz_values[np.isfinite(yz_values)]
    yz_half_span = nice_ceiling(np.max(np.abs(yz_values)) * 1.06)
    yz_span = 2.0 * yz_half_span
    x_lower, x_upper = padded_reference_limits(truth[:, 0], proposed[:, 0])
    ratio = max(1, int(np.ceil((x_upper - x_lower) / yz_span - 1e-12)))
    x_span = ratio * yz_span
    x_midpoint = .5 * (x_lower + x_upper)
    return ((x_midpoint - .5 * x_span, x_midpoint + .5 * x_span),
            (-yz_half_span, yz_half_span), ratio)


def plot_case(label, slug, start, end, gt_pos, gt_bv, gt_tf, proposed, imu, pm, im):
    gt_t = np.linspace(start, end, int((end - start) * 200) + 1)
    gt_p = interp(gt_pos["t"], gt_pos["x"], gt_t) + np.asarray(pm["position_offset_m"])
    gt_v = interp(gt_bv["t"], gt_bv["x"], gt_t)
    gt_rpy = rpy_deg(interp(gt_tf["t"], gt_tf["q"], gt_t)) + np.asarray(pm["attitude_offset_deg"])

    def make(kind, title, ylabel, truth, pvalue, ivalue, filename):
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        axes[0].set_title(title)
        labels = ("x", "y", "z") if kind != "attitude" else ("roll", "pitch", "yaw")
        if kind == "position":
            x_limits, yz_limits, scale_ratio = position_reference_limits(truth, pvalue)
        else:
            scale_ratio = None
        for axis, name, i in zip(axes, labels, range(3)):
            axis.plot(gt_t, truth[:, i], color=COLORS["gt"], lw=1.8, label="Ground Truth")
            axis.plot(proposed["t"], pvalue[:, i], color=COLORS["proposed"], lw=1.2, label="Proposed Method")
            axis.plot(imu["t"], ivalue[:, i], color=COLORS["imu"], lw=1.0, label="IMU Integration")
            if kind == "position":
                axis.set_ylim(x_limits if i == 0 else yz_limits)
            elif kind == "velocity":
                axis.set_ylim(padded_reference_limits(truth[:, i], pvalue[:, i]))
            axis.set_ylabel(ylabel.format(name=name))
            axis.grid(alpha=.25)
            axis.set_xlim(start, end)
        axes[0].legend(ncol=3, loc="best")
        axes[-1].set_xlabel("Time [s]")
        fig.tight_layout()
        fig.savefig(FIG / filename, dpi=220)
        fig.savefig(FIG / filename.replace(".png", ".pdf"))
        plt.close(fig)
        return scale_ratio

    scale_ratio = make(
        "position", "PositionComparison", "p{name} [m]", gt_p,
        proposed["p"],
        imu["p"] - np.asarray(im["position_offset_m"]) + np.asarray(pm["position_offset_m"]),
        f"fig_sim_{slug}_position.png")
    make("velocity", "Velocity Comparison", "v{name} [m/s]", gt_v,
         proposed["v"], imu["v"], f"fig_sim_{slug}_velocity.png")
    proposed_rpy_for_plot = rpy_deg(proposed["q"])
    imu_rpy_for_plot = (rpy_deg(imu["q"]) - np.asarray(im["attitude_offset_deg"])
                        + np.asarray(pm["attitude_offset_deg"]))
    make("attitude", "Attitude Comparison", "{name} [deg]", gt_rpy,
         proposed_rpy_for_plot, imu_rpy_for_plot, f"fig_sim_{slug}_attitude.png")
    return {
        "reference_methods": ["Ground Truth", "Proposed Method"],
        "imu_changes_visible_limits": False,
        "py_pz_zero_centered_common_scale": True,
        "px_to_yz_span_ratio": scale_ratio,
    }


def analyze_case(label, cfg):
    source, imu_db = cfg["source"], cfg["imu"]
    t0 = trigger_on(source)
    imu_t0 = trigger_on(imu_db)
    gt_pos = load_vec(source, "/sim/position", t0)
    gt_bv = load_vec(source, "/sim/body_velocity", t0)
    proposed = load_odom(source, "/ekf", t0)
    gt_tf = load_tf(source, t0, proposed)
    imu = load_odom(imu_db, "/imu_only/ekf", imu_t0)
    start = max(0.0, proposed["t"].min(), imu["t"].min(), gt_pos["t"].min(), gt_tf["t"].min())
    end = min(proposed["t"].max(), imu["t"].max(), gt_pos["t"].max(), gt_tf["t"].max())
    proposed, imu = crop(proposed, start, end), crop(imu, start, end)
    pm = estimator_metrics(proposed, gt_pos, gt_bv, gt_tf)
    im = estimator_metrics(imu, gt_pos, gt_bv, gt_tf)
    diag = imu_diagnostics(imu, im)
    existing = json.loads(cfg["existing"].read_text())
    plot_axis_policy = plot_case(
        label, cfg["slug"], start, end, gt_pos, gt_bv, gt_tf,
        proposed, imu, pm, im)
    result = {
        "source_bag": str(source), "imu_replay_bag": str(imu_db),
        "analysis_window_s": [float(start), float(end)], "duration_s": float(end - start),
        "proposed_method": {k: v for k, v in pm.items() if k != "series"},
        "imu_integration": {k: v for k, v in im.items() if k != "series"},
        "imu_diagnostics": diag,
        "plot_axis_policy": plot_axis_policy,
        "outer_fusion_existing": existing["outer_fusion"],
        "lidar_existing": existing["lidar"],
    }
    if "empty" in cfg:
        result["excluded_empty_file"] = str(cfg["empty"])
    return result


def vec(values, digits=3):
    return " / ".join(f"{x:.{digits}f}" for x in values)


def write_report(results):
    w, l = results["Walk"], results["WLW"]
    lines = [
        "# 5.3 平地實驗（模擬）", "",
        "## 5.3.1 資料選取與分析方法", "",
        "本節分析平地 Walk 與 WLW 模擬資料。由於模擬可重現性高，每種步態僅採一組資料，因此所有表格皆呈現單次結果，不計算平均值、標準差或顯著性檢定。全篇位置、速度與 bias 使用 SI 制，姿態角使用 deg。", "",
        "| 步態 | 原始資料 | 分析區間 [s] | 重複次數 |", "|---|---|---:|---:|",
        f"| Walk | `FLAT_WALK_NEW_SIM/walk_openloop.db3` | {w['analysis_window_s'][0]:.3f}–{w['analysis_window_s'][1]:.3f} | 1 |",
        f"| WLW | `FLAT_WLW_NEW_SIM/FLAT_WLW_NEW_SIM_0.db3` | {l['analysis_window_s'][0]:.3f}–{l['analysis_window_s'][1]:.3f} | 1 |", "",
        "Walk 目錄中的 `FLAT_WALK_NEW_SIM_0.db3` 為 0-byte 空檔，故排除；有效資料為同目錄的 `walk_openloop.db3`。Ground truth 使用 `/sim/position`、`/sim/body_velocity` 與 `/tf` 的 `odom → base_link`。位置以共同區間起始 500 筆的固定 offset 對齊，姿態同樣移除起始固定角度 offset；速度不做 offset 對齊。", "",
        "Proposed Method 使用原 bag 的 `/ekf`。IMU Integration 將既有 `/imu_noisy` 與 `/motor/state` 重播至同一個 `corgi_leg_odom` prediction-only 模式，只執行 IMU propagation，不使用腿部速度更新、ZUPT、GMO/contact、LiDAR、`/fusion/bv` 或 ground truth 校正。輸入 IMU 為 1 kHz CX5 規格雜訊模型；節點名義 propagation 頻率為 500 Hz。", "",
        "## 5.3.2 位置與速度估測結果", "",
        "| 步態 | 方法 | 位置 RMSE X / Y / Z [m] | 位置 RMSE 3D [m] | 速度 RMSE vx / vy / vz [m/s] | 速度 RMSE 3D [m/s] |", "|---|---|---:|---:|---:|---:|",
    ]
    for gait in ("Walk", "WLW"):
        r = results[gait]
        for method, key in (("Proposed Method（ES-EKF）", "proposed_method"), ("IMU Integration", "imu_integration")):
            m = r[key]
            lines.append(f"| {gait} | {method} | {vec(m['position_rmse_xyz_m'])} | {m['position_rmse_3d_m']:.3f} | {vec(m['velocity_rmse_xyz_mps'])} | {m['velocity_rmse_3d_mps']:.3f} |")
    lines += ["", "### 現有外部融合與 LiDAR 估測結果", "",
              "下表沿用各模擬資料既有 `metrics.json` 的單次結果，作為完整估測鏈的補充；主比較仍為 Proposed Method 與 IMU Integration。", "",
              "| 步態 | `/odom_mapping` 位置 RMSE 3D [m] | `/fusion/bv` RMSE vx / vy / vz [m/s] | LiDAR 位置 RMSE 3D [m] |", "|---|---:|---:|---:|"]
    for gait in ("Walk", "WLW"):
        r = results[gait]
        lines.append(f"| {gait} | {r['outer_fusion_existing']['position_rmse_3d_m']:.3f} | {vec(r['outer_fusion_existing']['body_velocity_rmse_xyz_mps'])} | {r['lidar_existing']['position_rmse_3d_m']:.3f} |")
    lines += ["", "位置與速度圖的顯示範圍僅由 Ground Truth 與 Proposed Method 決定；IMU Integration 僅疊加顯示，其超出範圍的漂移不會擴張座標軸。位置圖的 $p_y$ 與 $p_z$ 使用以 0 為中心的相同尺度，$p_x$ 顯示跨度則設定為 Y/Z 跨度的整數倍。Walk 與 WLW 的 X/YZ 跨度比分別為 **{} 倍**與 **{} 倍**。".format(w["plot_axis_policy"]["px_to_yz_span_ratio"], l["plot_axis_policy"]["px_to_yz_span_ratio"]), "",
              "### Walk 位置與速度時序", "",
              "![Walk position comparison](figures/fig_sim_walk_position.png)", "",
              "![Walk velocity comparison](figures/fig_sim_walk_velocity.png)", "",
              "### WLW 位置與速度時序", "",
              "![WLW position comparison](figures/fig_sim_wlw_position.png)", "",
              "![WLW velocity comparison](figures/fig_sim_wlw_velocity.png)", "",
              "### IMU 積分漂移分析", "",
              "| 步態 | 最終水平位置誤差 [m] | 最終速度誤差 X / Y / Z [m/s] | 水平等效殘差 [m/s²] | 等效重力傾角 [deg] |", "|---|---:|---:|---:|---:|"]
    for gait in ("Walk", "WLW"):
        r, m, d = results[gait], results[gait]["imu_integration"], results[gait]["imu_diagnostics"]
        lines.append(f"| {gait} | {m['final_horizontal_error_m']:.3f} | {vec(m['final_velocity_error_xyz_mps'])} | {d['horizontal_effective_acceleration_mps2']:.4f} | {d['equivalent_gravity_tilt_deg']:.3f} |")
    lines += ["", "IMU Integration 的位置誤差隨時間呈加速成長，而速度誤差近似線性累積，符合未受觀測約束的慣性積分特性。模擬輸入已包含 CX5 規格的白雜訊、初始 bias 與 bias random walk；靜態初始化只能估計初始平均 bias，後續微小的姿態與 bias 殘差仍會被一重與二重積分放大。", ""]
    for gait in ("Walk", "WLW"):
        d = results[gait]["imu_diagnostics"]
        lines.append(f"- {gait}：純 IMU 輸出 {d['messages']:,} 筆，正時間差中位數 {d['median_positive_dt_s']*1000:.3f} ms（約 {d['effective_median_rate_hz']:.1f} Hz），重複時間戳 {d['duplicate_timestamps']} 筆。")
    lines += ["", "因此「1 kHz IMU」是輸入資料率；本實驗的 prediction-only 節點實際名義 propagation 為 500 Hz，不能描述為逐筆 1 kHz 積分。", "",
              "## 5.3.3 姿態估測結果", "",
              "| 步態 | 方法 | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] |", "|---|---|---:|---:|---:|"]
    for gait in ("Walk", "WLW"):
        for method, key in (("Proposed Method（ES-EKF）", "proposed_method"), ("IMU Integration", "imu_integration")):
            m = results[gait][key]
            values = m["rpy_rmse_deg"]
            lines.append(f"| {gait} | {method} | {values[0]:.3f} | {values[1]:.3f} | {values[2]:.3f} |")
    lines += ["", "### Walk 姿態時序", "", "![Walk attitude comparison](figures/fig_sim_walk_attitude.png)", "",
              "### WLW 姿態時序", "", "![WLW attitude comparison](figures/fig_sim_wlw_attitude.png)", "",
              "## 5.3.4 小結", ""]
    for gait in ("Walk", "WLW"):
        p, i = results[gait]["proposed_method"], results[gait]["imu_integration"]
        lines.append(f"{gait} 單次模擬中，Proposed Method 的位置與速度 3D RMSE 分別為 **{p['position_rmse_3d_m']:.3f} m** 與 **{p['velocity_rmse_3d_mps']:.3f} m/s**；純 IMU 積分則為 **{i['position_rmse_3d_m']:.3f} m** 與 **{i['velocity_rmse_3d_mps']:.3f} m/s**。")
        lines.append("")
    lines += ["模擬結果顯示，當 IMU 使用相同雜訊模型與初始化流程時，純慣性 propagation 仍會快速累積速度與位置漂移；Proposed Method 透過腿部運動學速度觀測持續約束慣性狀態，因此能將位置與速度誤差維持在穩定範圍。由於每種步態僅一組固定資料，本節僅報告案例結果，不將其解讀為跨隨機種子的統計分布。"]
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    results = {label: analyze_case(label, cfg) for label, cfg in CASES.items()}
    SUMMARY_JSON.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_report(results)
    print(json.dumps({g: {"proposed_p3d": r["proposed_method"]["position_rmse_3d_m"],
                               "imu_p3d": r["imu_integration"]["position_rmse_3d_m"],
                               "imu_v3d": r["imu_integration"]["velocity_rmse_3d_mps"]}
                      for g, r in results.items()}, indent=2))
    print(REPORT)


if __name__ == "__main__":
    main()
