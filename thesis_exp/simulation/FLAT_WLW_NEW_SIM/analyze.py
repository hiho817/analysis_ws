#!/usr/bin/env python3
"""Analyse the walk_openloop simulation bag against its native ground truth."""
import json
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Vector3Stamped, TransformStamped
from tf2_msgs.msg import TFMessage
from corgi_msgs.msg import GMOContactStateStamped, SimLegContactStamped, TriggerStamped


BAG = Path("/home/hiho817/analysis_ws/thesis_exp/simulation/FLAT_WLW_NEW_SIM/FLAT_WLW_NEW_SIM_0.db3")
OUT = BAG.parent / "results"
LEGS = ("LF", "RF", "RH", "LH")
MODULES = ("module_a", "module_b", "module_c", "module_d")


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def rmse(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x * x))) if len(x) else float("nan")


def interp(t, x, tq):
    return interp1d(t, x, axis=0, bounds_error=False, fill_value=np.nan)(tq)


def rpy(q):
    return Rotation.from_quat(q[:, [1, 2, 3, 0]]).as_euler("ZYX")[:, ::-1]


def unwrap_deg(x):
    return np.degrees(np.unwrap(np.radians(x), axis=0))


def rows(cur, topic):
    return cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) ORDER BY timestamp", (topic,)).fetchall()


def load_vec(rows_, cls, t0):
    t, xyz = [], []
    for storage_t, raw in rows_:
        m = deserialize_message(raw, cls)
        t.append(stamp(m.header.stamp) - t0 if hasattr(m, "header") else storage_t / 1e9 - t0)
        xyz.append([m.x, m.y, m.z] if isinstance(m, Vector3) else [m.vector.x, m.vector.y, m.vector.z])
    return {"t": np.asarray(t), "xyz": np.asarray(xyz)}


def load_odom(rows_, t0):
    d = {k: [] for k in ("t", "p", "v", "q")}
    for _, raw in rows_:
        m = deserialize_message(raw, Odometry)
        d["t"].append(stamp(m.header.stamp) - t0)
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        d["p"].append([p.x, p.y, p.z]); d["v"].append([v.x, v.y, v.z]); d["q"].append([q.w, q.x, q.y, q.z])
    return {k: np.asarray(v) for k, v in d.items()}


def load_tf(rows_, t0):
    t, p, q = [], [], []
    for _, raw in rows_:
        msg = deserialize_message(raw, TFMessage)
        for tr in msg.transforms:
            if tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                t.append(stamp(tr.header.stamp) - t0)
                p.append([tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z])
                r = tr.transform.rotation; q.append([r.w, r.x, r.y, r.z])
    return {"t": np.asarray(t), "p": np.asarray(p), "q": np.asarray(q)}


def load_contact(rows_, cls, t0, simulated):
    d = {"t": []} | {leg: [] for leg in LEGS}
    seen = set()
    for storage_t, raw in rows_:
        if storage_t in seen: continue
        seen.add(storage_t)
        m = deserialize_message(raw, cls)
        d["t"].append(stamp(m.header.stamp) - t0 if simulated else (storage_t / 1e9 - t0))
        for leg, module in zip(LEGS, MODULES): d[leg].append(getattr(m, module).contact)
    return {k: np.asarray(v) for k, v in d.items()}


def align_initial(est, gt, n=500):
    """Remove the fixed origin offset using the beginning of the common window."""
    gi = interp(gt["t"], gt["p"], est["t"])
    valid = np.isfinite(gi).all(1)
    idx = np.flatnonzero(valid)[:n]
    offset = np.mean(est["p"][idx] - gi[idx], axis=0)
    return gi + offset, offset


def contact_metrics(gt, est, start, end):
    grid = np.arange(start, end, 0.002)
    out = {}
    for leg in LEGS:
        g = interp(gt["t"], gt[leg].astype(float), grid) > .5
        e = interp(est["t"], est[leg].astype(float), grid) > .5
        tp, fp, fn = (g & e).sum(), (~g & e).sum(), (g & ~e).sum()
        on_g = grid[np.diff(g.astype(int), prepend=0) == 1]
        on_e = grid[np.diff(e.astype(int), prepend=0) == 1]
        latency = []
        for x in on_g:
            near = on_e[np.abs(on_e - x) <= .1]
            if len(near): latency.append(near[np.argmin(np.abs(near))] - x)
        out[leg] = {"precision": float(tp/(tp+fp)) if tp+fp else float("nan"), "recall": float(tp/(tp+fn)) if tp+fn else float("nan"), "f1": float(2*tp/(2*tp+fp+fn)) if 2*tp+fp+fn else float("nan"), "mean_latency_ms": float(np.mean(latency)*1000) if latency else float("nan")}
    return out


def save(fig, name):
    fig.tight_layout(); fig.savefig(OUT / name, dpi=180); plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(BAG); cur = con.cursor()
    trig = [deserialize_message(x[1], TriggerStamped) for x in rows(cur, "/trigger")]
    t0 = stamp(next(x for x in trig if x.enable).header.stamp)
    gt_pos = load_vec(rows(cur, "/sim/position"), Vector3, t0)
    gt_vel = load_vec(rows(cur, "/sim/velocity"), Vector3, t0)
    gt_bv = load_vec(rows(cur, "/sim/body_velocity"), Vector3, t0)
    gt_tf = load_tf(rows(cur, "/tf"), t0)
    gt_ct = load_contact(rows(cur, "/sim/leg_contact"), SimLegContactStamped, t0, True)
    ekf = load_odom(rows(cur, "/ekf"), t0); odom = load_odom(rows(cur, "/odom_mapping"), t0); lidar = load_odom(rows(cur, "/lidar_odom"), t0)
    fv = load_vec(rows(cur, "/fusion/bv"), Vector3Stamped, t0)
    gmo = load_contact(rows(cur, "/gmo/contact_state"), GMOContactStateStamped, t0, False)
    ba = load_vec(rows(cur, "/ekf/ba"), Vector3, t0); bw = load_vec(rows(cur, "/ekf/bw"), Vector3, t0)
    con.close()

    start = max(ekf["t"].min(), gt_pos["t"].min(), gt_tf["t"].min())
    end = min(ekf["t"].max(), gt_pos["t"].max(), gt_tf["t"].max())
    em = (ekf["t"] >= start) & (ekf["t"] <= end); om = (odom["t"] >= start) & (odom["t"] <= end); lm = (lidar["t"] >= start) & (lidar["t"] <= end); fm = (fv["t"] >= start) & (fv["t"] <= end)
    for d, m in ((ekf, em), (odom, om), (lidar, lm), (fv, fm)):
        for k in d: d[k] = d[k][m]
    egp, eoff = align_initial(ekf, {"t": gt_pos["t"], "p": gt_pos["xyz"]})
    ogp, ooff = align_initial(odom, {"t": gt_pos["t"], "p": gt_pos["xyz"]})
    lgp, loff = align_initial(lidar, {"t": gt_pos["t"], "p": gt_pos["xyz"]})
    ep_err, op_err, lp_err = ekf["p"] - egp, odom["p"] - ogp, lidar["p"] - lgp
    gt_rpy_e = unwrap_deg(np.degrees(interp(gt_tf["t"], rpy(gt_tf["q"]), ekf["t"])))
    erpy = unwrap_deg(np.degrees(rpy(ekf["q"]))); orpy = unwrap_deg(np.degrees(rpy(odom["q"])))
    gt_rpy_o = unwrap_deg(np.degrees(interp(gt_tf["t"], rpy(gt_tf["q"]), odom["t"])))
    rpy_offset = np.nanmean(erpy[:500] - gt_rpy_e[:500], axis=0)
    orpy_offset = np.nanmean(orpy[:100] - gt_rpy_o[:100], axis=0)
    vel_gt_e = interp(gt_bv["t"], gt_bv["xyz"], ekf["t"]); vel_gt_f = interp(gt_bv["t"], gt_bv["xyz"], fv["t"])
    cm = contact_metrics(gt_ct, gmo, start, end)
    dt = np.diff(lidar["t"]); jumps = np.linalg.norm(np.diff(lidar["p"], axis=0), axis=1)
    metrics = {
        "analysis_window_s": [float(start), float(end)], "duration_s": float(end-start),
        "ground_truth": {"position": "/sim/position", "world_velocity": "/sim/velocity", "body_velocity": "/sim/body_velocity", "attitude": "/tf odom->base_link", "contact": "/sim/leg_contact"},
        "alignment_offsets_m": {"ekf": eoff.tolist(), "odom_mapping": ooff.tolist(), "lidar_odom": loff.tolist()},
        "inner_ekf": {"position_rmse_xyz_m": [rmse(ep_err[:,i]) for i in range(3)], "position_rmse_3d_m": rmse(np.linalg.norm(ep_err,axis=1)), "position_max_3d_m": float(np.max(np.linalg.norm(ep_err,axis=1))), "velocity_rmse_xyz_mps": [rmse(ekf["v"][:,i]-vel_gt_e[:,i]) for i in range(3)], "rpy_rmse_deg": [rmse(erpy[:,i]-gt_rpy_e[:,i]-rpy_offset[i]) for i in range(3)]},
        "outer_fusion": {"position_rmse_xyz_m": [rmse(op_err[:,i]) for i in range(3)], "position_rmse_3d_m": rmse(np.linalg.norm(op_err,axis=1)), "position_max_3d_m": float(np.max(np.linalg.norm(op_err,axis=1)),), "rpy_rmse_deg": [rmse(orpy[:,i]-gt_rpy_o[:,i]-orpy_offset[i]) for i in range(3)], "body_velocity_rmse_xyz_mps": [rmse(fv["xyz"][:,i]-vel_gt_f[:,i]) for i in range(3)]},
        "lidar": {"header_frame": "odom", "messages": int(len(lidar["t"])), "mean_interval_ms": float(np.mean(dt)*1000), "gaps_over_200ms": int((dt>.2).sum()), "jumps_over_5cm": int((jumps>.05).sum()), "position_rmse_3d_m": rmse(np.linalg.norm(lp_err,axis=1)), "position_max_3d_m": float(np.max(np.linalg.norm(lp_err,axis=1)))},
        "contact": cm,
        "bias_steady_last5s": {"ba_mean": np.mean(ba["xyz"][ba["t"]>=end-5],axis=0).tolist(), "ba_std": np.std(ba["xyz"][ba["t"]>=end-5],axis=0).tolist(), "bw_mean": np.mean(bw["xyz"][bw["t"]>=end-5],axis=0).tolist(), "bw_std": np.std(bw["xyz"][bw["t"]>=end-5],axis=0).tolist()}
    }
    (OUT / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False))

    fig, ax = plt.subplots(3,1,figsize=(12,7),sharex=True)
    for i, lab in enumerate("XYZ"):
        ax[i].plot(ekf["t"], ekf["p"][:,i], label="/ekf"); ax[i].plot(ekf["t"], egp[:,i],"--",label="GT aligned"); ax[i].set_ylabel(f"{lab} [m]"); ax[i].grid(alpha=.3); ax[i].legend()
    ax[-1].set_xlabel("Time from trigger [s]"); save(fig,"fig_ekf_position.png")
    fig, ax = plt.subplots(3,1,figsize=(12,7),sharex=True)
    for i, lab in enumerate(("roll","pitch","yaw")):
        ax[i].plot(ekf["t"], erpy[:,i],label="/ekf"); ax[i].plot(ekf["t"],gt_rpy_e[:,i]+rpy_offset[i],"--",label="/tf aligned"); ax[i].set_ylabel(f"{lab} [deg]"); ax[i].grid(alpha=.3); ax[i].legend()
    ax[-1].set_xlabel("Time from trigger [s]"); save(fig,"fig_ekf_attitude.png")
    fig, ax = plt.subplots(3,1,figsize=(12,7),sharex=True)
    for i, lab in enumerate("xyz"):
        ax[i].plot(ekf["t"],ekf["v"][:,i],label="/ekf"); ax[i].plot(ekf["t"],vel_gt_e[:,i],"--",label="GT body velocity"); ax[i].set_ylabel(f"v{lab} [m/s]"); ax[i].grid(alpha=.3); ax[i].legend()
    ax[-1].set_xlabel("Time from trigger [s]"); save(fig,"fig_ekf_velocity.png")
    fig, ax=plt.subplots(figsize=(8,6)); ax.plot(egp[:,0],egp[:,1],"k--",label="GT"); ax.plot(ekf["p"][:,0],ekf["p"][:,1],label="/ekf"); ax.plot(odom["p"][:,0],odom["p"][:,1],label="/odom_mapping"); ax.plot(lidar["p"][:,0],lidar["p"][:,1],label="/lidar_odom"); ax.axis("equal"); ax.grid(alpha=.3); ax.legend(); ax.set(xlabel="X [m]",ylabel="Y [m]",title="Trajectory (each estimator origin-aligned to GT)"); save(fig,"fig_trajectory_xy.png")
    fig, ax=plt.subplots(4,1,figsize=(13,7),sharex=True)
    for a, leg in zip(ax,LEGS):
        a.step(gt_ct["t"],gt_ct[leg].astype(int),where="post",label="sim GT"); a.step(gmo["t"],gmo[leg].astype(int),where="post",label="GMO",alpha=.8); a.set_ylabel(leg); a.grid(alpha=.3); a.legend(loc="upper right")
    ax[-1].set_xlabel("Time from trigger [s]"); save(fig,"fig_contact.png")
    fig, ax=plt.subplots(2,3,figsize=(12,5),sharex=True)
    for j in range(3):
        ax[0,j].plot(ba["t"],ba["xyz"][:,j]); ax[0,j].set_title(f"ba {'xyz'[j]}"); ax[1,j].plot(bw["t"],bw["xyz"][:,j]); ax[1,j].set_title(f"bw {'xyz'[j]}")
    for a in ax.flat: a.grid(alpha=.3); a.set_xlabel("Time [s]")
    save(fig,"fig_bias.png")
    fig, ax=plt.subplots(2,1,figsize=(12,5),sharex=True); ax[0].plot(lidar["t"][1:],dt*1000); ax[0].axhline(200,ls="--",c="r"); ax[0].set_ylabel("dt [ms]"); ax[1].plot(lidar["t"],np.linalg.norm(lp_err,axis=1)); ax[1].set_ylabel("3D error [m]"); ax[1].set_xlabel("Time from trigger [s]"); [a.grid(alpha=.3) for a in ax]; save(fig,"fig_lidar_quality.png")
    m=metrics
    def f(x, n=3): return f"{x:.{n}f}"
    lines=["# FLAT_WLW_NEW_SIM 模擬分析報告", "", "## 分析設定", "", f"- 量化區間：trigger 後 {start:.3f}–{end:.3f} s（{end-start:.3f} s）。", "- Ground truth：位置 `/sim/position`、世界速度 `/sim/velocity`、機體速度 `/sim/body_velocity`、姿態 `/tf` 的 `odom → base_link`、接觸 `/sim/leg_contact`。", "- 位置與姿態指標均以共同區間起始樣本的固定 offset 對齊，以排除各估測器初始化原點差異；因此量測的是追蹤與漂移誤差。", "", "## 內部 EKF", "", f"- 位置 RMSE (X/Y/Z/3D)：{', '.join(f(x) for x in m['inner_ekf']['position_rmse_xyz_m'])} / {f(m['inner_ekf']['position_rmse_3d_m'])} m；最大 3D 誤差 {f(m['inner_ekf']['position_max_3d_m'])} m。", f"- 機體速度 RMSE (vx/vy/vz)：{', '.join(f(x) for x in m['inner_ekf']['velocity_rmse_xyz_mps'])} m/s。", f"- 姿態 RMSE (roll/pitch/yaw)：{', '.join(f(x,2) for x in m['inner_ekf']['rpy_rmse_deg'])} deg。", "", "![EKF position](fig_ekf_position.png)", "![EKF attitude](fig_ekf_attitude.png)", "![EKF velocity](fig_ekf_velocity.png)", "", "## 外部融合與 LiDAR", "", f"- `/odom_mapping` 位置 3D RMSE：{f(m['outer_fusion']['position_rmse_3d_m'])} m；姿態 RMSE (R/P/Y)：{', '.join(f(x,2) for x in m['outer_fusion']['rpy_rmse_deg'])} deg。", f"- `/fusion/bv` 機體速度 RMSE (vx/vy/vz)：{', '.join(f(x) for x in m['outer_fusion']['body_velocity_rmse_xyz_mps'])} m/s。", f"- `/lidar_odom`：{m['lidar']['messages']} 筆，平均間隔 {f(m['lidar']['mean_interval_ms'],1)} ms，>200 ms gap {m['lidar']['gaps_over_200ms']}，>5 cm jump {m['lidar']['jumps_over_5cm']}，3D RMSE {f(m['lidar']['position_rmse_3d_m'])} m。", "", "![Trajectory](fig_trajectory_xy.png)", "![LiDAR quality](fig_lidar_quality.png)", "", "## 接觸狀態", "", "| 腿 | Precision | Recall | F1 | 平均接觸延遲 (ms) |", "|---|---:|---:|---:|---:|"]
    for leg in LEGS:
        x=m['contact'][leg]; lines.append(f"| {leg} | {f(x['precision'])} | {f(x['recall'])} | {f(x['f1'])} | {f(x['mean_latency_ms'],1)} |")
    lines += ["", "![Contact](fig_contact.png)", "", "## Bias", "", f"最後 5 s 的 accelerometer bias 平均值：{np.array(m['bias_steady_last5s']['ba_mean']).round(6).tolist()} m/s²；gyro bias 平均值：{np.array(m['bias_steady_last5s']['bw_mean']).round(6).tolist()} rad/s。", "", "![Bias](fig_bias.png)", "", "## 結論", "", "- 以上結果直接以模擬原生真值比較，不涉及 VICON 時間同步或座標轉換。", "- LiDAR topic 的 header frame 為 `odom`，故本次直接在 odom 座標比較；仍使用起始固定 offset 對齊以隔離初始化差異。"]
    (OUT / "analysis_report.md").write_text("\n".join(lines)+"\n")


if __name__ == "__main__": main()
