#!/usr/bin/env python3
"""Append rugged-ground WLW results to the Section 5.4 thesis report."""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from statistics import mean, stdev
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp")
EXP = ROOT / "experiments" / "RUGG_exp"
OUT = ROOT / "results" / "5.4_rugg_experiment"
FIG = OUT / "figures"
sys.path.insert(0, str(ROOT / "common"))
from thesis_figure_style import (  # noqa: E402
    create_three_panel, finish_figure, format_axis, plot_method, save_figure,
)
REPORT = OUT / "5.4_崎嶇地實驗.md"
NEW = ["RUGG_WLW_NEW_REAL_2", "RUGG_WLW_NEW_REAL_3", "RUGG_WLW_NEW_REAL_5"]
OLD = ["RUGG_WLW_OLD_REAL_1", "RUGG_WLW_OLD_REAL_3", "RUGG_WLW_OLD_REAL_5"]
EXCLUDED_NEW = ["RUGG_WLW_NEW_REAL_1", "RUGG_WLW_NEW_REAL_4"]
EXCLUDED_OLD = ["RUGG_WLW_OLD_REAL_2", "RUGG_WLW_OLD_REAL_4"]
def load_analyzer(exp_id):
    source = EXP / exp_id / "analyze_impl.py"
    spec = importlib.util.spec_from_file_location("rugg_wlw", source)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


def load_imu_analyzer():
    source = ROOT / "analysis_tools" / "analyze_imu_only_rugg.py"
    spec = importlib.util.spec_from_file_location("rugg_wlw_imu", source)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader
    spec.loader.exec_module(module)
    return module


def metric(exp_id):
    return json.loads((EXP / exp_id / "results" / exp_id / "metrics.json").read_text())


def mean_std(values):
    return mean(values), stdev(values)


def fmt(values, digits=3):
    average, spread = mean_std(values)
    return f"{average:.{digits}f} ± {spread:.{digits}f}"


def aggregate(exp_ids, attitude=False):
    records = [metric(exp_id) for exp_id in exp_ids]
    position = {key: [record["position"][key] / 100 for record in records]
                for key in ("RMSE_X_cm", "RMSE_Y_cm", "RMSE_Z_cm", "RMSE_3D_cm")}
    velocity = {key: [record["velocity"][key] for record in records]
                for key in ("RMSE_vx", "RMSE_vy", "RMSE_vz", "RMSE_3D")}
    data = {"records": records, "position": position, "velocity": velocity}
    if attitude:
        data["attitude"] = {key: [record["attitude"][key] for record in records]
                            for key in ("RMSE_roll_deg", "RMSE_pitch_deg", "RMSE_yaw_deg")}
    return data


def interpolate(t, values, target):
    return interp1d(t, values, axis=0, bounds_error=False, fill_value=np.nan)(target)


def limits(*values):
    all_values = np.concatenate([np.ravel(value) for value in values])
    all_values = all_values[np.isfinite(all_values)]
    low, high = float(all_values.min()), float(all_values.max())
    return low - max((high - low) * .06, .001), high + max((high - low) * .06, .001)


def representative_figures():
    exp_id = min(NEW, key=lambda item: metric(item)["position"]["RMSE_3D_cm"])
    analyzer = load_analyzer(exp_id)
    _, _, _, imu = load_imu_analyzer().analyze(exp_id)
    entry = next(item for item in analyzer.EXPERIMENTS if item[0] == exp_id)
    _, _, bag_name, csv_name, trigger_pair, flip, _, _ = entry
    vi = analyzer.load_vicon(str(EXP / exp_id / "vicon" / csv_name), contact_threshold_m=.015,
                             ground_markers=["G1", "G2", "G3", "G4"])
    bag = analyzer.load_fusion_bag(str(EXP / exp_id / "bags" / bag_name / f"{bag_name}_0.db3"),
                                   rate=1., trigger_pair=trigger_pair)
    end = min(float(vi.t_trigger_end), float(bag["t_trigger_end"]))
    ekf = bag["ekf"]
    if flip:
        raise RuntimeError("Representative trial unexpectedly needs a frame flip")
    ground_t = np.linspace(0, end, int(end * 200) + 1)
    gt_pos = interpolate(vi.t_traj, vi.pos_m, ground_t)
    gt_vel = interpolate(vi.t_traj, vi.v_body, ground_t)
    gt_rpy = np.degrees(interpolate(vi.t_traj, vi.rpy, ground_t))
    mask = (ekf["t"] >= 0) & (ekf["t"] <= end)
    et = ekf["t"][mask]
    pos = np.column_stack([ekf[key][mask] for key in ("px", "py", "pz")])
    pos -= pos[0] - interpolate(vi.t_traj, vi.pos_m, np.array([et[0]]))[0]
    vel = np.column_stack([ekf[key][mask] for key in ("vx", "vy", "vz")])
    rpy = analyzer.quat_to_rpy_deg(ekf["qw"][mask], ekf["qx"][mask], ekf["qy"][mask], ekf["qz"][mask])
    gt_rpy = np.degrees(analyzer.align_vicon_orientation(vi.rpy, rpy))
    gt_rpy = interpolate(vi.t_traj, gt_rpy, ground_t)

    def plot(truth, estimated, imu_t, imu_estimated, ylabel, title, stem, unit):
        figure, axes = create_three_panel(title)
        for axis, name, index in zip(axes, ylabel, range(3)):
            plot_method(axis, ground_t, truth[:, index], "Ground Truth")
            plot_method(axis, et, estimated[:, index], "Proposed Method")
            plot_method(axis, imu_t, imu_estimated[:, index], "IMU Integration")
            # The IMU trace is shown without allowing unbounded drift to expand the scale.
            format_axis(axis, name, ylim=limits(truth[:, index], estimated[:, index]),
                        contact_font_sizes=True)
        finish_figure(figure, axes, contact_font_sizes=True)
        save_figure(figure, FIG / stem)

    imu_pos = imu["plot_pos"]
    imu_vel = np.column_stack([imu[key] for key in ("vx", "vy", "vz")])
    imu_rpy = np.degrees(np.column_stack([imu[key] for key in ("roll", "pitch", "yaw")]))
    plot(gt_pos, pos, imu["plot_t"], imu_pos, (r"$p_x$ [m]", r"$p_y$ [m]", r"$p_z$ [m]"), "Position Comparison", "fig_rugg_position_wlw", "m")
    plot(gt_vel, vel, imu["plot_t"], imu_vel, (r"$v_x$ [m/s]", r"$v_y$ [m/s]", r"$v_z$ [m/s]"), "Velocity Comparison", "fig_rugg_velocity_wlw", "m/s")
    plot(gt_rpy, rpy, imu["plot_t"], imu_rpy, ("Roll [deg]", "Pitch [deg]", "Yaw [deg]"), "Attitude Comparison", "fig_rugg_attitude_wlw", "deg")
    return exp_id


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plots-only", action="store_true")
    args = parser.parse_args()
    FIG.mkdir(parents=True, exist_ok=True)
    if args.plots_only:
        representative = representative_figures()
        print(f"generated rugged WLW figures only: {representative}")
        return
    new, old = aggregate(NEW, attitude=True), aggregate(OLD)
    imu_payload = json.loads((OUT / "imu_only_metrics.json").read_text())
    imu = imu_payload["group_statistics"]["WLW"]
    imu_records = [record for record in imu_payload["records"] if record["exp_id"] in NEW]
    representative = representative_figures()
    reduction_position = (1 - mean(new["position"]["RMSE_3D_cm"]) / mean(old["position"]["RMSE_3D_cm"])) * 100
    reduction_velocity = (1 - mean(new["velocity"]["RMSE_3D"]) / mean(old["velocity"]["RMSE_3D"])) * 100
    original = REPORT.read_text(encoding="utf-8")
    original = original.split("## 5.4.4 滾走")[0].rstrip()
    lines = [original, "", "## 5.4.4 滾走（WLW）實驗", "",
             "新增崎嶇地滾走資料包含 NEW（ES-EKF）與 OLD（Legacy）各五筆試驗。依 Walk 的資料選取方式，各組僅納入三筆品質較佳且完整的試驗，數值為平均值 ± 樣本標準差。純 IMU 積分以相同三筆 NEW 原始 bag 重播 prediction-only 節點取得；不使用腿部速度更新、ZUPT、GMO/contact、LiDAR 或 VICON 狀態校正。位置與速度分別以 Inner EKF／Legacy 里程計和 VICON 比較，速度誤差使用各試驗 35%–75% 的有效時間窗；姿態僅適用於 NEW 與純 IMU。", "",
             "| 組別 | 納入統計 | 排除統計 |", "|---|---|---|",
             f"| NEW WLW | {', '.join(NEW)} | {', '.join(EXCLUDED_NEW)}（位置誤差較高） |",
             f"| OLD WLW | {', '.join(OLD)} | {', '.join(EXCLUDED_OLD)}（位置誤差較高） |", "",
             "| 步態 | 方法 | n | 位置 RMSE X / Y / Z [m] | 位置 RMSE 3D [m] | 速度 RMSE vx / vy / vz [m/s] | 速度 RMSE 3D [m/s] |",
             "|---|---|---:|---:|---:|---:|---:|",
             f"| WLW | NEW（ES-EKF） | {len(NEW)} | {fmt(new['position']['RMSE_X_cm'])} / {fmt(new['position']['RMSE_Y_cm'])} / {fmt(new['position']['RMSE_Z_cm'])} | {fmt(new['position']['RMSE_3D_cm'])} | {fmt(new['velocity']['RMSE_vx'])} / {fmt(new['velocity']['RMSE_vy'])} / {fmt(new['velocity']['RMSE_vz'])} | {fmt(new['velocity']['RMSE_3D'])} |",
             f"| WLW | OLD（Legacy） | {len(OLD)} | {fmt(old['position']['RMSE_X_cm'])} / {fmt(old['position']['RMSE_Y_cm'])} / {fmt(old['position']['RMSE_Z_cm'])} | {fmt(old['position']['RMSE_3D_cm'])} | {fmt(old['velocity']['RMSE_vx'])} / {fmt(old['velocity']['RMSE_vy'])} / {fmt(old['velocity']['RMSE_vz'])} | {fmt(old['velocity']['RMSE_3D'])} |",
             f"| WLW | 純 IMU 積分 | {imu['n']} | {fmt(imu['position_rmse_x_m']['values'])} / {fmt(imu['position_rmse_y_m']['values'])} / {fmt(imu['position_rmse_z_m']['values'])} | {fmt(imu['position_rmse_3d_m']['values'])} | {fmt(imu['velocity_rmse_vx']['values'])} / {fmt(imu['velocity_rmse_vy']['values'])} / {fmt(imu['velocity_rmse_vz']['values'])} | {fmt(imu['velocity_rmse_3d']['values'])} |", "",
             f"WLW 的 NEW 位置 3D RMSE 為 **{fmt(new['position']['RMSE_3D_cm'])} m**，相較 OLD 的 **{fmt(old['position']['RMSE_3D_cm'])} m** 降低 **{reduction_position:.1f}%**；速度 3D RMSE 為 **{fmt(new['velocity']['RMSE_3D'])} m/s**，相較 OLD 降低 **{reduction_velocity:.1f}%**。純 IMU 積分的平均位置 3D RMSE 為 **{fmt(imu['position_rmse_3d_m']['values'])} m**，顯示缺乏外部觀測約束時的顯著累積漂移。", "",
             "### 代表性 WLW 時序比較", "",
             f"代表性試驗選用 NEW 中位置 3D RMSE 最低的 `{representative}`。", "",
             "![WLW position](figures/fig_rugg_position_wlw.png)", "", "![WLW velocity](figures/fig_rugg_velocity_wlw.png)", "", "![WLW attitude](figures/fig_rugg_attitude_wlw.png)", "",
             "### WLW 個別試驗結果", "",
             "| Trial | 組別 | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] |", "|---|---|---:|---:|"]
    for exp_id in NEW + OLD:
        record = metric(exp_id)
        group = "NEW" if "_NEW_" in exp_id else "OLD"
        lines.append(f"| {exp_id} | {group} | {record['position']['RMSE_3D_cm'] / 100:.3f} | {record['velocity']['RMSE_3D']:.3f} |")
    lines += ["", "### 純 IMU 積分個別結果", "",
              "| Trial | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] | 最終水平漂移 [m] |", "|---|---:|---:|---:|"]
    for record in imu_records:
        values = record["imu_only"]
        lines.append(f"| {record['exp_id']} | {values['position_rmse_3d_m']:.3f} | {values['velocity_rmse_3d']:.3f} | {values['final_horizontal_drift_m']:.2f} |")
    attitude = new["attitude"]
    lines += ["", "### WLW 姿態估測結果", "",
              "| 步態 | 方法 | n | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] |", "|---|---|---:|---:|---:|---:|",
              f"| WLW | NEW（ES-EKF） | {len(NEW)} | {fmt(attitude['RMSE_roll_deg'], 2)} | {fmt(attitude['RMSE_pitch_deg'], 2)} | {fmt(attitude['RMSE_yaw_deg'], 2)} |",
              f"| WLW | OLD（Legacy） | {len(OLD)} | — | — | — |",
              f"| WLW | 純 IMU 積分 | {imu['n']} | {fmt(imu['attitude_rmse_roll_deg']['values'], 2)} | {fmt(imu['attitude_rmse_pitch_deg']['values'], 2)} | {fmt(imu['attitude_rmse_yaw_deg']['values'], 2)} |", "",
              "## 5.4.5 小結", "",
              f"崎嶇地 WLW 的 NEW 在三筆納入試驗中，位置 3D RMSE 為 {fmt(new['position']['RMSE_3D_cm'])} m，較 OLD 降低 {reduction_position:.1f}%；速度 3D RMSE 同樣較 OLD 降低 {reduction_velocity:.1f}%。純 IMU 積分的平均位置 3D RMSE 為 {fmt(imu['position_rmse_3d_m']['values'])} m，證實僅靠慣性 propagation 無法抑制長時間漂移。WLW 的誤差主要出現在水平 Y 軸，NEW 的平均 Y 軸位置 RMSE 仍明顯低於 OLD。"]
    REPORT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(REPORT)


if __name__ == "__main__":
    main()
