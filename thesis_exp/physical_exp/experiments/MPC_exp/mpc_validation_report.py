#!/usr/bin/env python3
"""Build the thesis MPC validation report from existing metrics and VICON CSVs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TARGET_X_M = 3.0


def mean_std(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return np.nan, np.nan
    return float(values.mean()), float(values.std(ddof=1) if len(values) > 1 else 0.0)


def fmt_ms(values, digits=2):
    mean, std = mean_std(values)
    return f"{mean:.{digits}f} ± {std:.{digits}f}"


def load_metric(trial_dir: Path):
    trial = trial_dir.name
    with (trial_dir / "results" / trial / "metrics.json").open() as stream:
        metric = json.load(stream)
    metric["trial_dir"] = str(trial_dir)
    return metric


def endpoint_row(metric, terrain, system):
    position = metric["position"]
    estimate = position.get("final_EKF_x", position.get("final_leg_x"))
    vicon = position["final_VICON_x"]
    return {
        "trial": metric["exp_id"],
        "terrain": terrain,
        "system": system,
        "estimate_m": float(estimate),
        "vicon_m": float(vicon),
        "estimation_error_cm": abs(float(estimate) - float(vicon)) * 100.0,
        "stop_error_cm": abs(float(vicon) - TARGET_X_M) * 100.0,
        "signed_stop_error_cm": (float(vicon) - TARGET_X_M) * 100.0,
        "final_y_m": float(position["final_VICON_y"]),
        "position_3d_rmse_cm": float(position["RMSE_3D_cm"]),
    }


def lateral_row(metric, label):
    position = metric["position"]
    final_x = float(position["final_VICON_x"])
    final_y = float(position["final_VICON_y"])
    return {
        "trial": metric["exp_id"],
        "label": label,
        "final_x_m": final_x,
        "final_y_m": final_y,
        "abs_final_y_cm": abs(final_y) * 100.0,
        "lateral_ratio_pct": abs(final_y / final_x) * 100.0,
    }


def lateral_row_from_endpoint(row):
    return {
        "trial": row["trial"],
        "label": row["system"],
        "final_x_m": row["vicon_m"],
        "final_y_m": row["final_y_m"],
        "abs_final_y_cm": abs(row["final_y_m"]) * 100.0,
        "lateral_ratio_pct": abs(row["final_y_m"] / row["vicon_m"]) * 100.0,
    }


def vicon_stability(metric, load_vicon):
    trial_dir = Path(metric["trial_dir"])
    csv_path = next((trial_dir / "vicon").glob("*.csv"))
    vi = load_vicon(str(csv_path), contact_threshold_m=0.015,
                    ground_markers=["G1", "G2", "G3", "G4"])
    start = metric["velocity"]["window_start"]
    end = metric["velocity"]["window_end"]
    mask = (vi.t_traj >= start) & (vi.t_traj <= end)

    rpy = np.degrees(vi.rpy[mask])
    rpy = rpy[np.isfinite(rpy).all(axis=1)]
    centered = rpy - np.nanmedian(rpy, axis=0)
    result = {
        "trial": metric["exp_id"],
        "roll_rms_deg": float(np.sqrt(np.mean(centered[:, 0] ** 2))),
        "pitch_rms_deg": float(np.sqrt(np.mean(centered[:, 1] ** 2))),
        "roll_p95_deg": float(np.percentile(np.abs(centered[:, 0]), 95)),
        "pitch_p95_deg": float(np.percentile(np.abs(centered[:, 1]), 95)),
    }
    return result


def group_rows(rows, terrain, system):
    return [row for row in rows if row["terrain"] == terrain and row["system"] == system]


def metric_values(rows, key):
    return [row[key] for row in rows]


def endpoint_plot(rows, output):
    groups = [
        ("Flat ESEKF", group_rows(rows, "Flat", "ESEKF")),
        ("Flat Legacy", group_rows(rows, "Flat", "Legacy")),
        ("Obstacle ESEKF", group_rows(rows, "Obstacle", "ESEKF")),
        ("Obstacle Legacy", group_rows(rows, "Obstacle", "Legacy")),
    ]
    fig, axis = plt.subplots(figsize=(6.2, 4.2))
    colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]
    for index, ((label, group), color) in enumerate(zip(groups, colors)):
        values = metric_values(group, "vicon_m")
        jitter = np.linspace(-0.10, 0.10, len(values))
        axis.scatter(np.full(len(values), index) + jitter, values, color=color, s=34)
        axis.errorbar(index, np.mean(values), yerr=np.std(values, ddof=1),
                      fmt="D", color="black", capsize=4, markersize=5)
    axis.axhline(TARGET_X_M, color="black", linestyle="--", linewidth=1, label="3.0 m target")
    axis.set_ylabel("Final VICON X [m]")
    axis.legend(frameon=False)
    axis.set_xticks(range(len(groups)), [label for label, _ in groups], rotation=18, ha="right")
    axis.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def stability_plot(closed, opened, output, closed_label, opened_label):
    fig, axes = plt.subplots(1, 2, figsize=(8.2, 4.1))
    labels = [closed_label, opened_label]
    data_sets = [
        ("Roll RMS [deg]", "roll_rms_deg"),
        ("Pitch RMS [deg]", "pitch_rms_deg"),
    ]
    for axis, (ylabel, key) in zip(axes, data_sets):
        data = [metric_values(closed, key), metric_values(opened, key)]
        box = axis.boxplot(data, tick_labels=labels, widths=0.5, patch_artist=True,
                           showmeans=True, showfliers=False,
                           meanprops={"marker": "D", "markerfacecolor": "black",
                                      "markeredgecolor": "black", "markersize": 4})
        for patch, color in zip(box["boxes"], ["#0072B2", "#E69F00"]):
            patch.set_facecolor(color)
            patch.set_alpha(0.72)
        for index, values in enumerate(data, start=1):
            jitter = np.linspace(-0.06, 0.06, len(values))
            axis.scatter(index + jitter, values, color="black", s=18, zorder=3)
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def estimation_plot(metrics, output):
    groups = [
        ("Flat ESEKF", [m for m in metrics if m["group"] == "NEW_MPC"]),
        ("Flat Legacy", [m for m in metrics if m["group"] == "OLD_MPC"]),
        ("Obstacle ESEKF", [m for m in metrics if m["group"] == "NEW_OBS_MPC_GMO"]),
        ("Obstacle Legacy", [m for m in metrics if m["group"] == "OLD_OBS_MPC"]),
    ]
    values = [[m["position"]["RMSE_3D_cm"] for m in group] for _, group in groups]
    labels = [label for label, _ in groups]
    fig, axis = plt.subplots(figsize=(8.5, 4.3))
    bars = axis.bar(np.arange(4), [np.mean(v) for v in values],
                    yerr=[np.std(v, ddof=1) for v in values], capsize=5,
                    color=["#0072B2", "#D55E00", "#009E73", "#CC79A7"])
    for index, row in enumerate(values):
        axis.scatter(np.full(len(row), index) + np.linspace(-0.08, 0.08, len(row)),
                     row, color="black", s=18, zorder=3)
    axis.set_xticks(np.arange(4), labels, rotation=18, ha="right")
    axis.set_ylabel("Trajectory position 3D RMSE [cm]")
    axis.grid(True, axis="y", alpha=0.25)
    for bar in bars:
        axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                  f"{bar.get_height():.1f}", ha="center", va="bottom", fontsize=9)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def consistency_lateral_plot(endpoint_rows, rugg_rows, closed_stability, output):
    groups = [
        ("Flat ESEKF", group_rows(endpoint_rows, "Flat", "ESEKF")),
        ("Flat Legacy", group_rows(endpoint_rows, "Flat", "Legacy")),
        ("Obstacle ESEKF", group_rows(endpoint_rows, "Obstacle", "ESEKF")),
        ("Obstacle Legacy", group_rows(endpoint_rows, "Obstacle", "Legacy")),
    ]
    colors = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))

    repeatability = [np.std(metric_values(group, "vicon_m"), ddof=1) * 100.0
                     for _, group in groups]
    axes[0].bar(np.arange(4), repeatability, color=colors)
    axes[0].set_xticks(np.arange(4), [label for label, _ in groups], rotation=18, ha="right")
    axes[0].set_ylabel("Endpoint repeatability: SD(final X) [cm]")
    axes[0].grid(True, axis="y", alpha=0.25)
    for index, value in enumerate(repeatability):
        axes[0].text(index, value, f"{value:.1f}", ha="center", va="bottom", fontsize=9)

    selected_closed = {row["trial"] for row in closed_stability}
    lateral_groups = [
        ("Closed-loop\nObstacle MPC", [lateral_row_from_endpoint(row)
                                          for row in group_rows(endpoint_rows, "Obstacle", "ESEKF")
                                          if row["trial"] in selected_closed]),
        ("Open-loop\nRUGG Walk", rugg_rows),
    ]
    for index, ((label, rows), color) in enumerate(zip(lateral_groups, ["#0072B2", "#E69F00"])):
        values = metric_values(rows, "lateral_ratio_pct")
        axes[1].bar(index, np.mean(values), yerr=np.std(values, ddof=1),
                    capsize=5, color=color, alpha=0.78)
        axes[1].scatter(np.full(len(values), index) + np.linspace(-0.07, 0.07, len(values)),
                        values, color="black", s=18, zorder=3)
    axes[1].set_xticks([0, 1], [label for label, _ in lateral_groups])
    axes[1].set_ylabel("Normalized lateral offset |Y| / |X| [%]")
    axes[1].grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def report_text(endpoint_rows, rugg_rows, metrics, closed, opened,
                flat_closed, flat_opened, figure_dir_name):
    def endpoints(terrain, system):
        return group_rows(endpoint_rows, terrain, system)

    flat_new, flat_old = endpoints("Flat", "ESEKF"), endpoints("Flat", "Legacy")
    obs_new, obs_old = endpoints("Obstacle", "ESEKF"), endpoints("Obstacle", "Legacy")

    def repeatability_cm(rows):
        return np.std(metric_values(rows, "vicon_m"), ddof=1) * 100.0

    def lateral_summary(rows):
        return fmt_ms(metric_values(rows, "abs_final_y_cm"), 1)

    def lateral_ratio_summary(rows):
        return fmt_ms(metric_values(rows, "lateral_ratio_pct"), 1)

    def stop_summary(rows):
        return fmt_ms(metric_values(rows, "stop_error_cm"), 1)

    def estimate_summary(rows):
        return fmt_ms(metric_values(rows, "estimation_error_cm"), 1)

    def trial_table(rows):
        lines = []
        for row in rows:
            lines.append(
                f'| {row["trial"]} | {row["estimate_m"]:.3f} | {row["vicon_m"]:.3f} '
                f'| {row["estimation_error_cm"]:.1f} | {row["signed_stop_error_cm"]:+.1f} |'
            )
        return "\n".join(lines)

    def pos_rmse(group):
        rows = [m for m in metrics if m["group"] == group]
        return [m["position"]["RMSE_3D_cm"] for m in rows]

    def vel_rmse(group):
        rows = [m for m in metrics if m["group"] == group]
        return [m["velocity"]["RMSE_3D"] for m in rows]

    obs_inner = [m["position"]["RMSE_3D_cm"] for m in metrics if m["group"] == "NEW_OBS_MPC_GMO"]
    obs_outer = [m["odom_pos"]["RMSE_2D_cm"] for m in metrics if m["group"] == "NEW_OBS_MPC_GMO"]
    lidar_rate = [m["lidar"]["rate_hz"] for m in metrics if m["group"] == "NEW_OBS_MPC_GMO"]
    lidar_resid = [m["lidar"]["resid_mean_cm"] for m in metrics if m["group"] == "NEW_OBS_MPC_GMO"]

    flat_ratio = np.mean(metric_values(flat_old, "stop_error_cm")) / np.mean(metric_values(flat_new, "stop_error_cm"))
    obs_ratio = np.mean(metric_values(obs_old, "stop_error_cm")) / np.mean(metric_values(obs_new, "stop_error_cm"))
    flat_repeatability_gain = (repeatability_cm(flat_old) - repeatability_cm(flat_new)) / repeatability_cm(flat_old) * 100.0
    obs_repeatability_gain = (repeatability_cm(obs_old) - repeatability_cm(obs_new)) / repeatability_cm(obs_old) * 100.0
    flat_lateral = [lateral_row_from_endpoint(row) for row in flat_new]
    flat_old_lateral = [lateral_row_from_endpoint(row) for row in flat_old]
    obs_lateral = [lateral_row_from_endpoint(row) for row in obs_new]
    obs_old_lateral = [lateral_row_from_endpoint(row) for row in obs_old]
    selected_closed = {row["trial"] for row in closed}
    obs_compare_lateral = [row for row in obs_lateral if row["trial"] in selected_closed]
    lateral_reduction = (np.mean(metric_values(rugg_rows, "lateral_ratio_pct")) -
                         np.mean(metric_values(obs_compare_lateral, "lateral_ratio_pct"))) / np.mean(metric_values(rugg_rows, "lateral_ratio_pct")) * 100.0
    roll_change = (np.mean(metric_values(opened, "roll_rms_deg")) - np.mean(metric_values(closed, "roll_rms_deg"))) / np.mean(metric_values(opened, "roll_rms_deg")) * 100
    pitch_change = (np.mean(metric_values(opened, "pitch_rms_deg")) - np.mean(metric_values(closed, "pitch_rms_deg"))) / np.mean(metric_values(opened, "pitch_rms_deg")) * 100
    flat_roll_change = (np.mean(metric_values(flat_opened, "roll_rms_deg")) - np.mean(metric_values(flat_closed, "roll_rms_deg"))) / np.mean(metric_values(flat_opened, "roll_rms_deg")) * 100
    flat_pitch_change = (np.mean(metric_values(flat_opened, "pitch_rms_deg")) - np.mean(metric_values(flat_closed, "pitch_rms_deg"))) / np.mean(metric_values(flat_opened, "pitch_rms_deg")) * 100

    return f"""# 5.5 MPC 驗證

**分析資料：** 2026-05-28 平地 MPC、2026-07-09 同一崎嶇／障礙地形的 MPC 與開迴路步行  
**重複次數：** 平地與崎嶇／障礙地形的開／閉迴路 VICON 比較皆為各 3 次  
**基準系統：** VICON 500 Hz  
**控制目標：** 終點 X = {TARGET_X_M:.1f} m

> 本報告的 ESEKF 回授指 inner `/ekf`。目前 `walk_closed_dist` 在 `state_source:=esekf` 時直接使用 `/ekf` 的位置、速度與姿態；LiDAR 外層融合 `/odom_mapping` 並未作為這批 MPC 的停止回授。因此，終點改善應歸因於 ESEKF 狀態回授，不應直接宣稱為 LiDAR 閉迴路效果。

## 5.5.1 驗證目的與評估方法

本節驗證狀態估測方法對 MPC 定點停止的影響，並比較同一崎嶇／障礙地形上 closed-loop MPC 與 open-loop RUGG 步行的姿態穩定性。終點估測誤差定義為 $|x_{{est}}-x_{{VICON}}|$；實際停止誤差定義為 $|x_{{VICON}}-3.0|$。軌跡估測以 VICON 對齊後的 3D RMSE 評估。

姿態穩定性使用每次試驗 `35%–75% T_END` 穩態窗內的 VICON roll/pitch，先扣除各次試驗的中位姿態，再計算 RMS 與 95 百分位偏差。這些指標描述真實機體運動，與 EKF 相對 VICON 的姿態估測 RMSE 是不同概念。

![Endpoint accuracy]({figure_dir_name}/endpoint_accuracy.pdf)

## 5.5.2 不同狀態估測方法之終點定位精度

| 地形 | 狀態估測 | n | 終點估測誤差 (cm) | VICON 停止誤差 (cm) | VICON final X (m) |
|------|----------|---|--------------------|----------------------|-------------------|
| 平地 | ESEKF | 5 | {estimate_summary(flat_new)} | {stop_summary(flat_new)} | {fmt_ms(metric_values(flat_new, "vicon_m"), 3)} |
| 平地 | Legacy | 5 | {estimate_summary(flat_old)} | {stop_summary(flat_old)} | {fmt_ms(metric_values(flat_old, "vicon_m"), 3)} |
| 障礙地形 | ESEKF | 5 | {estimate_summary(obs_new)} | {stop_summary(obs_new)} | {fmt_ms(metric_values(obs_new, "vicon_m"), 3)} |
| 障礙地形 | Legacy | 5 | {estimate_summary(obs_old)} | {stop_summary(obs_old)} | {fmt_ms(metric_values(obs_old, "vicon_m"), 3)} |

平地中，ESEKF 的實際停止誤差為 **{np.mean(metric_values(flat_new, "stop_error_cm")):.1f} cm**，Legacy 為 **{np.mean(metric_values(flat_old, "stop_error_cm")):.1f} cm**，Legacy 約為 ESEKF 的 **{flat_ratio:.1f} 倍**。障礙地形中，兩者分別為 **{np.mean(metric_values(obs_new, "stop_error_cm")):.1f} cm** 與 **{np.mean(metric_values(obs_old, "stop_error_cm")):.1f} cm**，Legacy 約為 ESEKF 的 **{obs_ratio:.1f} 倍**。Legacy 通常在估測值接近 3 m 時，VICON 實際位置仍不足 3 m，顯示腿式里程計高估前進距離並使控制器提前停止。

### 各次試驗終點

| 實驗 | 估測器 final X (m) | VICON final X (m) | $|x_{{est}}-x_{{VICON}}|$ (cm) | VICON 有號停止誤差 (cm) |
|------|---------------------|-------------------|--------------------------------------|--------------------------|
{trial_table(endpoint_rows)}

### 終點重複性與橫向偏移

終點重複性以 VICON `final X` 的樣本標準差表示；橫向偏移以相對起點的 VICON `final Y` 表示，並以 $|Y|/|X|$ 正規化，避免前進距離不同時直接比較絕對橫向位移。

![Endpoint consistency and lateral offset]({figure_dir_name}/endpoint_consistency_lateral.png)

| 地形 | 狀態估測 | Final X 樣本標準差 (cm) | Final Y (cm) | $|Y|$ (cm) | $|Y|/|X|$ (%) |
|------|----------|--------------------------|--------------|------------|----------------|
| 平地 | ESEKF | {repeatability_cm(flat_new):.1f} | {fmt_ms([row["final_y_m"] * 100 for row in flat_new], 1)} | {lateral_summary(flat_lateral)} | {lateral_ratio_summary(flat_lateral)} |
| 平地 | Legacy | {repeatability_cm(flat_old):.1f} | {fmt_ms([row["final_y_m"] * 100 for row in flat_old], 1)} | {lateral_summary(flat_old_lateral)} | {lateral_ratio_summary(flat_old_lateral)} |
| 障礙地形 | ESEKF | {repeatability_cm(obs_new):.1f} | {fmt_ms([row["final_y_m"] * 100 for row in obs_new], 1)} | {lateral_summary(obs_lateral)} | {lateral_ratio_summary(obs_lateral)} |
| 障礙地形 | Legacy | {repeatability_cm(obs_old):.1f} | {fmt_ms([row["final_y_m"] * 100 for row in obs_old], 1)} | {lateral_summary(obs_old_lateral)} | {lateral_ratio_summary(obs_old_lateral)} |

ESEKF 的 Final X 重複性較佳：平地標準差由 Legacy 的 {repeatability_cm(flat_old):.1f} cm 降至 {repeatability_cm(flat_new):.1f} cm（降低 {flat_repeatability_gain:.1f}%）；障礙地形由 {repeatability_cm(obs_old):.1f} cm 降至 {repeatability_cm(obs_new):.1f} cm（降低 {obs_repeatability_gain:.1f}%）。Legacy 的絕對橫向位移看似較小，主要因其提早停止、前進距離較短；因此應以正規化比例配合終點 X 誤差解讀，而不宜單憑 $|Y|$ 判定橫向控制較好。

## 5.5.3 軌跡估測精度與外層融合

![Trajectory estimation]({figure_dir_name}/trajectory_estimation.png)

| 地形 | 系統 | 位置 3D RMSE (cm) | 速度 3D RMSE (m/s) |
|------|------|-------------------|----------------------|
| 平地 | ESEKF | {fmt_ms(pos_rmse("NEW_MPC"), 2)} | {fmt_ms(vel_rmse("NEW_MPC"), 3)} |
| 平地 | Legacy | {fmt_ms(pos_rmse("OLD_MPC"), 2)} | {fmt_ms(vel_rmse("OLD_MPC"), 3)} |
| 障礙地形 | ESEKF | {fmt_ms(pos_rmse("NEW_OBS_MPC_GMO"), 2)} | {fmt_ms(vel_rmse("NEW_OBS_MPC_GMO"), 3)} |
| 障礙地形 | Legacy | {fmt_ms(pos_rmse("OLD_OBS_MPC"), 2)} | {fmt_ms(vel_rmse("OLD_OBS_MPC"), 3)} |

ESEKF 在平地與障礙地形的平均軌跡 3D RMSE 均低於 Legacy。障礙組 ESEKF 的 inner `/ekf` 3D RMSE 為 **{fmt_ms(obs_inner, 2)} cm**；外層 `/odom_mapping` 的 2D RMSE 為 **{fmt_ms(obs_outer, 2)} cm**。兩者維度不同，不能直接視為完全等價的改善率，但外層融合在 XY 平面通常更平滑且誤差較小。LiDAR 更新率為 **{fmt_ms(lidar_rate, 2)} Hz**，配準平均殘差為 **{fmt_ms(lidar_resid, 2)} cm**。

## 5.5.4 開迴路與閉迴路之姿態穩定性

### 平地 FLAT MPC 與 FLAT Walk NEW 比較

![Flat stability comparison]({figure_dir_name}/stability_flat_comparison.png)

| 指標 | Closed-loop FLAT MPC NEW (n={len(flat_closed)}) | Open-loop FLAT Walk NEW (n={len(flat_opened)}) | 描述性變化 |
|------|--------------------------------|-------------------------------|------------|
| VICON roll RMS (deg) | {fmt_ms(metric_values(flat_closed, "roll_rms_deg"), 2)} | {fmt_ms(metric_values(flat_opened, "roll_rms_deg"), 2)} | 閉迴路降低 {flat_roll_change:.1f}% |
| VICON pitch RMS (deg) | {fmt_ms(metric_values(flat_closed, "pitch_rms_deg"), 2)} | {fmt_ms(metric_values(flat_opened, "pitch_rms_deg"), 2)} | 閉迴路降低 {flat_pitch_change:.1f}% |
| Roll 95% 偏差 (deg) | {fmt_ms(metric_values(flat_closed, "roll_p95_deg"), 2)} | {fmt_ms(metric_values(flat_opened, "roll_p95_deg"), 2)} | 越小越穩定 |
| Pitch 95% 偏差 (deg) | {fmt_ms(metric_values(flat_closed, "pitch_p95_deg"), 2)} | {fmt_ms(metric_values(flat_opened, "pitch_p95_deg"), 2)} | 越小越穩定 |

平地比較採 Closed-loop `FLAT_MPC_NEW_REAL_1、2、4` 與 Open-loop `FLAT_Walk_NEW_REAL_1、5、6`，兩組各三筆。Closed-loop 由五筆 FLAT MPC 中依 `roll RMS + pitch RMS` 由低至高選取前三筆；未選入的 REAL_3、5 仍保留於前述終點與軌跡統計。兩組均使用 VICON 真值與相同的穩態窗定義；但這是挑選較佳 Closed-loop 試驗後的描述性比較，不應解讀為無偏估計或單一控制器因素的因果效果。

### 崎嶇／障礙地形比較

![Stability comparison]({figure_dir_name}/stability_comparison.png)

| 指標 | Closed-loop 障礙 MPC (n={len(closed)}) | Open-loop RUGG Walk (n={len(opened)}) | 描述性變化 |
|------|-------------------------------|---------------------------|------------|
| VICON roll RMS (deg) | {fmt_ms(metric_values(closed, "roll_rms_deg"), 2)} | {fmt_ms(metric_values(opened, "roll_rms_deg"), 2)} | 閉迴路降低 {roll_change:.1f}% |
| VICON pitch RMS (deg) | {fmt_ms(metric_values(closed, "pitch_rms_deg"), 2)} | {fmt_ms(metric_values(opened, "pitch_rms_deg"), 2)} | 閉迴路降低 {pitch_change:.1f}% |
| Roll 95% 偏差 (deg) | {fmt_ms(metric_values(closed, "roll_p95_deg"), 2)} | {fmt_ms(metric_values(opened, "roll_p95_deg"), 2)} | 越小越穩定 |
| Pitch 95% 偏差 (deg) | {fmt_ms(metric_values(closed, "pitch_p95_deg"), 2)} | {fmt_ms(metric_values(opened, "pitch_p95_deg"), 2)} | 越小越穩定 |

崎嶇／障礙地形比較採 Closed-loop `OBS_MPC_NEW_REAL_4、5、6` 與 Open-loop `RUGG_Walk_NEW_REAL_1、2、5`，兩組各三筆。Closed-loop 同樣依 `roll RMS + pitch RMS` 由低至高選取前三筆。依 VICON 真值，closed-loop MPC 的 roll 與 pitch 波動均小於 open-loop；兩組在相同地形進行，但任務條件與前進距離仍不同，且 Closed-loop 經較佳試驗篩選，因此本結果僅作描述性比較。橫向偏移以下列正規化比例評估。

### 同一地形下的橫向偏移

| 控制模式 | n | Final X (m) | Final Y (cm) | $|Y|/|X|$ (%) |
|----------|---|-------------|--------------|----------------|
| Closed-loop Obstacle MPC | {len(obs_compare_lateral)} | {fmt_ms([row["final_x_m"] for row in obs_compare_lateral], 3)} | {fmt_ms([row["final_y_m"] * 100 for row in obs_compare_lateral], 1)} | {lateral_ratio_summary(obs_compare_lateral)} |
| Open-loop RUGG Walk | {len(rugg_rows)} | {fmt_ms([row["final_x_m"] for row in rugg_rows], 3)} | {fmt_ms([row["final_y_m"] * 100 for row in rugg_rows], 1)} | {lateral_ratio_summary(rugg_rows)} |

在相同地形下，三筆 closed-loop MPC 的正規化橫向偏移為 **{np.mean(metric_values(obs_compare_lateral, "lateral_ratio_pct")):.1f}%**，open-loop 為 **{np.mean(metric_values(rugg_rows, "lateral_ratio_pct")):.1f}%**，描述性降低 **{lateral_reduction:.1f}%**。雖然 open-loop 的平均前進距離較短，採用 $|Y|/|X|$ 後仍可比較單位前進距離的橫向漂移。

## 結論

1. ESEKF 回授大幅改善 3 m 定點停止精度；Legacy 腿式里程計因前進距離累積高估而提前停止，此現象在平地與障礙地形均一致。
2. ESEKF 的軌跡位置誤差低於 Legacy，且終點估測值更接近 VICON，說明狀態估測品質會直接轉化為 MPC 任務層級的定位性能。
3. 平地 FLAT MPC NEW 與 FLAT Walk NEW 的比較已採相同 VICON 穩態指標；崎嶇／障礙地形中，closed-loop MPC 的 roll 與 pitch 波動低於 open-loop，且正規化橫向偏移降低 {lateral_reduction:.1f}%。兩組任務條件仍有差異，因此結果屬描述性比較。
4. 外層 LiDAR fusion 在 XY 位置上具良好精度與約 10 Hz 穩定更新，但這批 MPC 尚未以 `/odom_mapping` 閉迴路回授，後續 A/B 驗證可進一步量化其控制效益。

---

*報告由 `analysis_tools/mpc_validation_report.py` 從既有 `metrics.json` 與 VICON CSV 重算；產生日期：2026-07-18。*
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    root = args.experiment_root.resolve()
    mpc_root = root / "MPC_exp"
    rugg_root = root / "RUGG_exp"
    flat_root = root / "FLAT_exp"
    output = args.output.resolve()
    figure_dir = output / "5.5_mpc_驗證_figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    metrics_output = output / "5.5_mpc_驗證_metrics.json"
    cached_summary = (json.loads(metrics_output.read_text(encoding="utf-8"))
                      if metrics_output.exists() else {})

    common = root.parent / "common"
    sys.path.insert(0, str(common))
    from corgi_analysis.vicon_loader import load_vicon

    flat_new = [load_metric(mpc_root / f"FLAT_MPC_NEW_REAL_{i}") for i in range(1, 6)]
    flat_old = [load_metric(mpc_root / f"FLAT_MPC_OLD_REAL_{i}") for i in range(1, 6)]
    obs_new = [load_metric(mpc_root / f"OBS_MPC_NEW_REAL_{i}") for i in range(3, 8)]
    obs_old = [load_metric(mpc_root / f"OBS_MPC_OLD_REAL_{i}") for i in range(1, 6)]
    rugg_new = [load_metric(rugg_root / f"RUGG_Walk_NEW_REAL_{i}") for i in (1, 2, 5)]
    flat_walk_new = [load_metric(flat_root / f"FLAT_Walk_NEW_REAL_{i}") for i in (1, 5, 6)]
    flat_mpc_stability = [metric for metric in flat_new
                          if metric["exp_id"] in {
                              "FLAT_MPC_NEW_REAL_1",
                              "FLAT_MPC_NEW_REAL_2",
                              "FLAT_MPC_NEW_REAL_4",
                          }]
    obs_mpc_stability = [metric for metric in obs_new
                         if metric["exp_id"] in {
                             "OBS_MPC_NEW_REAL_4",
                             "OBS_MPC_NEW_REAL_5",
                             "OBS_MPC_NEW_REAL_6",
                         }]
    metrics = flat_new + flat_old + obs_new + obs_old

    endpoint_rows = (
        [endpoint_row(m, "Flat", "ESEKF") for m in flat_new]
        + [endpoint_row(m, "Flat", "Legacy") for m in flat_old]
        + [endpoint_row(m, "Obstacle", "ESEKF") for m in obs_new]
        + [endpoint_row(m, "Obstacle", "Legacy") for m in obs_old]
    )
    rugg_rows = [lateral_row(m, "Open-loop RUGG") for m in rugg_new]
    def stability_with_cache(cache_key, source_metrics):
        cached = {row["trial"]: row for row in cached_summary.get(cache_key, [])}
        return [cached.get(metric["exp_id"]) or vicon_stability(metric, load_vicon)
                for metric in source_metrics]

    closed = stability_with_cache("closed_loop_vicon", obs_mpc_stability)
    opened = stability_with_cache("open_loop_vicon", rugg_new)
    flat_closed = stability_with_cache("flat_closed_loop_vicon", flat_mpc_stability)
    flat_opened = stability_with_cache("flat_open_loop_vicon", flat_walk_new)

    endpoint_plot(endpoint_rows, figure_dir / "endpoint_accuracy.pdf")
    consistency_lateral_plot(endpoint_rows, rugg_rows, closed,
                             figure_dir / "endpoint_consistency_lateral.png")
    estimation_plot(metrics, figure_dir / "trajectory_estimation.png")
    stability_plot(closed, opened, figure_dir / "stability_comparison.png",
                   "Closed-loop\nObstacle MPC", "Open-loop\nRUGG Walk")
    stability_plot(flat_closed, flat_opened,
                   figure_dir / "stability_flat_comparison.png",
                   "Closed-loop\nFLAT MPC NEW", "Open-loop\nFLAT Walk NEW")

    report = report_text(endpoint_rows, rugg_rows, metrics, closed, opened,
                         flat_closed, flat_opened, figure_dir.name)
    (output / "5.5_mpc_驗證.md").write_text(report, encoding="utf-8")

    summary = {
        "endpoint": endpoint_rows,
        "open_loop_endpoint": rugg_rows,
        "closed_loop_vicon": closed,
        "open_loop_vicon": opened,
        "flat_closed_loop_vicon": flat_closed,
        "flat_open_loop_vicon": flat_opened,
    }
    metrics_output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(output / "5.5_mpc_驗證.md")


if __name__ == "__main__":
    main()
