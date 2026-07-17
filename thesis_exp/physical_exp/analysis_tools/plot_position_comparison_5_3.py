#!/usr/bin/env python3
"""Plot the best Proposed Method position comparison for each gait in Section 5.3.

Walk and WLW are selected separately by the smallest Proposed Method 3D
position RMSE. Ground truth, IMU integration, and the proposed method are
aligned to the VICON position at their first common sample and restricted to
one common time interval per gait.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp")
RESULT = ROOT / "results" / "5.3_flat_experiment"
FIGURE_DIR = RESULT / "figures"
ANALYZER = ROOT / "analysis_tools" / "analyze_imu_only_5_3.py"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_imu_only_5_3", ANALYZER)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def finite_position(data, keys, start, end):
    mask = (data["t"] >= start) & (data["t"] <= end)
    values = np.column_stack([data[key][mask] for key in keys])
    times = data["t"][mask]
    finite = np.isfinite(times) & np.isfinite(values).all(axis=1)
    return times[finite], values[finite]


def align_to_ground_truth(times, positions, vi):
    valid = np.isfinite(vi.pos_m).all(axis=1)
    gt_at_start = np.array([
        interp1d(vi.t_traj[valid], vi.pos_m[valid, axis], bounds_error=True)(times[0])
        for axis in range(3)
    ])
    return positions - (positions[0] - gt_at_start)


def padded_reference_limits(*series, padding=0.06):
    """Return finite limits from reference series only."""
    values = np.concatenate([np.asarray(value).ravel() for value in series])
    values = values[np.isfinite(values)]
    if values.size == 0:
        return -1.0, 1.0
    lower, upper = float(values.min()), float(values.max())
    span = upper - lower
    if span <= np.finfo(float).eps:
        span = max(abs(lower), 1.0) * 0.1
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


def position_reference_limits(gt, proposed):
    """Use common zero-centred Y/Z limits and an integer-multiple X span."""
    yz_values = np.concatenate((gt[:, 1:3].ravel(), proposed[:, 1:3].ravel()))
    yz_values = yz_values[np.isfinite(yz_values)]
    yz_half_span = nice_ceiling(np.max(np.abs(yz_values)) * 1.06)
    yz_span = 2.0 * yz_half_span

    x_lower, x_upper = padded_reference_limits(gt[:, 0], proposed[:, 0])
    ratio = max(1, int(np.ceil((x_upper - x_lower) / yz_span - 1e-12)))
    x_span = ratio * yz_span
    x_midpoint = 0.5 * (x_lower + x_upper)
    return ((x_midpoint - 0.5 * x_span, x_midpoint + 0.5 * x_span),
            (-yz_half_span, yz_half_span), ratio)


def save_position_plot(gt_t, gt, imu_t, imu, proposed_t, proposed,
                       output_stem):
    colors = {
        "Ground Truth": "#111111",
        "IMU Integration": "#D55E00",
        "Proposed Method": "#0072B2",
    }
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    fig.suptitle("PositionComparison")
    x_limits, yz_limits, scale_ratio = position_reference_limits(gt, proposed)
    for axis, (ax, component) in enumerate(zip(axes, ("x", "y", "z"))):
        ax.plot(gt_t, gt[:, axis], color=colors["Ground Truth"],
                linewidth=1.2, label="Ground Truth", zorder=3)
        ax.plot(proposed_t, proposed[:, axis],
                color=colors["Proposed Method"], linewidth=1.0,
                label="Proposed Method", zorder=2)
        ax.plot(imu_t, imu[:, axis], color=colors["IMU Integration"],
                linewidth=0.9, label="IMU Integration", zorder=1)
        ax.set_ylim(x_limits if axis == 0 else yz_limits)
        handles, legend_labels = ax.get_legend_handles_labels()
        order = [legend_labels.index(name) for name in
                 ("Ground Truth", "IMU Integration", "Proposed Method")]
        ax.legend([handles[index] for index in order],
                  [legend_labels[index] for index in order],
                  frameon=True, loc="upper right", fontsize=8)
        ax.set_ylabel(f"p{component} [m]")
        ax.grid(True, alpha=0.35, linewidth=0.7)
    axes[-1].set_xlabel("Time [s]")
    for suffix in ("png", "pdf"):
        fig.savefig(FIGURE_DIR / f"{output_stem}.{suffix}", dpi=300)
    plt.close(fig)
    return {
        "x_to_yz_span_ratio": scale_ratio,
        "x_limits_m": list(x_limits),
        "yz_limits_m": list(yz_limits),
    }


def save_velocity_plot(gt_t, gt_velocity, imu_t, imu_velocity,
                       proposed_t, proposed_velocity, output_stem):
    colors = {
        "Ground Truth": "#111111",
        "IMU Integration": "#D55E00",
        "Proposed Method": "#0072B2",
    }
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    fig.suptitle("Velocity Comparison")
    for axis, (ax, component) in enumerate(zip(axes, ("x", "y", "z"))):
        ax.plot(gt_t, gt_velocity[:, axis], color=colors["Ground Truth"],
                linewidth=1.2, label="Ground Truth", zorder=3)
        ax.plot(imu_t, imu_velocity[:, axis], color=colors["IMU Integration"],
                linewidth=0.9, label="IMU Integration", zorder=1)
        ax.plot(proposed_t, proposed_velocity[:, axis],
                color=colors["Proposed Method"], linewidth=1.0,
                label="Proposed Method", zorder=2)
        # IMU Integration is intentionally excluded from the visible-range
        # calculation because its unconstrained drift can dominate the plot.
        ax.set_ylim(padded_reference_limits(
            gt_velocity[:, axis], proposed_velocity[:, axis]))
        ax.set_ylabel(f"v{component} [m/s]")
        ax.grid(True, alpha=0.35, linewidth=0.7)
        ax.legend(frameon=True, loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Time [s]")
    for suffix in ("png", "pdf"):
        fig.savefig(FIGURE_DIR / f"{output_stem}.{suffix}", dpi=300)
    plt.close(fig)


def save_attitude_plot(gt_t, gt_rpy, imu_t, imu_rpy,
                       proposed_t, proposed_rpy, output_stem):
    colors = {
        "Ground Truth": "#111111",
        "IMU Integration": "#D55E00",
        "Proposed Method": "#0072B2",
    }
    fig, axes = plt.subplots(
        3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    fig.suptitle("Attitude Comparison")
    for axis, (ax, component) in enumerate(
            zip(axes, ("roll", "pitch", "yaw"))):
        ax.plot(gt_t, gt_rpy[:, axis], color=colors["Ground Truth"],
                linewidth=1.2, label="Ground Truth", zorder=3)
        ax.plot(imu_t, imu_rpy[:, axis], color=colors["IMU Integration"],
                linewidth=0.9, label="IMU Integration", zorder=1)
        ax.plot(proposed_t, proposed_rpy[:, axis],
                color=colors["Proposed Method"], linewidth=1.0,
                label="Proposed Method", zorder=2)
        ax.set_ylabel(f"{component} [deg]")
        ax.grid(True, alpha=0.35, linewidth=0.7)
        ax.legend(frameon=True, loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Time [s]")
    for suffix in ("png", "pdf"):
        fig.savefig(FIGURE_DIR / f"{output_stem}.{suffix}", dpi=300)
    plt.close(fig)


def update_report(selections):
    path = RESULT / "5.3_平地實驗.md"
    report = path.read_text(encoding="utf-8")
    start_marker = "<!-- POSITION_COMPARISON_FIGURES_START -->"
    end_marker = "<!-- POSITION_COMPARISON_FIGURES_END -->"
    walk = selections["WALK"]
    wlw = selections["WLW"]
    block = f"""{start_marker}

### 代表性位置時序比較

代表性試驗以 Proposed Method 的位置 3D RMSE 為選取標準，Walk 與 WLW 分別從各自納入比較的三組實驗中選出 RMSE 最低者。位置、速度與姿態皆以三列共用時間軸子圖呈現。位置與速度的顯示範圍僅由 Ground Truth 與 Proposed Method 決定，IMU Integration 僅疊加顯示，超出範圍的漂移不擴張座標軸。位置圖的 $p_y$ 與 $p_z$ 共用以 0 為中心的相同尺度；$p_x$ 顯示跨度則設為 Y/Z 跨度的整數倍，以保留前進方向較大的量級並便於比較。姿態角使用 deg。

#### Walk

Walk 選用 `{walk['experiment_id']}`，Proposed Method 的位置 3D RMSE 為 **{walk['proposed_method_position_rmse_3d_m']:.3f} m**。位置圖的 X 顯示跨度為 Y/Z 的 **{walk['position_axis_scale']['x_to_yz_span_ratio']} 倍**；位置、速度與姿態圖均呈現 0–30 s。

![Walk position time histories](figures/fig_position_walk.png)

Walk 的三軸速度以與位置相同的 0–30 s 時間窗呈現：

![Walk velocity time histories](figures/fig_velocity_walk.png)

Walk 的三軸姿態以相同的 0–30 s 時間窗呈現：

![Walk attitude time histories](figures/fig_attitude_walk.png)

#### WLW

WLW 選用 `{wlw['experiment_id']}`，Proposed Method 的位置 3D RMSE 為 **{wlw['proposed_method_position_rmse_3d_m']:.3f} m**。位置圖的 X 顯示跨度為 Y/Z 的 **{wlw['position_axis_scale']['x_to_yz_span_ratio']} 倍**；位置、速度與姿態圖均使用同一完整共同時間窗。

![WLW position time histories](figures/fig_position_wlw.png)

WLW 的三軸速度以與位置相同的共同時間窗呈現：

![WLW velocity time histories](figures/fig_velocity_wlw.png)

WLW 的三軸姿態以相同的共同時間窗呈現：

![WLW attitude time histories](figures/fig_attitude_wlw.png)

{end_marker}"""
    if start_marker in report and end_marker in report:
        before = report.split(start_marker, 1)[0].rstrip()
        after = report.split(end_marker, 1)[1].lstrip()
        report = before + "\n\n" + block + "\n\n" + after
    else:
        anchor = "### 納入試驗之個別位置與速度結果"
        report = report.replace(anchor, block + "\n\n" + anchor, 1)
    path.write_text(report.rstrip() + "\n", encoding="utf-8")


def main():
    metrics = json.loads((RESULT / "imu_only_metrics.json").read_text(encoding="utf-8"))
    analyzer = load_analyzer()
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    selections = {}
    for gait, group in (("WALK", "NEW_WALK"), ("WLW", "NEW_WLW")):
        candidates = [record for record in metrics["records"]
                      if record.get("valid_imu_only") and record.get("group") == group]
        selected = min(candidates,
                       key=lambda record: record["baseline"]["position_rmse_3d_m"])
        exp_id = selected["exp_id"]
        _, vi, baseline, imu = analyzer.analyze(exp_id)
        if baseline is None or imu is None:
            raise RuntimeError(f"{exp_id} lacks one of the required trajectories")

        common_start = max(0.0, float(baseline["t"][0]), float(imu["plot_t"][0]))
        common_end = min(float(vi.t_trigger_end), float(baseline["t"][-1]),
                         float(imu["plot_t"][-1]))
        if gait == "WALK":
            common_end = min(common_end, 30.0)
        vi_mask = ((vi.t_traj >= common_start) & (vi.t_traj <= common_end)
                   & np.isfinite(vi.pos_m).all(axis=1))
        gt_t = vi.t_traj[vi_mask]
        gt = vi.pos_m[vi_mask]

        proposed_t, proposed_raw = finite_position(
            baseline, ("px", "py", "pz"), common_start, common_end)
        proposed = align_to_ground_truth(proposed_t, proposed_raw, vi)
        imu_mask = ((imu["plot_t"] >= common_start) & (imu["plot_t"] <= common_end)
                    & np.isfinite(imu["plot_pos"]).all(axis=1))
        imu_t = imu["plot_t"][imu_mask]
        imu_position = align_to_ground_truth(imu_t, imu["plot_pos"][imu_mask], vi)

        gt_velocity_mask = ((vi.t_traj >= common_start)
                            & (vi.t_traj <= common_end)
                            & np.isfinite(vi.v_body).all(axis=1))
        gt_velocity_t = vi.t_traj[gt_velocity_mask]
        gt_velocity = vi.v_body[gt_velocity_mask]
        proposed_velocity_t, proposed_velocity = finite_position(
            baseline, ("vx", "vy", "vz"), common_start, common_end)
        imu_velocity_t, imu_velocity = finite_position(
            imu, ("vx", "vy", "vz"), common_start, common_end)

        gt_attitude_mask = ((vi.t_traj >= common_start)
                            & (vi.t_traj <= common_end)
                            & np.isfinite(vi.rpy).all(axis=1))
        gt_attitude_t = vi.t_traj[gt_attitude_mask]
        gt_attitude = np.degrees(vi.rpy[gt_attitude_mask])
        proposed_attitude_t, proposed_attitude = finite_position(
            baseline, ("roll", "pitch", "yaw"), common_start, common_end)
        proposed_attitude = np.degrees(proposed_attitude)
        imu_attitude_t, imu_attitude = finite_position(
            imu, ("roll", "pitch", "yaw"), common_start, common_end)
        imu_attitude = np.degrees(imu_attitude)

        stem = gait.lower()
        position_axis_scale = save_position_plot(
            gt_t, gt, imu_t, imu_position, proposed_t, proposed,
            f"fig_position_{stem}")
        save_velocity_plot(
            gt_velocity_t, gt_velocity,
            imu_velocity_t, imu_velocity,
            proposed_velocity_t, proposed_velocity,
            f"fig_velocity_{stem}")
        save_attitude_plot(
            gt_attitude_t, gt_attitude,
            imu_attitude_t, imu_attitude,
            proposed_attitude_t, proposed_attitude,
            f"fig_attitude_{stem}")
        selections[gait] = {
            "experiment_id": exp_id,
            "selection_rule": f"minimum Proposed Method 3D position RMSE among selected {gait} trials",
            "common_time_start_s": common_start,
            "common_time_end_s": common_end,
            "imu_integration_position_rmse_3d_m": selected["imu_only"]["position_rmse_3d_m"],
            "proposed_method_position_rmse_3d_m": selected["baseline"]["position_rmse_3d_m"],
            "imu_integration_velocity_rmse_3d_m_s": selected["imu_only"]["velocity_rmse_3d"],
            "proposed_method_velocity_rmse_3d_m_s": selected["baseline"]["velocity_rmse_3d"],
            "alignment": "each estimate translated to Ground Truth at its first common valid sample",
            "position_axis_scale": position_axis_scale,
            "presentation": "three-panel position, velocity, and attitude time histories with fixed method colors; angles in deg; position and velocity limits use Ground Truth plus Proposed Method only; py and pz share zero-centred limits; px span is an integer multiple of the common Y/Z span",
        }
    (FIGURE_DIR / "position_comparison_selection.json").write_text(
        json.dumps(selections, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    update_report(selections)
    print(json.dumps(selections, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
