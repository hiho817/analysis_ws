#!/usr/bin/env python3
"""Generate VICON-vs-GMO binary contact-state plots for selected experiments."""

import argparse
import importlib.util
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial import Delaunay


DEFAULT_EXPERIMENT_DIR = "/home/hiho817/analysis_ws/experiments/20260528"
SELECTED_EXPERIMENTS = {
    "FLAT_Walk_NEW_REAL_1",
    "FLAT_Walk_NEW_REAL_3",
    "FLAT_Walk_NEW_REAL_4",
    "FLAT_Walk_NEW_REAL_5",
    "FLAT_Walk_NEW_REAL_6",
    "FLAT_MPC_NEW_REAL_1",
    "FLAT_MPC_NEW_REAL_2",
    "FLAT_MPC_NEW_REAL_3",
    "FLAT_MPC_NEW_REAL_4",
    "FLAT_MPC_NEW_REAL_5",
}
LEGS = (("LF", "G1"), ("RF", "G2"), ("RH", "G3"), ("LH", "G4"))


def load_analysis_module(experiment_dir):
    path = os.path.join(experiment_dir, "analyze.py")
    spec = importlib.util.spec_from_file_location("analysis_20260528", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def interpolate_gmo(gmo_t, gmo_state, target_t, end_time):
    keep = (gmo_t >= -0.5) & (gmo_t <= end_time + 0.5)
    source_t = gmo_t[keep]
    source_state = gmo_state[keep].astype(float)
    if len(source_t) < 2:
        return np.zeros(len(target_t), dtype=bool)
    return (
        interp1d(
            source_t,
            source_state,
            kind="nearest",
            bounds_error=False,
            fill_value=0.0,
        )(target_t)
        > 0.5
    )


def shade_classification(ax, t, correct):
    """Shade contiguous valid intervals without bridging timestamp gaps."""
    if len(t) < 2:
        return

    dt = np.diff(t)
    gap_limit = max(0.01, 3.0 * float(np.nanmedian(dt)))
    segment_start = 0

    for i in range(1, len(t)):
        changed = bool(correct[i]) != bool(correct[i - 1])
        gap = (t[i] - t[i - 1]) > gap_limit
        if changed or gap:
            if not gap:
                ax.axvspan(
                    t[segment_start],
                    t[i],
                    color="#78c679" if correct[i - 1] else "#ef8a8a",
                    alpha=0.32,
                    linewidth=0,
                )
            elif i - segment_start > 1:
                ax.axvspan(
                    t[segment_start],
                    t[i - 1],
                    color="#78c679" if correct[i - 1] else "#ef8a8a",
                    alpha=0.32,
                    linewidth=0,
                )
            segment_start = i

    ax.axvspan(
        t[segment_start],
        t[-1],
        color="#78c679" if correct[-1] else "#ef8a8a",
        alpha=0.32,
        linewidth=0,
    )


def prepare_contact_data(module, experiment_dir, experiment):
    exp_id, _, bag_name, vicon_csv, trigger_pair, *_ = experiment
    bag_db = os.path.join(
        experiment_dir, "bags", bag_name, f"{bag_name}_0.db3"
    )
    csv_path = os.path.join(experiment_dir, "vicon", vicon_csv)

    vi = module.load_vicon(
        csv_path,
        contact_threshold_m=module.CONTACT_THRESHOLD_M,
        ground_markers=module.GROUND_MARKERS,
    )
    bag = module.load_fusion_bag(bag_db, rate=1.0, trigger_pair=trigger_pair)
    gmo = bag["gmo"]
    ekf = bag["ekf"]

    bag_end = bag["t_trigger_end"]
    vicon_end = vi.t_trigger_end
    if bag_end is None and len(ekf["t"]) > 0 and ekf["t"][-1] < 1.0:
        offset = float(vicon_end) if vicon_end is not None else 0.0
        if len(gmo["t"]) > 0:
            gmo["t"] = gmo["t"] + offset
    if bag_end is None and vicon_end is None:
        bag_end = float(ekf["t"][-1]) if len(ekf["t"]) else 30.0
    end_time = min(x for x in (vicon_end, bag_end) if x is not None)

    ground_xy = []
    for marker in module.GROUND_MARKERS:
        try:
            xyz = vi.get_xyz(marker)
            valid = ~np.isnan(xyz).any(axis=1)
            if valid.any():
                ground_xy.append(vi.to_robot(xyz[valid][0:1])[0, :2])
        except Exception:
            pass
    try:
        ground_hull = Delaunay(np.asarray(ground_xy)) if len(ground_xy) >= 3 else None
    except Exception:
        ground_hull = None

    result = {}
    analysis_window = (vi.t_traj >= 0.0) & (vi.t_traj <= end_time)
    for leg, marker in LEGS:
        foot_height = vi.foot_heights[leg]
        try:
            foot_xyz = vi.get_xyz(marker)
            foot_xy = np.full((len(vi.t_traj), 2), np.nan)
            valid_xyz = ~np.isnan(foot_xyz).any(axis=1)
            region_mask = np.zeros(len(vi.t_traj), dtype=bool)
            if valid_xyz.any():
                foot_xy[valid_xyz] = vi.to_robot(foot_xyz[valid_xyz])[:, :2]
                if ground_hull is None:
                    region_mask[valid_xyz] = True
                else:
                    region_mask[valid_xyz] = (
                        ground_hull.find_simplex(foot_xy[valid_xyz]) >= 0
                    )
        except Exception:
            region_mask = np.zeros(len(vi.t_traj), dtype=bool)

        overlap = (
            (vi.t_traj >= gmo["t"][0])
            & (vi.t_traj <= gmo["t"][-1])
            if len(gmo["t"])
            else np.zeros(len(vi.t_traj), dtype=bool)
        )
        valid = analysis_window & region_mask & np.isfinite(foot_height) & overlap
        t = vi.t_traj[valid]
        vicon = foot_height[valid] < module.CONTACT_THRESHOLD_M
        gmo_state = interpolate_gmo(gmo["t"], gmo[leg], t, end_time)
        result[leg] = (t, vicon, gmo_state)

    return exp_id, end_time, result


def plot_experiment(exp_id, end_time, contact, output_path):
    fig, axes = plt.subplots(4, 1, figsize=(16, 9), sharex=True)
    for ax, (leg, _) in zip(axes, LEGS):
        t, vicon, gmo = contact[leg]
        if len(t):
            shade_classification(ax, t, vicon == gmo)
            ax.step(
                t,
                vicon.astype(int),
                where="post",
                color="#1565c0",
                linewidth=1.25,
                label="VICON",
                zorder=3,
            )
            ax.step(
                t,
                gmo.astype(int),
                where="post",
                color="#ef6c00",
                linewidth=1.0,
                linestyle="--",
                label="GMO",
                zorder=4,
            )
        ax.set_ylabel(leg, rotation=0, labelpad=22, fontsize=11)
        ax.set_yticks([0, 1], labels=["0", "1"])
        ax.set_ylim(-0.12, 1.12)
        ax.grid(axis="x", alpha=0.25)

    axes[0].legend(loc="upper right", ncols=2)
    axes[-1].set_xlabel("Time [s]")
    axes[-1].set_xlim(0.0, end_time)
    fig.legend(
        handles=[
            Patch(facecolor="#78c679", alpha=0.32, label="Correct"),
            Patch(facecolor="#ef8a8a", alpha=0.32, label="Incorrect"),
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.965),
        ncols=2,
        frameon=False,
    )
    fig.suptitle(f"Contact State: VICON vs GMO — {exp_id}", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--experiment-dir", default=DEFAULT_EXPERIMENT_DIR)
    parser.add_argument(
        "--output-root",
        help="Defaults to <experiment-dir>/results.",
    )
    args = parser.parse_args()

    experiment_dir = os.path.abspath(args.experiment_dir)
    output_root = os.path.abspath(
        args.output_root or os.path.join(experiment_dir, "results")
    )
    module = load_analysis_module(experiment_dir)
    experiments = [
        exp for exp in module.EXPERIMENTS if exp[0] in SELECTED_EXPERIMENTS
    ]
    if len(experiments) != len(SELECTED_EXPERIMENTS):
        found = {exp[0] for exp in experiments}
        raise RuntimeError(f"Missing experiments: {sorted(SELECTED_EXPERIMENTS - found)}")

    for experiment in experiments:
        exp_id, end_time, contact = prepare_contact_data(
            module, experiment_dir, experiment
        )
        output_dir = os.path.join(output_root, exp_id)
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "fig_contact_state_comparison.png")
        plot_experiment(exp_id, end_time, contact, output_path)
        print(output_path)


if __name__ == "__main__":
    main()
