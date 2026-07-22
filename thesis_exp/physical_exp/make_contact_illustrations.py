#!/usr/bin/env python3
"""Regenerate the publication contact illustrations for selected real trials."""
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from scipy.interpolate import interp1d

ROOT = Path(__file__).resolve().parent
EXPERIMENT_ROOT = ROOT / "experiments" / "FLAT_exp"
sys.path.insert(0, str(ROOT / "common"))
from corgi_analysis.bag_loader import load_fusion_bag
from corgi_analysis.vicon_loader import load_vicon
from thesis_figure_style import (  # noqa: E402
    CONTACT_COLORS, LINE_WIDTH, create_contact_figure, finish_contact_figure,
    format_contact_axis, save_figure,
)

OUTPUT_DIR = ROOT / "results" / "5.2_contact_state_experiment" / "figures"

TRIALS = {
    "FLAT_Walk_NEW_REAL_1": {
        "bag": "odom_fusion20260528_150739",
        "csv": "FLAT_WALK_NEW_REAL_1.csv", "height_mm": 15,
        "rm": (35., 25.), "beta": (4., .5),
        "stem": "fig_contact_walk_real",
    },
    "FLAT_WLW_NEW_REAL_4": {
        "bag": "odom_fusion20260528_161450_replay",
        "csv": "FLAT_WLW_NEW_REAL_4.csv", "height_mm": 20,
        "rm": (50., 25.), "beta": (3., 1.5),
        "stem": "fig_contact_wlw_real",
    },
}


def schmitt(rm, beta, rm_high, rm_low, beta_high, beta_low):
    state = False
    out = np.empty(len(rm), dtype=bool)
    for i, (force, torque) in enumerate(zip(np.abs(rm), np.abs(beta))):
        if not state and (force > rm_high or torque > beta_high):
            state = True
        elif state and force < rm_low and torque < beta_low:
            state = False
        out[i] = state
    return out


def make(exp_id):
    cfg = TRIALS[exp_id]
    exp = EXPERIMENT_ROOT / exp_id
    vi = load_vicon(exp / "vicon" / cfg["csv"],
                    contact_threshold_m=cfg["height_mm"] / 1000,
                    ground_markers=["ground1", "ground2", "ground3", "ground4"])
    bag_name = cfg["bag"]
    bag = load_fusion_bag(exp / "bags" / bag_name / f"{bag_name}_0.db3")
    raw = bag["gmo_raw"]

    # OFF-only bags use t=0 at trigger OFF; align them to VICON's trigger-ON
    # time base, as in the trial analysis code.
    t = raw["t"].copy()
    if bag["t_trigger_end"] is None and t[-1] < 1.0:
        t += vi.t_trigger_end
    valid = (t >= 0) & (t <= vi.t_trigger_end)
    t = t[valid]
    rm = np.abs(raw["LF_rm_force"][valid])
    beta = np.abs(raw["LF_beta_torque"][valid])
    rm_high, rm_low = cfg["rm"]
    beta_high, beta_low = cfg["beta"]

    foot_mm = interp1d(vi.t_traj, vi.foot_heights["LF"] * 1000,
                        bounds_error=False, fill_value=np.nan)(t)
    gt = foot_mm < cfg["height_mm"]
    detected = schmitt(rm, beta, rm_high, rm_low, beta_high, beta_low)
    correct = gt == detected
    fig, axes = create_contact_figure()
    rm_ylim = (0, max(40, np.nanpercentile(rm, 99.8) * 1.05))
    beta_ylim = (0, max(beta_high * 1.25, np.nanpercentile(beta, 99.8) * 1.05))
    height_ylim = (0, max(cfg["height_mm"] * 1.5, np.nanpercentile(foot_mm, 99.8) * 1.05))
    axes[0].plot(t, rm, color=CONTACT_COLORS["sigma_rm"], lw=LINE_WIDTH, zorder=2)
    axes[0].axhline(rm_high, color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH)
    axes[0].axhline(rm_low, color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH)
    format_contact_axis(axes[0], r"$\sigma_{R_m}$ [N]", rm_ylim)
    axes[1].plot(t, beta, color=CONTACT_COLORS["sigma_beta"], lw=LINE_WIDTH, zorder=2)
    axes[1].axhline(beta_high, color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH)
    axes[1].axhline(beta_low, color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH)
    format_contact_axis(axes[1], r"$\sigma_\beta$ [N m]", beta_ylim)
    axes[2].plot(t, foot_mm, color=CONTACT_COLORS["g_height"], lw=LINE_WIDTH, zorder=2)
    axes[2].axhline(cfg["height_mm"], color=CONTACT_COLORS["g_height"], ls="--", lw=LINE_WIDTH)
    format_contact_axis(axes[2], "G Point Height [mm]", height_ylim)

    for ax in axes[:3]:
        y0, y1 = ax.get_ylim()
        ax.fill_between(t, y0, y1, where=gt, step="post", color=CONTACT_COLORS["contact"], zorder=0)
        ax.fill_between(t, y0, y1, where=~gt, step="post", color=CONTACT_COLORS["swing"], zorder=0)
    rgb = np.where(correct[:, None], np.array([0, 158, 115]),
                   np.array([213, 94, 0])).astype(np.uint8)
    axes[3].imshow(rgb[None, :, :], aspect="auto", interpolation="nearest",
                   extent=[t[0], t[-1], 0, 1])
    axes[3].set_yticks([])
    axes[3].set_ylabel("LF")
    handles = [
        Line2D([], [], color=CONTACT_COLORS["sigma_rm"], lw=LINE_WIDTH, label=r"$\sigma_{R_m}$"),
        Line2D([], [], color=CONTACT_COLORS["sigma_beta"], lw=LINE_WIDTH, label=r"$\sigma_\beta$"),
        Line2D([], [], color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH, label="high threshold"),
        Line2D([], [], color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH, label="low threshold"),
        Line2D([], [], color=CONTACT_COLORS["g_height"], lw=LINE_WIDTH, label="G Point height"),
        Line2D([], [], color=CONTACT_COLORS["g_height"], ls="--", lw=LINE_WIDTH, label=f"height threshold ({cfg['height_mm']} mm)"),
        Patch(facecolor=CONTACT_COLORS["contact"], label="GT contact"),
        Patch(facecolor=CONTACT_COLORS["swing"], label="GT swing"),
        Patch(facecolor=CONTACT_COLORS["correct"], label="correct"),
        Patch(facecolor=CONTACT_COLORS["incorrect"], label="incorrect"),
    ]
    finish_contact_figure(fig, axes, handles)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUTPUT_DIR / cfg["stem"]
    save_figure(fig, out)
    print(out.with_suffix(".pdf"))
    print(out.with_suffix(".png"))


if __name__ == "__main__":
    for trial in TRIALS:
        make(trial)
