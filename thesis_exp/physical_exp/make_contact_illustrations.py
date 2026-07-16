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
sys.path.insert(0, str(ROOT / "common"))
from corgi_analysis.bag_loader import load_fusion_bag
from corgi_analysis.vicon_loader import load_vicon

plt.rcParams.update({"font.size": 10, "axes.labelsize": 10,
                     "xtick.labelsize": 9, "ytick.labelsize": 9})

TRIALS = {
    "FLAT_Walk_NEW_REAL_1": {
        "bag": "odom_fusion20260528_150739",
        "csv": "FLAT_WALK_NEW_REAL_1.csv", "height_mm": 15,
        "rm": (30., 20.), "beta": (3.5, .5),
        "out": "FLAT_WALK_NEW_REAL_1.pdf",
    },
    "FLAT_WLW_NEW_REAL_4": {
        "bag": "odom_fusion20260528_161450_replay",
        "csv": "FLAT_WLW_NEW_REAL_4.csv", "height_mm": 20,
        "rm": (50., 25.), "beta": (3., 1.5),
        "out": "FLAT_WLW_NEW_REAL_4.pdf",
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
    exp = ROOT / "experiments" / exp_id
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
    contact_color, swing_color = "#dbe6f5", "#f7ead7"
    good_color, bad_color = "#4daf4a", "#e52b25"

    fig, axes = plt.subplots(4, 1, figsize=(14, 8), sharex=True,
                             gridspec_kw={"height_ratios": [2.2, 2.2, 1.45, .3]})
    axes[0].plot(t, rm, color="#1f77b4", lw=.75, zorder=2)
    axes[0].axhline(rm_high, color="#333", ls="--", lw=1.)
    axes[0].axhline(rm_low, color="#333", ls=":", lw=1.3)
    axes[0].set_ylabel(r"$\sigma_{R_m}\;[\mathrm{N}]$")
    axes[0].set_ylim(0, max(40, np.nanpercentile(rm, 99.8) * 1.05))
    axes[1].plot(t, beta, color="#222", lw=.75, zorder=2)
    axes[1].axhline(beta_high, color="#333", ls="--", lw=1.)
    axes[1].axhline(beta_low, color="#333", ls=":", lw=1.3)
    axes[1].set_ylabel(r"$\sigma_\beta\;[\mathrm{N,m}]$")
    axes[1].set_ylim(0, max(beta_high * 1.25, np.nanpercentile(beta, 99.8) * 1.05))
    axes[2].plot(t, foot_mm, color="#6f3dc4", lw=.9, zorder=2)
    axes[2].axhline(cfg["height_mm"], color="#6f3dc4", ls="--", lw=1.)
    axes[2].set_ylabel("G point height [mm]")
    axes[2].set_ylim(0, max(cfg["height_mm"] * 1.5, np.nanpercentile(foot_mm, 99.8) * 1.05))

    for ax in axes[:3]:
        y0, y1 = ax.get_ylim()
        ax.fill_between(t, y0, y1, where=gt, step="post", color=contact_color, zorder=0)
        ax.fill_between(t, y0, y1, where=~gt, step="post", color=swing_color, zorder=0)
        ax.grid(True, alpha=.32, zorder=1)
    rgb = np.where(correct[:, None], np.array([77, 175, 74]),
                   np.array([229, 43, 37])).astype(np.uint8)
    axes[3].imshow(rgb[None, :, :], aspect="auto", interpolation="nearest",
                   extent=[t[0], t[-1], 0, 1])
    axes[3].set_yticks([]); axes[3].set_ylabel("LF"); axes[3].set_xlabel("Time [s]")
    handles = [
        Line2D([], [], color="#333", ls="--", label="high threshold"),
        Line2D([], [], color="#333", ls=":", label="low threshold"),
        Line2D([], [], color="#1f77b4", label=r"$\sigma_{R_m}\;[\mathrm{N}]$"),
        Line2D([], [], color="#222", label=r"$\sigma_\beta\;[\mathrm{N,m}]$"),
        Line2D([], [], color="#6f3dc4", label="G point height"),
        Line2D([], [], color="#6f3dc4", ls="--", label=f"height threshold ({cfg['height_mm']} mm)"),
        Patch(facecolor=contact_color, label="contact"), Patch(facecolor=swing_color, label="swing / no contact"),
        Patch(facecolor=good_color, label="correct"), Patch(facecolor=bad_color, label="incorrect"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=5, frameon=False,
               fontsize=9, bbox_to_anchor=(.5, 1.01))
    fig.subplots_adjust(top=.86, hspace=.16, left=.09, right=.99, bottom=.08)
    out = exp / "results" / exp_id / cfg["out"]
    fig.savefig(out, bbox_inches="tight")
    print(out)


if __name__ == "__main__":
    for trial in TRIALS:
        make(trial)
