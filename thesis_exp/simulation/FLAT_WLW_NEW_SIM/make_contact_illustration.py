#!/usr/bin/env python3
"""Create a WLW-style contact illustration for the walk_openloop simulation."""
from pathlib import Path
import sqlite3
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
import numpy as np
from rclpy.serialization import deserialize_message
from corgi_msgs.msg import GMOContactStateStamped, SimLegContactStamped, TriggerStamped


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/simulation/FLAT_WLW_NEW_SIM")
DB = ROOT / "FLAT_WLW_NEW_SIM_0.db3"
STYLE_DIR = ROOT.parents[1] / "physical_exp" / "common"
OUTPUT_DIR = ROOT.parents[1] / "physical_exp" / "results" / "5.2_contact_state_experiment" / "figures"
sys.path.insert(0, str(STYLE_DIR))
from thesis_figure_style import (  # noqa: E402
    CONTACT_COLORS, LINE_WIDTH, create_contact_figure, finish_contact_figure,
    format_contact_axis, save_figure,
)
LEG = "LF"
MODULE = "module_a"


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def load_rows(cur, topic):
    return cur.execute(
        "SELECT data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name=?) ORDER BY timestamp",
        (topic,),
    ).fetchall()


def truth_at(t, gt_t, gt_c):
    return np.interp(t, gt_t, gt_c.astype(float)) >= 0.5


def schmitt_contact(rm, beta, rm_high, rm_low, beta_high, beta_low):
    """Match ContactSchmittTrigger: OR-activate, AND-deactivate."""
    state = False
    out = np.empty(len(rm), dtype=bool)
    for i, (r, b) in enumerate(zip(np.abs(rm), np.abs(beta))):
        if not state and (r > rm_high or b > beta_high):
            state = True
        elif state and r < rm_low and b < beta_low:
            state = False
        out[i] = state
    return out


def main():
    con = sqlite3.connect(DB)
    cur = con.cursor()
    triggers = [deserialize_message(raw, TriggerStamped) for (raw,) in load_rows(cur, "/trigger")]
    t0 = sec(next(m for m in triggers if m.enable).header.stamp)

    gt_rows = load_rows(cur, "/sim/leg_contact")
    gt_t, gt_c = [], []
    for (raw,) in gt_rows:
        m = deserialize_message(raw, SimLegContactStamped)
        gt_t.append(sec(m.header.stamp) - t0)
        gt_c.append(getattr(m, MODULE).contact)

    gmo_rows = load_rows(cur, "/gmo/contact_state")
    t, rm, beta = [], [], []
    for (raw,) in gmo_rows:
        m = deserialize_message(raw, GMOContactStateStamped)
        t.append(sec(m.header.stamp) - t0)
        mod = getattr(m, MODULE)
        rm.append(mod.rm_force)
        beta.append(mod.beta_torque)
    con.close()

    t = np.asarray(t); rm = np.asarray(rm); beta = np.asarray(beta)
    gt_t = np.asarray(gt_t); gt_c = np.asarray(gt_c, dtype=bool)
    keep = (t >= 0) & (t <= t[-1])
    t, rm, beta = t[keep], rm[keep], beta[keep]

    # Active simulator settings in config_online.yaml; the recorded Boolean
    # state is intentionally not reused, so this evaluates the new thresholds.
    rm_high, rm_low = 25.0, 15.0
    beta_high, beta_low = 1.5, 1.0
    gmo = schmitt_contact(rm, beta, rm_high, rm_low, beta_high, beta_low)
    gt = truth_at(t, gt_t, gt_c)
    correct = gt == gmo
    tp, fp, fn = np.sum(gt & gmo), np.sum(~gt & gmo), np.sum(gt & ~gmo)
    precision = tp / (tp + fp) if tp + fp else float("nan")
    recall = tp / (tp + fn) if tp + fn else float("nan")
    f1 = 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else float("nan")
    print(f"{LEG}: precision={precision:.4f}, recall={recall:.4f}, F1={f1:.4f}")
    fig, axes = create_contact_figure()
    rm_abs, beta_abs = np.abs(rm), np.abs(beta)
    axes[0].plot(t, rm_abs, color=CONTACT_COLORS["sigma_rm"], lw=LINE_WIDTH, zorder=2)
    axes[0].axhline(rm_high, color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH)
    axes[0].axhline(rm_low, color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH)
    format_contact_axis(axes[0], r"$\sigma_{R_m}$ [N]", (0, max(40, np.percentile(rm_abs, 99.8) * 1.05)))
    axes[1].plot(t, beta_abs, color=CONTACT_COLORS["sigma_beta"], lw=LINE_WIDTH, zorder=2)
    axes[1].axhline(beta_high, color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH)
    axes[1].axhline(beta_low, color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH)
    format_contact_axis(axes[1], r"$\sigma_\beta$ [N m]", (0, 6.0))
    axes[2].set_yticks([])
    format_contact_axis(axes[2], "GT", (-.15, 1.15))

    # Add GT contact/swing bands after each axis' data limits are fixed, so the
    # background occupies the whole subplot without affecting autoscaling.
    for ax in axes[:3]:
        y0, y1 = ax.get_ylim()
        ax.fill_between(t, y0, y1, where=gt, step="post", color=CONTACT_COLORS["contact"], zorder=0)
        ax.fill_between(t, y0, y1, where=~gt, step="post", color=CONTACT_COLORS["swing"], zorder=0)

    rgb = np.where(correct[:, None], np.array([0, 158, 115]), np.array([213, 94, 0])).astype(np.uint8)
    axes[3].imshow(rgb[None, :, :], aspect="auto", interpolation="nearest", extent=[t[0], t[-1], 0, 1])
    axes[3].set_yticks([]); axes[3].set_ylabel(LEG)

    handles = [
        Line2D([], [], color=CONTACT_COLORS["sigma_rm"], lw=LINE_WIDTH, label=r"$\sigma_{R_m}$"),
        Line2D([], [], color=CONTACT_COLORS["sigma_beta"], lw=LINE_WIDTH, label=r"$\sigma_\beta$"),
        Line2D([], [], color=CONTACT_COLORS["threshold"], ls="--", lw=LINE_WIDTH, label="high threshold"),
        Line2D([], [], color=CONTACT_COLORS["threshold"], ls=":", lw=LINE_WIDTH, label="low threshold"),
        Patch(facecolor=CONTACT_COLORS["contact"], label="GT contact"),
        Patch(facecolor=CONTACT_COLORS["swing"], label="GT swing"),
        Patch(facecolor=CONTACT_COLORS["correct"], label="correct"),
        Patch(facecolor=CONTACT_COLORS["incorrect"], label="incorrect"),
    ]
    finish_contact_figure(fig, axes, handles)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUTPUT_DIR / "fig_contact_wlw_sim"
    save_figure(fig, out)
    print(out.with_suffix(".pdf"))
    print(out.with_suffix(".png"))


if __name__ == "__main__":
    main()
