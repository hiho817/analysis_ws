#!/usr/bin/env python3
"""Create a WLW-style contact illustration for the walk_openloop simulation."""
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
import numpy as np
from rclpy.serialization import deserialize_message
from corgi_msgs.msg import GMOContactStateStamped, SimLegContactStamped, TriggerStamped


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/simulation/walk_openloop")
DB = ROOT / "walk_openloop.db3"
OUT = ROOT / "results" / "fig_sim_contact_illustration.pdf"
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
    beta_high, beta_low = 2.0, 1.0
    gmo = schmitt_contact(rm, beta, rm_high, rm_low, beta_high, beta_low)
    gt = truth_at(t, gt_t, gt_c)
    correct = gt == gmo
    tp, fp, fn = np.sum(gt & gmo), np.sum(~gt & gmo), np.sum(gt & ~gmo)
    precision = tp / (tp + fp) if tp + fp else float("nan")
    recall = tp / (tp + fn) if tp + fn else float("nan")
    f1 = 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else float("nan")
    print(f"{LEG}: precision={precision:.4f}, recall={recall:.4f}, F1={f1:.4f}")
    contact_color, swing_color = "#dbe6f5", "#f7ead7"
    good_color, bad_color = "#4daf4a", "#e52b25"

    fig, axes = plt.subplots(4, 1, figsize=(14, 8), sharex=True,
                             gridspec_kw={"height_ratios": [2.2, 2.2, 1.45, .3]})

    axes[0].plot(t, np.abs(rm), color="#1f77b4", lw=.75, zorder=2)
    axes[0].axhline(rm_high, color="#333333", ls="--", lw=1.0)
    axes[0].axhline(rm_low, color="#333333", ls=":", lw=1.3)
    axes[0].set_ylabel(r"$|F_{Rm}|$")
    axes[0].set_ylim(0, max(40, np.percentile(np.abs(rm), 99.8) * 1.05))

    axes[1].plot(t, np.abs(beta), color="#222222", lw=.75, zorder=2)
    axes[1].axhline(beta_high, color="#333333", ls="--", lw=1.0)
    axes[1].axhline(beta_low, color="#333333", ls=":", lw=1.3)
    axes[1].set_ylabel(r"$|\tau_\beta|$")
    axes[1].set_ylim(0, 6.0)

    axes[2].set_yticks([])
    axes[2].set_ylabel("Ground Truth")
    axes[2].set_ylim(-.15, 1.15)

    # Add GT contact/swing bands after each axis' data limits are fixed, so the
    # background occupies the whole subplot without affecting autoscaling.
    for ax in axes[:3]:
        y0, y1 = ax.get_ylim()
        ax.fill_between(t, y0, y1, where=gt, step="post", color=contact_color, zorder=0)
        ax.fill_between(t, y0, y1, where=~gt, step="post", color=swing_color, zorder=0)
        ax.grid(True, alpha=.32, zorder=1)

    rgb = np.where(correct[:, None], np.array([77, 175, 74]), np.array([229, 43, 37])).astype(np.uint8)
    axes[3].imshow(rgb[None, :, :], aspect="auto", interpolation="nearest", extent=[t[0], t[-1], 0, 1])
    axes[3].set_yticks([]); axes[3].set_ylabel(LEG)
    axes[3].set_xlabel("Time [s]")

    handles = [
        Line2D([], [], color="#333", ls="--", label="high threshold"),
        Line2D([], [], color="#333", ls=":", label="low threshold"),
        Line2D([], [], color="#1f77b4", label=r"$|F_{Rm}|$"),
        Line2D([], [], color="#222", label=r"$|\tau_\beta|$"),
        Patch(facecolor=contact_color, label="GT contact"), Patch(facecolor=swing_color, label="GT swing"),
        Patch(facecolor=good_color, label="correct"), Patch(facecolor=bad_color, label="incorrect"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(.5, 1.01))
    fig.subplots_adjust(top=.86, hspace=.16, left=.09, right=.99, bottom=.08)
    fig.savefig(OUT, bbox_inches="tight")
    print(OUT)


if __name__ == "__main__":
    main()
