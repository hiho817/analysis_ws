#!/usr/bin/env python3
"""Compute VICON position/velocity metrics for newly replayed NEW bags."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d


DATA_ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp/experiments")
COMMON = DATA_ROOT.parent / "common"
sys.path.insert(0, str(COMMON))
from corgi_analysis.bag_loader import load_fusion_bag  # noqa: E402
from corgi_analysis.vicon_loader import load_vicon  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
REPLAY_ROOT = ROOT / "results" / "5.3_flat_experiment" / "replayed_bags_isolated"
OUT = ROOT / "results" / "5.3_flat_experiment" / "replayed_new_metrics.json"
FLIP = {"FLAT_WLW_NEW_REAL_2", "FLAT_WLW_NEW_REAL_4", "FLAT_WLW_NEW_REAL_5"}
TRIGGER_PAIR = {"FLAT_Walk_NEW_REAL_3": 1}


def rmse(values) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values ** 2))) if len(values) else float("nan")


def valid_candidate(exp_id: str) -> Path | None:
    candidates = []
    for directory in REPLAY_ROOT.glob(f"{exp_id}*"):
        dbs = list(directory.glob("*.db3")) if directory.is_dir() else []
        if not dbs or not (directory / "metadata.yaml").exists():
            continue
        import sqlite3
        conn = sqlite3.connect(dbs[0])
        cur = conn.cursor()
        cur.execute("SELECT COUNT(*) FROM messages JOIN topics ON topics.id=messages.topic_id WHERE name='/ekf'")
        count = cur.fetchone()[0]
        cur.execute("SELECT COUNT(*) FROM messages JOIN topics ON topics.id=messages.topic_id WHERE name='/trigger'")
        triggers = cur.fetchone()[0]
        conn.close()
        if count >= 10_000 and triggers >= 2:
            candidates.append((count, directory))
    return max(candidates)[1] if candidates else None


def analyze(exp_id: str, replay_dir: Path) -> dict:
    gait = "WLW" if "_WLW_" in exp_id else "WALK"
    exp_dir = DATA_ROOT / exp_id
    csv = next((exp_dir / "vicon").glob("*.csv"))
    vi = load_vicon(str(csv), contact_threshold_m=0.020 if gait == "WLW" else 0.015,
                    ground_markers=["ground1", "ground2", "ground3", "ground4"])
    db = next(replay_dir.glob("*.db3"))
    bag = load_fusion_bag(str(db), rate=2.0,
                          trigger_pair=TRIGGER_PAIR.get(exp_id, 0))
    ekf = bag["ekf"]
    gmo = bag["gmo"]
    t_end = min(x for x in (vi.t_trigger_end, bag["t_trigger_end"]) if x is not None)
    mask_vi = (vi.t_traj >= 0) & (vi.t_traj <= t_end)
    t_vi = vi.t_traj[mask_vi]
    pos_vi = vi.pos_m[mask_vi]
    vel_vi = vi.v_body[mask_vi]
    valid_vi = np.isfinite(pos_vi).all(axis=1)
    valid_vel = np.isfinite(vel_vi).all(axis=1)
    mask_e = (ekf["t"] >= 0) & (ekf["t"] <= t_end)
    t = ekf["t"][mask_e]
    pos = np.column_stack([ekf[k][mask_e] for k in ("px", "py", "pz")])
    vel = np.column_stack([ekf[k][mask_e] for k in ("vx", "vy", "vz")])
    if exp_id in FLIP:
        pos[:, :2] *= -1
        vel[:, :2] *= -1
    pos_ref = np.column_stack([
        interp1d(t_vi[valid_vi], pos_vi[valid_vi, axis], bounds_error=False,
                 fill_value=np.nan)(t) for axis in range(3)])
    vel_ref = np.column_stack([
        interp1d(t_vi[valid_vel], vel_vi[valid_vel, axis], bounds_error=False,
                 fill_value=np.nan)(t) for axis in range(3)])
    z_offset = pos[0, 2] - pos_ref[0, 2] if np.isfinite(pos_ref[0, 2]) else 0.0
    pos_error = pos - pos_ref
    pos_error[:, 2] -= z_offset
    pvalid = np.isfinite(pos_error).all(axis=1)
    vwindow = (t >= t_end * 0.35) & (t <= t_end * 0.75)
    vel_error = vel - vel_ref
    vvalid = vwindow & np.isfinite(vel_error).all(axis=1)
    position = {
        "RMSE_X_cm": rmse(pos_error[pvalid, 0]) * 100,
        "RMSE_Y_cm": rmse(pos_error[pvalid, 1]) * 100,
        "RMSE_Z_cm": rmse(pos_error[pvalid, 2]) * 100,
        "RMSE_3D_cm": rmse(np.linalg.norm(pos_error[pvalid], axis=1)) * 100,
    }
    velocity = {
        "RMSE_vx": rmse(vel_error[vvalid, 0]),
        "RMSE_vy": rmse(vel_error[vvalid, 1]),
        "RMSE_vz": rmse(vel_error[vvalid, 2]),
        "RMSE_3D": rmse(np.linalg.norm(vel_error[vvalid], axis=1)),
        "window_start": t_end * 0.35,
        "window_end": t_end * 0.75,
    }
    contact = {}
    total = {key: 0 for key in ("N", "TP", "TN", "FP", "FN")}
    for leg in ("LF", "RF", "RH", "LH"):
        height = vi.foot_heights[leg]
        cmask = ((vi.t_traj >= 0) & (vi.t_traj <= t_end) & np.isfinite(height)
                 & (vi.t_traj >= gmo["t"][0]) & (vi.t_traj <= gmo["t"][-1]))
        ct = vi.t_traj[cmask]
        truth = height[cmask] < (0.020 if gait == "WLW" else 0.015)
        pred = interp1d(gmo["t"], gmo[leg].astype(float), kind="nearest",
                        bounds_error=False, fill_value=0.0)(ct) > 0.5
        values = {"N": int(len(ct)), "TP": int(np.sum(truth & pred)),
                  "TN": int(np.sum(~truth & ~pred)), "FP": int(np.sum(~truth & pred)),
                  "FN": int(np.sum(truth & ~pred))}
        values["accuracy"] = (values["TP"] + values["TN"]) / values["N"]
        values["fp_rate"] = values["FP"] / max(1, values["FP"] + values["TN"])
        values["fn_rate"] = values["FN"] / max(1, values["FN"] + values["TP"])
        contact[leg] = values
        for key in total:
            total[key] += values[key]
    total["accuracy"] = (total["TP"] + total["TN"]) / total["N"]
    total["fp_rate"] = total["FP"] / max(1, total["FP"] + total["TN"])
    total["fn_rate"] = total["FN"] / max(1, total["FN"] + total["TP"])
    return {"exp_id": exp_id, "group": f"NEW_{gait}", "source": str(replay_dir),
            "T_END": t_end, "position": position, "velocity": velocity,
            "contact": contact, "contact_total": total}


def main() -> None:
    results = []
    for pattern in ("FLAT_Walk_NEW_REAL_*", "FLAT_WLW_NEW_REAL_*"):
        for exp_dir in sorted(DATA_ROOT.glob(pattern)):
            exp_id = exp_dir.name
            if exp_id == "FLAT_Walk_NEW_REAL_2":
                continue
            candidate = valid_candidate(exp_id)
            if candidate is None:
                print(f"[unavailable] {exp_id}")
                continue
            result = analyze(exp_id, candidate)
            results.append(result)
            print(f"[analyzed] {exp_id}: pos={result['position']['RMSE_3D_cm']:.2f} cm, "
                  f"vel={result['velocity']['RMSE_3D']:.3f} m/s")
    OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
