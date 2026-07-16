#!/usr/bin/env python3
"""Replay NEW flat-ground bags while excluding all previously computed topics."""
from __future__ import annotations

import argparse
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path


ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"
DATA_ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp/experiments")
OUT_ROOT = Path(__file__).resolve().parents[1] / "results" / "5.3_flat_experiment" / "replayed_bags_isolated"
PLAY_TOPICS = "/imu_raw /motor/state /trigger /lidar_odom"
RECORD_TOPICS = (
    "/ekf /ekf/orientation /ekf/ba /ekf/bw /gmo/contact_state "
    "/odom_mapping /fusion/bv /trigger /lidar_odom"
)


def wrapped(command: str) -> list[str]:
    return ["bash", "-c", f"export ROS_LOG_DIR=/tmp/codex_ros_logs && "
            f"export ROS_DOMAIN_ID=77 && "
            f"source {ROS_SETUP} && source {WS_SETUP} && {command}"]


def start(command: str, log: Path) -> subprocess.Popen:
    handle = log.open("w")
    process = subprocess.Popen(wrapped(command), stdout=handle, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
    process._log_handle = handle  # type: ignore[attr-defined]
    return process


def stop(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
    process._log_handle.close()  # type: ignore[attr-defined]


def topic_counts(output: Path) -> dict[str, int]:
    db = next(output.glob("*.db3"))
    conn = sqlite3.connect(db)
    cur = conn.cursor()
    cur.execute("SELECT id, name FROM topics")
    result = {}
    for topic_id, name in cur.fetchall():
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,))
        result[name] = cur.fetchone()[0]
    conn.close()
    return result


def replay(exp_id: str, rate: float) -> None:
    exp_dir = DATA_ROOT / exp_id
    # Select the original complete source bag; concurrent jobs may add smaller
    # IMU-only replay directories under the same experiment.
    input_db = max((exp_dir / "bags").glob("*/*.db3"), key=lambda path: path.stat().st_size)
    input_bag = input_db.parent
    output = OUT_ROOT / exp_id
    for candidate in sorted(OUT_ROOT.glob(f"{exp_id}*")) if OUT_ROOT.exists() else []:
        if not candidate.is_dir() or not (candidate / "metadata.yaml").exists():
            continue
        counts = topic_counts(candidate)
        if (counts.get("/ekf", 0) >= 10_000
                and counts.get("/gmo/contact_state", 0) >= 10_000
                and counts.get("/trigger", 0) >= 2):
            print(f"[skip] {exp_id}: valid output already exists at {candidate.name}")
            return
    retry = 1
    while output.exists():
        output = OUT_ROOT / f"{exp_id}_retry{retry}"
        retry += 1
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    launch = recorder = None
    try:
        print(f"[start] {exp_id}: {input_bag}")
        launch = start("ros2 launch corgi_odometry odom_fusion_replay.launch.py",
                       OUT_ROOT / f"{exp_id}_nodes.log")
        time.sleep(3)
        recorder = start(f"ros2 bag record -o {output} {RECORD_TOPICS}",
                         OUT_ROOT / f"{exp_id}_record.log")
        time.sleep(2)
        played = subprocess.run(wrapped(
            f"ros2 bag play {input_bag} --clock --rate {rate:g} --topics {PLAY_TOPICS}"),
            timeout=300)
        if played.returncode != 0:
            raise RuntimeError(f"ros2 bag play returned {played.returncode}")
        time.sleep(3)
    finally:
        stop(recorder)
        stop(launch)
        time.sleep(1)
    counts = topic_counts(output)
    print(f"[done] {exp_id}: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    if counts.get("/ekf", 0) < 10_000 or counts.get("/gmo/contact_state", 0) < 10_000:
        raise RuntimeError(f"{exp_id}: insufficient output messages: {counts}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("gait", choices=("walk", "wlw"))
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--exp")
    args = parser.parse_args()
    pattern = "FLAT_Walk_NEW_REAL_*" if args.gait == "walk" else "FLAT_WLW_NEW_REAL_*"
    exp_ids = [p.name for p in sorted(DATA_ROOT.glob(pattern))]
    # REAL_2 is invalid; REAL_1 contains trigger-OFF only and cannot initialize
    # a fresh estimator replay. Its previously recorded NEW metrics remain
    # available for the aggregate comparison, but it is not replayable.
    exp_ids = [e for e in exp_ids if e not in {"FLAT_Walk_NEW_REAL_1", "FLAT_Walk_NEW_REAL_2"}]
    if args.exp:
        exp_ids = [e for e in exp_ids if e == args.exp]
    for exp_id in exp_ids:
        replay(exp_id, args.rate)


if __name__ == "__main__":
    main()
