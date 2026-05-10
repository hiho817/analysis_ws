#!/usr/bin/env python3
"""
Replay corrected input bag through corgi_leg_odom node and record output.
Scenario A: leg_odom only (no fusion node, no LiDAR).
"""

import subprocess
import time
import os
import signal
from datetime import datetime

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"

INPUT_BAG  = "/home/hiho817/analysis_ws/experiments/20260507/walk_2m_01/bags/corrected_input"
OUTPUT_DIR = "/home/hiho817/analysis_ws/experiments/20260507/walk_2m_01/bags"

RECORD_TOPICS = [
    "/ekf",
    "/ekf/orientation",
    "/ekf/ba",
    "/ekf/bw",
    "/gmo/contact_state",
    "/trigger",
]

def src_cmd(cmd):
    return f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"

def start(cmd, name=""):
    print(f"[start] {name}")
    p = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )
    return p

def kill_proc(p, name=""):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            p.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
    print(f"[done] {name} stopped")

def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_bag = os.path.join(OUTPUT_DIR, f"leg_odom_vel_corrected_{ts}")
    print(f"Output bag: {output_bag}")

    procs = {}

    # ── Check no stale leg_odom nodes ───────────────────────────────────────
    check = subprocess.run(
        ["bash", "-c", "ps aux | grep corgi_leg_odom | grep -v grep"],
        capture_output=True, text=True
    )
    if check.stdout.strip():
        print("[WARN] Stale corgi_leg_odom detected — killing...")
        subprocess.run(["bash", "-c",
            "ps aux | grep corgi_leg_odom | grep -v grep | awk '{print $2}' | xargs -r kill -9"])
        time.sleep(2)

    try:
        # 1. Start corgi_leg_odom
        procs['leg_odom'] = start(
            src_cmd("ros2 run corgi_odometry corgi_leg_odom "
                    "--ros-args -p use_sim_time:=true "
                    "-r /imu:=/imu_raw"),
            "leg_odom"
        )
        time.sleep(3)

        # 2. Start recorder
        record_cmd = src_cmd(
            "ros2 bag record " +
            " ".join(RECORD_TOPICS) +
            f" -o {output_bag} > /tmp/recorder_0507.log 2>&1"
        )
        procs['recorder'] = start(record_cmd, "recorder")
        time.sleep(3)

        # 3. Replay input bag (--clock for sim time, only input topics)
        print("[info] Starting bag replay...")
        play_cmd = src_cmd(
            f"ros2 bag play {INPUT_BAG} --clock --rate 2.0 "
            f"--topics /motor/state /imu_raw /trigger"
        )
        result = subprocess.run(["bash", "-c", play_cmd])
        print(f"[info] Replay finished (exit code {result.returncode})")

        time.sleep(5)  # let nodes flush remaining messages

    finally:
        for name, p in procs.items():
            kill_proc(p, name)

    # Verify output bag
    import sqlite3
    db = f"{output_bag}/{os.path.basename(output_bag)}_0.db3"
    if not os.path.exists(db):
        # try glob
        import glob
        dbs = glob.glob(f"{output_bag}/*.db3")
        db = dbs[0] if dbs else None

    if db and os.path.exists(db):
        conn = sqlite3.connect(db)
        topics = conn.execute("SELECT name FROM topics").fetchall()
        print("\n[verify] Topics in output bag:")
        for t in topics:
            name = t[0]
            cnt = conn.execute(
                f"SELECT COUNT(*) FROM messages WHERE topic_id="
                f"(SELECT id FROM topics WHERE name=?)", (name,)
            ).fetchone()[0]
            print(f"  {name}: {cnt} msgs")
        conn.close()
    else:
        print(f"[WARN] Output bag db3 not found at {output_bag}")

    print(f"\nOutput bag path: {output_bag}")
    print("Update analyze.py BAG_DB to this path.")

if __name__ == '__main__':
    main()
