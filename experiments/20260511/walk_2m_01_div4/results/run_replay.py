#!/usr/bin/env python3
"""
Replay experiment: walk_2m_01_div4 (20260511)
Input  : original leg_odom20260507_161231 bag (unmodified motor/state)
Algorithm: theta_d/beta_d divided by 4 (compiled into corgi_leg_odom)
Records : /ekf /ekf/orientation /ekf/ba /ekf/bw /trigger /gmo/contact_state
"""

import subprocess
import time
import os
import signal
from datetime import datetime

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"

INPUT_BAG = (
    "/home/hiho817/analysis_ws/experiments/20260507/walk_2m_01/bags/"
    "leg_odom20260507_161231"
)
OUTPUT_BASE = "/home/hiho817/analysis_ws/experiments/20260511/walk_2m_01_div4/bags"

REPLAY_TOPICS = ["/imu_raw", "/motor/state", "/trigger", "/gmo/contact_state"]
RECORD_TOPICS = ["/ekf", "/ekf/orientation", "/ekf/ba", "/ekf/bw",
                 "/trigger", "/gmo/contact_state"]


def make_cmd(cmd):
    return f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"


def start(cmd, name=""):
    print(f"[start] {name}")
    return subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def kill_proc(p, name=""):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
    print(f"[done]  {name} stopped")


def main():
    os.system("pkill -f corgi_leg_odom 2>/dev/null; sleep 1")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_bag = os.path.join(OUTPUT_BASE, f"replay_div4_{ts}")
    print(f"Output bag: {output_bag}")

    procs = {}
    try:
        procs["leg_odom"] = start(
            make_cmd(
                "ros2 run corgi_odometry corgi_leg_odom "
                "--ros-args -p use_sim_time:=true -r /imu:=/imu_raw"
                " > /tmp/leg_odom_div4.log 2>&1"
            ),
            "leg_odom",
        )
        time.sleep(3)
        print("[info] leg_odom started")

        procs["recorder"] = start(
            make_cmd(
                "ros2 bag record "
                + " ".join(RECORD_TOPICS)
                + f" -o {output_bag}"
                + " > /tmp/recorder_div4.log 2>&1"
            ),
            "recorder",
        )
        time.sleep(3)
        print("[info] Recorder started")

        replay_cmd = make_cmd(
            f"ros2 bag play {INPUT_BAG} --clock --rate 2.0 "
            "--topics " + " ".join(REPLAY_TOPICS)
        )
        print("[info] Starting replay...")
        procs["replay"] = start(replay_cmd, "replay")
        procs["replay"].wait()
        print("[info] Replay finished.")
        time.sleep(4)

    except KeyboardInterrupt:
        print("\n[interrupt] Stopping...")
    finally:
        for name, p in reversed(list(procs.items())):
            kill_proc(p, name)

    print(f"\n[result] Output bag: {output_bag}")
    try:
        r = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && ros2 bag info {output_bag}"],
            capture_output=True, text=True, timeout=30,
        )
        print(r.stdout)
    except Exception as e:
        print(f"[warn] {e}")
    return output_bag


if __name__ == "__main__":
    main()
