#!/usr/bin/env python3
"""
Run the bag replay experiment:
1. Start corgi_leg_odom + corgi_fusion_node (use_sim_time=true)
2. Start bag recorder for output topics
3. Replay the input bag (raw topics only)
4. Wait for completion, kill all processes, report output bag path
"""

import subprocess
import time
import os
import signal
import sys
from datetime import datetime

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"
INPUT_BAG = "/home/hiho817/analysis_ws/datas/0508/odom_fusion20260508_201239"
OUTPUT_DIR = "/home/hiho817/analysis_ws/datas/0508"

REPLAY_TOPICS = [
    "/imu_raw",
    "/motor/state",
    "/trigger",
    "/gmo/contact_state",
    "/lidar_odom",
]

RECORD_TOPICS = [
    "/ekf",
    "/ekf/orientation",
    "/ekf/ba",
    "/ekf/bw",
    "/lidar_odom",
    "/odom_mapping",
    "/fusion/bv",
]

def make_cmd(cmd):
    """Wrap command with ROS source."""
    return f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"

def start(cmd, name=""):
    print(f"[start] {name}: {cmd[:80]}...")
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
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=5)
        except Exception as e:
            print(f"[kill] {name}: {e}")
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except:
                pass
    print(f"[done] {name} stopped")

def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_bag = os.path.join(OUTPUT_DIR, f"replay_fixed_{ts}")
    print(f"Output bag: {output_bag}")

    procs = {}

    try:
        # 1. Start leg_odom node
        procs['leg_odom'] = start(
            make_cmd("ros2 run corgi_odometry corgi_leg_odom "
                     "--ros-args -p use_sim_time:=true "
                     "-r /imu:=/imu_raw"),
            "leg_odom"
        )
        time.sleep(2)

        # 2. Start fusion node
        procs['fusion'] = start(
            make_cmd("ros2 run corgi_odometry corgi_fusion_node "
                     "--ros-args -p use_sim_time:=true"),
            "fusion"
        )
        time.sleep(2)

        # 3. Start bag recorder
        record_cmd = make_cmd(
            "ros2 bag record " +
            " ".join(RECORD_TOPICS) +
            f" -o {output_bag}"
        )
        procs['recorder'] = start(record_cmd, "recorder")
        time.sleep(3)
        print("[info] Recorder started. Starting bag replay...")

        # 4. Replay the bag (blocking)
        replay_cmd = make_cmd(
            f"ros2 bag play {INPUT_BAG} "
            "--clock "
            "--topics " + " ".join(REPLAY_TOPICS) +
            " --rate 2.0"
        )
        procs['replay'] = start(replay_cmd, "replay")

        # Wait for replay to finish
        print("[info] Waiting for replay to complete...")
        procs['replay'].wait()
        print("[info] Replay finished.")

        # Give recorder a moment to flush
        time.sleep(3)

    except KeyboardInterrupt:
        print("\n[interrupt] Stopping all processes...")
    finally:
        for name, p in reversed(list(procs.items())):
            kill_proc(p, name)

    print(f"\n[result] Output bag: {output_bag}")

    # Print bag info
    try:
        result = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && ros2 bag info {output_bag}"],
            capture_output=True, text=True, timeout=30
        )
        print(result.stdout)
    except Exception as e:
        print(f"[warn] Could not get bag info: {e}")

    return output_bag

if __name__ == "__main__":
    bag = main()
    # Write the path to a file so we can pick it up
    with open("/tmp/replay_output_bag.txt", "w") as f:
        f.write(bag)
