#!/usr/bin/env python3
"""Replay a bag missing trigger ON by injecting one after IMU warm-up."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


WS = Path("/home/hiho817/corgi_ws/corgi_ros2_ws")
ENV = (
    "export ROS_DOMAIN_ID=79 ROS_LOG_DIR=/tmp/corgi_ros_logs && "
    "source /opt/ros/humble/setup.bash && "
    f"source {WS}/install/setup.bash"
)


def start(command: str, log: Path):
    handle = log.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-c", f"{ENV} && {command}"], stdout=handle,
        stderr=subprocess.STDOUT, start_new_session=True)
    return process, handle


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--warmup", type=float, default=1.0)
    args = parser.parse_args()
    if args.output_bag.exists():
        parser.error("output exists")

    node = recorder = player = None
    handles = []
    try:
        node, h = start("ros2 launch corgi_odometry imu_only_replay.launch.py",
                        Path("/tmp/FLAT_Walk_NEW_REAL_1_partial_node.log"))
        handles.append(h)
        time.sleep(3)
        recorder, h = start(
            f"ros2 bag record -o {args.output_bag} /imu_only/ekf "
            "/imu_only/orientation /imu_only/ba /imu_only/bw /trigger",
            Path("/tmp/FLAT_Walk_NEW_REAL_1_partial_recorder.log"))
        handles.append(h)
        time.sleep(3)
        player, h = start(
            f"ros2 bag play {args.input_bag} --clock --rate 1.0 "
            "--topics /imu_raw /motor/state",
            Path("/tmp/FLAT_Walk_NEW_REAL_1_partial_player.log"))
        handles.append(h)
        time.sleep(args.warmup)
        trigger = (
            "ros2 topic pub --once --qos-durability transient_local /trigger "
            "corgi_msgs/msg/TriggerStamped \"{enable: true}\""
        )
        result = subprocess.run(["bash", "-c", f"{ENV} && {trigger}"])
        if result.returncode:
            raise RuntimeError("synthetic trigger publication failed")
        if player.wait() != 0:
            raise RuntimeError("bag player failed")
        time.sleep(3)
        return 0
    finally:
        stop(recorder)
        stop(node)
        stop(player)
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
