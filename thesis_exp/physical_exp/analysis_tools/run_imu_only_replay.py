#!/usr/bin/env python3
"""Replay raw inputs through the prediction-only IMU integration node."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


WS = Path("/home/hiho817/corgi_ws/corgi_ros2_ws")
SOURCE = (
    "export ROS_LOG_DIR=/tmp/corgi_ros_logs && "
    "export ROS_DOMAIN_ID=77 && "
    "source /opt/ros/humble/setup.bash && "
    f"source {WS}/install/setup.bash"
)


def start(command: str, log_path: Path) -> tuple[subprocess.Popen, object]:
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-c", f"{SOURCE} && {command}"],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, log


def stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    if not (args.input_bag / "metadata.yaml").is_file():
        parser.error(f"input bag has no metadata.yaml: {args.input_bag}")
    if args.output_bag.exists():
        parser.error(f"refusing to overwrite existing output: {args.output_bag}")

    tag = args.output_bag.name.replace("/", "_")
    node_log = Path("/tmp") / f"{tag}_node.log"
    recorder_log = Path("/tmp") / f"{tag}_recorder.log"
    node = recorder = None
    logs = []
    try:
        node, node_handle = start(
            "ros2 launch corgi_odometry imu_only_replay.launch.py", node_log)
        logs.append(node_handle)
        time.sleep(3)
        if node.poll() is not None:
            raise RuntimeError(f"IMU-only node exited early; see {node_log}")

        topics = " ".join([
            "/imu_only/ekf", "/imu_only/orientation",
            "/imu_only/ba", "/imu_only/bw", "/trigger",
        ])
        recorder, recorder_handle = start(
            f"ros2 bag record -o {args.output_bag} {topics}", recorder_log)
        logs.append(recorder_handle)
        time.sleep(3)
        if recorder.poll() is not None:
            raise RuntimeError(f"recorder exited early; see {recorder_log}")

        play_cmd = (
            f"ros2 bag play {args.input_bag} --clock --rate {args.rate}"
            " --topics /imu_raw /motor/state /trigger"
        )
        completed = subprocess.run(
            ["bash", "-c", f"{SOURCE} && {play_cmd}"], check=False)
        if completed.returncode != 0:
            raise RuntimeError(f"ros2 bag play failed with code {completed.returncode}")
        time.sleep(3)
        return 0
    finally:
        if recorder is not None:
            stop(recorder)
        if node is not None:
            stop(node)
        for handle in logs:
            handle.close()
        print(f"node log: {node_log}")
        print(f"recorder log: {recorder_log}")


if __name__ == "__main__":
    raise SystemExit(main())
