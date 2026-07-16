#!/usr/bin/env python3
"""Replay a simulation bag through the prediction-only IMU estimator."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


WS = Path("/home/hiho817/corgi_ws/corgi_ros2_ws")
SOURCE = (
    "export ROS_LOG_DIR=/tmp/corgi_ros_logs && "
    "export ROS_DOMAIN_ID=78 && "
    "source /opt/ros/humble/setup.bash && "
    f"source {WS}/install/setup.bash"
)


def start(command: str, log_path: Path):
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-c", f"{SOURCE} && {command}"],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return process, log


def stop(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()
    if args.output_bag.exists():
        parser.error(f"output exists: {args.output_bag}")

    temporary = None
    input_bag = args.input_bag
    if input_bag.is_file():
        # Build valid metadata around a standalone db3.  This is required for
        # WALK, whose directory metadata points at an obsolete filename.
        temporary = tempfile.TemporaryDirectory(prefix="sim_imu_replay_")
        prepared = Path(temporary.name)
        link = prepared / f"{input_bag.stem}_0.db3"
        link.symlink_to(input_bag.resolve())
        completed = subprocess.run(
            ["bash", "-c", f"{SOURCE} && ros2 bag reindex {prepared}"],
            check=False)
        if completed.returncode:
            parser.error(f"failed to reindex {input_bag}")
        input_bag = prepared

    tag = args.output_bag.name
    node = recorder = None
    handles = []
    try:
        node, handle = start(
            "ros2 launch corgi_odometry imu_only_replay.launch.py",
            Path(f"/tmp/{tag}_node.log"))
        handles.append(handle)
        time.sleep(3)
        if node.poll() is not None:
            raise RuntimeError("IMU-only node exited early")

        recorder, handle = start(
            "ros2 bag record "
            f"-o {args.output_bag} /imu_only/ekf /imu_only/orientation "
            "/imu_only/ba /imu_only/bw /trigger",
            Path(f"/tmp/{tag}_recorder.log"))
        handles.append(handle)
        time.sleep(3)
        if recorder.poll() is not None:
            raise RuntimeError("recorder exited early")

        command = (
            f"{SOURCE} && ros2 bag play {input_bag} --clock "
            f"--rate {args.rate} --topics /imu_noisy /motor/state /trigger "
            "--remap /imu_noisy:=/imu_raw"
        )
        completed = subprocess.run(["bash", "-c", command], check=False)
        if completed.returncode:
            raise RuntimeError(f"bag play failed: {completed.returncode}")
        time.sleep(3)
    finally:
        stop(recorder)
        stop(node)
        for handle in handles:
            handle.close()
        if temporary is not None:
            temporary.cleanup()


if __name__ == "__main__":
    main()
