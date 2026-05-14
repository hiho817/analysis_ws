#!/usr/bin/env python3
"""
Ablation: replay walk_2m_01_01 bag with ONLY corgi_leg_odom (NO LiDAR / fusion node).

Usage:
  cd /home/hiho817/analysis_ws/experiments/20260513/walk_2m_01_01/bags
  python3 run_ablation_leg_only.py
"""

import subprocess, os, signal, time
from datetime import datetime

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"

INPUT_BAG  = "/home/hiho817/analysis_ws/experiments/20260513/walk_2m_01_01/bags/odom_fusion20260512_222613_trimmed"
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

REPLAY_TOPICS = [
    "/imu_raw",
    "/motor/state",
    "/trigger",
    "/gmo/contact_state",
]

RECORD_TOPICS = [
    "/ekf",
    "/ekf/orientation",
    "/ekf/ba",
    "/ekf/bw",
    "/gmo/contact_state",
    "/trigger",
]

def make_cmd(cmd):
    return f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"

def start(cmd, name="", log_file=None):
    print(f"[start] {name}")
    kwargs = dict(preexec_fn=os.setsid)
    if log_file:
        f = open(log_file, 'w')
        kwargs.update(stdout=f, stderr=subprocess.STDOUT)
    else:
        kwargs.update(stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return subprocess.Popen(["bash", "-c", cmd], **kwargs)

def kill_proc(p, name=""):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=5)
        except Exception as e:
            print(f"[kill] {name}: {e}")
            try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except: pass
    print(f"[done] {name} stopped")

def main():
    result = subprocess.run("ps aux | grep corgi_leg_odom | grep -v grep",
                            shell=True, capture_output=True, text=True)
    if result.stdout.strip():
        print("[WARN] Stale corgi_leg_odom detected — kill it first:")
        print(result.stdout)
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_bag = os.path.join(OUTPUT_DIR, f"ablation_leg_only_{ts}")
    print(f"[info] Output bag: {output_bag}")

    procs = {}
    try:
        procs['leg_odom'] = start(
            make_cmd("ros2 run corgi_odometry corgi_leg_odom "
                     "--ros-args -p use_sim_time:=true -r /imu:=/imu_raw"),
            "leg_odom", log_file="/tmp/ablation01_leg_odom.log"
        )
        time.sleep(3)
        log = open("/tmp/ablation01_leg_odom.log").read()
        print(f"[check] leg_odom log tail: {log[-200:]!r}")

        procs['recorder'] = start(
            make_cmd("ros2 bag record " + " ".join(RECORD_TOPICS) + f" -o {output_bag}"),
            "recorder", log_file="/tmp/ablation01_recorder.log"
        )
        time.sleep(3)
        print("[info] Recorder started. Replaying bag...")

        procs['replay'] = start(
            make_cmd("ros2 bag play " + INPUT_BAG + " --clock --rate 2.0 "
                     "--topics " + " ".join(REPLAY_TOPICS)),
            "replay"
        )
        procs['replay'].wait()
        print("[info] Replay finished.")
        time.sleep(3)

    except KeyboardInterrupt:
        print("\n[interrupt] Stopping...")
    finally:
        for name, p in reversed(list(procs.items())):
            kill_proc(p, name)

    print(f"\n[result] Output bag: {output_bag}")
    try:
        r = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && ros2 bag info {output_bag}"],
            capture_output=True, text=True, timeout=30
        )
        print(r.stdout)
    except Exception as e:
        print(f"[warn] bag info: {e}")

if __name__ == "__main__":
    main()
