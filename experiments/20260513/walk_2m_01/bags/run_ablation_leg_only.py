#!/usr/bin/env python3
"""
Ablation: replay walk_2m_01 bag with ONLY corgi_leg_odom (NO LiDAR / fusion node).

This allows comparison of:
  - Inner EKF only (leg odometry, no LiDAR correction)
  - vs Inner EKF + outer fusion (with LiDAR, from the original bag)

Usage:
  cd /home/hiho817/analysis_ws/experiments/20260513/walk_2m_01/bags
  python3 run_ablation_leg_only.py
"""

import subprocess, os, signal, time
from datetime import datetime

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP  = "/home/hiho817/corgi_ws/corgi_ros2_ws/install/setup.bash"

INPUT_BAG  = "/home/hiho817/analysis_ws/experiments/20260513/walk_2m_01/bag/odom_fusion20260512_205637"
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Only raw input topics — NO /lidar_odom, NO /ekf (to avoid double-publish)
REPLAY_TOPICS = [
    "/imu_raw",
    "/motor/state",
    "/trigger",
    "/gmo/contact_state",
]

# Record inner EKF outputs only
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
    p = subprocess.Popen(["bash", "-c", cmd], **kwargs)
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
    # Pre-check: no stale leg_odom node
    result = subprocess.run("ps aux | grep corgi_leg_odom | grep -v grep",
                            shell=True, capture_output=True, text=True)
    if result.stdout.strip():
        print("[WARN] Stale corgi_leg_odom detected:")
        print(result.stdout)
        print("Kill it first: kill -9 <PID>")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_bag = os.path.join(OUTPUT_DIR, f"ablation_leg_only_{ts}")
    print(f"[info] Output bag: {output_bag}")

    procs = {}
    try:
        # 1. Start ONLY corgi_leg_odom (no fusion node)
        procs['leg_odom'] = start(
            make_cmd("ros2 run corgi_odometry corgi_leg_odom "
                     "--ros-args -p use_sim_time:=true -r /imu:=/imu_raw"),
            "leg_odom",
            log_file="/tmp/ablation_leg_odom.log"
        )
        time.sleep(3)
        # Confirm node started
        log = open("/tmp/ablation_leg_odom.log").read()
        print(f"[check] leg_odom log tail: {log[-200:]!r}")

        # 2. Start recorder
        procs['recorder'] = start(
            make_cmd("ros2 bag record " + " ".join(RECORD_TOPICS) + f" -o {output_bag}"),
            "recorder",
            log_file="/tmp/ablation_recorder.log"
        )
        time.sleep(3)
        print("[info] Recorder started. Replaying bag...")

        # 3. Replay (blocking) — rate 2.0 to match original recording
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

    # Report bag stats
    print(f"\n[result] Output bag: {output_bag}")
    try:
        r = subprocess.run(
            ["bash", "-c", f"source {ROS_SETUP} && ros2 bag info {output_bag}"],
            capture_output=True, text=True, timeout=30
        )
        print(r.stdout)
    except Exception as e:
        print(f"[warn] bag info: {e}")

    with open("/tmp/ablation_output_bag.txt", "w") as f:
        f.write(output_bag)
    print(f"[info] Bag path saved to /tmp/ablation_output_bag.txt")
    return output_bag

if __name__ == "__main__":
    main()
