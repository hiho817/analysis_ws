#!/usr/bin/env python3
"""
Ablation replay: replay each odom_fusion bag WITHOUT /lidar_odom
so that corgi_fusion_node receives no LiDAR updates and /fusion/bv
is never published.  corgi_leg_odom therefore runs with bv_outer_=0
(pure inner ESEKF, no LiDAR body-velocity feedback).

Experiments processed: exp1, exp2, exp4, exp5  (all fusion-node trials)

Output bags are written to each experiment's bags/ folder with
the prefix  ablation_no_lidar_<timestamp>.
"""

import datetime
import os
import signal
import subprocess
import sys
import time

# ── Paths ──────────────────────────────────────────────────────────────────
ROS2_WS  = "/home/hiho817/corgi_ws/corgi_ros2_ws"
EXP_ROOT = "/home/hiho817/analysis_ws/experiments/20260514"

SRC = (
    "source /opt/ros/humble/setup.bash && "
    f"source {ROS2_WS}/install/setup.bash"
)

# Input bags (exp_name → bag_dir)
INPUT_BAGS = {
    "exp1": f"{EXP_ROOT}/exp1/bags/odom_fusion20260514_215405",
    "exp2": f"{EXP_ROOT}/exp2/bags/odom_fusion20260514_220252",
    "exp4": f"{EXP_ROOT}/exp4/bags/odom_fusion20260514_225104",
    "exp5": f"{EXP_ROOT}/exp5/bags/odom_fusion20260514_230340",
}

# Topics to replay (raw sensor inputs ONLY — no /lidar_odom)
REPLAY_TOPICS = "/imu_raw /motor/state /trigger /gmo/contact_state"

# Topics to record from the new run
RECORD_TOPICS = (
    "/ekf /ekf/ba /ekf/bw /ekf/orientation "
    "/gmo/contact_state /odom_mapping /trigger"
)

REPLAY_RATE  = 1.0
SETTLE_TIME  = 3.0   # seconds to let nodes settle after launch
FINISH_DELAY = 5.0   # extra seconds after bag play finishes before stopping


# ── Helpers ────────────────────────────────────────────────────────────────

def run(cmd: str) -> subprocess.Popen:
    """Start a bash process; return the Popen handle."""
    return subprocess.Popen(
        ["bash", "-c", f"{SRC} && {cmd}"],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def kill_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass


def kill_all_ros_nodes() -> None:
    """Kill any leftover ros2 node processes."""
    subprocess.run(
        ["bash", "-c",
         "pkill -9 -f 'corgi_leg_odom|corgi_fusion_node|ros2 bag' || true"],
        check=False,
    )
    time.sleep(1.5)


def make_output_dir(exp_root: str) -> str:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(exp_root, "bags", f"ablation_no_lidar_{ts}")
    return out


# ── Main replay loop ───────────────────────────────────────────────────────

def replay_experiment(exp_name: str, input_bag: str) -> str:
    exp_root   = os.path.join(EXP_ROOT, exp_name)
    output_bag = make_output_dir(exp_root)

    print(f"\n{'='*60}")
    print(f"[{exp_name}] Ablation replay (no lidar)")
    print(f"  Input  : {input_bag}")
    print(f"  Output : {output_bag}")
    print(f"{'='*60}")

    # 0. Kill any leftover nodes
    kill_all_ros_nodes()

    # 1. Launch leg_odom + fusion_node (use_sim_time=true)
    print(f"  Launching nodes...")
    launch_proc = run(
        "ros2 launch corgi_odometry odom_fusion_replay.launch.py"
    )
    time.sleep(SETTLE_TIME)

    # 2. Start recorder (redirect stdout to avoid terminal suspension)
    print(f"  Starting recorder -> {output_bag}")
    rec_cmd = (
        f"ros2 bag record -o {output_bag} {RECORD_TOPICS} "
        f"> /tmp/ablation_rec_{exp_name}.log 2>&1"
    )
    rec_proc = run(rec_cmd)
    time.sleep(1.5)

    # 3. Replay bag — NO /lidar_odom in topic list
    print(f"  Replaying bag (no lidar) at rate {REPLAY_RATE}x ...")
    play_cmd = (
        f"ros2 bag play {input_bag} --clock --rate {REPLAY_RATE} "
        f"--topics {REPLAY_TOPICS}"
    )
    play_result = subprocess.run(
        ["bash", "-c", f"{SRC} && {play_cmd}"],
        timeout=600,
    )
    print(f"  Bag play finished (exit={play_result.returncode})")
    time.sleep(FINISH_DELAY)

    # 4. Clean up
    print(f"  Stopping recorder and nodes...")
    kill_group(rec_proc)
    kill_group(launch_proc)
    kill_all_ros_nodes()
    time.sleep(2.0)

    print(f"  Done -> {output_bag}")
    return output_bag


def main():
    exps = list(INPUT_BAGS.keys())

    # Allow filtering: python3 run_ablation_replay.py exp1 exp4
    if len(sys.argv) > 1:
        exps = [e for e in sys.argv[1:] if e in INPUT_BAGS]
        if not exps:
            print(f"Valid experiment names: {list(INPUT_BAGS.keys())}")
            sys.exit(1)

    results = {}
    for exp_name in exps:
        out = replay_experiment(exp_name, INPUT_BAGS[exp_name])
        results[exp_name] = out

    print("\n" + "="*60)
    print("Ablation replay summary:")
    for exp, out in results.items():
        print(f"  {exp}: {out}")
    print("="*60)
    print("\nNext step: run analyze_ablation.py to compare with/without lidar.")


if __name__ == "__main__":
    main()
