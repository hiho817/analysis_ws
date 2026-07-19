#!/usr/bin/env bash
# Recreate the selected rugged-ground WLW IMU-only bags without interactive terminals.
set -eo pipefail

ROOT=/home/hiho817/analysis_ws/thesis_exp/physical_exp
WS=/home/hiho817/corgi_ws/corgi_ros2_ws
OUT_ROOT="$ROOT/results/5.4_rugg_experiment/imu_only_bags"
LOG_ROOT="$ROOT/results/5.4_rugg_experiment/imu_only_replay_logs"
export ROS_DOMAIN_ID=78
export ROS_LOG_DIR=/tmp/corgi_ros_logs

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

trials=(RUGG_WLW_NEW_REAL_2 RUGG_WLW_NEW_REAL_3 RUGG_WLW_NEW_REAL_5)
mkdir -p "$OUT_ROOT" "$LOG_ROOT"

cleanup() {
  local status=$?
  for pid in "${REC_PID:-}" "${NODE_PID:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "-$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

if pgrep -af 'corgi_leg_odom|ros2 bag (play|record)' >/dev/null; then
  echo 'Refusing to start: a replay or leg-odometry process is already running.' >&2
  pgrep -af 'corgi_leg_odom|ros2 bag (play|record)' >&2 || true
  exit 1
fi

for trial in "${trials[@]}"; do
  input=$(find "$ROOT/experiments/RUGG_exp/$trial/bags" -mindepth 1 -maxdepth 1 -type d | head -n 1)
  output="$OUT_ROOT/$trial"
  node_log="$LOG_ROOT/${trial}_node.log"
  recorder_log="$LOG_ROOT/${trial}_recorder.log"

  [[ -f "$input/metadata.yaml" ]] || { echo "Missing input metadata: $input" >&2; exit 1; }
  if [[ -e "$output" ]]; then
    mv "$output" "${output}.failed_$(date +%Y%m%d_%H%M%S)"
  fi

  echo "[$(date -Is)] replaying $trial"
  setsid ros2 launch corgi_odometry imu_only_replay.launch.py >"$node_log" 2>&1 &
  NODE_PID=$!
  sleep 4
  kill -0 "$NODE_PID"

  setsid ros2 bag record -o "$output" \
    /imu_only/ekf /imu_only/orientation /imu_only/ba /imu_only/bw /trigger \
    >"$recorder_log" 2>&1 &
  REC_PID=$!
  sleep 4
  kill -0 "$REC_PID"

  # Only raw estimator inputs are replayed: never replay or record original /ekf.
  ros2 bag play "$input" --clock --rate 1.0 \
    --topics /imu_raw /motor/state /trigger
  sleep 4

  kill -INT "-$REC_PID" 2>/dev/null || true
  wait "$REC_PID" || true
  REC_PID=''
  kill -INT "-$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" || true
  NODE_PID=''

  db="$output/${trial}_0.db3"
  python3 - "$db" "$trial" <<'PY'
import sqlite3
import sys

db, trial = sys.argv[1:]
connection = sqlite3.connect(db)
rows = dict(connection.execute(
    "SELECT name, COUNT(*) FROM messages JOIN topics ON messages.topic_id=topics.id GROUP BY name"))
connection.close()
for name in ("/imu_only/ekf", "/imu_only/orientation", "/imu_only/ba", "/imu_only/bw", "/trigger"):
    print(f"{trial}: {name} = {rows.get(name, 0)}")
if rows.get("/imu_only/ekf", 0) < 1000:
    raise SystemExit(f"{trial}: insufficient /imu_only/ekf output")
if rows.get("/ekf", 0):
    raise SystemExit(f"{trial}: original /ekf contamination detected")
PY
done

echo "[$(date -Is)] all WLW IMU-only replays completed"
