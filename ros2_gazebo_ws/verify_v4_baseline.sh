#!/usr/bin/env bash
set -Ee -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export TOMATO_SIM_WORKSPACE="$PWD"
OUT="${BASELINE_OUTPUT:-$ROOT/validation/v4_baseline/$(date -u +%Y%m%dT%H%M%S)-$$}"
mkdir -p "$OUT"
OUT="$(realpath "$OUT")"
printf 'Verification output: %s\n' "$OUT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}" ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT:-11683}" GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI= ROS_LOG_DIR="$OUT/ros"
if ss -Hlt | awk '{print $4}' | rg -q ":${GAZEBO_PORT:-11683}$"; then
  echo 'The baseline verification port is occupied; choose GAZEBO_PORT.' >&2
  exit 1
fi
launch_pid=""
cleanup() {
  local code=$?
  if [[ "$code" != 0 ]]; then printf 'FAIL\n' > "$OUT/status.txt"; fi
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
printf 'RUNNING\n' > "$OUT/status.txt"
source /opt/ros/humble/setup.bash
./build_overlay.sh > "$OUT/build.log" 2>&1
source .overlay/install/local_setup.bash
cp .overlay/dependencies.json "$OUT/dependencies.json"
env --default-signal=INT,TERM setsid ros2 launch tomato_sim_gazebo baseline.launch.py \
  world_record:="$OUT/world.json" > "$OUT/launch.log" 2>&1 &
launch_pid=$!
python3 "$ROOT/scripts/baseline_preflight.py" "$OUT"
