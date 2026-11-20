#!/usr/bin/env bash
set -Ee -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export TOMATO_SIM_WORKSPACE="$PWD"
config() { /usr/bin/python3 "$ROOT/scripts/simulation_config.py" "$@"; }
MIGRATION="$(config asset_root)"
BASELINE_ID="$(config baseline_id)"
HARVEST_WORLD="$(config harvest_world)"
STATUS="$ROOT/validation/phase3_status.txt"
SUMMARY="$ROOT/validation/phase3_summary.json"
LOG="$ROOT/validation/phase3_launch.log"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
GAZEBO_PORT="${GAZEBO_PORT:-11681}"
LAUNCH_PID=""
RESULT="FAIL"

mkdir -p "$ROOT/validation"
write_status() { printf '%s\n' "$RESULT" > "$STATUS"; }
cleanup() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  write_status
}
trap cleanup EXIT
trap 'exit 130' INT TERM
printf '{"phase":3,"baseline_id":"%s","passed":false,"state":"verification_started"}\n' "$BASELINE_ID" > "$SUMMARY"
write_status

source /opt/ros/humble/setup.bash
./build_overlay.sh
python3 "$ROOT/scripts/prepare_phase3_world.py"
python3 "$ROOT/scripts/audit_v4_harvest_world.py"
GAZEBO_MODEL_DATABASE_URI= GAZEBO_MODEL_PATH="$ROOT/validation/phase3/models:$MIGRATION/assets/models" \
  gz sdf -k "$HARVEST_WORLD"

while ss -Hlt | awk '{print $4}' | rg -q ":${GAZEBO_PORT}$"; do
  GAZEBO_PORT=$((GAZEBO_PORT + 1))
done
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH="$ROOT/validation/phase3/models:$MIGRATION/assets/models"
source "$ROOT/.overlay/install/local_setup.bash"

env --default-signal=INT,TERM setsid ros2 launch tomato_sim_gazebo phase3.launch.py \
  action_timeout_scale:=10.0 > "$LOG" 2>&1 &
LAUNCH_PID=$!

ready=false
for _ in $(seq 1 180); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  if /usr/bin/python3 "$ROOT/scripts/check_controllers.py" > "$ROOT/validation/phase3_controllers.log" 2>&1; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "$ready" != true ]]; then
  printf '{"phase":3,"baseline_id":"%s","passed":false,"error":"Gazebo controllers did not activate"}\n' "$BASELINE_ID" > "$SUMMARY"
  exit 1
fi

if python3 "$ROOT/scripts/phase3_checks.py" "$ROOT/validation" "$(config harvest_rail)"; then
  RESULT="PASS"
else
  RESULT="FAIL"
  exit 1
fi
