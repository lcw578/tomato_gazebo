#!/usr/bin/env bash
set -Ee -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export TOMATO_SIM_WORKSPACE="$PWD"
config() { /usr/bin/python3 "$ROOT/scripts/simulation_config.py" "$@"; }
MIGRATION="$(config asset_root)"
BASELINE_ID="$(config baseline_id)"
HARVEST_WORLD="$(config harvest_world)"
STATUS="$ROOT/validation/phase4_status.txt"
SUMMARY="$ROOT/validation/phase4_summary.json"
RUNTIME="$ROOT/validation/phase4_runtime.json"
LOG="$ROOT/validation/phase4_launch.log"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
GAZEBO_PORT="${GAZEBO_PORT:-11682}"
LAUNCH_PID=""
YOLO_PID=""
MAIN_PID=""
RESULT="FAIL"

mkdir -p "$ROOT/validation/phase4"
write_status() { printf '%s\n' "$RESULT" > "$STATUS"; }
cleanup() {
  for pid in "$MAIN_PID" "$YOLO_PID" "$LAUNCH_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || true
    fi
  done
  for pid in "$MAIN_PID" "$YOLO_PID" "$LAUNCH_PID"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  write_status
}
trap cleanup EXIT
trap 'exit 130' INT TERM
printf '{"phase":4,"baseline_id":"%s","passed":false,"state":"verification_started"}\n' "$BASELINE_ID" > "$SUMMARY"
printf '{}\n' > "$RUNTIME"
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
  remote_depth_units_per_metre:=1000 request_gripper_length:=0.10 \
  evidence_file:="$RUNTIME" \
  action_timeout_scale:=10.0 > "$LOG" 2>&1 &
LAUNCH_PID=$!

ready=false
for _ in $(seq 1 180); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  if /usr/bin/python3 "$ROOT/scripts/check_controllers.py" > "$ROOT/validation/phase4_controllers.log" 2>&1; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "$ready" != true ]]; then
  printf '{"phase":4,"baseline_id":"%s","passed":false,"error":"Gazebo controllers did not activate"}\n' "$BASELINE_ID" > "$SUMMARY"
  exit 1
fi

python3 "$ROOT/scripts/prepare_phase4_calibration.py"
yolo_python="$(config yolo_python)"
ros_python="$(config ros_python)"
"$yolo_python" -c "import rclpy, torch, ultralytics; print(torch.__version__, torch.cuda.is_available(), ultralytics.__version__)"

env --default-signal=INT,TERM setsid "$yolo_python" -c \
  'from xc_robot_main_control.yolov8_service import main; main()' \
  > "$ROOT/validation/phase4_yolo.log" 2>&1 &
YOLO_PID=$!
service_ready=false
for _ in $(seq 1 120); do
  if ros2 service list --no-daemon 2>/dev/null | rg -q '^/yolo_detection$'; then
    service_ready=true
    break
  fi
  sleep 0.5
done
if [[ "$service_ready" != true ]]; then
  printf '{"phase":4,"baseline_id":"%s","passed":false,"error":"YOLO service did not become ready"}\n' "$BASELINE_ID" > "$SUMMARY"
  exit 1
fi
env --default-signal=INT,TERM setsid "$ros_python" -c \
  'from xc_robot_main_control.xc_robot_main_control_node import main; main()' \
  --ros-args \
  -p "hand_eye_main_path:=$ROOT/validation/phase4/calibration/" \
  -p is_real_camera_data:=true \
  -p depth_scale:=0.001 \
  -p gripper_length:=0.10 \
  -p use_moge_depth_estimator:=false \
  -p manage_external_camera_processes:=false \
  -p camera_working_flag:=2 \
  > "$ROOT/validation/phase4_main_control.log" 2>&1 &
MAIN_PID=$!

if python3 "$ROOT/scripts/phase4_checks.py" "$SUMMARY" "${PHASE4_TIMEOUT_S:-480}"; then
  RESULT="PASS"
else
  RESULT="FAIL"
  exit 1
fi
