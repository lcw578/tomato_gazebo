#!/usr/bin/env bash
# Rebuild, launch and verify in an isolated ROS/Gazebo session.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TOMATO_SIM_WORKSPACE="$PWD"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83
export GAZEBO_MASTER_URI=http://127.0.0.1:11673 GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=
export RCUTILS_COLORIZED_OUTPUT=0
evidence="${SIM_VERIFICATION_OUTPUT:-$PWD/validation}"
model_python="$(/usr/bin/python3 scripts/simulation_config.py yolo_python)"
mkdir -p "$evidence"
exec 9>"$evidence/.phase1.lock"
flock -n 9 || { echo 'Phase 1 verification is already running.' >&2; exit 1; }
if ss -H -ltn 'sport = :11673' | awk 'END {exit !NR}'; then
    echo "Gazebo port 11673 is in use. Stop the other session before verification." >&2
    exit 1
fi
export ROS_LOG_DIR="$evidence/ros"
printf 'RUNNING\n' > "$evidence/phase1_status.txt"
launch_pid=
contact_pid=
cleanup() {
    local result=$?
    trap - EXIT
    if [[ -n "$contact_pid" ]]; then
        kill -TERM "$contact_pid" 2>/dev/null || true
        wait "$contact_pid" 2>/dev/null || true
    fi
    if [[ -n "$launch_pid" ]]; then
        kill -INT "$launch_pid" 2>/dev/null || true
        for _ in {1..50}; do
            kill -0 -- "-$launch_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    if [[ "$result" != 0 ]]; then
        printf 'FAIL (%s)\n' "$result" > "$evidence/phase1_status.txt"
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source /opt/ros/humble/setup.bash
./build_overlay.sh 2>&1 | tee "$evidence/phase1_build.log"
source .overlay/install/local_setup.bash

main_share="$(ros2 pkg prefix --share xc_robot_main_control)"
weight="$main_share/weights/best.pt"
test "$weight" -ef "$(/usr/bin/python3 scripts/simulation_config.py yolo_weights)"
sha256sum "$weight" | tee "$evidence/yolov8_weights.sha256"
YOLO_CONFIG_DIR="$evidence/ultralytics" "$model_python" - "$weight" <<'PY' | tee "$evidence/yolov8_inference.log"
import sys
import numpy as np
from ultralytics import YOLO

model = YOLO(sys.argv[1])
assert model.task == 'segment' and model.names == {0: 'stem', 1: 'tomato', 2: 'trunk'}
result, = model.predict(np.zeros((320, 320, 3), dtype=np.uint8), device='cpu', imgsz=320, verbose=False)
print(f'YOLOv8 CPU inference passed. Task={model.task}; classes={model.names}; boxes={len(result.boxes)}')
PY

description="$(ros2 pkg prefix --share tomato_sim_description)/urdf/tomato_picker_full.urdf.xacro"
xacro "$description" > "$evidence/tomato_picker.urdf"
xacro "$description" camera_angle_deg:=0 lift_height:=0.2 > "$evidence/tomato_picker_0deg_lift.urdf"
check_urdf "$evidence/tomato_picker.urdf" | tee "$evidence/check_urdf.log"
check_urdf "$evidence/tomato_picker_0deg_lift.urdf" > "$evidence/check_urdf_0deg_lift.log"
gz sdf -p "$evidence/tomato_picker.urdf" > "$evidence/tomato_picker.sdf"
export AUBO_SHARE="$(ros2 pkg prefix --share aubo_description)"
export TOMATO_SHARE="$(ros2 pkg prefix --share tomato_sim_description)"
/usr/bin/python3 scripts/phase1_structure.py "$evidence"
"$model_python" scripts/phase1_fk.py "$evidence/fk_reference.json"

env --default-signal=INT,TERM setsid ros2 launch tomato_sim_gazebo phase1.launch.py \
    world_record:="$evidence/phase1_world.json" > "$evidence/phase1_gazebo.log" 2>&1 9>&- &
launch_pid=$!
for _ in {1..24}; do
    kill -0 "$launch_pid" 2>/dev/null || { tail -50 "$evidence/phase1_gazebo.log"; exit 1; }
    if /usr/bin/python3 scripts/check_controllers.py > "$evidence/controllers.log" 2>&1; then
        break
    fi
    sleep 1
done
cat "$evidence/controllers.log"
awk '$1 ~ /^(joint_state_broadcaster|slide_velocity_controller|arm_trajectory_controller|gripper_controller)$/ && $NF=="active" {active[$1]=1} END {exit length(active)!=4}' "$evidence/controllers.log"

world_name="$(/usr/bin/python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["world_name"])' "$evidence/phase1_world.json")"
stdbuf -oL gz topic -e "/gazebo/$world_name/physics/contacts" > "$evidence/gazebo_contacts.log" 2>&1 &
contact_pid=$!
/usr/bin/python3 scripts/phase1_motion.py "$evidence" "$world_name" 2>&1 | tee "$evidence/phase1_motion.log"
kill -TERM "$contact_pid" 2>/dev/null || true
wait "$contact_pid" 2>/dev/null || true
contact_pid=
/usr/bin/python3 scripts/phase1_summary.py "$evidence"
if rg -n '\[ERROR\]|\[Err\]|No mesh specified|File or path does not exist' "$evidence/phase1_gazebo.log"; then
    exit 1
fi
printf 'PASS\n' > "$evidence/phase1_status.txt"
echo 'Phase 1 DoD passed.'
