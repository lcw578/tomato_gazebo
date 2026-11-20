#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TOMATO_SIM_WORKSPACE="$PWD"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83
export GAZEBO_MASTER_URI=http://127.0.0.1:11673 GAZEBO_IP=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=
export RCUTILS_COLORIZED_OUTPUT=0
evidence="${SIM_VERIFICATION_OUTPUT:-$PWD/validation}"
mkdir -p "$evidence"
exec 9>"$evidence/.phase1.lock"
flock -n 9
if ss -H -ltn 'sport = :11673' | awk 'END {exit !NR}'; then
    echo 'Gazebo port 11673 is already in use.' >&2
    exit 1
fi
export ROS_LOG_DIR="$evidence/ros"
printf 'RUNNING\n' > "$evidence/phase2_status.txt"
launch_pid=
cleanup() {
    local result=$?
    trap - EXIT
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
        printf 'FAIL (%s)\n' "$result" > "$evidence/phase2_status.txt"
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
source /opt/ros/humble/setup.bash
./build_overlay.sh 2>&1 | tee "$evidence/phase2_build.log"
source .overlay/install/local_setup.bash
env --default-signal=INT,TERM setsid ros2 launch tomato_sim_gazebo phase2.launch.py \
    > "$evidence/phase2_gazebo.log" 2>&1 9>&- &
launch_pid=$!
/usr/bin/python3 scripts/phase2_checks.py "$evidence" | tee "$evidence/phase2_checks.log"
if rg -n '\[ERROR\]|\[Err\]|Traceback' "$evidence/phase2_gazebo.log"; then
    exit 1
fi
printf 'PASS\n' > "$evidence/phase2_status.txt"
