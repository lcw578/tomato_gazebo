#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TOMATO_SIM_WORKSPACE="$PWD"
source /opt/ros/humble/setup.bash

exec 9>.overlay-build.lock
flock 9
/usr/bin/python3 scripts/prepare_overlay.py
colcon build --base-paths src/tomato_sim_adapters src/tomato_sim_description \
    src/tomato_sim_gazebo .overlay/src --build-base .overlay/build --install-base .overlay/install --symlink-install "$@"
