#!/bin/bash
set -e
p=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$p/runtime/install/local_setup.bash"
export PYTHONNOUSERSITE=1
cd "$p"
python3 tools/no_can.py colcon --log-base runtime/check_log build --base-paths tests/collision_check --build-base runtime/check_build --install-base runtime/check_install
python3 tools/no_can.py runtime/check_install/portable_collision_check/lib/portable_collision_check/check_model ros2_description/urdf/openarm_omnihand.urdf ros2_description/config/openarm_omnihand.srdf tests/collision_poses.yaml > reports/collision_reproduction.txt
