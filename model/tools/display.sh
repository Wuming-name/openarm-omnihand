#!/bin/bash
set -e
p=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$p/runtime/install/local_setup.bash"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-188}
cd "$p"
exec python3 "$p/tools/no_can.py" ros2 launch openarm_omnihand_description display.launch.py "$@"
