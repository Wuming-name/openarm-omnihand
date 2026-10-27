#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
p=$(cd "$(dirname "$0")/.." && pwd)
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-188}
case "${1:-}" in closed_hand|intermediate_hand|open_hand) ;; *) echo 'Usage: set_pose.sh closed_hand|intermediate_hand|open_hand (display gui:=false)'; exit 2;; esac
exec python3 "$p/tools/no_can.py" ros2 topic pub --once /model_pose std_msgs/msg/String "{data: $1}"
