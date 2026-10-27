#!/bin/bash
set -e
p=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
cd "$p"
python3 tools/no_can.py colcon --log-base runtime/log build --base-paths ros2_description --build-base runtime/build --install-base runtime/install
