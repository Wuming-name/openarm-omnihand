#!/bin/bash
set -euo pipefail
pivot_dir=$(cd "$(dirname "$0")/.." && pwd)
pivot_core=/isaac-sim/exts/isaacsim.ros2.core/humble
exec env -i DISPLAY=:1 HOME=/root PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 ACCEPT_EULA=Y OMNI_KIT_ALLOW_ROOT=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 LD_LIBRARY_PATH="$pivot_core/lib" PYTHONPATH="$pivot_core/rclpy" FASTRTPS_DEFAULT_PROFILES_FILE="$pivot_dir/tools/fastdds_udp.xml" /isaac-sim/python.sh "$pivot_dir/tools/no_can.py" /isaac-sim/kit/python/bin/python3 "$@"
