"""Run after sourcing the selected ROS distribution, before building the project."""
import importlib.util
from pathlib import Path
import os
import shutil
import sys
from ament_index_python.packages import get_package_prefix, PackageNotFoundError

missing = []
for package in ("ament_cmake", "rclpy", "robot_state_publisher", "joint_state_publisher_gui", "rviz2", "xacro"):
    try:
        prefix = get_package_prefix(package)
        print(f"{package}: {prefix}")
        # Do not accidentally build against an old same-name user description overlay.
        if not Path(prefix).is_relative_to(Path("/opt/ros/humble")):
            missing.append(f"{package}: unexpected prefix {prefix}")
    except PackageNotFoundError:
        missing.append(package)
if not shutil.which("colcon"):
    missing.append("colcon")
print("ROS_DISTRO:", os.environ.get("ROS_DISTRO"))
print("Python:", sys.executable)
if missing:
    print("Missing or unexpected dependencies:", ", ".join(missing), file=sys.stderr)
    print("See docs/运行与依赖.md. Nothing was installed automatically.", file=sys.stderr)
    sys.exit(2)
print("ROS_PREFLIGHT_PASS")
