"""Check ROS environment isolation without requiring ROS on the authoring host."""
from pathlib import Path
import os
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import project


class ProjectEnvironment(unittest.TestCase):
    def test_old_ros_catkin_and_conda_paths_are_removed(self):
        inherited = {
            "PATH": "/old/conda/bin:/old/ros/bin",
            "ROS_DISTRO": "noetic", "ROS_PACKAGE_PATH": "/catkin/src",
            "AMENT_PREFIX_PATH": "/old/overlay", "COLCON_PREFIX_PATH": "/old/install",
            "CMAKE_PREFIX_PATH": "/old/install", "PYTHONPATH": "/old/site-packages",
            "LD_LIBRARY_PATH": "/old/ros/lib", "CONDA_PREFIX": "/old/conda",
            "VIRTUAL_ENV": "/old/venv", "HOME": "/home/desktop", "DISPLAY": ":0",
        }
        with patch.dict(os.environ, inherited, clear=True):
            env = project.clean_ros_env()
        for key in inherited:
            if key not in ("PATH", "HOME", "DISPLAY"):
                self.assertNotIn(key, env)
        self.assertEqual(env["ROS_DOMAIN_ID"], "188")
        self.assertEqual(env["HOME"], "/home/desktop")
        self.assertEqual(env["DISPLAY"], ":0")
        self.assertNotIn("conda", env["PATH"])

    def test_explicit_project_domain_is_used(self):
        with patch.dict(os.environ, {"OPENARM_ROS_DOMAIN_ID": "189"}, clear=True):
            self.assertEqual(project.clean_ros_env()["ROS_DOMAIN_ID"], "189")

    def test_setup_path_and_user_arguments_are_passed_as_argv(self):
        setup = "/test ROS/humble/setup.bash"
        command = ["ros2", "launch", "pkg", "file.py", "view:=/path with spaces/hand.rviz"]
        with patch.dict(os.environ, {"OPENARM_ROS_SETUP": setup}, clear=True), \
             patch.object(project.platform, "system", return_value="Linux"), \
             patch.object(project.platform, "machine", return_value="x86_64"), \
             patch.object(Path, "is_file", return_value=True), \
             patch.object(project.subprocess, "call", return_value=0) as call:
            self.assertEqual(project.ros_run(command), 0)
        argv = call.call_args.args[0]
        self.assertIn(setup, argv)
        self.assertEqual(argv[-len(command):], command)
        self.assertNotIn(setup, argv[4])  # Setup is not interpolated into shell source.


if __name__ == "__main__":
    unittest.main()
