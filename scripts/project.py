"""Portable project commands. ROS and simulation use separate Python environments."""
from pathlib import Path
import argparse
import json
import os
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model"


def clean_ros_env():
    env = os.environ.copy()
    for name in list(env):
        if name.startswith(("ROS_", "AMENT_", "COLCON_", "CATKIN_")) or name in {
            "PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH",
            "CONDA_PREFIX", "CONDA_DEFAULT_ENV", "VIRTUAL_ENV", "RMW_IMPLEMENTATION",
        }:
            env.pop(name, None)
    env["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    # Use a separate domain from the ROS default and unrelated legacy projects.
    env["ROS_DOMAIN_ID"] = os.environ.get("OPENARM_ROS_DOMAIN_ID", "188")
    return env


def ros_run(command, overlay=False):
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise RuntimeError("ROS commands target Ubuntu 22.04 x86_64.")
    setup = os.environ.get("OPENARM_ROS_SETUP", "/opt/ros/humble/setup.bash")
    if not Path(setup).is_file():
        raise RuntimeError(f"ROS setup missing: {setup}")
    local_setup = ROOT / "workspace/install/local_setup.bash"
    if overlay and not local_setup.is_file():
        raise RuntimeError("Run 'bash project.sh build' before display/pose.")
    shell = ('set -e; source "$1"; shift; '
             'if [[ "${ROS_DISTRO:-}" != humble ]]; then echo "ROS Humble required" >&2; exit 2; fi; '
             'if [[ -n "$1" ]]; then source "$1"; fi; shift; exec "$@"')
    argv = ["/bin/bash", "--noprofile", "--norc", "-c", shell, "openarm-ros",
            setup, str(local_setup) if overlay else "",
            "/usr/bin/python3", str(MODEL / "tools/no_can.py"), *command]
    return subprocess.call(argv, env=clean_ros_env(), cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="help", choices=[
        "help", "check", "verify", "build", "display", "pose", "gpu-info",
        "setup-mujoco", "mujoco-check", "contract",
        "prepare-assets", "pack-usd",
    ])
    args, remaining = parser.parse_known_args()
    command = args.command
    if command not in ("display", "pose") and remaining:
        parser.error("Unexpected arguments: " + " ".join(remaining))
    if command == "help":
        parser.print_help()
        print("\nStart: check -> build -> display\n"
              "Exact poses: display gui:=false; another terminal: pose open_hand\n"
              "Optional CPU simulator: setup-mujoco -> mujoco-check\n"
              "No policy training or real robot driver is included in this stage.")
        return 0
    if command == "pack-usd":
        from large_assets import pack_usd
        print(json.dumps(pack_usd(ROOT), indent=2))
        return 0
    if command in ("prepare-assets", "check", "verify", "build", "display", "mujoco-check"):
        from large_assets import restore
        for path in restore(ROOT):
            print("Restored:", path)
        if command == "prepare-assets":
            return 0
    if command in ("check", "verify"):
        from check_project import check
        result = check(ROOT, runtime=command == "check", verify_hashes=command == "verify")
        print(json.dumps(result, indent=2, ensure_ascii=False))
        if command == "check":
            output = ROOT / "outputs"
            output.mkdir(exist_ok=True)
            (output / "desktop_check.json").write_text(
                json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        return 0 if result["asset_checks_passed"] else 1
    if command == "contract":
        print((ROOT / "config/policy_contract.json").read_text(encoding="utf-8"))
        return 0
    if command == "gpu-info":
        code = subprocess.call(["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
                                "--format=csv,noheader"])
        if code == 0:
            extra = subprocess.run(["nvidia-smi", "--query-gpu=compute_cap", "--format=csv,noheader"],
                                   capture_output=True, text=True)
            if extra.returncode == 0:
                print("Compute capability:", extra.stdout.strip())
            else:
                print("Compute capability query unavailable on this driver; full GPU name collected above.")
        return code
    if command == "build":
        code = ros_run(["/usr/bin/python3", str(ROOT / "scripts/ros_preflight.py")])
        if code:
            return code
        return ros_run(["colcon", "--log-base", str(ROOT / "workspace/log"), "build",
                        "--base-paths", str(MODEL / "ros2_description"),
                        "--build-base", str(ROOT / "workspace/build"),
                        "--install-base", str(ROOT / "workspace/install"),
                        "--executor", "sequential",
                        "--event-handlers", "console_direct+"])
    if command == "display":
        return ros_run(["ros2", "launch", "openarm_omnihand_description",
                        "display.launch.py", *remaining], overlay=True)
    if command == "pose":
        if len(remaining) != 1 or remaining[0] not in ("open_hand", "closed_hand", "intermediate_hand"):
            parser.error("pose requires open_hand, closed_hand, or intermediate_hand")
        return ros_run(["ros2", "topic", "pub", "--once", "/model_pose", "std_msgs/msg/String",
                        "{data: " + remaining[0] + "}"], overlay=True)
    venv_python = ROOT / ".venv-mujoco/bin/python"
    if command == "setup-mujoco":
        if platform.system() != "Linux":
            raise RuntimeError("Run setup-mujoco on the Ubuntu desktop.")
        if not venv_python.exists():
            subprocess.check_call(["/usr/bin/python3", "-m", "venv", str(venv_python.parents[1])])
        return subprocess.call([str(venv_python), "-m", "pip", "install", "-r",
                                str(ROOT / "requirements-mujoco.txt")])
    if command == "mujoco-check":
        if not venv_python.is_file():
            raise RuntimeError("First run 'bash project.sh setup-mujoco'.")
        env = clean_ros_env()
        return subprocess.call(["/usr/bin/python3", str(MODEL / "tools/no_can.py"),
                                str(venv_python), str(ROOT / "scripts/check_mujoco_cpu.py")],
                               cwd=ROOT, env=env)
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
