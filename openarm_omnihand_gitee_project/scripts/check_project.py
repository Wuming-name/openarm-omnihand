"""Dependency-free integrity, model-reference and policy-coordinate verification."""
from pathlib import Path
import hashlib
import json
import math
import os
import platform
import shutil
import xml.etree.ElementTree as ET


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def check(root, runtime=False, verify_hashes=True):
    root = Path(root)
    model = root / "model"
    errors = []
    manifest = read_json(root / "manifest/project_hashes.json")
    for name, digest in (manifest.items() if verify_hashes else []):
        path = root / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append("Missing or modified: " + name)
    urdf = ET.parse(model / "ros2_description/urdf/openarm_omnihand.urdf").getroot()
    links = {item.get("name") for item in urdf.findall("link")}
    joints = urdf.findall("joint")
    active = [item for item in joints if item.get("type") == "revolute"]
    independent = [item.get("name") for item in active if item.find("mimic") is None]
    children = [item.find("child").get("link") for item in joints]
    parents = [item.find("parent").get("link") for item in joints]
    if len(links) != 36 or len(joints) != 35 or len(active) != 23 or len(independent) != 17:
        errors.append("Unexpected robot dimensions")
    if len(set(children)) != len(children) or len(links - set(children)) != 1:
        errors.append("Invalid root or duplicate child links")
    if not set(parents + children) <= links:
        errors.append("Joint references unknown link")
    reached = links - set(children)
    while True:
        following = reached | {child for parent, child in zip(parents, children) if parent in reached}
        if following == reached:
            break
        reached = following
    if reached != links:
        errors.append("Disconnected or cyclic joint graph")
    unique_meshes = set()
    for mesh in urdf.findall(".//mesh"):
        uri = mesh.get("filename")
        prefix = "package://openarm_omnihand_description/"
        if not uri.startswith(prefix):
            errors.append("Unexpected mesh URI: " + uri)
            continue
        mesh_path = model / "ros2_description" / uri[len(prefix):]
        unique_meshes.add(str(mesh_path))
        if not mesh_path.is_file():
            errors.append("Missing mesh: " + uri)
    contract = read_json(root / "config/policy_contract.json")
    if contract["independent_joint_names"] != independent:
        errors.append("Policy independent order differs from URDF")
    if contract["state_joint_names"] != [item.get("name") for item in active]:
        errors.append("Policy state order differs from URDF")
    poses = read_json(model / "ros2_description/config/poses.json")
    for label, pose in poses.items():
        for joint in active:
            name = joint.get("name")
            value = pose[name]
            limit = joint.find("limit")
            if not math.isfinite(value) or not float(limit.get("lower")) - 1e-12 <= value <= float(limit.get("upper")) + 1e-12:
                errors.append(f"Invalid pose position: {label}/{name}")
            mimic = joint.find("mimic")
            if mimic is not None:
                target = pose[mimic.get("joint")] * float(mimic.get("multiplier", "1")) + float(mimic.get("offset", "0"))
                if abs(value - target) > 1e-12:
                    errors.append(f"Invalid mimic relation: {label}/{name}")
    mjcf = ET.parse(model / "simulation/mjcf/openarm_omnihand.xml").getroot()
    for mesh in mjcf.findall("asset/mesh"):
        if not (model / "simulation/mjcf/meshes" / mesh.get("file")).is_file():
            errors.append("Missing MJCF mesh: " + mesh.get("file"))
    actuator_names = [item.get("joint") for item in mjcf.find("actuator")]
    if actuator_names != independent:
        errors.append("MuJoCo actuator order differs from policy order")
    result = {
        "asset_checks_passed": not errors,
        "errors": errors,
        "verified_project_files": len(manifest) if verify_hashes else 0,
        "release_hash_verification": verify_hashes,
        "links": len(links), "joints": len(joints), "independent": len(independent),
        "movable_coordinates": len(active), "unique_urdf_meshes": len(unique_meshes),
        "rl_training_ready": False,
        "reasons": ["Existing MuJoCo wrist collision overlap is unresolved",
                    "Task, observation, reward, termination and reset are not implemented",
                    "Real robot actuator mapping, dynamics and deployment are not validated"],
    }
    if runtime:
        result["runtime"] = {
            "system": platform.system(), "machine": platform.machine(),
            "python": platform.python_version(),
            "configured_ros_setup": os.environ.get("OPENARM_ROS_SETUP", "/opt/ros/humble/setup.bash"),
            "ros_setup_exists": Path(os.environ.get("OPENARM_ROS_SETUP", "/opt/ros/humble/setup.bash")).is_file(),
            "colcon_on_current_path": shutil.which("colcon"),
            "isolated_mujoco_environment_exists": (root / ".venv-mujoco/bin/python").is_file(),
        }
    return result
