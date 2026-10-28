from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
OBSERVATION_FIELDS = [("joint_position_normalized", 7), ("joint_velocity_over_vmax", 7),
                      ("command_tracking_error_over_lead", 7), ("palm_tcp_position_base", 3),
                      ("goal_delta_base", 3), ("orientation_error_base_over_pi", 3),
                      ("last_applied_velocity_over_vmax", 7)]
DEFAULT_CONFIG = Path(__file__).parent / "configs/reach.json"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()


def root_path(explicit=None):
    if explicit:
        root = Path(explicit).resolve()
    else:
        root = Path(__file__).resolve().parent.parent
    if not (root / "model/simulation/mjcf/openarm_omnihand.xml").is_file():
        raise ValueError("Project root not found; run from the Git project or use --project-root")
    return root


def common_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--project-root", help="Default: repository containing rl_ppo")
    return parser


def load_config(path=DEFAULT_CONFIG):
    import numpy as np
    config = read_json(path)
    if config.get("schema_version") != 1:
        raise ValueError("Unsupported configuration schema")
    physics, task = config["physics"], config["task"]
    for key in ("kp", "kd", "vmax", "amax", "torque_limits"):
        values = np.asarray(physics[key], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all() or (values <= 0).any():
            raise ValueError(f"Invalid seven-axis physics setting: {key}")
    ratio = physics["control_dt"] / physics["timestep"]
    if physics["timestep"] <= 0 or ratio < 1 or abs(ratio - round(ratio)) > 1e-9:
        raise ValueError("control_dt must be an integer multiple of the positive physics timestep")
    if not 0 < physics["max_target_lead_rad"] < 0.5 or not 0 < physics["joint_margin_rad"] < 0.3:
        raise ValueError("Invalid lead or joint margin")
    if not 0 < task["distance_min_m"] < task["distance_max_m"] or task["episode_seconds"] <= 0:
        raise ValueError("Invalid task distance or horizon")
    for key in ("tcp_workspace_low", "tcp_workspace_high"):
        if len(task[key]) != 3 or not np.isfinite(task[key]).all():
            raise ValueError("Invalid workspace")
    if not np.all(np.array(task["tcp_workspace_low"]) < task["tcp_workspace_high"]):
        raise ValueError("Empty workspace")
    if not np.isfinite(config["tcp_offset_palm_m"]).all() or len(config["tcp_offset_palm_m"]) != 3:
        raise ValueError("Invalid TCP offset")
    if len(physics["gravity"]) != 3 or not np.isfinite(physics["gravity"]).all():
        raise ValueError("Invalid gravity vector")
    for key in ("mass_fraction", "gain_fraction", "damping_fraction"):
        if not 0 <= config["randomization"][key] < 0.5:
            raise ValueError(f"Randomization {key} must be in [0,0.5)")
    for key in ("position_noise_std_rad", "velocity_noise_std_rad_s"):
        if not np.isfinite(config["randomization"][key]) or config["randomization"][key] < 0:
            raise ValueError("Noise standard deviation must be finite and nonnegative")
    if not 0 < config["ppo"]["curriculum_start"] <= 1 or not 0 < config["ppo"]["curriculum_full_at_fraction"] <= 1:
        raise ValueError("Invalid curriculum schedule")
    return config
