"""CPU-only FK and contact inspection. No rendering, integration, or hardware access."""
from pathlib import Path
import json
import sys
import traceback
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model"


def inspect():
    model = mujoco.MjModel.from_xml_path(str(MODEL / "simulation/mjcf/openarm_omnihand.xml"))
    data = mujoco.MjData(model)
    poses = json.loads((MODEL / "ros2_description/config/poses.json").read_text())
    references = json.loads((MODEL / "reports/structural_validation.json").read_text())["FK"]
    rows = []
    for label, pose in poses.items():
        mujoco.mj_resetData(model, data)
        for name, value in pose.items():
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if index < 0:
                raise ValueError("Missing MuJoCo joint: " + name)
            data.qpos[model.jnt_qposadr[index]] = value
        # Forward evaluation only. Do not integrate an unvalidated collision model.
        mujoco.mj_forward(model, data)
        errors = []
        for name, reference in references[label].items():
            site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            if site < 0:
                raise ValueError("Missing reference site: " + name)
            actual = np.eye(4)
            actual[:3, :3] = data.site_xmat[site].reshape(3, 3)
            actual[:3, 3] = data.site_xpos[site]
            errors.append(float(np.max(np.abs(actual - np.asarray(reference)))))
        contacts = [{
            "a": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            "b": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
            "distance_m": float(contact.dist),
        } for contact in data.contact]
        rows.append({"pose": label, "max_fk_matrix_error": max(errors), "contacts": contacts})
    kinematics_pass = all(row["max_fk_matrix_error"] < 1e-8 for row in rows)
    contact_free = all(not row["contacts"] for row in rows)
    return {
        "mujoco_version": mujoco.__version__, "mode": "CPU forward only; no renderer",
        "nq": model.nq, "nu": model.nu, "neq": model.neq,
        "kinematics_pass": kinematics_pass, "sampled_poses_contact_free": contact_free,
        "poses": rows, "physics_integration_attempted": False, "rl_training_ready": False,
        "note": "Even contact-free endpoints do not validate dynamics, transitions, or a training task.",
    }


if __name__ == "__main__":
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    try:
        result = inspect()
        status = 0 if result["kinematics_pass"] and result["sampled_poses_contact_free"] else 3
    except Exception as exc:
        result = {"check_failed": True, "error": repr(exc), "rl_training_ready": False}
        traceback.print_exc()
        status = 2
    destination = output / "mujoco_cpu_check.json"
    destination.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("Report:", destination)
    sys.exit(status)
