"""Build a gravity-enabled frozen-hand model and search a collision-checked A/B pair."""
from pathlib import Path
import numpy as np
import mujoco

from .common import DEFAULT_CONFIG, common_parser, load_config, root_path, write_json
from .model import build_model, contacts, tcp_pose, rotation_vector, within_workspace


def select_task(model, indices, config):
    settings, physics = config["task"], config["physics"]
    rng = np.random.default_rng(settings["seed"])
    data = mujoco.MjData(model)
    ranges = model.jnt_range[indices["joints"]]
    lower = ranges[:, 0] + physics["joint_margin_rad"]
    upper = ranges[:, 1] - physics["joint_margin_rad"]
    counts = {"A_collision": 0, "B_collision": 0, "workspace": 0, "distance_or_orientation": 0, "path": 0}

    def pose(q):
        data.qpos[indices["q"]] = q
        mujoco.mj_forward(model, data)
        return tcp_pose(data, indices), contacts(model, data, physics["contact_tolerance_m"])

    # Joint-space candidates are simulation-only. They are never robot commands.
    center = np.array([0.3, 0.8, -0.2, 1.05, 0.0, 0.35, 0.25])
    spread = np.array([0.8, 0.5, 0.6, 0.6, 0.5, 0.30, 0.7])
    example_contacts = []
    for attempt in range(settings["search_attempts"]):
        qa = np.clip(center + rng.uniform(-1, 1, 7) * spread, lower, upper)
        (pa, ra), collision = pose(qa)
        if collision:
            counts["A_collision"] += 1
            example_contacts = collision[:3]
            continue
        if not within_workspace(pa, config):
            counts["workspace"] += 1
            continue
        for _ in range(30):
            qb = np.clip(qa + rng.uniform(-1, 1, 7) * [0.9, 0.65, 0.7, 0.8, 0.7, 0.30, 0.8], lower, upper)
            (pb, rb), collision = pose(qb)
            if collision:
                counts["B_collision"] += 1
                continue
            distance = np.linalg.norm(pb-pa)
            angle = np.linalg.norm(rotation_vector(rb @ ra.T))
            if not (settings["distance_min_m"] <= distance <= settings["distance_max_m"] and angle <= settings["orientation_change_max_rad"]):
                counts["distance_or_orientation"] += 1
                continue
            good_path = True
            for alpha in np.linspace(0, 1, settings["path_samples"]):
                (p, _), collision = pose((1-alpha)*qa + alpha*qb)
                if collision or not within_workspace(p, config):
                    good_path = False
                    break
            if not good_path:
                counts["path"] += 1
                continue
            return {"q_A": qa.tolist(), "q_B": qb.tolist(), "p_A_base_m": pa.tolist(),
                    "p_B_base_m": pb.tolist(), "R_A_base": ra.tolist(), "R_B_base": rb.tolist(),
                    "distance_m": float(distance), "orientation_change_rad": float(angle),
                    "path_samples": settings["path_samples"], "search_attempts_used": attempt + 1,
                    "rejections": counts, "hardware_validated": False}
    raise RuntimeError(f"No collision-free A/B found. No collision checks were disabled. Rejections: {counts}; example: {example_contacts}")


def prepare(root, config, output):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Task output is not empty; choose a new --output directory")
    model, indices, manifest = build_model(root, config, output / "model.xml")
    task = select_task(model, indices, config)
    task.update({"model_file": "model.mjb", "config_sha256": manifest["config_sha256"],
                 "model_sha256": manifest["training_binary_sha256"],
                 "source_model_sha256": manifest["source_model_sha256"], "mujoco_version": mujoco.__version__})
    write_json(output / "task.json", task)
    write_json(output / "config.json", config)
    # Dynamics check is deliberately separate from geometric pose search.
    from .env import PalmReachEnv
    env = PalmReachEnv(output / "task.json", randomize=False, reset_noise=0.)
    try:
        env.reset(seed=0)
        for _ in range(100):
            _, _, terminated, truncated, info = env.step(np.zeros(7))
            if terminated or truncated:
                raise RuntimeError(f"Zero-action hold dynamics failed: {info}")
        task["zero_action_hold_2s"] = {"passed": True, "tcp_drift_m": float(np.linalg.norm(env.position - task["p_A_base_m"])),
                                      "max_joint_drift_rad": float(np.max(np.abs(env.data.qpos - task["q_A"])))}
        if task["zero_action_hold_2s"]["max_joint_drift_rad"] > 0.02:
            raise RuntimeError("Nominal hold drift exceeds 0.02 rad; dynamics check did not pass")
        task["preflight_passed"] = True
        write_json(output / "task.json", task)
    finally:
        env.close()
    print(f"TASK: {output / 'task.json'}", flush=True)
    print(f"A={task['p_A_base_m']}, B={task['p_B_base_m']}, distance={task['distance_m']:.3f} m", flush=True)
    return task


def main():
    parser = common_parser(__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output")
    args = parser.parse_args()
    root = root_path(args.project_root)
    prepare(root, load_config(args.config), args.output or root / "outputs/ppo/task")


if __name__ == "__main__":
    main()
