"""Evaluate a saved PPO run using its bundled task and exact observation/control contract."""
from pathlib import Path
import argparse
import time
import numpy as np

from .common import read_json, write_json, sha256


def apply_evaluation_gain_scale(env, scale):
    """Explicit runtime override; never edits the saved task/config or weights."""
    if not np.isfinite(scale) or not .1 <= scale <= 1.:
        raise ValueError('Evaluation gain scale must be 0.1..1.0')
    env.base_kp = env.base_kp * scale
    env.base_kd = env.base_kd * scale


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--model", help="Relative to run; default from result.json")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--randomized", action="store_true")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--gain-scale", type=float, default=1., help="Evaluate a runtime PD gain fraction (0.1..1); saved training artifacts remain unchanged")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("episodes must be positive")
    if not np.isfinite(args.gain_scale) or not .1 <= args.gain_scale <= 1.:
        parser.error('--gain-scale must be 0.1..1.0')
    from stable_baselines3 import PPO
    import torch
    torch.set_num_threads(1)
    from .env import PalmReachEnv
    run = Path(args.run).resolve()
    model_path = run / (args.model or read_json(run / "result.json")["model"])
    contract = read_json(run / "policy_contract.json")
    if sha256(run / "task/task.json") != contract["task_sha256"]:
        raise ValueError("A/B task snapshot changed after training")
    env = PalmReachEnv(run / "task/task.json", randomize=args.randomized, render_mode="human" if args.render else None)
    if contract["model_sha256"] != env.task["model_sha256"] or contract["config_sha256"] != env.task["config_sha256"]:
        raise ValueError("Checkpoint task/contract mismatch")
    apply_evaluation_gain_scale(env, args.gain_scale)
    model = PPO.load(model_path, device="cpu")
    if model.observation_space.shape != (37,) or model.action_space.shape != (7,):
        raise ValueError("Policy dimensions do not match task")
    results = []
    trajectory = []
    try:
        for episode in range(args.episodes):
            obs, _ = env.reset(seed=args.seed+episode)
            total = 0.
            while True:
                begin = time.monotonic()
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                total += reward
                if episode == 0:
                    trajectory.append({"time_s": float(env.data.time), "q_rad": env.data.qpos.tolist(),
                                       "tcp_base_m": info["tcp_base_m"], "goal_base_m": env.goal.tolist()})
                if args.render:
                    time.sleep(max(0., env.dt-(time.monotonic()-begin)))
                if terminated or truncated:
                    results.append({"episode": episode, "seed": args.seed+episode, "return": total,
                                    "truncated": truncated, **info})
                    print(f"episode={episode} success={info['is_success']} distance={info['distance_m']:.4f} failure={info['failure']}", flush=True)
                    break
    finally:
        env.close()
    report = {"model": str(model_path), "randomized": args.randomized, "episodes": results,
              "gain_scale": args.gain_scale, "training_gains_unchanged": args.gain_scale == 1.,
              "effective_nominal_kp": env.base_kp.tolist(), "effective_nominal_kd": env.base_kd.tolist(),
              "saved_model_sha256": env.task["model_sha256"], "saved_config_sha256": env.task["config_sha256"],
              "success_rate": float(np.mean([r["is_success"] for r in results])),
              "mean_final_distance_m": float(np.mean([r["distance_m"] for r in results])),
              "hardware_ready": False}
    tag = "randomized" if args.randomized else "nominal"
    if args.gain_scale != 1.:
        tag += f'_gain_{args.gain_scale:g}'
    stamp = time.time_ns()
    output = run / f"evaluation_{tag}_{stamp}.json"
    write_json(output, report)
    write_json(output.with_name(output.stem + "_trajectory.json"), trajectory)
    print("Success rate:", report["success_rate"], "REPORT:", output)


if __name__ == "__main__":
    main()
