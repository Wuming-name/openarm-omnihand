"""Train a CPU PPO policy in simulation. No hardware connection or ROS node."""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import os
import shutil

from .common import ARM_JOINTS, OBSERVATION_FIELDS, common_parser, config_hash, read_json, root_path, write_json, sha256


def make_env(task_path, rank, seed, randomize=True):
    from stable_baselines3.common.monitor import Monitor
    from .env import PalmReachEnv
    env = Monitor(PalmReachEnv(task_path, randomize=randomize))
    env.action_space.seed(seed + rank)
    return env


def main():
    parser = common_parser(__doc__)
    parser.add_argument("--task", help="Default outputs/ppo/task/task.json; run prepare first")
    parser.add_argument("--run-dir")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--envs", type=int)
    parser.add_argument("--rollout", type=int)
    args = parser.parse_args()
    root = root_path(args.project_root)
    task_path = Path(args.task or root / "outputs/ppo/task/task.json").resolve()
    task = read_json(task_path)
    if not task.get("preflight_passed"):
        raise RuntimeError("Task has not passed geometric/dynamics preflight; run prepare")
    config = read_json(task_path.with_name("config.json"))
    ppo = config["ppo"]
    steps = ppo["total_timesteps"] if args.steps is None else args.steps
    count = ppo["n_envs"] if args.envs is None else args.envs
    rollout = ppo["n_steps"] if args.rollout is None else args.rollout
    if steps < 1 or not 1 <= count <= 8 or rollout < 2:
        raise ValueError("Positive steps, 1..8 environments and rollout >= 2 required")
    batch = min(ppo["batch_size"], count*rollout)
    if count*rollout % batch:
        raise ValueError("n_envs * rollout must be divisible by batch_size")
    run = Path(args.run_dir or root / "outputs/ppo" / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")).resolve()
    if run.exists():
        raise FileExistsError("Run directory exists; choose a new directory")
    # Validate model/config before creating an output run.
    from .env import PalmReachEnv
    check = PalmReachEnv(task_path, randomize=False)
    check.reset(seed=ppo["seed"])
    check.close()
    run.mkdir(parents=True)
    shutil.copytree(task_path.parent, run / "task")
    task_path = run / "task" / task_path.name
    (run / "code_snapshot").mkdir()
    for file in Path(__file__).parent.glob("*.py"):
        shutil.copy2(file, run / "code_snapshot" / file.name)
    for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        os.environ[key] = "1"
    import torch
    torch.set_num_threads(1)
    import mujoco
    import gymnasium
    import stable_baselines3
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    from functools import partial

    class Curriculum(BaseCallback):
        def _fraction(self):
            progress = min(1., self.num_timesteps / max(1., steps*ppo["curriculum_full_at_fraction"]))
            return ppo["curriculum_start"] + progress*(1-ppo["curriculum_start"])
        def _on_training_start(self):
            self.training_env.env_method("set_curriculum", self._fraction())
            # SB3 has already reset at learn startup; reset again to apply initial curriculum.
            self.model._last_obs = self.training_env.reset()
        def _on_rollout_start(self):
            self.training_env.env_method("set_curriculum", self._fraction())
        def _on_step(self):
            return True

    factories = [partial(make_env, str(task_path), rank, ppo["seed"], config["randomization"]["enabled"]) for rank in range(count)]
    vector = DummyVecEnv(factories) if count == 1 else SubprocVecEnv(factories, start_method="spawn")
    vector.seed(ppo["seed"])
    evaluation = DummyVecEnv([partial(make_env, str(task_path), 100, ppo["seed"], False)])
    evaluation.seed(ppo["seed"] + 100)
    callbacks = CallbackList([Curriculum(),
        CheckpointCallback(max(1, ppo["checkpoint_every_steps"]//count), str(run / "checkpoints"), name_prefix="ppo"),
        EvalCallback(evaluation, best_model_save_path=str(run / "best"), log_path=str(run / "evaluation"),
                     eval_freq=max(1, ppo["eval_every_steps"]//count), n_eval_episodes=ppo["eval_episodes"], deterministic=True)])
    contract = {"observation_size": 37, "observation_fields": OBSERVATION_FIELDS, "action_size": 7,
                "joint_order": ARM_JOINTS, "action": "normalized desired joint velocity; acceleration/lead/soft-limit constrained integration",
                "normalization": "fixed physical scales defined in env.py and task/config.json; no VecNormalize",
                "policy_dt": config["physics"]["control_dt"], "config_sha256": config_hash(config),
                "task_sha256": sha256(task_path),
                "model_sha256": task["model_sha256"], "hardware_ready": False}
    write_json(run / "policy_contract.json", contract)
    write_json(run / "training_settings.json", {"requested_steps": steps, "n_envs": count, "n_steps": rollout,
               "batch_size": batch, "seed": ppo["seed"], "device": "cpu", "config": config,
               "versions": {"mujoco": mujoco.__version__, "torch": torch.__version__,
                            "gymnasium": gymnasium.__version__, "stable_baselines3": stable_baselines3.__version__}})
    model = PPO("MlpPolicy", vector, learning_rate=ppo["learning_rate"], n_steps=rollout, batch_size=batch,
                n_epochs=ppo["n_epochs"], gamma=ppo["gamma"], gae_lambda=ppo["gae_lambda"], clip_range=ppo["clip_range"],
                ent_coef=ppo["ent_coef"], policy_kwargs={"net_arch": dict(pi=ppo["net_arch"], vf=ppo["net_arch"])},
                seed=ppo["seed"], device="cpu", verbose=1)
    model.set_logger(configure(str(run / "logs"), ["stdout", "csv"]))
    interrupted, failed = False, False
    print("RUN:", run, flush=True)
    try:
        model.learn(total_timesteps=steps, callback=callbacks, progress_bar=False)
    except KeyboardInterrupt:
        interrupted = True
        print("Interrupted; saving simulation policy.", flush=True)
    except BaseException:
        failed = True
        raise
    finally:
        filename = "failed_model.zip" if failed else ("interrupted_model.zip" if interrupted else "final_model.zip")
        model.save(run / filename)
        write_json(run / "result.json", {"model": filename, "timesteps": model.num_timesteps,
                                         "interrupted": interrupted, "failed": failed, "hardware_ready": False})
        vector.close()
        evaluation.close()
    print("Saved:", run / filename)


if __name__ == "__main__":
    main()
