"""Inspect A/B geometry or run a classical joint-space reference as a simulation diagnostic."""
import argparse
from pathlib import Path
import time
import numpy as np
import mujoco

from .common import read_json, write_json
from .env import PalmReachEnv
from .model import contacts, tcp_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--pose", choices=["A", "B"])
    parser.add_argument("--baseline", action="store_true", help="Execute a non-RL joint-space reference in simulation only")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    env = PalmReachEnv(args.task, randomize=False, reset_noise=0., render_mode="human" if args.render else None)
    try:
        env.reset(seed=0)
        if args.pose:
            env.data.qpos[:] = env.task["q_"+args.pose]
            mujoco.mj_forward(env.model, env.data)
            p, r = tcp_pose(env.data, env.indices)
            print("POSE", args.pose, "TCP base m", p.tolist())
            print("CONTACTS", contacts(env.model, env.data, env.physics["contact_tolerance_m"]))
            if args.render:
                env.render()
                while env.viewer.is_running():
                    env.viewer.sync()
                    time.sleep(.03)
        elif args.baseline:
            trajectory, info = [], {}
            for _ in range(round(env.settings["episode_seconds"]/env.dt)):
                begin = time.monotonic()
                action = np.clip(2.0*(np.asarray(env.task["q_B"])-env.data.qpos)/env.vmax, -1, 1)
                _, _, terminated, truncated, info = env.step(action)
                trajectory.append({"time_s": float(env.data.time), "q_rad": env.data.qpos.tolist(), **info})
                if args.render:
                    time.sleep(max(0., env.dt-(time.monotonic()-begin)))
                if terminated or truncated:
                    break
            path = Path(args.task).parent / f"baseline_{time.time_ns()}.json"
            write_json(path, {"controller": "classical joint-space reference; NOT a PPO result", "final": info, "trajectory": trajectory})
            print(info, "REPORT:", path)
            if not info.get("is_success"):
                raise RuntimeError("Classical reference did not complete task; inspect geometry/dynamics before long training")
        else:
            print(read_json(args.task))
    finally:
        env.close()


if __name__ == "__main__":
    main()
