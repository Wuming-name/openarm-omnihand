"""Exercise startup/hold/A/policy/B-hold in MuJoCo without ROS or CAN.

--baseline uses a classical controller ONLY as an integration diagnostic.
"""
import argparse
import copy
import time
from pathlib import Path
from datetime import datetime
import numpy as np
import mujoco

from rl_ppo.common import write_json
from .control import RunBundle, Mission
from .servo import Servo
from .settings import DEFAULT, load


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--config", default=str(DEFAULT))
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    hardware = load(args.config)
    bundle = RunBundle(args.run, load_policy=not args.baseline)
    if args.baseline:
        class Baseline:
            def predict(self, observation, deterministic=True):
                action = np.clip(2*(bundle.q_b-bundle.env.data.qpos)/bundle.env.vmax, -1, 1)
                return action, None
        bundle.policy = Baseline()
    model = copy.copy(bundle.env.model)
    data = mujoco.MjData(model)
    data.qpos[:] = bundle.q_a
    data.qpos[0] += .03
    data.qpos[1] -= .02
    mujoco.mj_forward(model, data)
    mission = Mission(bundle, hardware)
    servo = Servo(hardware, bundle.env.physics, bundle.env.low, bundle.env.high)
    report = {"backend": "MuJoCo only", "baseline": args.baseline, "hardware_commands_sent": False,
              "phases": [], "samples": [], "success": False}
    viewer = None
    next_command = 0.
    hand_ready = False
    try:
        if args.render:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(model, data)
        for _ in range(round(60/model.opt.timestep)):
            before = time.monotonic()
            now = float(data.time)
            if now+1e-8 >= next_command:
                if mission.phase == "FISTING":
                    hand_ready = True  # mock hand, no physical communication
                command, velocity, ff = mission.tick(data.qpos.copy(), data.qvel.copy(), hand_ready, True, now)
                servo.accept(command, velocity, ff, data.qpos, now)
                if not report["phases"] or report["phases"][-1] != mission.phase:
                    report["phases"].append(mission.phase)
                    print("STATE:", mission.phase, flush=True)
                report["samples"].append({"time": now, "phase": mission.phase, "q": data.qpos.tolist(),
                                          "tcp": bundle.env.position.tolist()})
                next_command += bundle.env.dt
            torque = servo.tick(data.qpos.tolist(), data.qvel.tolist(), now)
            if servo.fault:
                raise RuntimeError(servo.fault)
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            if viewer:
                viewer.sync()
                time.sleep(max(0., model.opt.timestep-(time.monotonic()-before)))
            if mission.phase == "DONE":
                report["success"] = True
                break
        if not report["success"]:
            report["error"] = "Full mission did not finish within 60 simulated seconds"
    except (RuntimeError, ValueError) as exc:
        report["error"] = str(exc)
    finally:
        if viewer:
            viewer.close()
        bundle.env.close()
    output = Path("outputs/real_robot") / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_dry_run.json")
    write_json(output, report)
    print({k: v for k, v in report.items() if k != "samples"}, "REPORT:", output)
    return 0 if report["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
