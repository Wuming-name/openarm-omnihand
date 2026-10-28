"""Check hardware controller math against the trained MuJoCo step without ROS or CAN."""
import argparse
import copy
import numpy as np
import mujoco

from rl_ppo.env import PalmReachEnv
from .control import RunBundle
from .settings import DEFAULT, load
from .servo import Servo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--config", default=str(DEFAULT))
    args = parser.parse_args()
    hardware = load(args.config)
    bundle = RunBundle(args.run, load_policy=False)
    reference = PalmReachEnv(bundle.env.task_path, randomize=False, reset_noise=0.)
    reference.reset(seed=0)
    model = copy.copy(bundle.env.model)
    data = mujoco.MjData(model)
    data.qpos[:] = reference.data.qpos
    mujoco.mj_forward(model, data)
    servo = Servo(hardware, reference.physics, reference.low, reference.high)
    try:
        rng = np.random.default_rng(28)
        for step in range(20):
            bundle.update_state(data.qpos, data.qvel)
            # All 37 input fields must agree before applying each test action.
            np.testing.assert_allclose(bundle.observation(), reference._observation(), atol=1e-6)
            desired_action = rng.uniform(-.1, .1, 7)
            command, velocity, ff = bundle.action_command(desired_action)
            now = step*reference.dt
            servo.accept(command, velocity, ff, data.qpos, now)
            # Compare one-step torque integration by using the same target and
            # nominal feedforward, then compare against env.step on the SAME action.
            _, _, ended, truncated, _ = reference.step(desired_action)
            for substep in range(reference.substeps):
                data.ctrl[:] = servo.tick(data.qpos.tolist(), data.qvel.tolist(), now+substep*model.opt.timestep)
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
            np.testing.assert_allclose(data.qpos, reference.data.qpos, atol=1e-8)
            np.testing.assert_allclose(data.qvel, reference.data.qvel, atol=1e-7)
            if ended or truncated:
                raise AssertionError("Unexpected reference termination during controller equivalence check")
        print("PASS: 37-D observations and torque-limited 500 Hz software PD match trained environment over 20 policy steps; no CAN/ROS used")
    finally:
        reference.close()
        bundle.env.close()


if __name__ == "__main__":
    main()
