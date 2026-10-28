"""Verify the real MuJoCo task, Gymnasium API, numerical bounds, and reset behavior."""
import argparse
import numpy as np
from stable_baselines3.common.env_checker import check_env

from .common import write_json
from .env import PalmReachEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    args = parser.parse_args()
    env = PalmReachEnv(args.task, randomize=False, reset_noise=0.)
    checks = []
    try:
        check_env(env, warn=True)
        checks.append("SB3 Gymnasium check_env")
        obs, _ = env.reset(seed=99)
        other, _ = env.reset(seed=99)
        np.testing.assert_array_equal(obs, other)
        assert obs.shape == (37,) and obs.dtype == np.float32
        assert env.model.nq == env.model.nv == env.model.nu == 7
        np.testing.assert_allclose(env.model.opt.gravity, [0, 0, -9.81])
        checks.append("37 observations, seven frozen-hand DOFs, deterministic reset, gravity")
        for bad in ([0]*6, [float("nan")]*7, [float("inf")]*7):
            try:
                env.step(bad)
            except ValueError:
                pass
            else:
                raise AssertionError("Malformed action accepted")
        checks.append("Malformed/nonfinite actions rejected")
        for seed in range(3):
            env.reset(seed=seed)
            initial = env.position.copy()
            for _ in range(100):
                _, reward, terminated, truncated, info = env.step(np.zeros(7))
                assert np.isfinite(reward) and not terminated and not truncated, info
            assert np.linalg.norm(env.position-initial) < .01
        checks.append("Nominal zero-action hold across resets")
        env.reset(seed=4)
        rng = np.random.default_rng(4)
        for _ in range(100):
            before_velocity = env.velocity_command.copy()
            observation, reward, terminated, truncated, _ = env.step(rng.uniform(-1, 1, 7))
            assert np.isfinite(observation).all() and np.isfinite(reward)
            assert np.all(np.abs(env.data.ctrl) <= env.limits + 1e-9)
            assert np.all(np.abs(env.velocity_command) <= env.vmax + 1e-9)
            assert np.all(np.abs(env.velocity_command-before_velocity) <= env.amax*env.dt+1e-9)
            assert np.all(env.command >= env.low) and np.all(env.command <= env.high)
            if terminated or truncated:
                env.reset(seed=5)
        checks.append("Random actions: finite state, total torque, velocity, acceleration and target bounds")
    finally:
        env.close()
    randomized = PalmReachEnv(args.task, randomize=True)
    try:
        a, _ = randomized.reset(seed=2026)
        mass_a = randomized.model.body_mass.copy()
        b, _ = randomized.reset(seed=2026)
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(mass_a, randomized.model.body_mass)
        for _ in range(100):
            obs, reward, terminated, truncated, info = randomized.step(np.zeros(7))
            assert np.isfinite(obs).all() and np.isfinite(reward)
            assert not terminated and not truncated, info
        checks.append("Seeded mass/gain/noise randomization and randomized hold")
    finally:
        randomized.close()
    result = {"passed": True, "checks": checks, "hardware_ready": False}
    output = env.task_path.with_name("verification.json")
    write_json(output, result)
    print(result, "REPORT:", output)


if __name__ == "__main__":
    main()
