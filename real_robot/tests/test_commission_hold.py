import unittest
from pathlib import Path
import mujoco
import numpy as np

from real_robot.commission_hold import HoldController
from real_robot.control import RunBundle
from real_robot.settings import load


class CommissionController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_run = Path(__file__).resolve().parents[2]/'ppo_validation/smoke_run'
        if not cls.fixture_run.is_dir():
            raise unittest.SkipTest('Local fixture absent; use self_check and mock CLI with a saved run')

    def setUp(self):
        self.bundle = RunBundle(self.fixture_run, load_policy=False)
        self.env = self.bundle.env
        self.q = self.env.data.qpos.copy()
        self.controller = HoldController(self.bundle, load(), self.q, np.zeros(7), 0.)

    def tearDown(self):
        self.env.close()

    def test_nominal_training_hold_100_physics_steps(self):
        for i in range(100):
            torque = self.controller.step(self.env.data.qpos.copy(), self.env.data.qvel.copy(), i*.002)
            self.env.data.ctrl[:] = torque
            mujoco.mj_step(self.env.model, self.env.data)
        np.testing.assert_allclose(self.env.data.qpos, self.q, atol=1e-8)

    def test_drift_rejected(self):
        q = self.q.copy()
        q[0] += .06
        with self.assertRaises(RuntimeError):
            self.controller.step(q, np.zeros(7), .002)

    def test_speed_rejected(self):
        with self.assertRaises(RuntimeError):
            self.controller.step(self.q, np.ones(7)*.16, .002)

    def test_late_tick_is_not_treated_as_valid_hold(self):
        self.controller.step(self.q, np.zeros(7), 0.)
        with self.assertRaises(RuntimeError):
            self.controller.step(self.q, np.zeros(7), .03)
