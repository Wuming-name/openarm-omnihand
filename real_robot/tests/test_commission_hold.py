import unittest
from pathlib import Path
import mujoco
import numpy as np

from real_robot.commission_hold import HoldController, StationaryOverlapGuard
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

    def test_stationary_baseline_accepts_only_near_zero_and_fixed_target(self):
        controller = HoldController(self.bundle, load(), np.zeros(7), np.zeros(7), 0., True)
        self.assertTrue(controller.initial_overlaps)
        self.assertTrue(np.isfinite(controller.step(np.zeros(7), np.zeros(7), 0.)).all())
        with self.assertRaises(RuntimeError):
            HoldController(self.bundle, load(), self.q, np.zeros(7), 0., True)
        with self.assertRaises(RuntimeError):
            controller.step(np.array([.006,0,0,0,0,0,0]), np.zeros(7), .002)


class BaselineGuardTests(unittest.TestCase):
    def item(self, depth=.018):
        return {'a': 'collision_openarm_body_link0_0',
                'b': 'collision_omni_R_thumb_mcp_0', 'distance_m': -depth}

    def test_same_or_reduced_initial_overlap_accepted(self):
        guard = StationaryOverlapGuard([self.item()])
        guard.check([self.item(.017)])
        guard.check([])

    def test_increasing_overlap_rejected(self):
        with self.assertRaises(RuntimeError):
            StationaryOverlapGuard([self.item()]).check([self.item(.019)])

    def test_new_pair_rejected_even_if_in_known_allowlist(self):
        item = self.item()
        item['b'] = 'collision_omni_R_thumb_pip_0'
        with self.assertRaises(RuntimeError):
            StationaryOverlapGuard([self.item()]).check([item])

    def test_unknown_or_deep_or_nonfinite_initial_contact_rejected(self):
        for item in [dict(self.item(), b='other_link'), self.item(.026), self.item(float('nan'))]:
            with self.assertRaises(RuntimeError):
                StationaryOverlapGuard([item])
