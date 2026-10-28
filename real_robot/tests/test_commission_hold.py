import unittest
from unittest import mock
from pathlib import Path
import mujoco
import numpy as np

from real_robot.commission_hold import HoldController, StationaryOverlapGuard, ModelWorker, HoldViolation
from real_robot.control import RunBundle
from real_robot.settings import load
from rl_ppo.evaluate import apply_evaluation_gain_scale


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

    def test_rejected_sample_preserves_exact_axis_and_drift(self):
        q = self.q.copy()
        q[3] += .06
        with self.assertRaises(HoldViolation) as caught:
            self.controller.check_measured(q, np.zeros(7))
        self.assertEqual(caught.exception.details['drift_axis'], 4)
        self.assertTrue(caught.exception.details['drift_exceeded'])
        self.assertFalse(caught.exception.details['speed_exceeded'])
        self.assertAlmostEqual(self.controller.max_drift, .06)
        velocity = np.zeros(7)
        velocity[5] = -.16
        with self.assertRaises(HoldViolation) as caught:
            self.controller.check_measured(self.q, velocity)
        self.assertEqual(caught.exception.details['speed_axis'], 6)
        self.assertTrue(caught.exception.details['speed_exceeded'])
        self.assertFalse(caught.exception.details['drift_exceeded'])

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

    def test_fast_servo_does_not_execute_model_work(self):
        ff = self.controller.reference(self.q, np.zeros(7))
        with mock.patch.object(self.bundle, 'update_state', side_effect=AssertionError('model work in fast loop')):
            torque = self.controller.step_fast(self.q, np.zeros(7), 0., ff)
            np.testing.assert_allclose(torque, ff, atol=1e-9)
            self.assertTrue(np.isfinite(self.controller.step_fast(self.q, np.zeros(7), .002)).all())

    def test_scaled_commissioning_gains_preserve_training_and_feedforward(self):
        kp = list(self.env.physics['kp'])
        kd = list(self.env.physics['kd'])
        scaled = HoldController(self.bundle, load(), self.q, np.zeros(7), 0., gain_scale=.2)
        ff = scaled.reference(self.q, np.zeros(7))
        actual = self.q-np.ones(7)*.001
        velocity = np.ones(7)*.01
        torque = scaled.step_fast(actual, velocity, 0., ff)
        expected = ff+.2*(np.asarray(kp)*.001-np.asarray(kd)*.01)
        np.testing.assert_allclose(torque, expected, atol=1e-8)
        self.assertEqual(self.env.physics['kp'], kp)
        self.assertEqual(self.env.physics['kd'], kd)
        with self.assertRaises(HoldViolation):
            scaled.check_measured(self.q, np.ones(7)*.16)

    def test_invalid_gain_scale_rejected_before_control(self):
        for value in (0., 1.1, float('nan')):
            with self.assertRaises(ValueError):
                HoldController(self.bundle, load(), self.q, np.zeros(7), 0., gain_scale=value)

    def test_gain_override_evaluation_matches_commissioning_after_reset(self):
        kp = list(self.env.physics['kp'])
        kd = list(self.env.physics['kd'])
        apply_evaluation_gain_scale(self.env, .2)
        self.env.reset(seed=123)
        np.testing.assert_allclose(self.env.kp, np.array(kp)*.2)
        np.testing.assert_allclose(self.env.kd, np.array(kd)*.2)
        self.assertEqual(self.env.physics['kp'], kp)
        self.assertEqual(self.env.physics['kd'], kd)
        for value in (0., 1.1, float('nan')):
            with self.assertRaises(ValueError):
                apply_evaluation_gain_scale(self.env, value)

    def test_spawned_model_worker_supplies_reference_and_stops_on_bad_state(self):
        worker = ModelWorker(self.fixture_run, load(), self.q, np.zeros(7), False)
        try:
            np.testing.assert_allclose(worker.latest['ff'], self.controller.reference(self.q, np.zeros(7)))
            worker.request(self.q, np.zeros(7))
            self.assertTrue(worker.connection.poll(3.))
            self.assertEqual(len(worker.receive()['ff']), 7)
            bad = self.q.copy()
            bad[0] += .06
            worker.request(bad, np.zeros(7))
            self.assertTrue(worker.connection.poll(3.))
            with self.assertRaises(HoldViolation) as caught:
                worker.receive()
            self.assertEqual(caught.exception.details['source'], 'model_worker')
            self.assertTrue(caught.exception.details['drift_exceeded'])
        finally:
            worker.close()


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
