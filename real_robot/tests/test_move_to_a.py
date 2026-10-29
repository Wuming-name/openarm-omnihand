import copy
import json
from pathlib import Path
import time
import unittest

import numpy as np

from real_robot.commission_hold import ModelWorker
from real_robot.control import RunBundle
from real_robot.move_to_a import ToAController, reference_at, validate_plan, _worker, at_a
from real_robot.settings import load


class MoveToAModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_run = Path(__file__).resolve().parents[2]/'ppo_validation/smoke_run'
        cls.plan_path = cls.fixture_run.parent/'startup_plan_test.json'
        if not cls.fixture_run.is_dir() or not cls.plan_path.is_file():
            raise unittest.SkipTest('Local saved-model fixture absent')

    def setUp(self):
        self.bundle = RunBundle(self.fixture_run,load_policy=False)
        self.plan = json.loads(self.plan_path.read_text())
        self.start,self.verified = validate_plan(self.bundle,self.plan)

    def tearDown(self):
        self.bundle.env.close()

    def test_plan_tampering_rejected(self):
        for field in ('model_sha256','config_sha256'):
            bad = copy.deepcopy(self.plan)
            bad[field] = 'bad'
            with self.assertRaises(ValueError):
                validate_plan(self.bundle,bad)
        bad = copy.deepcopy(self.plan)
        bad['segments'][2]['end_q_rad'][0] += .01
        with self.assertRaises(ValueError):
            validate_plan(self.bundle,bad)

    def test_reference_reaches_a_and_never_returns(self):
        segments = self.verified['segments']
        elapsed = 1.
        for segment in segments:
            q,v,phase = reference_at(self.start,segments,elapsed+.1)
            self.assertEqual(phase,segment['name'])
            elapsed += segment['duration_s']
        for t in (elapsed+1.,elapsed+100.):
            q,v,phase = reference_at(self.start,segments,t)
            self.assertEqual(phase,'HOLD_A')
            np.testing.assert_allclose(q,self.bundle.q_a)
            np.testing.assert_allclose(v,0.)

    def test_exact_training_gains_and_no_normal_tracking_rejection(self):
        original = copy.deepcopy(self.bundle.env.physics)
        c = ToAController(self.bundle,load(),self.start)
        for key in ('kp','kd','torque_limits'):
            np.testing.assert_allclose(c.servo.physics[key],original[key])
            np.testing.assert_allclose(self.bundle.env.physics[key],original[key])
        target = self.start.copy()
        target[1] += .03
        packet = {'command':target.tolist(),'velocity_command':[0.]*7,'ff':[0.]*7}
        c.accept_reference(packet,self.start.tolist(),[0.]*7,0.)
        output = c.step_fast(self.start.tolist(),[0.]*7,0.)
        self.assertAlmostEqual(output[1],original['kp'][1]*.03)

    def test_endpoint_gate_checks_position_tcp_and_speed(self):
        packet = {'a_distance_m':0.}
        self.assertTrue(at_a(self.bundle,self.bundle.q_a,[0.]*7,packet))
        bad = self.bundle.q_a.copy()
        bad[1] += .011
        self.assertFalse(at_a(self.bundle,bad,[0.]*7,packet))
        self.assertFalse(at_a(self.bundle,self.bundle.q_a,[0.]*7,{'a_distance_m':.016}))
        self.assertFalse(at_a(self.bundle,self.bundle.q_a,[.051]*7,packet))

    def test_only_j6_outer_kd_changes_and_saved_training_is_untouched(self):
        original = copy.deepcopy(self.bundle.env.physics)
        c = ToAController(self.bundle,load(),self.start,j6_kd=.5)
        expected = list(original['kd'])
        expected[5] = .5
        np.testing.assert_allclose(c.servo.physics['kd'],expected)
        np.testing.assert_allclose(c.servo.physics['kp'],original['kp'])
        np.testing.assert_allclose(self.bundle.env.physics['kd'],original['kd'])
        packet = {'command':self.start.tolist(),'velocity_command':[0.]*7,'ff':[0.]*7}
        velocity = [0.]*7
        velocity[5] = .1
        c.accept_reference(packet,self.start.tolist(),velocity,0.)
        output = c.step_fast(self.start.tolist(),velocity,0.)
        self.assertAlmostEqual(output[5],-.05)
        for invalid in (0.,1.,5.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                ToAController(self.bundle,load(),self.start,j6_kd=invalid)

    def test_shared_servo_rejects_gross_lead_and_overspeed(self):
        c = ToAController(self.bundle,load(),self.start)
        bad = self.start.copy()
        bad[1] += .3
        with self.assertRaises(RuntimeError):
            c.accept_reference({'command':bad.tolist(),'velocity_command':[0.]*7,'ff':[0.]*7},
                               self.start.tolist(),[0.]*7,0.)
        with self.assertRaises(RuntimeError):
            c.check_measured(self.start,np.ones(7)*1.1)

    def test_spawned_worker_follows_plan_then_holds_a(self):
        segments = self.verified['segments']
        worker = ModelWorker(self.fixture_run,load(),self.start,np.zeros(7),True,
                             worker_target=_worker,worker_args=(segments,))
        try:
            self.assertEqual(worker.latest['phase'],'HOLD_INITIAL')
            worker.connection.send({'q':self.bundle.q_a.tolist(),'velocity':[0.]*7,
                                    'start_time':time.monotonic()-self.verified['total_duration_s']-2.})
            worker.pending = True
            self.assertTrue(worker.connection.poll(3.))
            packet = worker.receive()
            self.assertEqual(packet['phase'],'HOLD_A')
            self.assertAlmostEqual(packet['a_distance_m'],0.)
            np.testing.assert_allclose(packet['command'],self.bundle.q_a)
        finally:
            worker.close()
