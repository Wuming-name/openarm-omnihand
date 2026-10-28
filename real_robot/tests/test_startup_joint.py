import copy
import json
from pathlib import Path
import unittest
import time

import numpy as np

from real_robot.commission_hold import HoldViolation, ModelWorker
from real_robot.control import RunBundle
from real_robot.settings import load
from real_robot.startup_joint_test import reference_at, TrackingController, validate_first_segment, _startup_worker


class J2Reference(unittest.TestCase):
    def test_only_j2_moves_and_returns(self):
        start = np.zeros(7)
        end = start.copy()
        end[1] = .06
        for t in np.linspace(0,15,151):
            q,v,phase = reference_at(start,end,6.,t)
            np.testing.assert_allclose(q[[0,2,3,4,5,6]],0)
            np.testing.assert_allclose(v[[0,2,3,4,5,6]],0)
            self.assertTrue(0 <= q[1] <= .06)
            self.assertLessEqual(abs(v[1]), .019)
        q,v,phase = reference_at(start,end,6.,15.)
        np.testing.assert_allclose(q,start)
        self.assertEqual(phase,'RETURN_HOLD')


class J2ModelScope(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_run = Path(__file__).resolve().parents[2]/'ppo_validation/smoke_run'
        cls.plan_path = cls.fixture_run.parents[0]/'startup_plan_test.json'
        if not cls.fixture_run.is_dir() or not cls.plan_path.is_file():
            raise unittest.SkipTest('Local saved-run and startup-plan fixture absent')

    def setUp(self):
        self.bundle = RunBundle(self.fixture_run,load_policy=False)
        self.plan = json.loads(self.plan_path.read_text())

    def tearDown(self):
        self.bundle.env.close()

    def test_validated_first_segment_and_tampered_other_joint_rejected(self):
        start,end,duration,initial = validate_first_segment(self.plan,self.bundle)
        self.assertAlmostEqual(end[1]-start[1],.06)
        self.assertEqual(duration,6.)
        bad = copy.deepcopy(self.plan)
        bad['segments'][0]['end_q_rad'][6] += .01
        with self.assertRaises(ValueError):
            validate_first_segment(bad,self.bundle)

    def test_reference_outside_corridor_rejected(self):
        start,end,_,_ = validate_first_segment(self.plan,self.bundle)
        controller = TrackingController(self.bundle,load(),start,end)
        bad = start.copy()
        bad[6] += .01
        packet = {'command':bad.tolist(),'velocity_command':[0.]*7,'ff':[0.]*7}
        with self.assertRaises(RuntimeError):
            controller.accept_motion(packet,start,np.zeros(7),0.)

    def test_motion_and_fixed_joint_tracking_limits_are_distinct(self):
        start,end,_,_ = validate_first_segment(self.plan,self.bundle)
        controller = TrackingController(self.bundle,load(),start,end)
        command = start.copy()
        command[1] += .015
        packet = {'command':command.tolist(),'velocity_command':[0.]*7,'ff':[0.]*7}
        controller.accept_motion(packet,start,np.zeros(7),0.)
        bad = start.copy()
        bad[5] += .006
        with self.assertRaises(HoldViolation):
            controller.check_measured(bad,np.zeros(7))
        with self.assertRaises(HoldViolation):
            controller.check_measured(start,np.ones(7)*.11)

    def test_spawn_worker_starts_with_stationary_reference(self):
        start,end,duration,_ = validate_first_segment(self.plan,self.bundle)
        worker = ModelWorker(self.fixture_run,load(),start,np.zeros(7),True,
                             worker_target=_startup_worker,worker_args=(end,duration))
        try:
            np.testing.assert_allclose(worker.latest['command'],start)
            self.assertEqual(worker.latest['phase'],'INITIAL_HOLD')
            worker.request(start,np.zeros(7))
            self.assertTrue(worker.connection.poll(3.))
            self.assertEqual(len(worker.receive()['ff']),7)
            worker.connection.send({'q':start.tolist(),'velocity':[0.]*7,'start_time':time.monotonic()-2.})
            worker.pending = True
            self.assertTrue(worker.connection.poll(3.))
            packet = worker.receive()
            self.assertEqual(packet['phase'],'J2_OUTWARD')
            self.assertTrue(start[1] < packet['command'][1] < end[1])
            self.assertGreater(packet['velocity_command'][1],0.)
        finally:
            worker.close()
