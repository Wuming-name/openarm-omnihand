import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import contextlib
import io

import numpy as np

from real_robot.control import RunBundle
from real_robot.move_to_a import validate_plan
from real_robot.policy_mission import PolicyMission, a_metrics, a_accepted
from real_robot.mit_servo import OFFICIAL_KP, OFFICIAL_KD, PROFILE
from real_robot.evaluate_policy import verify_evaluation, model_file
from real_robot import evaluate_policy
from rl_ppo.common import sha256


class FakePolicy:
    def predict(self,obs,deterministic=True):
        self.observation = obs.copy()
        return np.full(7,.2),None


class PolicyHandoff(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_run = Path(__file__).resolve().parents[2]/'ppo_validation/smoke_run'
        if not cls.fixture_run.is_dir():
            raise unittest.SkipTest('Saved model fixture absent')

    def setUp(self):
        self.b = RunBundle(self.fixture_run,load_policy=False)
        self.plan = json.loads((self.fixture_run.parent/'startup_plan_test.json').read_text())
        self.start,self.verified = validate_plan(self.b,self.plan)
        self.b.policy = FakePolicy()
        self.m = PolicyMission(self.b,self.start,self.verified['segments'],True)
        self.m.begin = 0.
        self.now = self.m.duration+1.
        self.q = np.array([-.4915312428473335,1.1198214694438082,-.24891279468986127,
                           1.0423819333180742,.18444342717631734,.1993209735255963,.18177309834439548])

    def tearDown(self):
        self.b.env.close()

    def enter_policy(self):
        first = self.m.tick(self.q,np.zeros(7),self.now)
        self.assertEqual(first['phase'],'HOLD_A')
        second = self.m.tick(self.q,np.zeros(7),self.now+.6)
        self.assertEqual(second['phase'],'POLICY')
        return second

    def test_accept_actual_palm_error_without_joint_exactness(self):
        metrics = a_metrics(self.b,self.q,np.zeros(7))
        self.assertAlmostEqual(metrics['a_distance_m'],.005396344322272382,places=7)
        self.assertTrue(a_accepted(metrics,np.zeros(7)))
        self.assertGreater(np.max(np.abs(self.q-self.b.q_a)),.02)
        packet = self.enter_policy()
        np.testing.assert_allclose(packet['command'],self.b.q_a)
        np.testing.assert_allclose(self.b.env.velocity_command,0.)
        np.testing.assert_allclose(packet['accepted_A']['q_rad'],self.q)

    def test_observation_is_measured_and_first_target_is_acceleration_limited(self):
        self.enter_policy()
        packet = self.m.tick(self.q,np.zeros(7),self.now+.62)
        expected = (2*self.q-self.b.env.low-self.b.env.high)/(self.b.env.high-self.b.env.low)
        np.testing.assert_allclose(self.b.policy.observation[:7],expected,atol=1e-6)
        self.assertTrue(np.all(np.abs(np.array(packet['command'])-self.b.q_a)
                               <= self.b.env.amax*self.b.env.dt**2+1e-10))

    def test_policy_timeout_is_failure_not_b_success(self):
        self.enter_policy()
        with self.assertRaisesRegex(RuntimeError,'did not reach B'):
            self.m.tick(self.q,np.zeros(7),self.now+14.)

    def test_b_requires_stable_success_then_holds_last_target(self):
        self.enter_policy()
        self.b.start_policy(command=self.b.q_b)
        self.m.tick(self.b.q_b,np.zeros(7),self.now+1.)
        packet = self.m.tick(self.b.q_b,np.zeros(7),self.now+1.6)
        self.assertEqual(packet['phase'],'HOLD_B')
        np.testing.assert_allclose(packet['velocity_command'],0.)
        saved = packet['command']
        packet = self.m.tick(self.b.q_b,np.zeros(7),self.now+1.62)
        self.assertEqual(packet['command'],saved)

    def test_offline_report_binds_policy_and_rejects_failure(self):
        report = {'control_profile':PROFILE,'kp':list(OFFICIAL_KP),'kd':list(OFFICIAL_KD),
                  'model_sha256':self.b.env.task['model_sha256'],'config_sha256':self.b.env.task['config_sha256'],
                  'task_sha256':sha256(self.b.run/'task/task.json'),'policy_sha256':sha256(model_file(self.b)),
                  'accepted_start_q_rad':self.q.tolist(),'all_passed':True,
                  'nominal':[{'is_success':True} for _ in range(20)],
                  'randomized':[{'is_success':True} for _ in range(20)]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'evaluation.json'
            path.write_text(json.dumps(report))
            verify_evaluation(path,self.b)
            for key,value in (('policy_sha256','bad'),('kp',[0.]*7),('all_passed',False),
                              ('randomized',[{'is_success':False}]*20)):
                bad = copy.deepcopy(report)
                bad[key] = value
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    verify_evaluation(path,self.b)

    def test_offline_evaluation_cli_writes_failure_and_never_claims_hardware_ready(self):
        self.b.env.settings['episode_seconds'] = .04  # Test-only short episodes, saved files unchanged.
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'accepted.json'
            output = Path(directory)/'evaluation.json'
            source.write_text(json.dumps({'control_profile':PROFILE,
                'model_sha256':self.b.env.task['model_sha256'],
                'config_sha256':self.b.env.task['config_sha256'],
                'final_q_rad':self.q.tolist(),'final_velocity_rad_s':[0.]*7}))
            argv = ['evaluate_policy','--run',str(self.fixture_run),'--start-report',str(source),
                    '--output',str(output)]
            with patch('sys.argv',argv),patch.object(evaluate_policy,'RunBundle',return_value=self.b),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(evaluate_policy.main(),1)
            report = json.loads(output.read_text())
            self.assertEqual(len(report['nominal']),20)
            self.assertEqual(len(report['randomized']),20)
            self.assertFalse(report['all_passed'])
            self.assertFalse(report['hardware_ready'])
