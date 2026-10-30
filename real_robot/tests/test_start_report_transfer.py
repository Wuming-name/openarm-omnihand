import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from real_robot.evaluate_policy import validate_start_report
from real_robot.mit_servo import PROFILE, OFFICIAL_KP, OFFICIAL_KD
from rl_ppo.common import config_hash, sha256
from rl_ppo.prepare_official import official_config


class TransferAReport(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.old,self.new = root/'old/task',root/'new/task'
        self.old.mkdir(parents=True)
        self.new.mkdir(parents=True)
        config = {'physics':{'kp':[100.]*7,'kd':[10.]*7},'ppo':{'seed':28},'reward':{'progress':20}}
        self.variant = official_config(config)
        task = {'model_sha256':'unchanged-model','config_sha256':config_hash(config),
                'q_A':[0.]*7,'q_B':[.1]*7,'p_A_base_m':[0.]*3,'p_B_base_m':[.1]*3,
                'R_A_base':[[1,0,0],[0,1,0],[0,0,1]],'R_B_base':[[1,0,0],[0,1,0],[0,0,1]]}
        self.write(self.old/'task.json',task)
        self.write(self.old/'config.json',config)
        newtask = dict(task,config_sha256=config_hash(self.variant))
        self.write(self.new/'task.json',newtask)
        self.audit = {'variant':'official_kp_kd_only','source_run':str(self.old.parent),
                      'source_task_sha256':sha256(self.old/'task.json'),
                      'source_config_sha256':config_hash(config),'new_config_sha256':config_hash(self.variant),
                      'source_model_sha256':task['model_sha256'],
                      'variant_task_sha256':sha256(self.new/'task.json')}
        self.write(self.new/'official_variant.json',self.audit)
        self.bundle = SimpleNamespace(env=SimpleNamespace(task=newtask,config=self.variant,task_path=self.new/'task.json'))
        self.source = {'control_profile':PROFILE,'host_pd_enabled':False,
                       'native_mit_kp':list(OFFICIAL_KP),'native_mit_kd':list(OFFICIAL_KD),
                       'model_sha256':task['model_sha256'],'config_sha256':task['config_sha256']}

    @staticmethod
    def write(path,value):
        path.write_text(json.dumps(value),encoding='utf-8')

    def tearDown(self):
        self.temp.cleanup()

    def test_verified_transfer_does_not_mutate_report(self):
        old = copy.deepcopy(self.source)
        result = validate_start_report(self.source,self.bundle)
        self.assertEqual(result['mode'],'verified_gain_only_derivation')
        self.assertEqual(self.source,old)

    def test_reject_different_actual_control(self):
        for key,value in (('host_pd_enabled',True),('native_mit_kd',[0.]*7),('model_sha256','other')):
            source = dict(self.source,**{key:value})
            with self.assertRaises(ValueError):
                validate_start_report(source,self.bundle)

    def test_reject_changed_reward_despite_audit_labels(self):
        self.bundle.env.config['reward']['progress'] = 99
        self.bundle.env.task['config_sha256'] = config_hash(self.bundle.env.config)
        self.audit['new_config_sha256'] = self.bundle.env.task['config_sha256']
        self.write(self.new/'task.json',self.bundle.env.task)
        self.audit['variant_task_sha256'] = sha256(self.new/'task.json')
        self.write(self.new/'official_variant.json',self.audit)
        with self.assertRaisesRegex(ValueError,'more than'):
            validate_start_report(self.source,self.bundle)

    def test_reject_changed_target_even_when_task_hash_updated(self):
        self.bundle.env.task['q_B'] = [.2]*7
        self.write(self.new/'task.json',self.bundle.env.task)
        self.audit['variant_task_sha256'] = sha256(self.new/'task.json')
        self.write(self.new/'official_variant.json',self.audit)
        with self.assertRaisesRegex(ValueError,'geometry'):
            validate_start_report(self.source,self.bundle)
