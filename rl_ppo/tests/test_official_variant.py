import copy
from pathlib import Path
import tempfile
import unittest

from rl_ppo.common import read_json, sha256
from rl_ppo.prepare_official import official_config, assert_only_gains_changed, prepare_official
from rl_ppo.train_official import training_arguments
from real_robot.mit_servo import OFFICIAL_KP, OFFICIAL_KD


class OfficialConfig(unittest.TestCase):
    def test_only_two_gain_arrays_change(self):
        source = read_json(Path(__file__).resolve().parents[1]/'configs/reach.json')
        original = copy.deepcopy(source)
        variant = official_config(source)
        assert_only_gains_changed(source,variant)
        self.assertEqual(variant['physics']['kp'],list(OFFICIAL_KP))
        self.assertEqual(variant['physics']['kd'],list(OFFICIAL_KD))
        self.assertEqual(variant['ppo'],source['ppo'])
        self.assertEqual(source,original)
        variant['ppo']['learning_rate'] *= .5
        with self.assertRaises(ValueError):
            assert_only_gains_changed(source,variant)


class OfficialTask(unittest.TestCase):
    def test_new_task_preserves_model_targets_and_original_and_uses_same_training_settings(self):
        source = Path(__file__).resolve().parents[2]/'ppo_validation/smoke_run'
        if not source.is_dir():
            self.skipTest('Local saved-task fixture absent')
        before = {name:sha256(source/'task'/name) for name in ('task.json','config.json','model.mjb')}
        old = read_json(source/'task/task.json')
        with tempfile.TemporaryDirectory() as directory:
            task = prepare_official(source,Path(directory)/'official_task')
            new = read_json(task)
            variant = read_json(task.with_name('config.json'))
            original = read_json(source/'task/config.json')
            assert_only_gains_changed(original,variant)
            for key in ('q_A','q_B','p_A_base_m','p_B_base_m','R_A_base','R_B_base'):
                self.assertEqual(new[key],old[key])
            self.assertTrue(new['preflight_passed'])
            self.assertEqual(new['model_sha256'],old['model_sha256'])
            audit = read_json(task.with_name('official_variant.json'))
            args = training_arguments(task,run_dir=Path(directory)/'run')
            self.assertEqual(int(args[args.index('--steps')+1]),audit['training_defaults']['steps'])
            self.assertEqual(int(args[args.index('--envs')+1]),audit['training_defaults']['envs'])
            self.assertEqual(training_arguments(task,steps=2048,envs=1,rollout=128)[-6:],
                             ['--steps','2048','--envs','1','--rollout','128'])
            with self.assertRaises(FileExistsError):
                prepare_official(source,task.parent)
        after = {name:sha256(source/'task'/name) for name in before}
        self.assertEqual(before,after)
