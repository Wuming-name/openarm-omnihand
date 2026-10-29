"""Create a new PPO task changing only Kp/Kd from a saved training snapshot.

Preserves exact A/B poses and compiled model. No hardware, no old-file edits.
"""
import argparse
import copy
from datetime import datetime
from pathlib import Path
import shutil

import mujoco
import numpy as np

from .common import config_hash, load_config, read_json, root_path, sha256, write_json
from .env import PalmReachEnv
from .model import contacts, within_workspace, tcp_pose
from real_robot.mit_servo import OFFICIAL_KP, OFFICIAL_KD, PROFILE


def official_config(source):
    result = copy.deepcopy(source)
    result['physics']['kp'] = list(OFFICIAL_KP)
    result['physics']['kd'] = list(OFFICIAL_KD)
    return result


def assert_only_gains_changed(source,variant):
    unchanged = copy.deepcopy(variant)
    for name in ('kp','kd'):
        unchanged['physics'][name] = copy.deepcopy(source['physics'][name])
    if unchanged != source:
        raise ValueError('Variant changes settings beyond physics.kp/physics.kd')


def prepare_official(source_run,output):
    source_run,output = Path(source_run).resolve(),Path(output).resolve()
    source_task = source_run/'task/task.json'
    source_config = source_task.with_name('config.json')
    task = read_json(source_task)
    config = load_config(source_config)
    if not task.get('preflight_passed') or task['config_sha256'] != config_hash(config):
        raise ValueError('Source task preflight/config hash invalid')
    if task['mujoco_version'] != mujoco.__version__:
        raise ValueError('Use the same pinned MuJoCo version as the source run')
    filename = task['model_file']
    if Path(filename).name != filename:
        raise ValueError('Source model must be a local task filename')
    binary = source_task.parent/filename
    if sha256(binary) != task['model_sha256']:
        raise ValueError('Source compiled model hash mismatch')
    contract_file = source_run/'policy_contract.json'
    if contract_file.is_file():
        contract = read_json(contract_file)
        if (contract['task_sha256'] != sha256(source_task)
                or contract['model_sha256'] != task['model_sha256']
                or contract['config_sha256'] != task['config_sha256']):
            raise ValueError('Source run contract does not match saved task')
    if output.exists():
        raise FileExistsError('Variant output already exists; choose a NEW --output directory')
    if output == source_run or source_run in output.parents:
        raise ValueError('Do not write variant inside the original run')
    variant = official_config(config)
    assert_only_gains_changed(config,variant)
    ppo = config['ppo']
    defaults = {'steps':ppo['total_timesteps'],'envs':ppo['n_envs'],'rollout':ppo['n_steps']}
    settings_file = source_run/'training_settings.json'
    if settings_file.is_file():
        settings = read_json(settings_file)
        if settings.get('config') != config:
            raise ValueError('Source training_settings differs from task config')
        defaults.update(steps=settings['requested_steps'],envs=settings['n_envs'],rollout=settings['n_steps'])
    audit = {'variant':'official_kp_kd_only','control_profile':PROFILE,
             'source_run':str(source_run),'source_task_sha256':sha256(source_task),
             'source_config_sha256':config_hash(config),'source_model_sha256':task['model_sha256'],
             'new_config_sha256':config_hash(variant),'training_defaults':defaults,
             'changed_parameters':{
                 'physics.kp':{'old':config['physics']['kp'],'new':variant['physics']['kp']},
                 'physics.kd':{'old':config['physics']['kd'],'new':variant['physics']['kd']}},
             'ppo_unchanged':True,'task_targets_unchanged':True,'compiled_model_unchanged':True,
             'hardware_ready':False,'created_at':datetime.now().astimezone().isoformat()}
    output.mkdir(parents=True)
    shutil.copy2(binary,output/filename)
    for original,renamed in (('model.xml','model.xml'),('model.manifest.json','source_model.manifest.json')):
        path = source_task.parent/original
        if path.is_file():
            shutil.copy2(path,output/renamed)
    variant_task = copy.deepcopy(task)
    variant_task['config_sha256'] = config_hash(variant)
    variant_task['preflight_passed'] = False
    variant_task.pop('zero_action_hold_2s',None)
    write_json(output/'config.json',variant)
    write_json(output/'task.json',variant_task)
    write_json(output/'official_variant.json',audit)
    env = PalmReachEnv(output/'task.json',randomize=False,reset_noise=0.)
    try:
        qa,qb = np.array(task['q_A']),np.array(task['q_B'])
        for alpha in np.linspace(0.,1.,config['task']['path_samples']):
            env.data.qpos[:] = (1-alpha)*qa+alpha*qb
            mujoco.mj_forward(env.model,env.data)
            position,_ = tcp_pose(env.data,env.indices)
            if (contacts(env.model,env.data,env.physics['contact_tolerance_m'])
                    or not within_workspace(position,config)
                    or np.any(env.data.qpos < env.low) or np.any(env.data.qpos > env.high)):
                raise RuntimeError('Preserved A/B path failed new task preflight')
        env.reset(seed=0)
        initial = env.position.copy()
        for _ in range(100):
            _,_,terminated,truncated,info = env.step(np.zeros(7))
            if terminated or truncated:
                raise RuntimeError('Official-gain zero-action hold failed: '+str(info))
        drift = float(np.max(np.abs(env.data.qpos-qa)))
        if drift > .02:
            raise RuntimeError('Official-gain hold drift exceeds 0.02rad')
        variant_task['zero_action_hold_2s'] = {'passed':True,'max_joint_drift_rad':drift,
                                              'tcp_drift_m':float(np.linalg.norm(env.position-initial))}
        variant_task['preflight_passed'] = True
        write_json(output/'task.json',variant_task)
        audit['preflight_passed'] = True
        audit['variant_task_sha256'] = sha256(output/'task.json')
        write_json(output/'official_variant.json',audit)
    finally:
        env.close()
    print('TASK:',output/'task.json')
    print('Kp:',variant['physics']['kp'],'Kd:',variant['physics']['kd'])
    print('PPO/A/B/model unchanged; original run untouched. Training defaults:',defaults)
    return output/'task.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',default='outputs/ppo/run_20261028_164711_538719')
    parser.add_argument('--output')
    args = parser.parse_args()
    prepare_official(args.source_run,args.output or root_path()/'outputs/ppo_official/task')


if __name__ == '__main__':
    main()
