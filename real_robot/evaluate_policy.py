"""Offline PPO evaluation with official gains and the accepted measured A pose.

Ideal MuJoCo actuator only: no CAN, no verification of real firmware dynamics.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from .control import RunBundle
from .mit_servo import OFFICIAL_KP, OFFICIAL_KD, PROFILE
from .policy_mission import a_metrics, a_accepted
from rl_ppo.common import sha256


def model_file(bundle):
    result = json.loads((bundle.run/'result.json').read_text())
    return bundle.run/result['model']


def verify_evaluation(path,bundle):
    report = json.loads(Path(path).read_text(encoding='utf-8'))
    expected = {'control_profile':PROFILE,'model_sha256':bundle.env.task['model_sha256'],
                'config_sha256':bundle.env.task['config_sha256'],
                'task_sha256':sha256(bundle.run/'task/task.json'),
                'policy_sha256':sha256(model_file(bundle))}
    if any(report.get(k) != v for k,v in expected.items()):
        raise ValueError('Offline evaluation belongs to different policy/model/config/profile')
    if report.get('kp') != list(OFFICIAL_KP) or report.get('kd') != list(OFFICIAL_KD):
        raise ValueError('Offline evaluation gain mismatch')
    if report.get('all_passed') is not True:
        raise ValueError('Offline evaluation did not pass')
    for name in ('nominal','randomized'):
        episodes = report.get(name,[])
        if len(episodes) < 20 or not all(e.get('is_success') is True for e in episodes):
            raise ValueError('At least 20 successful nominal AND randomized episodes required')
    q = report.get('accepted_start_q_rad')
    metrics = a_metrics(bundle,q,[0.]*7)
    if not a_accepted(metrics,[0.]*7):
        raise ValueError('Evaluated start is outside accepted A pose')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True)
    parser.add_argument('--start-report',required=True,help='Saved native-MIT A test, including accepted residual errors')
    parser.add_argument('--episodes',type=int,default=20)
    parser.add_argument('--seed',type=int,default=6000)
    parser.add_argument('--output',default='outputs/real_robot/ppo_native_evaluation.json')
    args = parser.parse_args()
    if args.episodes < 20:
        parser.error('At least 20 episodes per condition required')
    b = RunBundle(args.run,load_policy=True)
    report = {'control_profile':PROFILE,'kp':list(OFFICIAL_KP),'kd':list(OFFICIAL_KD),
              'hardware_commands_sent':False,'hardware_ready':False,
              'actuator_model':'ideal MuJoCo torque; native PD formula, not actual motor firmware',
              'model_sha256':b.env.task['model_sha256'],'config_sha256':b.env.task['config_sha256'],
              'task_sha256':sha256(b.run/'task/task.json'),'policy_sha256':sha256(model_file(b)),
              'created_at':datetime.now().astimezone().isoformat(),'all_passed':False}
    env = b.env
    try:
        source = json.loads(Path(args.start_report).read_text(encoding='utf-8'))
        if (source.get('control_profile') != PROFILE or source.get('model_sha256') != env.task['model_sha256']
                or source.get('config_sha256') != env.task['config_sha256']):
            raise ValueError('Accepted A report belongs to different model/config/control mode')
        q = np.asarray(source['final_q_rad'],dtype=float)
        metrics = a_metrics(b,q,source['final_velocity_rad_s'])
        if not a_accepted(metrics,source['final_velocity_rad_s']):
            raise ValueError('Saved report does not satisfy palm A acceptance: '+str(metrics))
        b.check_state()
        report.update(accepted_start_q_rad=q.tolist(),accepted_A_metrics=metrics,
                      source_start_report=str(args.start_report),source_start_sha256=sha256(args.start_report))
        env.base_kp,env.base_kd = np.array(OFFICIAL_KP),np.array(OFFICIAL_KD)
        for name,randomized in (('nominal',False),('randomized',True)):
            env.randomize = randomized
            report[name] = []
            for episode in range(args.episodes):
                seed = args.seed+episode+(10000 if randomized else 0)
                env.reset(seed=seed)
                pose = q.copy()
                if randomized:
                    pose += env.np_random.uniform(-.015,.015,7)
                b.update_state(pose,np.zeros(7))
                b.check_state()
                # Same handoff as hardware: preserve the previous q_A target.
                b.start_policy(command=b.q_a)
                env.ended = False
                env.last_distance = float(np.linalg.norm(env.goal-env.position))
                obs = env._observation()
                for _ in range(int(np.ceil(env.settings['episode_seconds']/env.dt))+2):
                    action,_ = b.policy.predict(obs,deterministic=True)
                    obs,reward,terminated,truncated,info = env.step(action)
                    if np.max(np.abs(env.data.qvel)) > 1.:
                        info.update(is_success=False,failure='deployment_overspeed_at_policy_sample')
                        terminated = True
                    if terminated or truncated:
                        break
                else:
                    raise RuntimeError('Evaluation failed to terminate within episode')
                report[name].append({'episode':episode,'seed':seed,**info})
                print(name,episode,'success=',info['is_success'],'distance_m=',round(info['distance_m'],5),flush=True)
        report['all_passed'] = all(e['is_success'] for name in ('nominal','randomized') for e in report[name])
    except (ValueError,RuntimeError,OSError,FloatingPointError) as exc:
        report['error'] = str(exc)
    finally:
        env.close()
        output = Path(args.output)
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print('ALL_PASSED:',report['all_passed'],'REPORT:',output)
    return 0 if report['all_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
