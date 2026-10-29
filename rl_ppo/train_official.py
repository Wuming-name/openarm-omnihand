"""Fresh PPO training: official Kp/Kd task, otherwise original saved settings."""
import argparse
from datetime import datetime
from pathlib import Path

from .common import read_json, root_path, sha256
from .train import main as train_main
from real_robot.mit_servo import OFFICIAL_KP, OFFICIAL_KD


def training_arguments(task,run_dir=None,steps=None,envs=None,rollout=None):
    task = Path(task).resolve()
    config = read_json(task.with_name('config.json'))
    audit = read_json(task.with_name('official_variant.json'))
    if (config['physics']['kp'] != list(OFFICIAL_KP) or config['physics']['kd'] != list(OFFICIAL_KD)
            or not audit.get('preflight_passed') or audit['variant_task_sha256'] != sha256(task)):
        raise ValueError('Prepare and verify the official-gains variant before training')
    defaults = audit['training_defaults']
    run = Path(run_dir or Path(__file__).resolve().parent.parent/'outputs/ppo_official'/datetime.now().strftime('run_%Y%m%d_%H%M%S_%f'))
    return ['--task',str(task),'--run-dir',str(run),'--steps',str(defaults['steps'] if steps is None else steps),
            '--envs',str(defaults['envs'] if envs is None else envs),
            '--rollout',str(defaults['rollout'] if rollout is None else rollout)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',help='Optional project root, normally the repository containing this module')
    parser.add_argument('--task')
    parser.add_argument('--run-dir')
    parser.add_argument('--steps',type=int,help='Explicit smoke-test override only')
    parser.add_argument('--envs',type=int,help='Explicit smoke-test override only')
    parser.add_argument('--rollout',type=int,help='Explicit smoke-test override only')
    args = parser.parse_args()
    root = root_path(args.project_root)
    task = args.task or root/'outputs/ppo_official/task/task.json'
    run = args.run_dir or root/'outputs/ppo_official'/datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')
    train_main(training_arguments(task,run,args.steps,args.envs,args.rollout)+['--project-root',str(root)])


if __name__ == '__main__':
    main()
