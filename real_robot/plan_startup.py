"""Offline startup-path screening. Reads saved probe; never opens CAN or enables motors.

Initial known model overlaps are explicitly permitted ONLY while retreating from
their initial depths. This is not a claim of physical collision-free motion.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from .commission_hold import StationaryOverlapGuard
from .control import RunBundle
from .control import minimum_jerk
from .inspect_start import read_probe
from rl_ppo.model import contacts


class EscapeContactGuard:
    def __init__(self, initial):
        StationaryOverlapGuard(initial)  # Restrict identities and initial depths.
        self.best = {}
        self.exited = set()
        for c in initial:
            pair = frozenset((c['a'], c['b']))
            self.best[pair] = max(self.best.get(pair, 0.), -c['distance_m'])

    def check(self, current):
        present = set()
        for c in current:
            pair = frozenset((c['a'], c['b']))
            depth = -float(c['distance_m'])
            if pair not in self.best or pair in self.exited:
                raise RuntimeError('New contact or re-entry after clearing initial overlap')
            if not math.isfinite(depth) or depth > self.best[pair]+.0001:
                raise RuntimeError('Initial model penetration worsens >0.1mm from shallowest observed depth')
            self.best[pair] = min(self.best[pair], depth)
            present.add(pair)
        self.exited.update(set(self.best)-present)


def model_contacts(bundle, q):
    bundle.update_state(q, np.zeros(7))
    if np.any(q < bundle.hard_low) or np.any(q > bundle.hard_high):
        raise RuntimeError('Startup path outside hard joint limits')
    return contacts(bundle.env.model, bundle.env.data, bundle.env.physics['contact_tolerance_m'])


def screen_candidate(bundle, start, shoulder_delta, wrist_delta):
    first = start.copy()
    first[1] += shoulder_delta
    second = first.copy()
    second[5] += wrist_delta
    points = [start.copy(), first, second, bundle.q_a.copy()]
    initial = model_contacts(bundle, start)
    guard = EscapeContactGuard(initial)
    records, segments = [], []
    total_time = 0.
    for segment in range(3):
        a, b = points[segment], points[segment+1]
        # Slow quintic references; planning records only, no motion execution.
        duration = max(6. if segment < 2 else 20., float(np.max(1.875*np.abs(b-a)/(bundle.env.vmax*.2))),
                       float(np.max(np.sqrt(5.8*np.abs(b-a)/(bundle.env.amax*.2)))))
        count = max(100, int(math.ceil(np.max(np.abs(b-a))/.002)))
        max_contacts = 0
        for index in range(count+1):
            u = index/count
            q = a+u*(b-a)
            found = model_contacts(bundle, q)
            if segment < 2:
                if np.max(np.abs(q-start)) > .12+1e-9:
                    raise RuntimeError('Escape leaves 0.12rad neighborhood of measured start')
                guard.check(found)
            elif found:
                raise RuntimeError('Path after escape must satisfy strict original collision tolerance')
            max_contacts = max(max_contacts, len(found))
            if index in (0, count):
                records.append({'segment': segment, 'endpoint': index == count,
                                'q_rad': q.tolist(), 'palm_base_m': bundle.env.position.tolist(), 'contacts': found})
        if segment == 1 and model_contacts(bundle, b):
            raise RuntimeError('Escape endpoint still has model overlaps')
        segments.append({'name': ['shoulder_retreat', 'wrist_retreat', 'to_A'][segment],
                         'start_q_rad': a.tolist(), 'end_q_rad': b.tolist(),
                         'duration_s': duration, 'model_samples': count+1,
                         'max_contact_count': max_contacts,
                         'initial_overlap_exception': segment < 2,
                         'strict_collision_check': segment == 2})
        total_time += duration
    return {'initial_contacts': initial, 'segments': segments, 'endpoints': records,
            'total_duration_s': total_time}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--probe', help='Explicit historical successful probe; otherwise use newest from last 60 seconds')
    parser.add_argument('--output', default='outputs/real_robot/startup_plan.json')
    parser.add_argument('--render', action='store_true', help='Kinematic preview only; requires graphical desktop, never sends CAN')
    args = parser.parse_args()
    output = Path(args.output)
    report = {'mode': 'offline_startup_path_screening', 'hardware_commands_sent': False,
              'passed': False, 'motion_authorized': False,
              'entire_path_strictly_collision_free': False,
              'requires_initial_overlap_allowance': True,
              'note': 'Known initial model overlaps only retreat; no physical clearance or actuator validation implied.'}
    bundle = None
    try:
        if args.probe:
            probe = Path(args.probe)
        else:
            files = list(Path('outputs/real_robot').glob('*_probe.json'))
            if not files:
                raise RuntimeError('Run real_robot.probe first')
            probe = max(files, key=lambda p: p.stat().st_mtime)
            if time.time()-probe.stat().st_mtime > 60:
                raise RuntimeError('Refresh probe; latest is older than 60 seconds')
        start = read_probe(probe)
        raw = json.loads(probe.read_text(encoding='utf-8'))
        if any(m['state']['status'] != 0 for m in raw['motors']):
            raise RuntimeError('Expected disabled-motor probe')
        if np.max(np.abs(start)) > .1:
            raise RuntimeError('This candidate search is limited to current near-zero posture')
        bundle = RunBundle(args.run, load_policy=False)
        report.update(source_probe=str(probe), source_probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),
                      model_sha256=bundle.env.task['model_sha256'], config_sha256=bundle.env.task['config_sha256'],
                      q_start_rad=start.tolist(), q_A_rad=bundle.q_a.tolist(), intended_gain_scale=.2,
                      generated_at=datetime.now().astimezone().isoformat())
        rejected = []
        for shoulder in (.04, .06, .08):
            for wrist in (.08, .10, .12):
                try:
                    candidate = screen_candidate(bundle, start, shoulder, wrist)
                    report.update(candidate, passed=True, shoulder_delta_rad=shoulder, wrist_delta_rad=wrist)
                    break
                except RuntimeError as exc:
                    rejected.append({'shoulder_delta_rad': shoulder, 'wrist_delta_rad': wrist, 'reason': str(exc)})
            if report['passed']:
                break
        report['rejected_candidates'] = rejected
        if not report['passed']:
            report['error'] = 'No candidate satisfied the restricted model screening rules'
    except (RuntimeError, ValueError, OSError) as exc:
        report['error'] = str(exc)
    finally:
        if bundle:
            bundle.env.close()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('endpoints', 'rejected_candidates')}, ensure_ascii=False, indent=2))
    print('REPORT:', output)
    if args.render and report['passed']:
        import mujoco
        from mujoco import viewer as mj_viewer
        preview = RunBundle(args.run, load_policy=False)
        try:
            with mj_viewer.launch_passive(preview.env.model, preview.env.data) as window:
                window.cam.lookat[:] = [0., -.2, .3]
                window.cam.distance = 1.4
                window.cam.azimuth, window.cam.elevation = 0., -15.
                for segment in report['segments']:
                    print('PREVIEW ONLY:', segment['name'], flush=True)
                    begin = time.monotonic()
                    while window.is_running():
                        elapsed = min(time.monotonic()-begin, segment['duration_s'])
                        q, velocity = minimum_jerk(np.array(segment['start_q_rad']),
                                                   np.array(segment['end_q_rad']), elapsed, segment['duration_s'])
                        preview.update_state(q, velocity)
                        window.sync()
                        if elapsed >= segment['duration_s']:
                            break
                        time.sleep(.02)
                    if not window.is_running():
                        break
        finally:
            preview.env.close()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
