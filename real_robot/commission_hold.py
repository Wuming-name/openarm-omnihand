"""Bounded commissioning: training PD + bias feedforward, fixed measured pose.

Default mock mode never opens CAN. Live mode requires --backend socketcan
--execute and a physically supported arm. TIMEOUT is not modified or certified.
On exit, live mode attempts to disable all motors enabled by this test.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from . import protocol as p
from .can_bus import MotorBus
from .control import RunBundle
from .hold_test import snapshot
from .servo import Servo
from .settings import DEFAULT, load
from rl_ppo.model import contacts


class StationaryOverlapGuard:
    """Narrow stationary commissioning allowance, NOT a motion collision model."""
    ALLOWED = {
        frozenset(('collision_openarm_body_link0_0', f'collision_omni_R_thumb_{part}_0')): .025
        for part in ('mcp', 'pip', 'dip')
    }
    ALLOWED[frozenset(('collision_openarm_right_link5_0_piece0', 'collision_openarm_right_link7_0_piece7'))] = .004

    def __init__(self, initial):
        self.depths = {}
        for item in initial:
            pair = frozenset((item['a'], item['b']))
            depth = -float(item['distance_m'])
            if pair not in self.ALLOWED or not math.isfinite(depth) or depth > self.ALLOWED[pair]:
                raise RuntimeError('Initial overlap outside stationary commissioning scope')
            self.depths[pair] = max(depth, self.depths.get(pair, 0.))

    def check(self, current):
        for item in current:
            pair = frozenset((item['a'], item['b']))
            depth = -float(item['distance_m'])
            if pair not in self.depths or not math.isfinite(depth) or depth > self.depths[pair]+.0005:
                raise RuntimeError('New contact or initial model overlap worsened >0.5mm')


class HoldController:
    def __init__(self, bundle, hardware, q, velocity, now, stationary_baseline=False):
        self.bundle = bundle
        self.overlap_guard = None
        self.initial_overlaps = []
        self.stationary_baseline = stationary_baseline
        bundle.update_state(q, velocity)
        if stationary_baseline:
            if np.max(np.abs(q)) > .1:
                raise RuntimeError('Stationary baseline is limited to current near-zero pose, all |q|<=0.1rad')
            self.initial_overlaps = contacts(bundle.env.model, bundle.env.data, bundle.env.physics['contact_tolerance_m'])
            self.overlap_guard = StationaryOverlapGuard(self.initial_overlaps)
        else:
            bundle.check_state(startup=True)
        self.target = np.asarray(q, dtype=float).copy()
        if np.any(self.target < bundle.hard_low) or np.any(self.target > bundle.hard_high):
            raise RuntimeError('Initial target outside model hard limits')
        self.servo = Servo(hardware, bundle.env.physics, bundle.hard_low, bundle.hard_high)
        self.started, self.last_reference = now, -math.inf
        self.max_drift, self.max_torque = 0., np.zeros(7)

    def step(self, q, velocity, now):
        b = self.bundle
        b.update_state(q, velocity)
        if self.overlap_guard is None:
            b.check_state(startup=True)
        else:
            if np.any(np.asarray(q) < b.hard_low) or np.any(np.asarray(q) > b.hard_high):
                raise RuntimeError('Measured joint outside model hard limits')
            self.overlap_guard.check(contacts(b.env.model, b.env.data, b.env.physics['contact_tolerance_m']))
        drift = float(np.max(np.abs(np.asarray(q)-self.target)))
        drift_limit = .005 if self.stationary_baseline else .05
        speed_limit = .1 if self.stationary_baseline else .15
        if drift > drift_limit or np.max(np.abs(velocity)) > speed_limit:
            raise RuntimeError(f'Supported hold drift >{drift_limit}rad or speed >{speed_limit}rad/s')
        self.max_drift = max(self.max_drift, drift)
        if now-self.last_reference >= b.env.dt-1e-8:
            # Match the trained nominal controller immediately. A ramp from zero
            # FF would remove gravity support and change the closed-loop test.
            ff = b.feedforward()
            self.servo.accept(self.target.tolist(), [0.]*7, ff.tolist(), list(q), now)
            self.last_reference = now
        torque = self.servo.tick(list(q), list(velocity), now)
        if self.servo.fault:
            raise RuntimeError('Servo latched fault: '+self.servo.fault)
        self.max_torque = np.maximum(self.max_torque, np.abs(torque))
        return torque


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--backend', choices=['mock', 'socketcan'], default='mock')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--stationary-baseline', action='store_true', help='Near-zero fixed-pose test only; monitor known initial model overlaps without certifying them safe')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 5:
        parser.error('--seconds must be 2..5')
    live = args.backend == 'socketcan'
    if live and not args.execute:
        parser.error('Live test requires --execute and physically supported arm')
    if args.stationary_baseline and args.seconds > 3:
        parser.error('Stationary baseline test limited to at most 3 seconds')
    if args.stationary_baseline and not live:
        parser.error('Stationary baseline is a physical supported-pose test; mock integration retains original contact forces. Use ordinary mock for dynamics tests.')
    hardware = load(args.config)
    report = {'backend': args.backend, 'timing_clock': 'monotonic' if live else 'simulation', 'TIMEOUT_changed': False,
              'watchdog_verified': False, 'is_ppo_test': False, 'passed': False,
              'hardware_commands_attempted': False, 'disable_attempts': [],
              'stationary_baseline': args.stationary_baseline,
              'collision_model_validated_for_motion': False}
    bundle = bus = controller = None
    own_enable = False
    intervals, observed = [], [0]*7
    previous_received = [None]*7
    try:
        bundle = RunBundle(args.run, load_policy=False)
        if live:
            bus = MotorBus(hardware, allow_control=True)
            bus.inspect()
            bus.validate_parameters(live=False)
            bus.states.clear()
            bus.refresh()
            deadline = time.monotonic()+.5
            while len(bus.states) < 7 and time.monotonic() < deadline:
                bus.drain()
                time.sleep(.001)
            states = snapshot(bus, hardware, time.monotonic())
            if any(s['status'] != 0 for s in states):
                raise RuntimeError('All motors must initially be disabled; no takeover of another controller')
            q, velocity = [s['q'] for s in states], [s['velocity'] for s in states]
        else:
            q, velocity = bundle.env.data.qpos.copy(), bundle.env.data.qvel.copy()
        # Preserve the measured input even if collision preflight rejects it.
        report['measured_q_rad'] = np.asarray(q, dtype=float).tolist()
        report['measured_velocity_rad_s'] = np.asarray(velocity, dtype=float).tolist()
        report['model_sha256'] = bundle.env.task['model_sha256']
        start = time.monotonic() if live else 0.
        controller = HoldController(bundle, hardware, q, velocity, start, args.stationary_baseline)
        report['initial_model_overlaps'] = controller.initial_overlaps
        report['target_q_rad'] = controller.target.tolist()
        report['kp'] = bundle.env.physics['kp']
        report['kd'] = bundle.env.physics['kd']
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if live:
            print('SUPPORTED ARM: 500Hz software PD/MIT torque test, DISABLE on exit.', flush=True)
            # Clear previous firmware targets before the single initial enable.
            report['hardware_commands_attempted'] = True
            bus.torques([0.]*7)
            own_enable = True
            bus.enable()
            deadline = time.monotonic()+.1
            while time.monotonic() < deadline:
                bus.refresh()
                time.sleep(.002)
                bus.drain()
                states = snapshot(bus, hardware, time.monotonic())
                if all(s['status'] == 1 for s in states):
                    break
            else:
                raise RuntimeError('All-seven enable not acknowledged')
            # A first fresh state is required before sending any holding torque.
            states = snapshot(bus, hardware, time.monotonic())
            q, velocity = [s['q'] for s in states], [s['velocity'] for s in states]
            start = time.monotonic()
            controller.started = start
        previous, next_log, steps = start, start, 0
        period = 1/hardware['servo_hz']
        while True:
            now = time.monotonic() if live else start+steps*period
            if now-start >= args.seconds:
                break
            if live:
                bus.drain()
                states = snapshot(bus, hardware, time.monotonic())
                if any(s['status'] != 1 for s in states):
                    raise RuntimeError('Motor enable lost')
                for i, state in enumerate(states):
                    stamp = state['received_monotonic']
                    if stamp != previous_received[i]:
                        observed[i] += 1
                        previous_received[i] = stamp
                q, velocity = [s['q'] for s in states], [s['velocity'] for s in states]
            torque = controller.step(q, velocity, now)
            if live:
                bus.torques(torque)
            else:
                bundle.env.data.ctrl[:] = torque
                mujoco.mj_step(bundle.env.model, bundle.env.data)
                q, velocity = bundle.env.data.qpos.copy(), bundle.env.data.qvel.copy()
            if steps:
                intervals.append(now-previous)
            previous = now
            steps += 1
            if now >= next_log:
                print(f'HOLD_500HZ t={now-start:.2f}s max_drift={controller.max_drift:.5f}rad', flush=True)
                next_log = now+.5
            if live:
                # Absolute cadence; never burst old commands to catch up.
                remaining = start+steps*period-time.monotonic()
                if remaining < -period:
                    raise RuntimeError('Cannot sustain 500Hz; test stops rather than sends a catch-up burst')
                if remaining > 0:
                    time.sleep(remaining)
        report.update(cycles=steps,
                      measured_loop_hz=1/float(np.mean(intervals)) if intervals else None,
                      max_loop_gap_s=max(intervals) if intervals else None,
                      observed_feedback_updates=observed if live else None)
        if live and (report['measured_loop_hz'] < 475 or min(observed) < .8*steps):
            raise RuntimeError('Insufficient loop rate or fresh per-axis feedback updates for 500Hz acceptance')
        report['passed'] = True
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        print('STOP:', str(exc), flush=True)
    finally:
        if bus:
            if own_enable:
                for motor in hardware['motors']:
                    result = {'motor': motor['tx'], 'sent': False}
                    try:
                        bus.send(motor['tx'], p.disable_payload())
                        result['sent'] = True
                    except (RuntimeError, ValueError, OSError) as exc:
                        result['error'] = str(exc)
                    report['disable_attempts'].append(result)
                report['disable_delivery_verified'] = False
                print('Disable attempted; arm must remain supported. Delivery not guaranteed.', flush=True)
            bus.close()
        if controller:
            report['max_drift_rad'] = controller.max_drift
            report['max_commanded_torque_nm'] = controller.max_torque.tolist()
        if bundle:
            bundle.env.close()
        output = Path('outputs/real_robot') / (datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_commission_hold.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print('REPORT:', output, flush=True)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
