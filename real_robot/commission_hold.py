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


class HoldController:
    def __init__(self, bundle, hardware, q, velocity, now):
        self.bundle = bundle
        bundle.update_state(q, velocity)
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
        b.check_state(startup=True)
        drift = float(np.max(np.abs(np.asarray(q)-self.target)))
        if drift > .05 or np.max(np.abs(velocity)) > .15:
            raise RuntimeError('Supported hold drift >0.05rad or speed >0.15rad/s')
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
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 5:
        parser.error('--seconds must be 2..5')
    live = args.backend == 'socketcan'
    if live and not args.execute:
        parser.error('Live test requires --execute and physically supported arm')
    hardware = load(args.config)
    report = {'backend': args.backend, 'timing_clock': 'monotonic' if live else 'simulation', 'TIMEOUT_changed': False,
              'watchdog_verified': False, 'is_ppo_test': False, 'passed': False,
              'hardware_commands_attempted': False, 'disable_attempts': []}
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
        start = time.monotonic() if live else 0.
        controller = HoldController(bundle, hardware, q, velocity, start)
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
