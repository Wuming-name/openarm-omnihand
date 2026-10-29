"""Bounded commissioning: training PD + bias feedforward, fixed measured pose.

Default mock mode never opens CAN. Live mode requires --backend socketcan
--execute and a physically supported arm. TIMEOUT is not modified or certified.
On exit, live mode attempts to disable all motors enabled by this test.
"""
import argparse
from collections import deque
from datetime import datetime
import json
import math
import multiprocessing as mp
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
from .startup_scope import classify_start_pose
from rl_ppo.model import contacts


class HoldViolation(RuntimeError):
    def __init__(self, message, details):
        super().__init__(message)
        self.details = details


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
    def __init__(self, bundle, hardware, q, velocity, now, stationary_baseline=False, gain_scale=1.):
        if not math.isfinite(gain_scale) or not .1 <= gain_scale <= 1.:
            raise ValueError('Commissioning gain scale must be 0.1..1.0')
        self.bundle = bundle
        self.overlap_guard = None
        self.initial_overlaps = []
        self.stationary_baseline = stationary_baseline
        bundle.update_state(q, velocity)
        if stationary_baseline:
            try:
                classify_start_pose(q)
            except ValueError as exc:
                raise RuntimeError(str(exc)) from exc
            self.initial_overlaps = contacts(bundle.env.model, bundle.env.data, bundle.env.physics['contact_tolerance_m'])
            self.overlap_guard = StationaryOverlapGuard(self.initial_overlaps)
        else:
            bundle.check_state(startup=True)
        self.target = np.asarray(q, dtype=float).copy()
        if np.any(self.target < bundle.hard_low) or np.any(self.target > bundle.hard_high):
            raise RuntimeError('Initial target outside model hard limits')
        physics = dict(bundle.env.physics)
        physics['kp'] = [gain_scale*x for x in bundle.env.physics['kp']]
        physics['kd'] = [gain_scale*x for x in bundle.env.physics['kd']]
        self.servo = Servo(hardware, physics, bundle.hard_low, bundle.hard_high)
        self.started, self.last_reference = now, -math.inf
        self.max_drift, self.max_torque = 0., np.zeros(7)

    def reference(self, q, velocity):
        """Expensive model work; live caller runs this in a separate process."""
        b = self.bundle
        b.update_state(q, velocity)
        if self.overlap_guard is None:
            b.check_state(startup=True)
        else:
            if np.any(np.asarray(q) < b.hard_low) or np.any(np.asarray(q) > b.hard_high):
                raise RuntimeError('Measured joint outside model hard limits')
            self.overlap_guard.check(contacts(b.env.model, b.env.data, b.env.physics['contact_tolerance_m']))
        self.check_measured(q, velocity)
        return b.feedforward()

    def check_measured(self, q, velocity):
        q, velocity = np.asarray(q), np.asarray(velocity)
        if q.shape != (7,) or velocity.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(velocity).all():
            raise RuntimeError('Malformed/nonfinite measured state')
        if np.any(q < self.bundle.hard_low) or np.any(q > self.bundle.hard_high):
            raise RuntimeError('Measured joint outside model hard limits')
        drift = float(np.max(np.abs(np.asarray(q)-self.target)))
        drift_limit = .005 if self.stationary_baseline else .05
        speed_limit = .1 if self.stationary_baseline else .15
        # Include the rejected sample, not just previously accepted samples.
        self.max_drift = max(self.max_drift, drift)
        if drift > drift_limit or np.max(np.abs(velocity)) > speed_limit:
            drift_axis = int(np.argmax(np.abs(q-self.target)))
            speed_axis = int(np.argmax(np.abs(velocity)))
            details = {'source': 'measured_state_guard', 'q_rad': q.tolist(),
                       'target_q_rad': self.target.tolist(), 'velocity_rad_s': velocity.tolist(),
                       'position_errors_rad': (q-self.target).tolist(),
                       'max_drift_rad': drift, 'drift_axis': drift_axis+1,
                       'max_abs_speed_rad_s': float(np.max(np.abs(velocity))), 'speed_axis': speed_axis+1,
                       'drift_limit_rad': drift_limit, 'speed_limit_rad_s': speed_limit,
                       'drift_exceeded': bool(drift > drift_limit),
                       'speed_exceeded': bool(np.max(np.abs(velocity)) > speed_limit)}
            raise HoldViolation(f"Measured guard: J{drift_axis+1} drift={drift:.7f}rad (limit {drift_limit}); "
                                f"J{speed_axis+1} speed={float(velocity[speed_axis]):.7f}rad/s (limit ±{speed_limit})", details)

    def step_fast(self, q, velocity, now, fresh_ff=None):
        """500Hz measured-state checks and PD only; no MuJoCo or printing."""
        self.check_measured(q, velocity)
        if fresh_ff is not None:
            self.servo.accept(self.target.tolist(), [0.]*7, list(fresh_ff), list(q), now)
        torque = self.servo.tick(list(q), list(velocity), now)
        if self.servo.fault:
            raise RuntimeError('Servo latched fault: '+self.servo.fault)
        self.max_torque = np.maximum(self.max_torque, np.abs(torque))
        return torque

    def step(self, q, velocity, now):
        b = self.bundle
        ff = self.reference(q, velocity)
        fresh_ff = None
        if now-self.last_reference >= b.env.dt-1e-8:
            # Match the trained nominal controller immediately. A ramp from zero
            # FF would remove gravity support and change the closed-loop test.
            fresh_ff = ff
            self.last_reference = now
        return self.step_fast(q, velocity, now, fresh_ff)


def _model_worker(connection, run, hardware, q, velocity, stationary_baseline):
    bundle = None
    try:
        bundle = RunBundle(run, load_policy=False)
        controller = HoldController(bundle, hardware, q, velocity, 0., stationary_baseline)
        while True:
            begin = time.perf_counter()
            ff = controller.reference(q, velocity)
            connection.send({'ff': ff.tolist(), 'computed': time.monotonic(),
                             'model_compute_s': time.perf_counter()-begin})
            request = connection.recv()
            if request is None:
                break
            q, velocity = request
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({'error': str(exc), 'stop_details': getattr(exc, 'details', None)})
        except (EOFError, BrokenPipeError):
            pass
    finally:
        if bundle:
            bundle.env.close()
        connection.close()


class ModelWorker:
    """One outstanding small request: no unbounded queue or CAN in child."""
    def __init__(self, run, hardware, q, velocity, stationary_baseline, worker_target=None, worker_args=()):
        ctx = mp.get_context('spawn')
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=worker_target or _model_worker, args=(child, str(Path(run).resolve()), hardware,
                                   list(q), list(velocity), stationary_baseline, *worker_args), daemon=True)
        self.pending = True
        self.latest = None
        self.process.start()
        child.close()
        if not self.connection.poll(15.):
            self.close()
            raise RuntimeError('Model worker initialization timed out before motor enable')
        try:
            self.receive()
        except BaseException:
            self.close()
            raise

    def receive(self):
        if self.connection.poll():
            self.latest = self.connection.recv()
            self.pending = False
        if self.latest and 'error' in self.latest:
            details = self.latest.get('stop_details') or {}
            details['source'] = 'model_worker'
            raise HoldViolation('Model worker: '+self.latest['error'], details)
        return self.latest

    def request(self, q, velocity):
        if not self.pending:
            self.connection.send((list(q), list(velocity)))
            self.pending = True

    def close(self):
        try:
            self.connection.close()
        finally:
            self.process.join(timeout=.3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--backend', choices=['mock', 'socketcan'], default='mock')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--gain-scale', type=float, default=1., help='Commissioning PD gain fraction, 0.1..1; does not change saved training files or gravity FF')
    parser.add_argument('--stationary-baseline', action='store_true', help='Near-zero fixed-pose test only; monitor known initial model overlaps without certifying them safe')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 5:
        parser.error('--seconds must be 2..5')
    if not math.isfinite(args.gain_scale) or not .1 <= args.gain_scale <= 1.:
        parser.error('--gain-scale must be 0.1..1.0')
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
              'gain_scale': args.gain_scale,
              'training_gains_unchanged': args.gain_scale == 1.,
              'collision_model_validated_for_motion': False}
    bundle = bus = controller = worker = None
    own_enable = False
    intervals, observed = [], [0]*7
    previous_received = [None]*7
    steps = 0
    costs = {'receive_s': [], 'controller_s': [], 'send_s': [], 'cycle_work_s': [], 'model_compute_s': []}
    progress = []
    trace = deque(maxlen=500)
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
        controller = HoldController(bundle, hardware, q, velocity, start, args.stationary_baseline, args.gain_scale)
        report['initial_model_overlaps'] = controller.initial_overlaps
        report['target_q_rad'] = controller.target.tolist()
        report['kp'] = controller.servo.physics['kp']
        report['kd'] = controller.servo.physics['kd']
        report['training_kp'] = bundle.env.physics['kp']
        report['training_kd'] = bundle.env.physics['kd']
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if live:
            # Initialize and warm model process BEFORE enabling any motor.
            worker = ModelWorker(args.run, hardware, q, velocity, args.stationary_baseline)
            bus.states.clear()
            bus.refresh()
            deadline = time.monotonic()+.5
            while len(bus.states) < 7 and time.monotonic() < deadline:
                bus.drain()
                time.sleep(.001)
            states = snapshot(bus, hardware, time.monotonic())
            if any(s['status'] != 0 for s in states):
                raise RuntimeError('Motor enabled by another controller during model preparation')
            q, velocity = [s['q'] for s in states], [s['velocity'] for s in states]
            controller.check_measured(q, velocity)
            worker.request(q, velocity)
            if not worker.connection.poll(.5):
                raise RuntimeError('Model worker warmup failed before motor enable')
            worker.receive()
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
        last_model_request, last_generation = -math.inf, None
        period = 1/hardware['servo_hz']
        while True:
            now = time.monotonic() if live else start+steps*period
            if now-start >= args.seconds:
                break
            begin_receive = time.perf_counter()
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
            trace.append({'elapsed_s': now-start, 'q_rad': list(q), 'velocity_rad_s': list(velocity),
                          'commanded_torque_nm': None})
            begin_controller = time.perf_counter()
            if live:
                packet = worker.receive()
                if not packet or time.monotonic()-packet['computed'] > hardware['command_timeout_s']:
                    raise RuntimeError('Model reference/collision monitor stale; control stopped')
                fresh_ff = None
                if packet['computed'] != last_generation:
                    fresh_ff = packet['ff']
                    last_generation = packet['computed']
                    costs['model_compute_s'].append(packet['model_compute_s'])
                if now-last_model_request >= bundle.env.dt and not worker.pending:
                    worker.request(q, velocity)
                    last_model_request = now
                torque = controller.step_fast(q, velocity, now, fresh_ff)
            else:
                torque = controller.step(q, velocity, now)
            begin_send = time.perf_counter()
            trace[-1]['commanded_torque_nm'] = list(torque)
            if live:
                bus.torques(torque)
            else:
                bundle.env.data.ctrl[:] = torque
                mujoco.mj_step(bundle.env.model, bundle.env.data)
                q, velocity = bundle.env.data.qpos.copy(), bundle.env.data.qvel.copy()
            end_work = time.perf_counter()
            costs['receive_s'].append(begin_controller-begin_receive)
            costs['controller_s'].append(begin_send-begin_controller)
            costs['send_s'].append(end_work-begin_send)
            costs['cycle_work_s'].append(end_work-begin_receive)
            if steps:
                intervals.append(now-previous)
            previous = now
            steps += 1
            if now >= next_log:
                # Never synchronously print inside the live servo cadence.
                progress.append({'elapsed_s': now-start, 'max_drift_rad': controller.max_drift})
                next_log = now+.5
            if live:
                # Absolute cadence; never burst old commands to catch up.
                remaining = start+steps*period-time.monotonic()
                if remaining < -period:
                    report['deadline_overrun_s'] = -remaining
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
    except (RuntimeError, ValueError, OSError, EOFError, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        if hasattr(exc, 'details'):
            report['stop_details'] = exc.details
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
        if worker:
            worker.close()
        # Preserve partial diagnostics even when a timing fault stops early.
        report.update(cycles=steps,
                      measured_loop_hz=1/float(np.mean(intervals)) if intervals else None,
                      max_loop_gap_s=max(intervals) if intervals else None,
                      observed_feedback_updates=observed if live else None,
                      model_reference_updates=len(costs['model_compute_s']),
                      progress=progress,
                      timing_costs={key: {'max_s': max(values), 'p95_s': float(np.percentile(values, 95)),
                                         'mean_s': float(np.mean(values))} for key, values in costs.items() if values})
        if controller:
            report['max_drift_rad'] = controller.max_drift
            report['max_commanded_torque_nm'] = controller.max_torque.tolist()
        if bundle:
            bundle.env.close()
        output = Path('outputs/real_robot') / (datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_commission_hold.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        trace_path = output.with_name(output.stem+'_trace.json')
        trace_path.write_text(json.dumps({'note': 'Last at most 500 cycles; rejected sample may have no new torque command',
                                         'gain_scale': args.gain_scale, 'samples': list(trace)}, indent=2)+'\n', encoding='utf-8')
        report['trace_file'] = str(trace_path)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print('REPORT:', output, flush=True)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
