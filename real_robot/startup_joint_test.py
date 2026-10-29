"""Only J2: slow planned outward move and return; NOT full A/B or PPO execution.

Preview default does not open CAN. --execute requires a supported arm and an
explicitly screened plan; motors must initially be disabled. Disable on exit.
"""
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np

from . import protocol as p
from .can_bus import MotorBus
from .commission_hold import HoldController, HoldViolation, ModelWorker, StationaryOverlapGuard
from .control import RunBundle, minimum_jerk
from .hold_test import snapshot
from .plan_startup import model_contacts
from .settings import DEFAULT, load
from .startup_scope import classify_start_pose
from rl_ppo.model import contacts


def validate_first_segment(plan, bundle):
    if not plan.get('passed') or plan.get('mode') != 'offline_startup_path_screening':
        raise ValueError('Expected successful offline startup screening report')
    if plan['model_sha256'] != bundle.env.task['model_sha256'] or plan['config_sha256'] != bundle.env.task['config_sha256']:
        raise ValueError('Plan belongs to a different trained model/configuration')
    if plan.get('intended_gain_scale') != .2:
        raise ValueError('This test supports only commissioned 20% gains')
    segment = plan['segments'][0]
    start = np.asarray(plan['q_start_rad'], dtype=float)
    end = np.asarray(segment['end_q_rad'], dtype=float)
    duration = float(segment['duration_s'])
    if start.shape != (7,) or end.shape != (7,) or not np.isfinite(start).all() or not np.isfinite(end).all():
        raise ValueError('Malformed plan joint vectors')
    classify_start_pose(start)
    if segment['name'] != 'shoulder_retreat' or not 6 <= duration <= 10:
        raise ValueError('Plan outside limited J2 test scope')
    if not np.allclose(segment['start_q_rad'], start, atol=1e-10, rtol=0):
        raise ValueError('Plan start vectors disagree')
    delta = end-start
    if not .02 <= delta[1] <= .08 or np.max(np.abs(delta[[0,2,3,4,5,6]])) > 1e-10:
        raise ValueError('Only positive J2 increments 0.02..0.08rad are supported')
    initial = model_contacts(bundle, start)
    guard = StationaryOverlapGuard(initial)
    # Entire outbound AND return corridor is explicitly re-screened. Return may
    # restore initial virtual overlaps; depth cannot exceed initial +0.5mm.
    for a, b in ((start, end), (end, start)):
        for u in np.linspace(0., 1., 161):
            guard.check(model_contacts(bundle, a+u*(b-a)))
    return start, end, duration, initial


def reference_at(start, end, duration, elapsed):
    zero = np.zeros(7)
    if elapsed < 1.:
        return start.copy(), zero, 'INITIAL_HOLD'
    if elapsed < 1.+duration:
        q, v = minimum_jerk(start, end, elapsed-1., duration)
        return q, v, 'J2_OUTWARD'
    if elapsed < 2.+duration:
        return end.copy(), zero, 'END_HOLD'
    if elapsed < 2.+2*duration:
        q, v = minimum_jerk(end, start, elapsed-2.-duration, duration)
        return q, v, 'J2_RETURN'
    return start.copy(), zero, 'RETURN_HOLD'


class TrackingController(HoldController):
    def __init__(self, bundle, hardware, start, end, j2_gain_scale=.2, endpoint_only=False):
        if j2_gain_scale not in (.2, .4):
            raise ValueError('J2 commissioning gain scale must be 0.2 or 0.4')
        super().__init__(bundle, hardware, start, np.zeros(7), 0., True, .2)
        # Only the moving axis changes; saved training parameters stay intact.
        self.servo.physics['kp'][1] = bundle.env.physics['kp'][1]*j2_gain_scale
        self.servo.physics['kd'][1] = bundle.env.physics['kd'][1]*j2_gain_scale
        self.start, self.end = start.copy(), end.copy()
        self.endpoint_only = endpoint_only

    def accept_motion(self, packet, q, velocity, now):
        command, v = np.asarray(packet['command']), np.asarray(packet['velocity_command'])
        if command.shape != (7,) or v.shape != (7,) or not np.isfinite(command).all() or not np.isfinite(v).all():
            raise RuntimeError('Malformed motion reference')
        if (np.any(command < np.minimum(self.start,self.end)-1e-9)
                or np.any(command > np.maximum(self.start,self.end)+1e-9)
                or np.max(np.abs(v[[0,2,3,4,5,6]])) > 1e-9 or abs(v[1]) > .03):
            raise RuntimeError('Reference outside bounded J2 corridor')
        self.target = command.copy()
        self.check_measured(q, velocity)
        self.servo.accept(command.tolist(), v.tolist(), packet['ff'], list(q), now)

    def check_measured(self, q, velocity):
        q, velocity = np.asarray(q), np.asarray(velocity)
        if q.shape != (7,) or velocity.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(velocity).all():
            raise RuntimeError('Malformed measured state')
        if np.any(q < self.bundle.hard_low) or np.any(q > self.bundle.hard_high):
            raise RuntimeError('Measured state outside model hard limits')
        errors = np.abs(q-self.target)
        limits = np.full(7,.005)
        limits[1] = .02  # Moving-axis tracking window; other six stay fixed.
        self.max_drift = max(self.max_drift,float(np.max(errors)))
        checked = np.ones(7,dtype=bool)
        if self.endpoint_only:
            checked[1] = False
        if np.any(errors[checked] > limits[checked]) or np.max(np.abs(velocity)) > .1:
            raise HoldViolation('Bounded J2 tracking or speed limit exceeded',
                                {'q_rad':q.tolist(),'target_q_rad':self.target.tolist(),
                                 'position_errors_rad':(q-self.target).tolist(),
                                 'tracking_limits_rad':limits.tolist(),'velocity_rad_s':velocity.tolist(),
                                 'tracking_check_axes':(np.flatnonzero(checked)+1).tolist(),
                                 'speed_limit_rad_s':.1})
        if np.any(q < np.minimum(self.start,self.end)-.005) or np.any(q > np.maximum(self.start,self.end)+.005):
            raise RuntimeError('Measured pose outside bounded J2 corridor')

    def check_outward_endpoint(self, q):
        error = float(abs(np.asarray(q)[1]-self.end[1]))
        if not np.isfinite(error) or error > .02:
            raise HoldViolation('J2 outward endpoint error exceeds 0.02rad after hold',
                                {'endpoint':'outward','actual_j2_rad':float(q[1]),
                                 'target_j2_rad':float(self.end[1]),
                                 'endpoint_error_rad':error,'endpoint_limit_rad':.02})
        return error


def _startup_worker(connection, run, hardware, q, velocity, stationary_baseline, end, duration):
    bundle = None
    try:
        bundle = RunBundle(run, load_policy=False)
        start = np.asarray(q)
        guard = StationaryOverlapGuard(model_contacts(bundle,start))
        begin_motion = None
        while True:
            begin = time.perf_counter()
            q = np.asarray(q)
            bundle.update_state(q,velocity)
            if np.any(q < bundle.hard_low) or np.any(q > bundle.hard_high):
                raise RuntimeError('Measured joint outside model hard limits')
            found = contacts(bundle.env.model,bundle.env.data,bundle.env.physics['contact_tolerance_m'])
            try:
                guard.check(found)
            except RuntimeError as exc:
                raise HoldViolation(str(exc), {'q_rad':q.tolist(),'contacts':found}) from exc
            if np.any(q < np.minimum(start,end)-.005) or np.any(q > np.maximum(start,end)+.005):
                raise RuntimeError('Model worker: measured pose left J2 corridor')
            elapsed = max(0.,time.monotonic()-begin_motion) if begin_motion is not None else 0.
            command, v, phase = reference_at(start,end,duration,elapsed)
            connection.send({'ff': bundle.feedforward().tolist(), 'command': command.tolist(),
                             'velocity_command': v.tolist(), 'phase': phase, 'elapsed_s': elapsed,
                             'computed': time.monotonic(), 'model_compute_s': time.perf_counter()-begin})
            request = connection.recv()
            if request is None:
                break
            if isinstance(request, dict):
                q, velocity, begin_motion = request['q'], request['velocity'], request['start_time']
            else:
                q, velocity = request
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({'error': str(exc), 'stop_details': getattr(exc,'details',None)})
        except (EOFError,BrokenPipeError):
            pass
    finally:
        if bundle:
            bundle.env.close()
        connection.close()


def fresh_disabled(bus, hardware):
    bus.states.clear()
    bus.refresh()
    end = time.monotonic()+.5
    while len(bus.states) < 7 and time.monotonic() < end:
        bus.drain()
        time.sleep(.001)
    states = snapshot(bus,hardware,time.monotonic())
    if any(s['status'] != 0 for s in states):
        raise RuntimeError('All seven motors must initially be disabled; no takeover')
    return [s['q'] for s in states], [s['velocity'] for s in states]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--plan', default='outputs/real_robot/startup_plan.json')
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--j2-gain-scale', type=float, choices=(.2, .4), default=.2,
                        help='J2-only commissioning PD fraction; other six axes remain at 0.2')
    parser.add_argument('--endpoint-only', action='store_true',
                        help='Check J2 accuracy after outward hold and final return, not along the moving reference')
    args = parser.parse_args()
    hardware = load(args.config)
    report = {'test': 'only_J2_out_and_return', 'passed': False, 'gain_scale': .2,
              'j2_gain_scale': args.j2_gain_scale,
              'j2_accuracy_mode': 'endpoints' if args.endpoint_only else 'continuous',
              'outward_tolerance_rad': .02, 'return_tolerance_rad': .005,
              'joint_gain_scales': [.2,args.j2_gain_scale,.2,.2,.2,.2,.2],
              'hardware_commands_attempted': False, 'disable_attempts': [],
              'is_ppo_test': False, 'initial_model_overlap_allowance': True,
              'collision_model_validated_for_full_motion': False}
    bundle = bus = worker = controller = None
    own_enable = False
    intervals, trace, updates = [], deque(maxlen=500), [0]*7
    previous_stamps, phases, steps = [None]*7, [], 0
    max_feedback_torque = np.zeros(7)
    try:
        bundle = RunBundle(args.run, load_policy=False)
        plan = json.loads(Path(args.plan).read_text(encoding='utf-8'))
        start, end, duration, initial = validate_first_segment(plan,bundle)
        report['kp'] = [.2*x for x in bundle.env.physics['kp']]
        report['kd'] = [.2*x for x in bundle.env.physics['kd']]
        report['kp'][1] = args.j2_gain_scale*bundle.env.physics['kp'][1]
        report['kd'][1] = args.j2_gain_scale*bundle.env.physics['kd'][1]
        report.update(start_scope=classify_start_pose(start),
                      start_q_rad=start.tolist(), outward_q_rad=end.tolist(), leg_duration_s=duration,
                      total_duration_s=3.+2*duration, initial_model_overlaps=initial,
                      model_sha256=bundle.env.task['model_sha256'])
        if args.execute:
            bus = MotorBus(hardware,allow_control=True)
            bus.inspect()
            bus.validate_parameters(live=False)
            q, velocity = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.asarray(q)-start)) > .002:
                raise RuntimeError('Measured start differs from plan by >0.002rad; refresh probe and plan')
            controller = TrackingController(bundle,hardware,start,end,args.j2_gain_scale,args.endpoint_only)
            controller.check_measured(q,velocity)
            worker = ModelWorker(args.run,hardware,start,np.zeros(7),True,
                                 worker_target=_startup_worker,worker_args=(end,duration))
            q, velocity = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.asarray(q)-start)) > .002:
                raise RuntimeError('Start moved during worker preparation; re-plan')
            worker.request(q,velocity)
            if not worker.connection.poll(.5):
                raise RuntimeError('Model worker warmup timed out')
            worker.receive()
            print('SUPPORTED ARM ONLY: J2 slow outward AND RETURN. Disable attempted on exit.',flush=True)
            report['hardware_commands_attempted'] = True
            bus.torques([0.]*7)
            own_enable = True
            bus.enable()
            deadline = time.monotonic()+.1
            while time.monotonic() < deadline:
                bus.refresh()
                time.sleep(.002)
                bus.drain()
                states = snapshot(bus,hardware,time.monotonic())
                if all(s['status'] == 1 for s in states):
                    break
            else:
                raise RuntimeError('All-seven enable not acknowledged')
            begin = previous = time.monotonic()
            worker.connection.send({'q':q,'velocity':velocity,'start_time':begin})
            worker.pending = True
            last_request, generation = -1., None
            while time.monotonic()-begin < 3.+2*duration:
                now = time.monotonic()
                bus.drain()
                states = snapshot(bus,hardware,time.monotonic())
                if any(s['status'] != 1 for s in states):
                    raise RuntimeError('Motor enable lost')
                q, velocity = [s['q'] for s in states], [s['velocity'] for s in states]
                feedback_torque = [s['torque'] for s in states]
                max_feedback_torque = np.maximum(max_feedback_torque,np.abs(feedback_torque))
                for i,s in enumerate(states):
                    if s['received_monotonic'] != previous_stamps[i]:
                        updates[i] += 1
                        previous_stamps[i] = s['received_monotonic']
                packet = worker.receive()
                if not packet or time.monotonic()-packet['computed'] > .1:
                    raise RuntimeError('Motion/model reference stale')
                trace.append({'elapsed_s':now-begin,'q_rad':q,'velocity_rad_s':velocity,
                              'feedback_torque_nm':feedback_torque,
                              'command_q_rad':packet['command'],'commanded_torque_nm':None})
                if packet['computed'] != generation:
                    if (args.endpoint_only and packet['phase'] in ('J2_RETURN','RETURN_HOLD')
                            and 'outward_endpoint_error_rad' not in report):
                        report['outward_endpoint_error_rad'] = controller.check_outward_endpoint(q)
                    controller.accept_motion(packet,q,velocity,now)
                    generation = packet['computed']
                    if not phases or phases[-1]['phase'] != packet['phase']:
                        phases.append({'phase':packet['phase'],'elapsed_s':now-begin})
                if now-last_request >= .02 and not worker.pending:
                    worker.request(q,velocity)
                    last_request = now
                torque = controller.step_fast(q,velocity,now)
                trace[-1]['commanded_torque_nm'] = torque
                bus.torques(torque)
                if steps:
                    intervals.append(now-previous)
                previous = now
                steps += 1
                remaining = begin+steps*.002-time.monotonic()
                if remaining < -.002:
                    raise RuntimeError('Cannot sustain 500Hz; motion stopped')
                if remaining > 0:
                    time.sleep(remaining)
            report['return_error_rad'] = float(np.max(np.abs(np.asarray(q)-start)))
            if args.endpoint_only and 'outward_endpoint_error_rad' not in report:
                raise RuntimeError('Outward endpoint was not validated; cannot report success')
            if report['return_error_rad'] > .005:
                raise RuntimeError('Did not return to planned initial pose')
            if not intervals or 1/np.mean(intervals) < 475 or min(updates) < .8*steps:
                raise RuntimeError('Loop/feedback update rate below acceptance threshold')
            report['passed'] = True
        else:
            report['preview_only'] = True
            report['note'] = 'Only verified the first outbound/return model corridor. No CAN opened; add --execute only with supported arm.'
    except (RuntimeError,ValueError,OSError,EOFError,KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        if hasattr(exc,'details'):
            report['stop_details'] = exc.details
    finally:
        if bus:
            if own_enable:
                for motor in hardware['motors']:
                    result = {'motor':motor['tx'],'sent':False}
                    try:
                        bus.send(motor['tx'],p.disable_payload())
                        result['sent'] = True
                    except (RuntimeError,ValueError,OSError) as exc:
                        result['error'] = str(exc)
                    report['disable_attempts'].append(result)
                report['disable_delivery_verified'] = False
            bus.close()
        if worker:
            worker.close()
        report.update(cycles=steps, measured_loop_hz=float(1/np.mean(intervals)) if intervals else None,
                      max_loop_gap_s=max(intervals) if intervals else None, observed_feedback_updates=updates,
                      phases=phases)
        if controller:
            report['max_tracking_error_rad'] = controller.max_drift
            report['max_commanded_torque_nm'] = controller.max_torque.tolist()
        report['max_abs_feedback_torque_nm'] = max_feedback_torque.tolist()
        if bundle:
            bundle.env.close()
        output = Path('outputs/real_robot')/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_startup_J2.json')
        output.parent.mkdir(parents=True,exist_ok=True)
        trace_path = output.with_name(output.stem+'_trace.json')
        trace_path.write_text(json.dumps(list(trace),indent=2)+'\n',encoding='utf-8')
        report['trace_file'] = str(trace_path)
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('REPORT:',output)
    return 0 if report.get('passed') or report.get('preview_only') else 2


if __name__ == '__main__':
    main()
