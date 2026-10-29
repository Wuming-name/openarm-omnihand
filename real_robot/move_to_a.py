"""Planned startup to A using training PD with J5/J6 outer-loop Kd overrides.

No CAN without --execute. At A, maintain torque until Ctrl+C; exit attempts
disable, which cannot support the arm against gravity. No PPO or hand commands.
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
from .plan_startup import model_contacts, screen_candidate
from .settings import DEFAULT, load
from .startup_joint_test import fresh_disabled
from .startup_scope import classify_start_pose
from rl_ppo.model import contacts


def validate_plan(bundle, plan):
    if plan.get('mode') != 'offline_startup_path_screening' or not plan.get('passed'):
        raise ValueError('Successful startup plan required')
    for name in ('model_sha256', 'config_sha256'):
        if plan.get(name) != bundle.env.task[name]:
            raise ValueError('Startup plan does not match trained run: '+name)
    start = np.asarray(plan['q_start_rad'], dtype=float)
    classify_start_pose(start)
    shoulder, wrist = plan['shoulder_delta_rad'], plan['wrist_delta_rad']
    if shoulder not in (.04,.06,.08) or wrist not in (.08,.10,.12):
        raise ValueError('Startup escape increments outside screened scope')
    # Recompute all endpoints, durations and contacts instead of trusting JSON.
    verified = screen_candidate(bundle,start,shoulder,wrist)
    if len(plan['segments']) != 3:
        raise ValueError('Expected three startup segments')
    for saved, actual in zip(plan['segments'],verified['segments']):
        if saved['name'] != actual['name']:
            raise ValueError('Startup segment name mismatch')
        for key in ('start_q_rad','end_q_rad'):
            if not np.allclose(saved[key],actual[key],rtol=0,atol=1e-10):
                raise ValueError('Startup endpoint mismatch')
        if not np.isclose(saved['duration_s'],actual['duration_s'],rtol=0,atol=1e-8):
            raise ValueError('Startup duration mismatch')
    return start, verified


def reference_at(start, segments, elapsed):
    if elapsed < 1.:
        return start.copy(),np.zeros(7),'HOLD_INITIAL'
    remaining = elapsed-1.
    for segment in segments:
        if remaining < segment['duration_s']:
            q,v = minimum_jerk(np.array(segment['start_q_rad']),
                               np.array(segment['end_q_rad']),remaining,segment['duration_s'])
            return q,v,segment['name']
        remaining -= segment['duration_s']
    return np.array(segments[-1]['end_q_rad']),np.zeros(7),'HOLD_A'


class ToAController(HoldController):
    def __init__(self,bundle,hardware,start,j6_kd=None,j5_kd=None):
        if j6_kd is not None and j6_kd not in (.5,2.5):
            raise ValueError('J6 outer Kd must be 0.5 or 2.5')
        if j5_kd is not None and j5_kd not in (.5,3.):
            raise ValueError('J5 outer Kd must be 0.5 or 3.0')
        self.hardware = hardware
        super().__init__(bundle,hardware,start,np.zeros(7),0.,True,1.)
        if j6_kd is not None:
            self.servo.physics['kd'][5] = j6_kd
        if j5_kd is not None:
            self.servo.physics['kd'][4] = j5_kd

    def check_measured(self,q,velocity):
        q,v = np.asarray(q),np.asarray(velocity)
        if q.shape != (7,) or v.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(v).all():
            raise RuntimeError('Malformed measured state')
        if np.any(q < self.bundle.hard_low) or np.any(q > self.bundle.hard_high):
            raise RuntimeError('Measured joint outside hard limits')
        if np.max(np.abs(v)) > self.hardware['max_joint_speed_rad_s']:
            raise HoldViolation('Measured overspeed',{'q_rad':q.tolist(),
                                'velocity_rad_s':v.tolist(),
                                'speed_limit_rad_s':self.hardware['max_joint_speed_rad_s']})
        self.max_drift = max(self.max_drift,float(np.max(np.abs(q-self.target))))

    def accept_reference(self,packet,q,velocity,now):
        self.target = np.asarray(packet['command'],dtype=float)
        self.check_measured(q,velocity)
        # Shared ROS servo: bounds gross target lead, velocity and torque.
        # No ordinary 0.02rad moving-reference tracking-error rejection.
        try:
            self.servo.accept(packet['command'],packet['velocity_command'],packet['ff'],q,now)
        except ValueError as exc:
            raise HoldViolation(str(exc),{'q_rad':list(q),'target_q_rad':self.target.tolist(),
                                'position_errors_rad':(self.target-np.asarray(q)).tolist()}) from exc


def _worker(connection,run,hardware,q,velocity,stationary_baseline,segments):
    bundle = None
    try:
        bundle = RunBundle(run,load_policy=False)
        start = np.array(q)
        guard = StationaryOverlapGuard(model_contacts(bundle,start))
        bundle.update_state(bundle.q_a,np.zeros(7))
        palm_a = bundle.env.position.copy()
        begin = None
        while True:
            command,v,phase = reference_at(start,segments,0. if begin is None else time.monotonic()-begin)
            bundle.update_state(q,velocity)
            measured = np.asarray(q)
            if np.any(measured < bundle.hard_low) or np.any(measured > bundle.hard_high):
                raise RuntimeError('Measured joint outside hard limits')
            found = contacts(bundle.env.model,bundle.env.data,bundle.env.physics['contact_tolerance_m'])
            if phase in ('HOLD_INITIAL','shoulder_retreat','wrist_retreat'):
                guard.check(found)
                if np.max(np.abs(measured-start)) > .125:
                    raise RuntimeError('Measured escape leaves startup neighborhood')
            elif found:
                raise HoldViolation('Collision after escape',{'contacts':found,'q_rad':list(q)})
            connection.send({'command':command.tolist(),'velocity_command':v.tolist(),
                             'ff':bundle.feedforward().tolist(),'phase':phase,
                             'tcp_base_m':bundle.env.position.tolist(),
                             'a_distance_m':float(np.linalg.norm(bundle.env.position-palm_a)),
                             'computed':time.monotonic()})
            request = connection.recv()
            if request is None:
                break
            if isinstance(request,dict):
                q,velocity,begin = request['q'],request['velocity'],request['start_time']
            else:
                q,velocity = request
    except (EOFError,BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send({'error':str(exc),'stop_details':getattr(exc,'details',None)})
        except (EOFError,BrokenPipeError):
            pass
    finally:
        if bundle:
            bundle.env.close()
        connection.close()


def live_states(bus,hardware):
    now = time.monotonic()
    result = [bus.states.get(m['tx']) for m in hardware['motors']]
    for state in result:
        if state is None or not 0 <= now-state['received_monotonic'] <= hardware['feedback_timeout_s']:
            raise RuntimeError('Motor feedback missing or stale')
        if state['status'] != 1:
            raise RuntimeError('Motor disabled or faulted; no automatic re-enable')
        if not all(np.isfinite(state[k]) for k in ('q','velocity','torque')):
            raise RuntimeError('Nonfinite motor feedback')
        if max(state['mos_c'],state['rotor_c']) >= hardware['max_temperature_c']:
            raise RuntimeError('Motor temperature threshold exceeded')
    return result


def at_a(bundle,q,velocity,packet):
    return (np.max(np.abs(np.array(q)-bundle.q_a)) <= .01
            and packet['a_distance_m'] <= .015
            and np.max(np.abs(velocity)) <= .05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True)
    parser.add_argument('--plan',default='outputs/real_robot/startup_plan.json')
    parser.add_argument('--config',default=str(DEFAULT))
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--j6-kd',type=float,choices=(.5,2.5),default=.5,
                        help='J6 host outer-loop Kd; default 0.5 after observed oscillation; native MIT Kd remains zero')
    parser.add_argument('--j5-kd',type=float,choices=(.5,3.),default=.5,
                        help='J5 host outer-loop Kd; default 0.5 after observed oscillation; native MIT Kd remains zero')
    args = parser.parse_args()
    hardware = load(args.config)
    report = {'test':'training_gains_move_to_A','passed':False,'is_ppo_test':False,
              'gain_scale':1.,'hardware_commands_attempted':False,'disable_attempts':[],
              'j6_outer_kd_override':args.j6_kd,'native_mit_kp':0.,'native_mit_kd':0.,
              'j5_outer_kd_override':args.j5_kd,
              'TIMEOUT_changed':False,'watchdog_verified':False,
              'hand_commands_sent':False,'hand_monitored':False}
    bundle = bus = worker = controller = None
    enabled = False
    trace = deque(maxlen=1000)
    phases = []
    steps = 0
    stable_since = None
    q = v = None
    try:
        bundle = RunBundle(args.run,load_policy=False)
        start,verified = validate_plan(bundle,json.loads(Path(args.plan).read_text(encoding='utf-8')))
        segments = verified['segments']
        duration = 1.+verified['total_duration_s']
        report.update(kp=list(bundle.env.physics['kp']),kd=list(bundle.env.physics['kd']),
                      start_q_rad=start.tolist(),q_A_rad=bundle.q_a.tolist(),
                      model_sha256=bundle.env.task['model_sha256'],
                      config_sha256=bundle.env.task['config_sha256'],
                      trajectory_seconds=duration,initial_model_overlaps=verified['initial_contacts'])
        report['training_kd'] = list(bundle.env.physics['kd'])
        report['kd'][5] = args.j6_kd
        report['kd'][4] = args.j5_kd
        if not args.execute:
            report['preview_only'] = True
        else:
            bus = MotorBus(hardware,allow_control=True)
            bus.inspect()
            bus.validate_parameters(live=False)
            q,v = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.array(q)-start)) > .002:
                raise RuntimeError('Actual start moved >0.002rad from new plan; re-probe and re-plan')
            controller = ToAController(bundle,hardware,start,j6_kd=args.j6_kd,j5_kd=args.j5_kd)
            worker = ModelWorker(args.run,hardware,start,np.zeros(7),True,
                                 worker_target=_worker,worker_args=(segments,))
            q,v = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.array(q)-start)) > .002:
                raise RuntimeError('Actual start moved during preparation; re-plan')
            worker.request(q,v)
            if not worker.connection.poll(.5):
                raise RuntimeError('Model worker did not warm up')
            worker.receive()
            print(f'TRAINING PD / J5 Kd={args.j5_kd}, J6 Kd={args.j6_kd}: move to A, then KEEP HOLDING. Ctrl+C attempts motor DISABLE; support arm before exit.',flush=True)
            report['hardware_commands_attempted'] = True
            bus.torques([0.]*7)
            enabled = True
            bus.enable()
            deadline = time.monotonic()+.1
            while True:
                bus.refresh()
                time.sleep(.002)
                bus.drain()
                try:
                    states = live_states(bus,hardware)
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
            begin = previous = time.monotonic()
            worker.connection.send({'q':q,'velocity':v,'start_time':begin})
            worker.pending = True
            generation = None
            last_request = -1.
            while True:
                now = time.monotonic()
                if now-previous > hardware['max_timer_gap_s']:
                    raise RuntimeError('Servo loop deadline exceeded')
                previous = now
                bus.drain()
                states = live_states(bus,hardware)
                q,v = [s['q'] for s in states],[s['velocity'] for s in states]
                packet = worker.receive()
                if not packet or now-packet['computed'] > hardware['command_timeout_s']:
                    raise RuntimeError('Model/reference stale')
                if packet['computed'] != generation:
                    controller.accept_reference(packet,q,v,now)
                    generation = packet['computed']
                    if not phases or phases[-1]['phase'] != packet['phase']:
                        phases.append({'phase':packet['phase'],'elapsed_s':now-begin})
                if now-last_request >= .02 and not worker.pending:
                    worker.request(q,v)
                    last_request = now
                torque = controller.step_fast(q,v,now)
                bus.torques(torque)
                trace.append({'elapsed_s':now-begin,'q_rad':q,'velocity_rad_s':v,
                              'command_q_rad':packet['command'],'commanded_torque_nm':torque,
                              'feedback_torque_nm':[s['torque'] for s in states]})
                if packet['phase'] == 'HOLD_A':
                    stable_since = (now if stable_since is None else stable_since) if at_a(bundle,q,v,packet) else None
                    if not report['passed'] and stable_since is not None and now-stable_since >= .5:
                        report.update(passed=True,a_reached_seconds=now-begin,
                                      a_joint_errors_rad=(np.array(q)-bundle.q_a).tolist(),
                                      a_palm_distance_m=packet['a_distance_m'])
                        print('A_REACHED: maintaining A. Support arm before Ctrl+C; exit attempts DISABLE.',flush=True)
                    if report['passed'] and not at_a(bundle,q,v,packet):
                        report['passed'] = False
                        raise RuntimeError('A hold left endpoint acceptance region')
                    if not report['passed'] and now-begin > duration+15.:
                        raise RuntimeError('Failed to settle at A within 15 seconds')
                steps += 1
                # Skip missed scheduler periods; do not issue catch-up bursts.
                remaining = .002-(time.monotonic()-now)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        report['operator_stopped'] = True
    except (ValueError,RuntimeError,OSError,EOFError) as exc:
        report['passed'] = False
        report['error'] = str(exc)
        report['stop_details'] = getattr(exc,'details',None)
    finally:
        if bus:
            if enabled:
                for motor in hardware['motors']:
                    attempt = {'motor':motor['tx'],'sent':False}
                    try:
                        bus.send(motor['tx'],p.disable_payload())
                        attempt['sent'] = True
                    except (ValueError,RuntimeError,OSError) as exc:
                        attempt['error'] = str(exc)
                    report['disable_attempts'].append(attempt)
                report['disable_delivery_verified'] = False
            bus.close()
        if worker:
            worker.close()
        if controller:
            report['max_commanded_torque_nm'] = controller.max_torque.tolist()
        if q is not None and bundle:
            report['final_q_rad'] = list(q)
            report['final_velocity_rad_s'] = list(v)
            report['final_A_errors_rad'] = (np.asarray(q)-bundle.q_a).tolist()
        if bundle:
            bundle.env.close()
        report.update(cycles=steps,phases=phases)
        output = Path('outputs/real_robot')/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_move_to_A.json')
        output.parent.mkdir(parents=True,exist_ok=True)
        trace_path = output.with_name(output.stem+'_trace.json')
        trace_path.write_text(json.dumps(list(trace),indent=2)+'\n',encoding='utf-8')
        report['trace_file'] = str(trace_path)
        output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        print('REPORT:',output)
    return 0 if report.get('preview_only') or report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
