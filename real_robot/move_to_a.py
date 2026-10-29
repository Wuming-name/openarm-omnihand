"""Planned startup to A using full native MIT and official OpenArm V1 gains.

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
from .commission_hold import HoldViolation, ModelWorker, StationaryOverlapGuard
from .control import RunBundle, minimum_jerk
from .plan_startup import model_contacts, screen_candidate
from .settings import DEFAULT, load
from .mit_servo import NativeMITServo, OFFICIAL_KP, OFFICIAL_KD, PROFILE
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


class ToAController(NativeMITServo):
    """To-A and ROS bridge share native MIT control and official gains."""
    def __init__(self,bundle,hardware,start):
        classify_start_pose(start)
        super().__init__(hardware,bundle.env.physics,bundle.hard_low,bundle.hard_high)

    def accept_reference(self,packet,q,velocity,now):
        self.accept(packet['command'],packet['velocity_command'],packet['ff'],q,now)


def _worker(connection,run,hardware,q,velocity,stationary_baseline,segments,with_policy=False):
    from .policy_mission import PolicyMission
    bundle = None
    try:
        bundle = RunBundle(run,load_policy=with_policy)
        mission = PolicyMission(bundle,q,segments,enable_policy=with_policy)
        while True:
            packet = mission.tick(q,velocity,time.monotonic())
            packet['computed'] = time.monotonic()
            connection.send(packet)
            request = connection.recv()
            if request is None:
                break
            if isinstance(request,dict):
                q,velocity,mission.begin = request['q'],request['velocity'],request['start_time']
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
    from .policy_mission import a_accepted
    return a_accepted(packet,velocity)


def main(with_policy=False):
    parser = argparse.ArgumentParser(description=(
        'Startup -> accepted palm A -> PPO -> B hold, full native MIT throughout; no CAN without --execute.'
        if with_policy else __doc__))
    parser.add_argument('--run',required=True)
    parser.add_argument('--plan',default='outputs/real_robot/startup_plan.json')
    parser.add_argument('--config',default=str(DEFAULT))
    parser.add_argument('--execute',action='store_true')
    if with_policy:
        parser.add_argument('--evaluation-report',default='outputs/real_robot/ppo_native_evaluation.json')
    args = parser.parse_args()
    hardware = load(args.config)
    report = {'test':'official_native_MIT_move_to_A','passed':False,'is_ppo_test':with_policy,
              'control_profile':PROFILE,'host_pd_enabled':False,
              'native_mit_kp':list(OFFICIAL_KP),'native_mit_kd':list(OFFICIAL_KD),
              'firmware_torque_limit_verified':False,
              'hardware_commands_attempted':False,'disable_attempts':[],
              'TIMEOUT_changed':False,'watchdog_verified':False,
              'hand_commands_sent':False,'hand_monitored':False}
    if with_policy:
        report['test'] = 'native_MIT_startup_A_PPO_B'
    bundle = bus = worker = controller = None
    enabled = False
    trace = deque(maxlen=1000)
    phases = []
    steps = 0
    stable_since = None
    q = v = None
    try:
        bundle = RunBundle(args.run,load_policy=False)
        if with_policy and args.execute:
            from .evaluate_policy import verify_evaluation
            verify_evaluation(args.evaluation_report,bundle)
            report['offline_evaluation_report'] = args.evaluation_report
        start,verified = validate_plan(bundle,json.loads(Path(args.plan).read_text(encoding='utf-8')))
        segments = verified['segments']
        duration = 1.+verified['total_duration_s']
        report.update(kp=list(OFFICIAL_KP),kd=list(OFFICIAL_KD),
                      start_q_rad=start.tolist(),q_A_rad=bundle.q_a.tolist(),
                      model_sha256=bundle.env.task['model_sha256'],
                      config_sha256=bundle.env.task['config_sha256'],
                      trajectory_seconds=duration,initial_model_overlaps=verified['initial_contacts'])
        report['training_kp'] = list(bundle.env.physics['kp'])
        report['training_kd'] = list(bundle.env.physics['kd'])
        if not args.execute:
            report['preview_only'] = True
        else:
            bus = MotorBus(hardware,allow_control=True)
            bus.inspect()
            bus.validate_parameters(live=False)
            q,v = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.array(q)-start)) > .002:
                raise RuntimeError('Actual start moved >0.002rad from new plan; re-probe and re-plan')
            controller = ToAController(bundle,hardware,start)
            worker = ModelWorker(args.run,hardware,start,np.zeros(7),True,
                                 worker_target=_worker,worker_args=(segments,with_policy))
            q,v = fresh_disabled(bus,hardware)
            if np.max(np.abs(np.array(q)-start)) > .002:
                raise RuntimeError('Actual start moved during preparation; re-plan')
            worker.request(q,v)
            if not worker.connection.poll(.5):
                raise RuntimeError('Model worker did not warm up')
            worker.receive()
            destination = 'A then PPO to B' if with_policy else 'A'
            print(f'OFFICIAL V1 NATIVE MIT: {destination}, then KEEP HOLDING. Support arm before Ctrl+C; exit attempts DISABLE.',flush=True)
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
                    controller.accept(packet['command'],packet['velocity_command'],packet['ff'],q,now)
                    generation = packet['computed']
                    if not phases or phases[-1]['phase'] != packet['phase']:
                        phases.append({'phase':packet['phase'],'elapsed_s':now-begin})
                        if with_policy and packet['phase'] in ('POLICY','HOLD_B'):
                            print('STATE: '+packet['phase'],flush=True)
                if now-last_request >= .02 and not worker.pending:
                    worker.request(q,v)
                    last_request = now
                commands = controller.tick(q,v,now,[s['torque'] for s in states])
                if controller.fault:
                    raise RuntimeError('MIT servo latched fault: '+controller.fault)
                bus.mit_commands(commands)
                trace.append({'elapsed_s':now-begin,'q_rad':q,'velocity_rad_s':v,
                              'command_q_rad':packet['command'],'command_velocity_rad_s':packet['velocity_command'],
                              'feedforward_torque_nm':list(controller.ff),
                              'estimated_total_torque_nm':list(controller.last_estimated_torque),
                              'feedback_torque_nm':[s['torque'] for s in states]})
                if with_policy:
                    report['accepted_A'] = packet.get('accepted_A')
                    report['final_B_distance_m'] = packet['b_distance_m']
                    if packet['phase'] == 'HOLD_B':
                        stable_since = (now if stable_since is None else stable_since) if packet['b_goal_reached'] else None
                        if stable_since is not None and now-stable_since >= .5:
                            if not report['passed']:
                                print('B_REACHED: maintaining B. Support arm before Ctrl+C.',flush=True)
                            report['passed'] = True
                        elif not packet['b_goal_reached']:
                            report['passed'] = False
                    else:
                        stable_since = None
                elif packet['phase'] == 'HOLD_A':
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
            report['max_estimated_total_torque_nm'] = list(controller.max_estimated_torque)
            report['max_feedforward_torque_nm'] = list(controller.max_feedforward)
        if q is not None and bundle:
            report['final_q_rad'] = list(q)
            report['final_velocity_rad_s'] = list(v)
            report['final_A_errors_rad'] = (np.asarray(q)-bundle.q_a).tolist()
        if bundle:
            bundle.env.close()
        report.update(cycles=steps,phases=phases)
        output = Path('outputs/real_robot')/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+('_policy_to_B.json' if with_policy else '_move_to_A.json'))
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
