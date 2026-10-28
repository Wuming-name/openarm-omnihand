"""Supported-arm MIT smoke test: measured position, low native gains, no PPO.

TIMEOUT is read but not changed or treated as verified. Executed test enables
initially disabled motors and disables them on exit. Mechanically support arm.
"""
import argparse
import json
import math
import time

from .can_bus import MotorBus
from .settings import DEFAULT, load
from . import protocol as p


def snapshot(bus, config, now):
    states = [bus.states.get(m['tx']) for m in config['motors']]
    for state in states:
        if state is None or not 0 <= now-state['received_monotonic'] <= .05:
            raise RuntimeError('Missing or stale motor feedback')
        if state['status'] not in (0, 1):
            raise RuntimeError('Motor fault/status: ' + str(state['status']))
        if not all(math.isfinite(state[k]) for k in ('motor_q', 'velocity', 'torque')):
            raise RuntimeError('Nonfinite motor feedback')
        if abs(state['velocity']) > .15 or max(state['mos_c'], state['rotor_c']) >= 70:
            raise RuntimeError('Arm moving or motor too hot for supported hold test')
    return states


def packets(config, targets, states):
    if len(targets) != 7 or len(states) != 7:
        raise ValueError('Seven measured positions required')
    result = []
    for motor, target, state in zip(config['motors'], targets, states):
        if not math.isfinite(target) or abs(state['motor_q']-target) > .03:
            raise RuntimeError('Position drift over 0.03 rad; support arm before retrying')
        if abs(state['torque']) > motor['continuous_nm']:
            raise RuntimeError('Measured torque exceeds continuous rating')
        # Bench-test gains, not official payload gains or the trained controller.
        result.append((motor['tx'], p.mit_payload(motor['protocol'], target, 0., 10., .5, 0.)))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seconds', type=float, default=3.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 5:
        parser.error('--seconds must be between 1 and 5')
    config = load(args.config)
    bus, own_enable = None, False
    try:
        bus = MotorBus(config, allow_control=args.execute)
        bus.inspect()
        bus.validate_parameters(live=False)
        # Refresh all states together; inspect() may retain older first-axis data.
        bus.states.clear()
        bus.refresh()
        end = time.monotonic()+.5
        while len(bus.states) < 7 and time.monotonic() < end:
            bus.drain()
            time.sleep(.001)
        states = snapshot(bus, config, time.monotonic())
        targets = [s['motor_q'] for s in states]
        prepared = packets(config, targets, states)
        print(json.dumps({'native_motor_targets_rad': targets, 'kp': 10., 'kd': .5,
                          'feedforward_nm': 0., 'TIMEOUT_changed': False,
                          'enable_then_disable': args.execute}, indent=2), flush=True)
        if not args.execute:
            print('Read-only preflight complete. No enable or MIT command sent.')
            return 0
        if any(s['status'] != 0 for s in states):
            raise RuntimeError('Test requires all seven motors initially disabled; will not take over an active controller')
        print('SUPPORTED ARM ONLY: native low-gain MIT test starts; motors DISABLE on exit.', flush=True)
        # Preload measured-position MIT targets while motors are disabled.
        for can_id, payload in prepared:
            bus.send(can_id, payload)
        own_enable = True  # Also covers a partially failed enable transmission.
        bus.enable()
        start = previous = time.monotonic()
        last_log = start-1.
        while time.monotonic()-start < args.seconds:
            now = time.monotonic()
            if now-previous > .05:
                raise RuntimeError('Host scheduling gap over 50 ms')
            previous = now
            bus.drain()
            states = snapshot(bus, config, time.monotonic())
            if now-start > .15 and any(s['status'] != 1 for s in states):
                raise RuntimeError('Not all motors acknowledged enable')
            prepared = packets(config, targets, states)
            for can_id, payload in prepared:
                bus.send(can_id, payload)
            if now-last_log >= .5:
                drift = max(abs(s['motor_q']-q) for s, q in zip(states, targets))
                print(f'HOLD elapsed={now-start:.2f}s max_drift={drift:.5f}rad', flush=True)
                last_log = now
            time.sleep(max(0., .01-(time.monotonic()-now)))
        print('MIT hold communication test completed; this is not PPO/controller acceptance.', flush=True)
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        print('STOP:', str(exc), flush=True)
        return 2
    finally:
        if bus:
            if own_enable:
                for motor in config['motors']:
                    try:
                        bus.send(motor['tx'], p.disable_payload())
                    except (RuntimeError, ValueError, OSError) as exc:
                        print(f"DISABLE FAILED J{motor['tx']}: {exc}; hardware support required", flush=True)
                print('Disable commands attempted on all seven motors; delivery is not guaranteed.', flush=True)
            bus.close()


if __name__ == '__main__':
    raise SystemExit(main())
