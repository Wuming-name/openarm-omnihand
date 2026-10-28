"""Observe disabled seven-axis position/velocity feedback. No enable or MIT frames."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

from .can_bus import MotorBus
from .settings import DEFAULT, load
from .protocol import LIMITS


def summarize(samples, motors):
    result = []
    for motor in motors:
        values = samples[motor['tx']]
        if not values:
            raise RuntimeError(f"No feedback samples for J{motor['tx']}")
        q = [s['q'] for s in values]
        v = [s['velocity'] for s in values]
        result.append({'joint': motor['joint'], 'samples': len(values),
                       'position_min_rad': min(q), 'position_max_rad': max(q),
                       'position_span_rad': max(q)-min(q),
                       'velocity_min_rad_s': min(v), 'velocity_max_rad_s': max(v),
                       'max_abs_velocity_rad_s': max(abs(x) for x in v),
                       'velocity_code_step_rad_s': 2*LIMITS[motor['protocol']][1]/4095,
                       'all_samples_disabled': all(s['status'] == 0 for s in values)})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(DEFAULT))
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 10:
        parser.error('--seconds must be 2..10')
    config = load(args.config)
    report = {'mode': 'disabled_feedback_read_only', 'motion_commands_sent': False, 'passed': False}
    bus = None
    samples = {m['tx']: [] for m in config['motors']}
    stamps = {m['tx']: None for m in config['motors']}
    try:
        bus = MotorBus(config, allow_control=False)
        bus.inspect()
        bus.validate_parameters(live=False)
        bus.states.clear()
        start = time.monotonic()
        while time.monotonic()-start < args.seconds:
            begin = time.monotonic()
            bus.refresh()  # Official 0xCC status request only, not MIT position/torque.
            end = min(begin+.01, start+args.seconds)
            while time.monotonic() < end:
                bus.drain()
                for motor in config['motors']:
                    state = bus.states.get(motor['tx'])
                    if time.monotonic()-start > .1 and (state is None or time.monotonic()-state['received_monotonic'] > .05):
                        raise RuntimeError(f"J{motor['tx']} feedback missing or stale")
                    if state is None:
                        continue
                    if state['status'] != 0:
                        raise RuntimeError(f"J{motor['tx']} is not disabled (status {state['status']}); observation stopped")
                    if state['received_monotonic'] != stamps[motor['tx']]:
                        samples[motor['tx']].append(dict(state))
                        stamps[motor['tx']] = state['received_monotonic']
                time.sleep(.001)
        report['motors'] = summarize(samples, config['motors'])
        report['passed'] = True
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        if all(samples.values()):
            report['motors'] = summarize(samples, config['motors'])
    finally:
        if bus:
            bus.close()
        report['samples'] = samples
        output = Path('outputs/real_robot') / (datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_feedback_probe.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        # Keep terminal output short; full timestamped samples are in JSON.
        print(json.dumps({k:v for k,v in report.items() if k != 'samples'}, ensure_ascii=False, indent=2))
        print('REPORT:', output)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
