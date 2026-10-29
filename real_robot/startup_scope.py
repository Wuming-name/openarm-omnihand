"""Measured initial-pose region; never authorizes motion or waives contacts."""
import math


# Test envelope, not motor hard limits. J5 includes its measured ~0.124rad pose.
# Acceptance here still requires fresh planning, hard limits and contact checks.
START_ABS_LIMITS_RAD = (.1, .1, .1, .1, .15, .1, .1)


def classify_start_pose(q):
    values = tuple(float(x) for x in q)
    if len(values) != 7 or not all(math.isfinite(x) for x in values):
        raise ValueError('Expected seven finite initial joint positions')
    for axis, (value, limit) in enumerate(zip(values, START_ABS_LIMITS_RAD), 1):
        if abs(value) > limit:
            raise ValueError(f'J{axis} measured start {value:.6f}rad outside test region +/-{limit}rad')
    return 'measured_start_region_v1'
