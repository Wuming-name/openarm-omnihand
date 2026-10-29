"""Explicit initial-pose scopes; these never authorize motion or waive contacts."""
import math


SCREENED_20261029 = (
    0.005149919890135024, 0.0165941863126573, 0.015449759670405072,
    0.05893797207598972, 0.12417029068436669, 0.00705729762722207,
    -0.028801403830014394,
)
SCREENED_START_TOLERANCE_RAD = .002


def classify_start_pose(q):
    values = tuple(float(x) for x in q)
    if len(values) != 7 or not all(math.isfinite(x) for x in values):
        raise ValueError('Expected seven finite initial joint positions')
    if max(abs(x) for x in values) <= .1:
        return 'near_zero_0.1rad'
    if all(abs(a-b) <= SCREENED_START_TOLERANCE_RAD
           for a, b in zip(values, SCREENED_20261029)):
        return 'screened_pose_20261029_153744'
    raise ValueError('Start outside near-zero scope and the narrowly screened 20261029 pose')
