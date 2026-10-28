import json
import math
from pathlib import Path

DEFAULT = Path(__file__).parent / "config/hardware.json"
NAMES = [f"openarm_right_joint{i}" for i in range(1, 8)]


def load(path=DEFAULT, live=False):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if config["arm_channel"] != "can0" or config["hand_channel"] != "can1":
        raise ValueError("This deployment is bound to verified arm CAN1/can0 and hand CAN2/can1")
    if config["hand_id"] != 9:
        raise ValueError("This hand deployment is bound to verified O10 ID 0x09")
    motors = config["motors"]
    if len(motors) != 7 or [m["joint"] for m in motors] != NAMES:
        raise ValueError("Seven ordered arm motor mappings required")
    if len({m["tx"] for m in motors}) != 7 or len({m["rx"] for m in motors}) != 7:
        raise ValueError("Duplicate motor IDs")
    for motor in motors:
        if motor["sign"] not in (-1, 1) or not math.isfinite(motor["offset_rad"]):
            raise ValueError("Invalid joint sign/offset")
        peak, continuous = {"DM8009": (40.,20.), "DM4340": (27.,9.), "DM4310": (7.,3.)}[motor["protocol"]]
        if not 0 < motor["mechanical_peak_nm"] <= peak or not 0 < motor["continuous_nm"] <= min(continuous, motor["mechanical_peak_nm"]):
            raise ValueError("Mechanical limits exceed published motor ratings")
    if config["servo_hz"] != 500 or config["command_timeout_s"] > .2 or config["feedback_timeout_s"] > .1:
        raise ValueError("Unexpected servo/watchdog timing")
    if live:
        calibration = config["calibration"]
        for key in ("joint_mapping_and_zero_verified", "hand_pose_and_tcp_match_training", "hardware_stop_and_support_verified"):
            if calibration.get(key) is not True:
                raise ValueError(f"Live execution blocked: calibration.{key} is not verified; see deployment README")
        seconds = calibration.get("motor_timeout_seconds_verified")
        raw = calibration.get("motor_timeout_raw_verified")
        sec_values = seconds if isinstance(seconds, list) else [seconds]*7
        raw_values = raw if isinstance(raw, list) else [raw]*7
        if (len(sec_values) != 7 or len(raw_values) != 7
                or any(not isinstance(v, (float, int)) or not .02 <= v <= .2 for v in sec_values)
                or any(not isinstance(v, int) or v <= 0 for v in raw_values)):
            raise ValueError("Motor TIMEOUT raw value and measured/documented seconds must be verified; no unit is guessed")
    return config
