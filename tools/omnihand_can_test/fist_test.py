#!/usr/bin/env python3
"""Slow, one-shot RIGHT O10 fist test using verified can1 / node 0x09."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import sys
import time

import read_only_test as ro

PROFILE = Path(__file__).with_name("right_hand_gestures.json")
NODE = 9
PRESERVED_AXES = (1, 4, 7, 9)


def positions_checked(values):
    if len(values) != 10 or any(type(v) is not int or not 0 <= v <= 4096 for v in values):
        raise ValueError("Expected ten integer command positions in 0..4096")
    return list(values)


def initial_positions(values):
    # Preserve original values in report. Only the planning start is clamped;
    # up to 16 counts of readback overshoot are accepted, never sent as targets.
    if len(values) != 10 or any(type(v) is not int or not 0 <= v <= 4112 for v in values):
        raise RuntimeError("Initial feedback outside accepted 0..4112 readback range; no movement")
    return [min(4096, v) for v in values]


def single_position_frame(axis, target):
    if type(axis) is not int or not 1 <= axis <= 10:
        raise ValueError("Axis must be 1..10")
    if type(target) is not int or not 0 <= target <= 4096:
        raise ValueError("Command target must be in 0..4096")
    payload = bytes([0x06, axis]) + struct.pack("<H", target)
    return ro.FD_FRAME.pack(NODE, len(payload), ro.CANFD_BRS, payload.ljust(64, b"\0"))


def valid_peer(frame):
    return (frame and frame["fd"] and frame["standard"] and not frame["local_origin"]
            and not frame["error_frame"] and not frame["remote_request"] and frame["can_id"] == NODE)


class MotionTransport(ro.SocketTransport):
    def set_axis(self, axis, target):
        end = time.monotonic() + .02
        while time.monotonic() < end:
            if self.receive(.001) is None:
                break
        packet = single_position_frame(axis, target)
        if self.sock.send(packet) != len(packet):
            raise RuntimeError("Incomplete position write")
        self.events.append({"direction": "tx_position", "command": "0x06", "can_id": "0x9",
                            "axis": axis, "target": target})
        end = time.monotonic() + .3
        while time.monotonic() < end:
            frame = self.receive(end - time.monotonic())
            if valid_peer(frame):
                data = frame["data"]
                if len(data) >= 4 and data[0] == 0x06 and data[1] == axis:
                    value = struct.unpack("<H", data[2:4])[0]
                    self.events.append({"direction": "rx_position", "axis": axis,
                                        "position": value, "data_hex": data.hex(" "),
                                        "monotonic_time": time.monotonic()})
                    if value > 4112:
                        raise RuntimeError(f"Axis {axis} feedback outside accepted readback range: {value}")
                    return value
        raise RuntimeError(f"Axis {axis} position response timed out; no further targets sent")

    def check_error(self):
        # Read-only command 0x0D. Does not clear errors or enable/calibrate.
        end = time.monotonic() + .02
        while time.monotonic() < end:
            if self.receive(.001) is None:
                break
        packet = ro.FD_FRAME.pack(NODE, 1, ro.CANFD_BRS, b"\x0d".ljust(64, b"\0"))
        if self.sock.send(packet) != len(packet):
            raise RuntimeError("Incomplete error query")
        self.events.append({"direction": "tx_read_query", "command": "0x0d", "can_id": "0x9"})
        end = time.monotonic() + .3
        while time.monotonic() < end:
            frame = self.receive(end - time.monotonic())
            if valid_peer(frame) and len(frame["data"]) >= 3 and frame["data"][0] == 0x0D:
                code = struct.unpack("<H", frame["data"][1:3])[0]
                if code:
                    raise RuntimeError(f"Hand error code {code}; no error clearing attempted")
                return
        raise RuntimeError("Hand error query timed out; no further targets sent")


def trajectory(start, target, step=12):
    start, target = positions_checked(start), positions_checked(target)
    count = max(1, math.ceil(max(abs(a-b) for a, b in zip(start, target)) / step))
    for i in range(1, count + 1):
        yield [round(a + (b-a)*i/count) for a, b in zip(start, target)]


def run_motion(transport, target, report, sleep=time.sleep, now=time.monotonic):
    preflight = ro.probe(transport, NODE, timeout=.4, attempts=1)
    report["preflight"] = preflight
    if not preflight["communication_passed"]:
        raise RuntimeError("Read-only identity/position preflight failed; no movement")
    info = preflight["device_info"]
    if info["software_version"] != "1.1.9" or info["hardware_version"] != "3.0.0":
        raise RuntimeError("This test is restricted to verified firmware 1.1.9 / hardware 3.0.0")
    if preflight.get("enable_state", {}).get("enable_state") != 1:
        raise RuntimeError("Hand must already be enabled; this program never enables it")
    feedback = list(preflight["joint_positions"]["positions_raw"])
    start = initial_positions(feedback)
    transport.check_error()
    report["planning_start"] = start
    report["samples"] = []
    report["requested_preset"] = list(target)
    target = positions_checked(target)
    for axis in PRESERVED_AXES:
        target[axis - 1] = start[axis - 1]
    report["target"] = list(target)
    report["preserved_axes"] = list(PRESERVED_AXES)
    report["motion_scope"] = "Six-axis hand closure; full official fist geometry is not validated"
    print("Keeping axes 1/4/7/9 at current positions; effective target:", target, flush=True)
    # Replace any previous run's residual targets with current-position targets.
    # Do not leave these axes tracking the interrupted official preset.
    for axis in PRESERVED_AXES:
        feedback[axis - 1] = transport.set_axis(axis, start[axis - 1])
        if abs(feedback[axis - 1] - start[axis - 1]) > 128:
            raise RuntimeError(f"Preserved axis {axis} did not remain near its starting position")
    transport.check_error()
    moved_axes = [i for i in range(10) if start[i] != target[i]]
    lag_since = {}
    last = list(start)

    def tick(command):
        nonlocal feedback
        begin = now()
        sample = {"target": list(command), "feedback": list(feedback), "sent_axes": [], "complete": False}
        report["samples"].append(sample)
        for i in moved_axes:
            # Only changed goals or lagging axes need a write, with reply per axis.
            if command[i] != last[i] or abs(feedback[i] - command[i]) > 64:
                sample["sent_axes"].append(i + 1)
                feedback[i] = transport.set_axis(i + 1, command[i])
                sample["feedback"] = list(feedback)
                last[i] = command[i]
            error = abs(feedback[i] - command[i])
            if error > 512:
                raise RuntimeError(f"Axis {i+1} tracking error {error} exceeds 512 counts")
            if error > 128:
                lag_since.setdefault(i, now())
                if now() - lag_since[i] > 1.:
                    raise RuntimeError(f"Axis {i+1} lag persisted; possible obstruction")
            else:
                lag_since.pop(i, None)
        transport.check_error()
        sample["complete"] = True
        sleep(max(0., .1 - (now() - begin)))

    # Max 12 counts per step, max ten steps/sec (120 counts/sec).
    for command in trajectory(start, target):
        tick(command)
    end = now() + 2.
    while max(abs(a-b) for a, b in zip(feedback, target)) > 64 and now() < end:
        tick(target)
    # Final fresh readback, not a statement of success based solely on TX/ACK.
    replies = transport.exchange(NODE, 0x09, .4)
    if not replies:
        raise RuntimeError("No final position response")
    report["final_positions_raw"] = replies[-1]["positions_raw"]
    report["max_final_error_counts"] = max(abs(a-b) for a, b in zip(report["final_positions_raw"], target))
    if report["max_final_error_counts"] > 128:
        raise RuntimeError("Final position error exceeds 128 counts; visual fist not confirmed")
    report["position_test_passed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", required=True, choices=["can1"], help="Verified hand-only physical CAN2")
    parser.add_argument("--gesture", choices=["fist", "open"], default="fist")
    parser.add_argument("--execute", action="store_true", help="Actually send position commands; default only previews offline")
    args = parser.parse_args()
    profile = json.loads(PROFILE.read_text(encoding="utf-8"))
    target = positions_checked(profile[args.gesture])
    print("RIGHT O10 / can1 / ID 0x09 /", args.gesture, "reference preset:", target, flush=True)
    print("Axes 1/4/7/9 will retain live starting positions. Only axes 2/3/5/6/8/10 follow the preset.", flush=True)
    if not args.execute:
        print("PREVIEW ONLY: no CAN socket opened. Add --execute to move the empty hand.")
        return 0
    report = {"gesture": args.gesture, "target": target, "profile": profile,
              "position_test_passed": False, "time": datetime.now().astimezone().isoformat()}
    transport = None
    status = 0
    try:
        ro.validate_channel(ro.interface_inventory(), args.channel, True)
        transport = MotionTransport(args.channel)
        print("Motion starts after read-only preflight. Keep fingers clear; Ctrl+C stops future targets, not motor torque.", flush=True)
        run_motion(transport, target, report)
    except KeyboardInterrupt:
        report["error"] = "Interrupted; future commands stopped. Hand remains enabled at last target."
        status = 130
    except (RuntimeError, OSError, ValueError) as exc:
        report["error"] = str(exc)
        status = 2
    finally:
        if transport is not None:
            transport.close()
            report["frames"] = transport.events
        directory = ro.PROJECT / "outputs/omnihand_can_test"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_motion.json")
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in report.items() if k not in ("frames", "samples", "profile", "preflight")}, ensure_ascii=False, indent=2))
        print("REPORT:", path)
    return status


if __name__ == "__main__":
    sys.exit(main())
