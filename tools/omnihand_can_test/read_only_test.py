#!/usr/bin/env python3
"""OmniHand 2025/O10 SocketCAN-FD read-only probe. No motor writes or link changes."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import json
import os
from pathlib import Path
import platform
import re
import socket
import struct
import subprocess
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]
FD_FRAME = struct.Struct("=IBB2x64s")
CLASSIC_FRAME = struct.Struct("=IB3x8s")
CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG = 0x80000000, 0x40000000, 0x20000000
CANFD_BRS = 1
READ_COMMANDS = {0x02: "enable_state", 0x09: "joint_positions", 0xCD: "device_info"}
MOTOR_NAMES = ["thumb_roll", "thumb_abad", "thumb_flexion", "index_abad", "index_flexion",
               "middle_flexion", "ring_abad", "ring_flexion", "pinky_abad", "pinky_flexion"]


def query_frame(device_id: int, command: int, brs=True) -> bytes:
    if command not in READ_COMMANDS:
        raise ValueError("Only read commands 0x02, 0x09 and 0xCD are implemented")
    if not 0 <= device_id <= 0x7FF:
        raise ValueError("A standard 11-bit CAN ID is required")
    return FD_FRAME.pack(device_id, 1, CANFD_BRS if brs else 0, bytes([command]).ljust(64, b"\0"))


def unpack_frame(raw: bytes, message_flags=0):
    if len(raw) == FD_FRAME.size:
        can_id, length, flags, data = FD_FRAME.unpack(raw)
        fd = True
    elif len(raw) == CLASSIC_FRAME.size:
        can_id, length, data = CLASSIC_FRAME.unpack(raw)
        flags, fd = 0, False
    else:
        return None
    if length > (64 if fd else 8):
        return None
    return {"can_id": can_id & 0x1FFFFFFF, "standard": not bool(can_id & CAN_EFF_FLAG),
            "remote_request": bool(can_id & CAN_RTR_FLAG), "error_frame": bool(can_id & CAN_ERR_FLAG),
            "fd": fd, "brs": bool(flags & CANFD_BRS), "data": data[:length],
            "local_origin": bool(message_flags & (getattr(socket, "MSG_DONTROUTE", 4) |
                                                   getattr(socket, "MSG_CONFIRM", 0x800)))}


def decode_reply(frame, command, expected_id=None):
    if not frame or frame["local_origin"] or not frame["fd"] or not frame["standard"]:
        return None
    if frame["remote_request"] or frame["error_frame"] or frame["can_id"] == 0x7FF:
        return None
    if expected_id is not None and frame["can_id"] != expected_id:
        return None
    data = frame["data"]
    if not data or data[0] != command:
        return None
    payload = data[1:]
    if command == 0xCD:
        # The published table inconsistently labels total size; keep raw response and
        # parse only documented fields present. Model=2 is O10, not O12/OmniPicker.
        if len(payload) < 9 or payload[0] != 2:
            return None
        result = {"product_type": 2, "product": "OmniHand O10",
                  "product_status": payload[1:3].decode("ascii", errors="replace"),
                  "software_version": ".".join(str(x) for x in payload[3:6]),
                  "hardware_version": ".".join(str(x) for x in payload[6:9])}
        if len(payload) >= 10 and payload[9] == 10:
            result["active_dof"] = 10
    elif command == 0x09:
        if len(payload) < 20:
            return None
        positions = list(struct.unpack("<10H", payload[:20]))
        outside = [i + 1 for i, value in enumerate(positions) if value > 4096]
        result = {"positions_raw": positions, "unit": "O10 raw uint16 position counts, not radians",
                  "documented_position_range": [0, 4096],
                  "positions_in_documented_range": not outside,
                  "by_motor": {str(i+1): {"label": name, "raw": value}
                               for i, (name, value) in enumerate(zip(MOTOR_NAMES, positions))}}
        if outside:
            result["warnings"] = [{"code": "POSITION_OUTSIDE_DOCUMENTED_RANGE", "motor_ids": outside,
                                   "message": "Raw reply retained without clamping. Communication can pass; position calibration and validity for control remain unverified."}]
    elif command == 0x02:
        if not payload or payload[0] not in (0, 1, 2):
            return None
        result = {"enable_state": payload[0], "meaning": {0: "disabled", 1: "enabled", 2: "calibration"}[payload[0]]}
    else:
        return None
    result.update(can_id=frame["can_id"], can_id_hex=hex(frame["can_id"]), command=hex(command))
    return result


def run_command(argv):
    try:
        process = subprocess.run(argv, capture_output=True, text=True, timeout=8, check=False)
        return {"command": argv, "returncode": process.returncode,
                "stdout": process.stdout, "stderr": process.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": argv, "error": str(exc)}


def interface_inventory():
    result = {"system": platform.system(), "python": platform.python_version(), "interfaces": []}
    if platform.system() != "Linux":
        result["note"] = "Run hardware inspection on Ubuntu. Offline unit tests also work on Windows."
        return result
    response = run_command(["ip", "-json", "-details", "-statistics", "link", "show", "type", "can"])
    result["ip_command"] = response
    if response.get("returncode") == 0:
        try:
            result["interfaces"] = json.loads(response["stdout"])
        except json.JSONDecodeError:
            result["note"] = "Could not parse ip JSON; inspect ip_command output."
    result["usb"] = run_command(["lsusb"])
    result["sysfs"] = {}
    for link in result["interfaces"]:
        name = link["ifname"]
        directory = Path("/sys/class/net") / name
        fields = {}
        for filename in ("dev_id", "dev_port", "operstate", "mtu"):
            try:
                fields[filename] = (directory / filename).read_text().strip()
            except OSError:
                pass
        try:
            fields["device_path"] = str((directory / "device").resolve(strict=True))
        except OSError:
            pass
        result["sysfs"][name] = fields
    return result


def validate_channel(inventory, channel, active):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", channel):
        raise ValueError("Invalid interface name")
    links = [link for link in inventory["interfaces"] if link["ifname"] == channel]
    if len(links) != 1:
        raise RuntimeError(f"SocketCAN interface {channel!r} not found; run inspect first")
    link = links[0]
    if "UP" not in link.get("flags", []):
        raise RuntimeError("Interface is DOWN. This tool never changes interface configuration.")
    if link.get("mtu") != 72:
        raise RuntimeError("CAN FD is not enabled (expected MTU=72).")
    details = link.get("linkinfo", {}).get("info_data", {})
    if details.get("state") in ("BUS-OFF", "STOPPED", "SLEEPING"):
        raise RuntimeError("CAN interface state is " + details["state"])
    if active:
        nominal = details.get("bittiming", {}).get("bitrate")
        data = details.get("data_bittiming", {}).get("bitrate")
        if nominal != 1_000_000 or data != 5_000_000:
            raise RuntimeError(f"Expected O10 nominal/data rates 1000000/5000000; observed {nominal}/{data}.")
        modes = details.get("ctrlmode", [])
        modes = modes if isinstance(modes, list) else []
        if any(str(mode).upper() in ("LOOPBACK", "LISTEN-ONLY", "FD-NON-ISO") for mode in modes):
            raise RuntimeError("Active O10 probe is not supported in loopback/listen-only/non-ISO mode.")
    return link


class SocketTransport:
    def __init__(self, channel, brs=True):
        if platform.system() != "Linux" or not hasattr(socket, "AF_CAN"):
            raise RuntimeError("SocketCAN requires Linux. No device was accessed.")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(getattr(socket, "SOL_CAN_RAW", 101), getattr(socket, "CAN_RAW_FD_FRAMES", 5), 1)
            self.sock.setsockopt(getattr(socket, "SOL_CAN_RAW", 101), getattr(socket, "CAN_RAW_RECV_OWN_MSGS", 4), 0)
            self.sock.bind((channel,))
        except BaseException:
            self.sock.close()
            raise
        self.brs = brs
        self.events = []

    def close(self):
        self.sock.close()

    def receive(self, timeout):
        self.sock.settimeout(max(.001, timeout))
        try:
            raw, _, flags, _ = self.sock.recvmsg(FD_FRAME.size)
        except socket.timeout:
            return None
        frame = unpack_frame(raw, flags)
        if frame and len(self.events) < 200:
            self.events.append({"direction": "rx", "can_id": hex(frame["can_id"]),
                                "fd": frame["fd"], "local_origin": frame["local_origin"],
                                "data_hex": frame["data"].hex(" ")})
        return frame

    def exchange(self, device_id, command, timeout):
        # Drop queued frames before this query. Bound draining even on a busy bus.
        end = time.monotonic() + .04
        while time.monotonic() < end:
            if self.receive(.001) is None:
                break
        raw = query_frame(device_id, command, self.brs)
        sent = self.sock.send(raw)
        if sent != len(raw):
            raise RuntimeError("Incomplete CAN socket write")
        self.events.append({"direction": "tx_read_query", "can_id": hex(device_id), "command": hex(command)})
        replies = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            frame = self.receive(end-time.monotonic())
            parsed = decode_reply(frame, command, None if device_id == 0x7FF else device_id)
            if parsed and parsed not in replies:
                replies.append(parsed)
        return replies


def probe(transport, requested_id, timeout=.7, attempts=2):
    identities = {}
    target = 0x7FF if requested_id is None else requested_id
    for _ in range(attempts):
        for reply in transport.exchange(target, 0xCD, timeout):
            identities[reply["can_id"]] = reply
        if identities:
            break
    if len(identities) != 1:
        return {"communication_passed": False, "identified_devices": list(identities.values()),
                "failure": "No unique O10 identity reply. Check channel, ID, rates and firmware; no motion command was sent."}
    device_id, identity = next(iter(identities.items()))
    result = {"device_info": identity, "communication_passed": False}
    for command in (0x02, 0x09):
        for _ in range(attempts):
            replies = transport.exchange(device_id, command, timeout)
            if replies:
                result[READ_COMMANDS[command]] = replies[-1]
                break
    result["communication_passed"] = "joint_positions" in result
    if not result["communication_passed"]:
        result["failure"] = "O10 identity received but no valid ten-axis position response. Inspect raw events for firmware differences."
    return result


def device_id(text):
    if text.lower() == "auto":
        return None
    value = int(text, 0)
    if not 0 <= value < 0x7FF:
        raise argparse.ArgumentTypeError("Use auto or a standard node ID below 0x7FF")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    commands.add_parser("inspect", help="List interfaces/configuration; no CAN socket or frame transmission")
    listen = commands.add_parser("listen", help="Receive frames only; silence does not mean the hand is absent")
    listen.add_argument("--channel", required=True)
    listen.add_argument("--seconds", type=float, default=3.)
    active = commands.add_parser("probe", help="Send three allowlisted read-only O10 queries")
    active.add_argument("--channel", required=True)
    active.add_argument("--hand-only", action="store_true", help="Confirm this physical channel contains the hand, not the powered arm")
    active.add_argument("--device-id", type=device_id, default=None, metavar="auto|0x09", help="Default auto uses read-only paging address 0x7FF")
    active.add_argument("--timeout", type=float, default=.7)
    active.add_argument("--attempts", type=int, default=2)
    active.add_argument("--no-brs", action="store_true", help="Diagnostic: FD frames without bit-rate switching; link rates are still checked")
    args = parser.parse_args()
    if args.mode == "probe" and not args.hand_only:
        parser.error("probe requires --hand-only after physical channel identification; do not guess can0/can1")
    if args.mode == "probe" and (not .1 <= args.timeout <= 5 or not 1 <= args.attempts <= 3):
        parser.error("timeout must be 0.1..5 seconds, attempts 1..3")
    if args.mode == "listen" and not .1 <= args.seconds <= 30:
        parser.error("seconds must be 0.1..30")
    report = {"mode": args.mode, "queries_change_motor_state": False,
              "time": datetime.now().astimezone().isoformat(), "inventory": interface_inventory()}
    transport = None
    status = 0
    try:
        if args.mode in ("listen", "probe"):
            report["channel"] = args.channel
            report["selected_link"] = validate_channel(report["inventory"], args.channel, args.mode == "probe")
            transport = SocketTransport(args.channel, brs=not getattr(args, "no_brs", False))
            if args.mode == "probe":
                report["result"] = probe(transport, args.device_id, args.timeout, args.attempts)
                status = 0 if report["result"]["communication_passed"] else 3
            else:
                ids = Counter()
                end = time.monotonic() + args.seconds
                while time.monotonic() < end:
                    frame = transport.receive(end-time.monotonic())
                    if frame and not frame["local_origin"]:
                        ids[hex(frame["can_id"])] += 1
                report["received_ids"] = dict(ids)
                report["note"] = "Receive-only application. No frames does not imply communication failure; O10 may only reply to queries."
    except (OSError, RuntimeError, ValueError) as exc:
        report["error"] = str(exc)
        status = 2
    except KeyboardInterrupt:
        report["error"] = "Interrupted"
        status = 130
    finally:
        if transport is not None:
            transport.close()
            report["frames"] = transport.events
        output = PROJECT / "outputs/omnihand_can_test"
        output.mkdir(parents=True, exist_ok=True)
        path = output / (datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}_{args.mode}.json")
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("\nREPORT:", path)
    return status


if __name__ == "__main__":
    sys.exit(main())
