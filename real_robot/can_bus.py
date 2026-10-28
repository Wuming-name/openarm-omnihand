"""Linux SocketCAN backend. Read requests are available separately from torque writes."""
import json
import socket
import subprocess
import time

from . import protocol as p


class MotorBus:
    def __init__(self, config, allow_control=False):
        import fcntl
        self.config = config
        self.allow_control = allow_control
        self.checked = False
        self.states, self.parameters = {}, {}
        name = config["arm_channel"]
        self.lock = open(f"/tmp/openarm_rl_{name}.lock", "a")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = subprocess.run(["ip", "-json", "-details", "link", "show", name], capture_output=True, text=True, check=True)
        link = json.loads(result.stdout)[0]
        details = link.get("linkinfo", {}).get("info_data", {})
        if ("UP" not in link["flags"] or link["mtu"] != 72 or details.get("bittiming", {}).get("bitrate") != config["bitrate"]
                or details.get("data_bittiming", {}).get("bitrate") != config["data_bitrate"]
                or details.get("state") != "ERROR-ACTIVE"
                or any(mode in details.get("ctrlmode", []) for mode in ("LOOPBACK", "LISTEN-ONLY", "FD-NON-ISO"))):
            raise RuntimeError("Arm CAN interface not UP / FD / 1M-5M / normal mode; no configuration changed")
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(101, 5, 1)
            self.sock.setsockopt(101, 4, 0)
            self.sock.bind((name,))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            self.lock.close()
            raise

    def send(self, can_id, payload):
        packet = p.pack_fd(can_id, payload)
        if self.sock.send(packet) != len(packet):
            raise RuntimeError("Incomplete CAN write")

    def drain(self):
        for _ in range(256):
            try:
                raw, _, flags, _ = self.sock.recvmsg(72)
            except BlockingIOError:
                break
            packet = p.unpack(raw, flags)
            if not packet:
                continue
            can_id, data = packet
            for motor in self.config["motors"]:
                if can_id != motor["rx"]:
                    continue
                parameter = p.parameter_reply(motor, can_id, data)
                if parameter:
                    self.parameters[(motor["tx"], parameter[0])] = parameter[1]
                else:
                    state = p.state_reply(motor, can_id, data)
                    if state:
                        state["received_monotonic"] = time.monotonic()
                        self.states[motor["tx"]] = state

    def inspect(self):
        for motor in self.config["motors"]:
            for rid in p.READ_RIDS:
                key = (motor["tx"], rid)
                self.parameters.pop(key, None)
                self.send(0x7FF, p.read_param(motor["tx"], rid))
                deadline = time.monotonic()+.25
                while time.monotonic() < deadline and key not in self.parameters:
                    self.drain()
                    time.sleep(.001)
                if key not in self.parameters:
                    raise RuntimeError(f"No parameter reply: TX={motor['tx']} RX={motor['rx']} RID={rid}; no motion sent")
            self.send(0x7FF, p.refresh_payload(motor["tx"]))
        deadline = time.monotonic()+.5
        while time.monotonic() < deadline:
            self.drain()
            if len(self.states) == 7:
                break
            time.sleep(.001)
        if len(self.states) != 7:
            raise RuntimeError("Not all seven joint feedback frames received")
        return [{"mapping": m, "parameters": {p.READ_RIDS[r]: self.parameters[(m["tx"], r)] for r in p.READ_RIDS},
                 "state": self.states[m["tx"]]} for m in self.config["motors"]]

    def validate_parameters(self, live=False):
        self.checked = False
        for index, motor in enumerate(self.config["motors"]):
            tx = motor["tx"]
            actual = lambda rid: self.parameters[(tx, rid)]
            if actual(7) != motor["rx"] or actual(8) != tx or actual(10) != 1:
                raise RuntimeError(f"Motor {tx}: ID pair or MIT mode differs; no automatic parameter writes")
            for rid, expected in zip((21, 22, 23), p.LIMITS[motor["protocol"]]):
                if abs(actual(rid)-expected) > 1e-3:
                    raise RuntimeError(f"Motor {tx}: protocol range RID {rid} is {actual(rid)}, expected {expected}; refusing incorrect quantization")
            expected_timeout = self.config["calibration"]["motor_timeout_raw_verified"]
            if isinstance(expected_timeout, list):
                expected_timeout = expected_timeout[index]
            if live and actual(9) != expected_timeout:
                raise RuntimeError(f"Motor {tx}: TIMEOUT differs from reviewed configuration")
        self.checked = True

    def refresh(self):
        for motor in self.config["motors"]:
            self.send(0x7FF, p.refresh_payload(motor["tx"]))

    def enable(self):
        if not self.allow_control or not self.checked:
            raise RuntimeError("Read-only CAN session")
        for motor in self.config["motors"]:
            self.send(motor["tx"], p.enable_payload())

    def torques(self, torque):
        if not self.allow_control or not self.checked or len(torque) != 7:
            raise RuntimeError("Control not enabled or wrong torque size")
        packets = []
        for motor, value in zip(self.config["motors"], torque):
            if abs(value) > motor["mechanical_peak_nm"]:
                raise ValueError("Mechanical peak limit exceeded")
            packets.append((motor["tx"], p.mit_payload(motor["protocol"], 0., 0., 0., 0., motor["sign"]*value)))
        for can_id, payload in packets:
            self.send(can_id, payload)

    def close(self):
        self.sock.close()
        self.lock.close()
