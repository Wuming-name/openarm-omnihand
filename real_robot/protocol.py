"""Independent implementation of the documented Damiao MIT wire format.

Reference: enactic/openarm_can bfd4e52194aefe38389fa6e670be3738136facb9.
Protocol mapping limits are NOT mechanical operating limits.
"""
import math
import struct

LIMITS = {"DM8009": (12.5, 45., 54.), "DM4340": (12.5, 10., 28.), "DM4310": (12.5, 30., 10.)}
READ_RIDS = {7: "MST_ID", 8: "ESC_ID", 9: "TIMEOUT", 10: "CTRL_MODE", 21: "PMAX", 22: "VMAX", 23: "TMAX"}
FD = struct.Struct("=IBB2x64s")
CLASSIC = struct.Struct("=IB3x8s")


def quantize(value, low, high, bits):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"Value {value} outside protocol range [{low}, {high}]")
    return int((value-low)/(high-low)*((1 << bits)-1))


def decode_uint(value, maximum, bits):
    return value/((1 << bits)-1)*(2*maximum)-maximum


def mit_payload(kind, q, velocity, kp, kd, torque):
    pmax, vmax, tmax = LIMITS[kind]
    p, v = quantize(q, -pmax, pmax, 16), quantize(velocity, -vmax, vmax, 12)
    k, d = quantize(kp, 0., 500., 12), quantize(kd, 0., 5., 12)
    t = quantize(torque, -tmax, tmax, 12)
    return bytes([p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k >> 8),
                  k & 255, d >> 4, ((d & 15) << 4) | (t >> 8), t & 255])


def enable_payload():
    return b"\xff"*7 + b"\xfc"


def disable_payload():
    return b"\xff"*7 + b"\xfd"


def read_param(tx, rid):
    if not 1 <= tx <= 0x7FE or rid not in READ_RIDS:
        raise ValueError("Unsupported read-only parameter request")
    return bytes([tx & 255, tx >> 8, 0x33, rid, 0, 0, 0, 0])


def refresh_payload(tx):
    if not 1 <= tx <= 0x7FE:
        raise ValueError("Invalid node ID")
    return bytes([tx & 255, tx >> 8, 0xCC, 0, 0, 0, 0, 0])


def pack_fd(can_id, payload):
    if not 0 <= can_id <= 0x7FF or not 1 <= len(payload) <= 8:
        raise ValueError("This motor protocol uses standard IDs and up to eight data bytes")
    return FD.pack(can_id, len(payload), 1, payload.ljust(64, b"\0"))


def unpack(raw, flags=0):
    if flags & (4 | 0x800):  # Linux MSG_DONTROUTE / MSG_CONFIRM: local origin
        return None
    if len(raw) == FD.size:
        can_id, length, _, data = FD.unpack(raw)
    elif len(raw) == CLASSIC.size:
        can_id, length, data = CLASSIC.unpack(raw)
    else:
        return None
    if can_id & 0xE0000000 or can_id > 0x7FF or length > len(data):
        return None
    return can_id, data[:length]


def parameter_reply(motor, can_id, data):
    if (can_id != motor["rx"] or len(data) < 8 or data[2] != 0x33
            or int.from_bytes(data[:2], "little") != motor["tx"] or data[3] not in READ_RIDS):
        return None
    rid = data[3]
    value = struct.unpack("<I" if rid <= 10 else "<f", data[4:8])[0]
    if not math.isfinite(value):
        raise ValueError("Nonfinite parameter response")
    return rid, value


def state_reply(motor, can_id, data):
    if can_id != motor["rx"] or len(data) < 8 or parameter_reply(motor, can_id, data):
        return None
    if data[0] & 15 != motor["tx"] & 15:
        return None
    pmax, vmax, tmax = LIMITS[motor["protocol"]]
    q = decode_uint(int.from_bytes(data[1:3], "big"), pmax, 16)
    v = decode_uint((data[3] << 4) | (data[4] >> 4), vmax, 12)
    tau = decode_uint(((data[4] & 15) << 8) | data[5], tmax, 12)
    return {"q": motor["sign"]*(q-motor["offset_rad"]), "velocity": motor["sign"]*v,
            "torque": motor["sign"]*tau, "status": data[0] >> 4,
            "mos_c": data[6], "rotor_c": data[7], "motor_q": q}
