from pathlib import Path
import json
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import read_only_test as testtool


def reply(command, payload, node=9, local=False):
    data = bytes([command]) + payload
    length = next(size for size in [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64] if size >= len(data))
    raw = testtool.FD_FRAME.pack(node, length, 1, data.ljust(64, b"\0"))
    return testtool.unpack_frame(raw, 4 if local else 0)


INFO = bytes([2]) + b"P1" + bytes([1, 2, 14, 3, 0, 0, 10])


class FakeTransport:
    def __init__(self, positions=True, ids=(9,)):
        self.calls = []
        self.positions = positions
        self.ids = ids

    def exchange(self, node, command, timeout):
        # Exercise the same non-bypassable read-only encoder as real transport.
        testtool.query_frame(node, command)
        self.calls.append((node, command))
        if command == 0xCD:
            return [testtool.decode_reply(reply(command, INFO, identity), command) for identity in self.ids]
        if command == 0x02:
            return [testtool.decode_reply(reply(command, b"\x01", node), command, node)]
        if command == 0x09 and self.positions:
            return [testtool.decode_reply(reply(command, struct.pack("<10H", *range(10)), node), command, node)]
        return []


class ReadOnlyProtocol(unittest.TestCase):
    def test_only_read_opcodes_can_be_encoded(self):
        for command in range(256):
            if command in testtool.READ_COMMANDS:
                raw = testtool.query_frame(9, command)
                self.assertEqual(len(raw), 72)
                _, length, flags, payload = testtool.FD_FRAME.unpack(raw)
                self.assertEqual(length, 1)
                self.assertEqual(flags, 1)
                self.assertEqual(payload[0], command)
            else:
                with self.assertRaises(ValueError):
                    testtool.query_frame(9, command)

    def test_local_echo_wrong_id_and_classic_frames_cannot_pass(self):
        self.assertIsNone(testtool.decode_reply(reply(0xCD, INFO, local=True), 0xCD))
        self.assertIsNone(testtool.decode_reply(reply(0xCD, INFO, node=1), 0xCD, 9))
        classic = testtool.CLASSIC_FRAME.pack(9, 2, b"\x02\x01".ljust(8, b"\0"))
        self.assertIsNone(testtool.decode_reply(testtool.unpack_frame(classic), 0x02, 9))
        self.assertIsNone(testtool.decode_reply(testtool.unpack_frame(testtool.query_frame(9, 0x09)), 0x09, 9))

    def test_positions_are_little_endian_and_truncated_reply_rejected(self):
        positions = [0, 1, 256, 1000, 2000, 3000, 4095, 4096, 25, 1500]
        decoded = testtool.decode_reply(reply(0x09, struct.pack("<10H", *positions)), 0x09, 9)
        self.assertEqual(decoded["positions_raw"], positions)
        self.assertTrue(decoded["positions_in_documented_range"])
        self.assertIsNone(testtool.decode_reply(reply(0x09, b"\0\0"), 0x09, 9))

    def test_recorded_firmware_119_reply_passes_with_range_warning(self):
        recorded = {
            0xCD: bytes.fromhex("cd 02 54 31 01 01 09 03 00 00 0a 00"),
            0x02: bytes.fromhex("02 01"),
            0x09: bytes.fromhex("09 d2 07 f1 07 f8 0f 6b 07 fe 0f fe 0f cc 07 01 10 09 08 02 10 00 00 00"),
        }
        class RecordedTransport:
            def exchange(self, node, command, timeout):
                testtool.query_frame(node, command)
                raw = testtool.FD_FRAME.pack(node, len(recorded[command]), 1, recorded[command].ljust(64, b"\0"))
                return [testtool.decode_reply(testtool.unpack_frame(raw), command, node)]
        result = testtool.probe(RecordedTransport(), 9)
        self.assertTrue(result["communication_passed"])
        self.assertEqual(result["device_info"]["software_version"], "1.1.9")
        positions = result["joint_positions"]
        self.assertEqual(positions["positions_raw"], [2002, 2033, 4088, 1899, 4094, 4094, 1996, 4097, 2057, 4098])
        self.assertFalse(positions["positions_in_documented_range"])
        self.assertEqual(positions["warnings"][0]["motor_ids"], [8, 10])

    def test_o10_identification_and_enable_state(self):
        info = testtool.decode_reply(reply(0xCD, INFO), 0xCD)
        self.assertEqual(info["software_version"], "1.2.14")
        self.assertEqual(info["hardware_version"], "3.0.0")
        self.assertEqual(info["active_dof"], 10)
        self.assertIsNone(testtool.decode_reply(reply(0xCD, bytes([3])+INFO[1:]), 0xCD))
        self.assertIsNone(testtool.decode_reply(reply(0x02, b"\xff"), 0x02))

    def test_no_identity_stops_before_other_queries(self):
        transport = FakeTransport(ids=())
        result = testtool.probe(transport, None)
        self.assertFalse(result["communication_passed"])
        self.assertTrue(all(command == 0xCD for _, command in transport.calls))

    def test_multiple_devices_are_not_automatically_selected(self):
        transport = FakeTransport(ids=(1, 9))
        result = testtool.probe(transport, None)
        self.assertFalse(result["communication_passed"])
        self.assertEqual(transport.calls, [(0x7FF, 0xCD)])

    def test_passing_requires_identity_and_ten_positions(self):
        transport = FakeTransport()
        result = testtool.probe(transport, None)
        self.assertTrue(result["communication_passed"])
        self.assertEqual(transport.calls, [(0x7FF, 0xCD), (9, 0x02), (9, 0x09)])
        self.assertEqual(len(result["joint_positions"]["positions_raw"]), 10)
        self.assertFalse(testtool.probe(FakeTransport(positions=False), 9)["communication_passed"])

    def test_error_and_remote_request_frames_rejected(self):
        for flag in [testtool.CAN_ERR_FLAG, testtool.CAN_RTR_FLAG, testtool.CAN_EFF_FLAG]:
            frame = reply(0xCD, INFO)
            raw = testtool.FD_FRAME.pack(9 | flag, 12, 1, (b"\xcd"+INFO).ljust(64, b"\0"))
            self.assertIsNone(testtool.decode_reply(testtool.unpack_frame(raw), 0xCD))


class LinkValidation(unittest.TestCase):
    def inventory(self):
        return {"interfaces": [{"ifname": "can1", "flags": ["UP"], "mtu": 72,
                "linkinfo": {"info_data": {"state": "ERROR-ACTIVE", "ctrlmode": ["FD"],
                "bittiming": {"bitrate": 1000000}, "data_bittiming": {"bitrate": 5000000}}}}]}

    def test_configured_channel_is_required(self):
        inventory = self.inventory()
        self.assertEqual(testtool.validate_channel(inventory, "can1", True)["ifname"], "can1")
        with self.assertRaises(RuntimeError):
            testtool.validate_channel(inventory, "can0", True)
        with self.assertRaises(ValueError):
            testtool.validate_channel(inventory, "../../can0", True)

    def test_inactive_classic_bus_off_and_bad_rate_rejected(self):
        modifications = [("flags", []), ("mtu", 16)]
        for key, value in modifications:
            inventory = self.inventory()
            inventory["interfaces"][0][key] = value
            with self.assertRaises(RuntimeError):
                testtool.validate_channel(inventory, "can1", True)
        for key, value in [("state", "BUS-OFF"), ("data_bittiming", {"bitrate": 2000000}),
                           ("ctrlmode", ["FD", "LOOPBACK"]), ("ctrlmode", ["FD", "LISTEN-ONLY"])]:
            inventory = self.inventory()
            inventory["interfaces"][0]["linkinfo"]["info_data"][key] = value
            with self.assertRaises(RuntimeError):
                testtool.validate_channel(inventory, "can1", True)


if __name__ == "__main__":
    unittest.main()
