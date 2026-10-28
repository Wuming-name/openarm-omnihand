import ast
import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest

from real_robot import protocol as p
from real_robot.settings import DEFAULT, load
from real_robot.servo import Servo
from real_robot.can_bus import MotorBus


PHYSICS = {"kp": [100.]*7, "kd": [12.]*7, "vmax": [.3]*7, "max_target_lead_rad": .2, "torque_limits": [40.,40.,27.,27.,7.,7.,7.]}


class Protocol(unittest.TestCase):
    def test_known_zero_and_endpoint_packets(self):
        self.assertEqual(p.mit_payload("DM4310", 0., 0., 0., 0., 0.).hex(), "7fff7ff0000007ff")
        self.assertEqual(p.mit_payload("DM8009", 12.5, 45., 500., 5., 54.), b"\xff"*8)
        self.assertEqual(p.mit_payload("DM4340", -12.5, -10., 0., 0., -28.), b"\0"*8)

    def test_native_kd_cannot_silently_clip_trained_gain(self):
        for kind in p.LIMITS:
            with self.assertRaises(ValueError):
                p.mit_payload(kind, 0., 0., 1., 12., 0.)
        with self.assertRaises(ValueError):
            p.mit_payload("DM8009", float("nan"), 0., 0., 0., 0.)

    def test_query_is_read_only_and_zero_command_not_exposed(self):
        self.assertEqual(p.read_param(7, 21), bytes.fromhex("0700331500000000"))
        self.assertEqual(p.refresh_payload(1), bytes.fromhex("0100cc0000000000"))
        with self.assertRaises(ValueError):
            p.read_param(1, 55)
        self.assertFalse(hasattr(p, "set_zero"))

    def test_parameter_reply_routing_and_little_endian(self):
        motor = load()["motors"][0]
        data = bytes.fromhex("01003317")+struct.pack("<f", 54.)
        self.assertEqual(p.parameter_reply(motor, 17, data), (23,54.))
        self.assertIsNone(p.parameter_reply(motor, 18, data))
        self.assertIsNone(p.state_reply(motor, 17, data))

    def test_motor_state_sign_offset_status_and_temperature(self):
        motor = copy.deepcopy(load()["motors"][0])
        motor.update(sign=-1, offset_rad=.5)
        data = bytes.fromhex("11bfff7ff7ff2832")
        result = p.state_reply(motor, 17, data)
        expected = -((0xbfff/65535*25-12.5)-.5)
        self.assertAlmostEqual(result["q"], expected)
        self.assertEqual((result["status"],result["mos_c"],result["rotor_c"]), (1,40,50))
        self.assertIsNone(p.state_reply(motor, 18, data))
        self.assertIsNone(p.state_reply(motor, 17, b"\0"))

    def test_local_echo_extended_remote_and_error_frames_rejected(self):
        raw=p.pack_fd(17, b"\x11"+b"\0"*7)
        self.assertIsNotNone(p.unpack(raw))
        self.assertIsNone(p.unpack(raw,4))
        self.assertIsNone(p.unpack(raw,0x800))
        for flag in (0x80000000,0x40000000,0x20000000):
            self.assertIsNone(p.unpack(p.FD.pack(17|flag,8,1,b"\0"*64)))


class Safety(unittest.TestCase):
    def servo(self):
        return Servo(load(), PHYSICS, [-1.]*7, [1.]*7)

    def test_no_torque_before_valid_reference(self):
        self.assertIsNone(self.servo().tick([0.]*7,[0.]*7,0.))

    def test_software_pd_supports_training_kd_and_total_torque_clip(self):
        s=self.servo()
        s.accept([.1]*7,[.1]*7,[30.]*7,[0.]*7,0.)
        self.assertEqual(s.tick([0.]*7,[0.]*7,0.), [40.,40.,27.,27.,7.,7.,7.])

    def test_stale_commands_latch_current_pose_hold(self):
        s=self.servo()
        s.accept([.1]*7,[0.]*7,[1.]*7,[0.]*7,0.)
        torque=s.tick([.04]*7,[0.]*7,.11)
        self.assertEqual(s.fault,"command_timeout")
        self.assertEqual(s.target[0],[.04]*7)
        self.assertEqual(torque,[1.]*7)
        s.accept([.05]*7,[0.]*7,[0.]*7,[.04]*7,.12)
        self.assertEqual(s.target[0],[.04]*7)

    def test_bad_reference_not_accepted(self):
        s=self.servo()
        for q,v,ff in [([1.1]*7,[0.]*7,[0.]*7),([.3]*7,[0.]*7,[0.]*7),([0.]*7,[.4]*7,[0.]*7),([0.]*7,[0.]*7,[float('nan')]*7)]:
            with self.assertRaises(ValueError):
                s.accept(q,v,ff,[0.]*7,0.)

    def test_loop_deadline_latches_fault(self):
        s=self.servo();s.accept([0.]*7,[0.]*7,[0.]*7,[0.]*7,0.)
        s.tick([0.]*7,[0.]*7,0.)
        s.tick([0.]*7,[0.]*7,.03)
        self.assertEqual(s.fault,"servo_deadline")

    def test_continuous_rating_guard_is_not_protocol_full_scale(self):
        s=self.servo()
        for k in range(260):
            now=k*.002
            s.accept([0.]*7,[0.]*7,[0.,0.,0.,0.,4.,0.,0.],[0.]*7,now)
            if k<=250:
                s.tick([0.]*7,[0.]*7,now)
            else:
                with self.assertRaisesRegex(RuntimeError,"continuous"):
                    s.tick([0.]*7,[0.]*7,now)
                break

    def test_live_mode_requires_real_calibration_not_only_official_defaults(self):
        with self.assertRaisesRegex(ValueError,"calibration"):
            load(live=True)
        config=load()
        config["calibration"].update(joint_mapping_and_zero_verified=True, hand_pose_and_tcp_match_training=True,
            hardware_stop_and_support_verified=True,motor_timeout_raw_verified=123,motor_timeout_seconds_verified=.05)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'hardware.json'
            path.write_text(json.dumps(config),encoding='utf-8')
            self.assertEqual(load(path,live=True)["arm_channel"],"can0")

    def test_all_sources_parse_as_python310(self):
        for path in DEFAULT.parent.parent.glob('*.py'):
            ast.parse(path.read_text(encoding='utf-8'),feature_version=(3,10))

    def bus_without_socket(self):
        bus=MotorBus.__new__(MotorBus)
        bus.config=load()
        bus.parameters={}
        bus.checked=False
        bus.allow_control=True
        for motor in bus.config['motors']:
            for rid,value in zip((7,8,9,10,21,22,23),(motor['rx'],motor['tx'],123,1,*p.LIMITS[motor['protocol']])):
                bus.parameters[(motor['tx'],rid)]=value
        return bus

    def test_live_parameter_verification_handles_per_axis_timeout(self):
        bus=self.bus_without_socket()
        bus.config['calibration']['motor_timeout_raw_verified']=[123]*7
        bus.validate_parameters(live=True)
        self.assertTrue(bus.checked)
        bus.parameters[(7,23)]=7.0  # 7 Nm mechanical peak is NOT TMAX=10 mapping
        with self.assertRaisesRegex(RuntimeError,'protocol range'):
            bus.validate_parameters(live=True)
        self.assertFalse(bus.checked)

    def test_read_only_and_unverified_session_never_send_torque(self):
        bus=self.bus_without_socket()
        sent=[]
        bus.send=lambda cid,payload:sent.append((cid,payload))
        with self.assertRaises(RuntimeError):bus.torques([0.]*7)
        bus.checked=True
        bus.allow_control=False
        with self.assertRaises(RuntimeError):bus.enable()
        with self.assertRaises(RuntimeError):bus.torques([0.]*7)
        self.assertEqual(sent,[])

    def test_invalid_seventh_axis_does_not_partially_transmit_first_six(self):
        bus=self.bus_without_socket()
        bus.checked=True
        sent=[]
        bus.send=lambda cid,payload:sent.append((cid,payload))
        with self.assertRaises(ValueError):bus.torques([1.]*6+[8.])
        with self.assertRaises(ValueError):bus.torques([1.]*6+[float('nan')])
        self.assertEqual(sent,[])


if __name__ == '__main__':
    unittest.main()
