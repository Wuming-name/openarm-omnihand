import copy
import unittest

from real_robot.can_bus import MotorBus
from real_robot.mit_servo import NativeMITServo, OFFICIAL_KP, OFFICIAL_KD
from real_robot.settings import load


class NativeMIT(unittest.TestCase):
    def setUp(self):
        self.hardware = load()
        self.physics = {'kp':[100.]*7,'kd':[12.]*7,'vmax':[.35]*7,
                        'max_target_lead_rad':.2,'torque_limits':[40.,40.,27.,27.,7.,7.,7.]}
        self.servo = NativeMITServo(self.hardware,self.physics,[-2.]*7,[2.]*7)

    def bus(self):
        bus = MotorBus.__new__(MotorBus)
        bus.config = copy.deepcopy(self.hardware)
        bus.checked = bus.allow_control = True
        self.sent = []
        bus.send = lambda cid,data:self.sent.append((cid,data))
        return bus

    def command(self):
        self.servo.accept([.01]*7,[.02]*7,[.1]*7,[0.]*7,0.)
        return self.servo.tick([0.]*7,[0.]*7,0.,[0.]*7)

    def test_official_gains_and_feedforward_only_no_double_pd(self):
        commands = self.command()
        for i,c in enumerate(commands):
            self.assertEqual(c['kp'],OFFICIAL_KP[i])
            self.assertEqual(c['kd'],OFFICIAL_KD[i])
            self.assertEqual(c['q'],.01)
            self.assertEqual(c['velocity'],.02)
            self.assertEqual(c['torque'],.1)
        self.assertEqual(self.physics['kd'],[12.]*7)

    def test_wire_mapping_sign_offset_and_nonzero_native_gains(self):
        commands = self.command()
        commands[0].update(q=.2,velocity=.1,torque=.25)
        bus = self.bus()
        bus.config['motors'][0].update(sign=-1,offset_rad=.3)
        bus.mit_commands(commands)
        cid,d = self.sent[0]
        self.assertEqual(cid,1)
        qcode = (d[0]<<8)|d[1]
        vcode = (d[2]<<4)|(d[3]>>4)
        kcode = ((d[3]&15)<<8)|d[4]
        dcode = (d[5]<<4)|(d[6]>>4)
        tcode = ((d[6]&15)<<8)|d[7]
        self.assertAlmostEqual(qcode/65535*25-12.5,.1,delta=.0004)
        self.assertAlmostEqual(vcode/4095*90-45,-.1,delta=.022)
        self.assertAlmostEqual(kcode/4095*500,70.,delta=.123)
        self.assertAlmostEqual(dcode/4095*5,2.75,delta=.002)
        self.assertAlmostEqual(tcode/4095*108-54,-.25,delta=.027)

    def test_bad_last_axis_never_sends_partial_group(self):
        for key,value in (('kd',5.1),('q',float('nan')),('torque',8.)):
            commands = self.command()
            commands[-1][key] = value
            bus = self.bus()
            with self.assertRaises(ValueError):
                bus.mit_commands(commands)
            self.assertEqual(self.sent,[])

    def test_read_only_session_cannot_send_mit(self):
        commands = self.command()
        bus = self.bus()
        for attr in ('checked','allow_control'):
            setattr(bus,attr,False)
            with self.assertRaises(RuntimeError):
                bus.mit_commands(commands)
            setattr(bus,attr,True)
        self.assertEqual(self.sent,[])

    def test_component_budget_rejects_hidden_large_cancellation(self):
        # J7 P=+2Nm and FF=-6Nm total is only -4Nm, but budget is 8>6.9.
        q = [0.]*7
        target = [0.]*7
        target[6] = .2
        ff = [0.]*7
        ff[6] = -6.
        self.servo.accept(target,[0.]*7,ff,q,0.)
        with self.assertRaisesRegex(RuntimeError,'budget'):
            self.servo.tick(q,[0.]*7,0.)

    def test_measured_torque_peak_guard_and_continuous_duration(self):
        self.command()
        with self.assertRaisesRegex(RuntimeError,'mechanical peak'):
            self.servo.tick([0.]*7,[0.]*7,.002,[41.]+[0.]*6)
        servo = NativeMITServo(self.hardware,self.physics,[-2.]*7,[2.]*7)
        with self.assertRaisesRegex(RuntimeError,'continuous'):
            for i in range(60):
                now = i*.01
                servo.accept([0.]*7,[0.]*7,[0.]*7,[0.]*7,now)
                servo.tick([0.]*7,[0.]*7,now,[21.]+[0.]*6)

    def test_stale_command_latches_hold_and_cannot_rearm(self):
        self.command()
        self.servo.tick([0.]*7,[0.]*7,.11)
        self.assertIsNotNone(self.servo.fault)
        self.assertEqual(self.servo.target[0],[0.]*7)
        self.servo.accept([.01]*7,[0.]*7,[0.]*7,[0.]*7,.12)
        self.assertEqual(self.servo.target[0],[0.]*7)

    def test_nonfinite_states_and_overspeed_rejected(self):
        self.command()
        with self.assertRaises(RuntimeError):
            self.servo.tick([float('nan')]*7,[0.]*7,.002)
        with self.assertRaises(RuntimeError):
            self.servo.tick([0.]*7,[1.1]*7,.002)
