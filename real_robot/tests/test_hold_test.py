import copy
import unittest
from types import SimpleNamespace

from real_robot.hold_test import snapshot, packets
from real_robot.settings import load
from real_robot import protocol as p


class SupportedHold(unittest.TestCase):
    def setUp(self):
        self.config = load()
        self.states = [dict(motor_q=.1, velocity=0., torque=0., status=0,
                            mos_c=25, rotor_c=25, received_monotonic=1.) for _ in range(7)]

    def test_native_targets_and_gains(self):
        encoded = packets(self.config, [.1]*7, self.states)
        self.assertEqual(len(encoded), 7)
        self.assertEqual(encoded[0], (1, p.mit_payload('DM8009', .1, 0., 10., .5, 0.)))

    def test_drift_last_axis_prevents_packet_return(self):
        self.states[-1]['motor_q'] = .14
        with self.assertRaises(RuntimeError):
            packets(self.config, [.1]*7, self.states)

    def test_nonfinite_target_rejected(self):
        with self.assertRaises(RuntimeError):
            packets(self.config, [.1]*6+[float('nan')], self.states)

    def test_continuous_torque_guard(self):
        self.states[-1]['torque'] = 4.
        with self.assertRaises(RuntimeError):
            packets(self.config, [.1]*7, self.states)

    def test_fresh_states_and_fault_rejection(self):
        bus = SimpleNamespace(states={i+1:s for i,s in enumerate(self.states)})
        self.assertEqual(len(snapshot(bus, self.config, 1.01)), 7)
        for key, value in [('received_monotonic', .8), ('status', 8), ('velocity', .2), ('mos_c', 71)]:
            bad = copy.deepcopy(bus)
            bad.states[1][key] = value
            with self.assertRaises(RuntimeError):
                snapshot(bad, self.config, 1.01)


if __name__ == '__main__':
    unittest.main()
