import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fist_test as motion


class Clock:
    def __init__(self):
        self.value = 0.
    def now(self):
        return self.value
    def sleep(self, duration):
        self.value += duration


class Hand:
    def __init__(self, enabled=1):
        self.feedback = [2000] * 10
        self.enabled = enabled
        self.writes = []
        self.errors = 0
        self.fail_error_at = None
        self.no_ack = False
        self.lag = False

    def exchange(self, node, command, timeout):
        if command == 0xCD:
            return [{"software_version": "1.1.9", "hardware_version": "3.0.0", "can_id": 9}]
        if command == 0x02:
            return [{"enable_state": self.enabled}]
        if command == 0x09:
            return [{"positions_raw": list(self.feedback)}]
        raise AssertionError(command)

    def check_error(self):
        self.errors += 1
        if self.fail_error_at == self.errors:
            raise RuntimeError("Hand error code 1")

    def set_axis(self, axis, target):
        motion.single_position_frame(axis, target)
        self.writes.append((axis, target))
        if self.no_ack:
            raise RuntimeError("position response timed out")
        if not self.lag:
            self.feedback[axis-1] = target
        return self.feedback[axis-1]


class FistTest(unittest.TestCase):
    def run_hand(self, hand, target):
        report = {}
        clock = Clock()
        motion.run_motion(hand, target, report, clock.sleep, clock.now)
        return report

    def test_single_axis_frame_has_only_position_command_and_fixed_node(self):
        node, length, flags, data = motion.ro.FD_FRAME.unpack(motion.single_position_frame(3, 1446))
        self.assertEqual((node, length, flags), (9, 4, 1))
        self.assertEqual(data[:4], b"\x06\x03" + struct.pack("<H", 1446))
        for axis, target in [(0, 100), (11, 100), (1, -1), (1, 4097), (True, 100)]:
            with self.assertRaises(ValueError):
                motion.single_position_frame(axis, target)

    def test_trajectory_step_bound_and_exact_endpoint(self):
        start = [2000] * 10
        target = [0, 4096, 1446, 0, 46, 46, 0, 46, 4096, 46]
        last = start
        for step in motion.trajectory(start, target):
            self.assertTrue(all(abs(a-b) <= 24 for a, b in zip(last, step)))
            motion.positions_checked(step)
            last = step
        self.assertEqual(last, target)

    def test_recorded_overshoot_is_bounded_for_planning_not_transmitted(self):
        self.assertEqual(motion.initial_positions([4097, 4098] + [2000]*8)[:2], [4096, 4096])
        with self.assertRaises(RuntimeError):
            motion.initial_positions([4113]*10)

    def test_disabled_and_calibration_states_never_write(self):
        for state in (0, 2):
            hand = Hand(state)
            with self.assertRaisesRegex(RuntimeError, "already be enabled"):
                self.run_hand(hand, [2100]*10)
            self.assertEqual(hand.writes, [])

    def test_initial_error_prevents_movement(self):
        hand = Hand()
        hand.fail_error_at = 1
        with self.assertRaisesRegex(RuntimeError, "error code"):
            self.run_hand(hand, [2100]*10)
        self.assertEqual(hand.writes, [])

    def test_missing_ack_stops_after_first_axis(self):
        hand = Hand()
        hand.no_ack = True
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.run_hand(hand, [2100]*10)
        self.assertEqual(len(hand.writes), 1)

    def test_persistent_lag_aborts_before_final_goal(self):
        hand = Hand()
        hand.lag = True
        with self.assertRaisesRegex(RuntimeError, "lag persisted|tracking error"):
            self.run_hand(hand, [4000]*10)
        self.assertLess(max(v for _, v in hand.writes), 4000)

    def test_feedback_success_and_no_enable_calibration_calls(self):
        hand = Hand()
        report = self.run_hand(hand, [2060]*10)
        self.assertTrue(report["position_test_passed"])
        self.assertEqual(report["final_positions_raw"], [2060]*10)

    def test_dry_preview_does_not_inspect_or_open_can(self):
        with patch.object(sys, "argv", ["fist_test.py", "--channel", "can1"]), \
             patch.object(motion.ro, "interface_inventory", side_effect=AssertionError("CAN access")), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(motion.main(), 0)
        self.assertIn("PREVIEW ONLY", output.getvalue())

    def test_checked_in_targets_match_official_right_solver_output(self):
        profile = json.loads(motion.PROFILE.read_text(encoding="utf-8"))
        self.assertEqual(profile["hand"], "right")
        self.assertEqual(profile["fist"], [2458, 849, 1446, 0, 46, 46, 0, 46, 4096, 46])
        self.assertEqual(profile["open"], [1924, 631, 4093, 0, 4069, 4069, 0, 4069, 4096, 4069])
        for name in ("fist", "open"):
            motion.positions_checked(profile[name])


if __name__ == "__main__":
    unittest.main()
