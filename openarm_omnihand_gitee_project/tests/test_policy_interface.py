"""Check coordinate mapping invariants that matter for later simulator adapters."""
from pathlib import Path
import json
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from policy_interface import load_contract, expand_positions, normalized_to_positions


class PolicyCoordinates(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_contract(ROOT)

    def test_delivered_poses_roundtrip_and_six_mimics(self):
        poses = json.loads((ROOT / "model/ros2_description/config/poses.json").read_text())
        for pose in poses.values():
            independent = {name: pose[name] for name in self.contract["independent_joint_names"]}
            expanded = expand_positions(independent, self.contract)
            self.assertEqual(len(expanded), 23)
            for name, value in pose.items():
                self.assertAlmostEqual(expanded[name], value, places=12)

    def test_normalized_endpoints_respect_all_source_limits(self):
        for value in (-1, 0, 1):
            expanded = normalized_to_positions([value] * 17, self.contract)
            for name, position in expanded.items():
                bounds = self.contract["source_position_limits"][name]
                self.assertGreaterEqual(position, bounds["lower"] - 1e-12)
                self.assertLessEqual(position, bounds["upper"] + 1e-12)

    def test_bad_dimensions_nan_and_out_of_range_are_rejected(self):
        for action in ([0] * 16, [0] * 18, [math.nan] * 17, [math.inf] * 17, [1.01] * 17):
            with self.assertRaises(ValueError):
                normalized_to_positions(action, self.contract)
        with self.assertRaises(ValueError):
            expand_positions({}, self.contract)

    def test_mimic_joint_cannot_be_supplied_as_independent_input(self):
        pose = dict.fromkeys(self.contract["independent_joint_names"], 0)
        pose[next(iter(self.contract["mimic"]))] = 0
        with self.assertRaises(ValueError):
            expand_positions(pose, self.contract)


if __name__ == "__main__":
    unittest.main()
