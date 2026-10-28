from pathlib import Path
import ast
import copy
import tempfile
import unittest
import numpy as np

from rl_ppo.common import DEFAULT_CONFIG, OBSERVATION_FIELDS, config_hash, load_config, write_json
from rl_ppo.model import rotation_vector


class ContractTest(unittest.TestCase):
    def test_observation_contract_size(self):
        self.assertEqual(sum(size for _, size in OBSERVATION_FIELDS), 37)

    def test_fixed_config_and_invalid_control_period(self):
        config = load_config()
        self.assertEqual(config["physics"]["control_dt"], .02)
        config["physics"]["control_dt"] = .019
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"config.json"
            write_json(path, config)
            with self.assertRaises(ValueError):
                load_config(path)

    def test_invalid_gain_dimensions_or_nonpositive_limit(self):
        for key, value in [("kp", [1]*6), ("torque_limits", [-1]*7)]:
            config = load_config()
            config["physics"][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"config.json"
                write_json(path, config)
                with self.assertRaises(ValueError):
                    load_config(path)

    def test_rotation_error_handles_identity_and_pi(self):
        np.testing.assert_allclose(rotation_vector(np.eye(3)), [0, 0, 0], atol=1e-10)
        self.assertAlmostEqual(np.linalg.norm(rotation_vector(np.diag([1., -1., -1.]))), np.pi)

    def test_config_fingerprint_order_independent_but_value_sensitive(self):
        config = load_config()
        self.assertEqual(config_hash(config), config_hash(dict(reversed(list(config.items())))))
        changed = copy.deepcopy(config)
        changed["tcp_offset_palm_m"][0] += .001
        self.assertNotEqual(config_hash(config), config_hash(changed))

    def test_target_python_310_syntax_and_no_hardware_imports(self):
        directory = DEFAULT_CONFIG.parent.parent
        for path in directory.glob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), feature_version=(3, 10))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [alias.name.split(".")[0] for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [(node.module or "").split(".")[0]]
                else:
                    continue
                self.assertFalse(set(names) & {"rclpy", "can", "socket", "serial"}, str(path))


if __name__ == "__main__":
    unittest.main()
