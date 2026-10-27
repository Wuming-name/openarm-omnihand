"""Name-based model coordinates only; no simulator stepping or hardware commands."""
from pathlib import Path
import json
import math

ROOT = Path(__file__).resolve().parents[1]


def load_contract(root=ROOT):
    return json.loads((Path(root) / "config/policy_contract.json").read_text(encoding="utf-8"))


def expand_positions(independent, contract=None):
    """Validate 17 independent radian positions and expand the six mimic joints."""
    contract = load_contract() if contract is None else contract
    names = contract["independent_joint_names"]
    if set(independent) != set(names):
        raise ValueError("Independent positions must contain exactly the 17 named joints")
    positions = {name: float(independent[name]) for name in names}
    for name, value in positions.items():
        limits = contract["effective_limits"][name]
        if not math.isfinite(value) or not limits["lower"] - 1e-12 <= value <= limits["upper"] + 1e-12:
            raise ValueError(f"Invalid model position: {name}={value}")
    for name, mimic in contract["mimic"].items():
        positions[name] = positions[mimic["joint"]] * mimic["multiplier"] + mimic["offset"]
    return {name: positions[name] for name in contract["state_joint_names"]}


def normalized_to_positions(action, contract=None):
    """Illustrative absolute-position mapping; zero action means range midpoint."""
    contract = load_contract() if contract is None else contract
    names = contract["independent_joint_names"]
    if len(action) != len(names):
        raise ValueError("Expected 17 action coordinates")
    positions = {}
    for name, coordinate in zip(names, action):
        value = float(coordinate)
        if not math.isfinite(value) or not -1 <= value <= 1:
            raise ValueError(f"Action for {name} must be finite and in [-1,1]")
        limits = contract["effective_limits"][name]
        positions[name] = limits["lower"] + (value + 1) * 0.5 * (limits["upper"] - limits["lower"])
    return expand_positions(positions, contract)
