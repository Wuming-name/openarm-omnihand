"""Read seven motor IDs, ranges, mode, TIMEOUT and states. No enable, zero, or torque command."""
import argparse
import json
import subprocess
from pathlib import Path
from datetime import datetime

from .settings import DEFAULT, load
from .can_bus import MotorBus


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT))
    args = parser.parse_args()
    config = load(args.config)
    bus = None
    report = {"mode": "read_only", "motion_commands_sent": False}
    try:
        bus = MotorBus(config)
        report["motors"] = bus.inspect()
        bus.validate_parameters()
        report["official_protocol_match"] = True
    except (RuntimeError, ValueError, OSError, ImportError, subprocess.SubprocessError) as exc:
        report["error"] = str(exc)
        if bus:
            report["partial_parameters"] = {f"node_{tx}_RID_{rid}": value for (tx,rid),value in bus.parameters.items()}
            report["partial_states"] = bus.states
    finally:
        if bus:
            bus.close()
    output = Path("outputs/real_robot") / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_probe.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("REPORT:", output)
    return 0 if report.get("official_protocol_match") else 2


if __name__ == "__main__":
    raise SystemExit(main())
