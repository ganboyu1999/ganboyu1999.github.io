import argparse
import json
import math
import shlex
import sys
from pathlib import Path

import frs


def inspect_robot(robot, max_age_ns=100_000_000):
    status = robot.system.status()
    hardware = robot.capabilities.hardware("whole_body")
    state = robot.state.snapshot()
    count = int(hardware.parameters["joint_count"])
    positions = state.joints.get("position", [])
    if state.quality != "good" or not 0 <= state.age_ns <= max_age_ns:
        raise RuntimeError("state is unavailable or stale")
    if count <= 0 or len(positions) != count:
        raise RuntimeError("joint position dimension does not match capability")
    if not all(math.isfinite(value) for value in positions):
        raise RuntimeError("joint position contains NaN or Inf")
    return {
        "health": status.health,
        "product_model": status.product_model,
        "control_owner": status.control_owner,
        "joint_count": count,
        "quality": state.quality,
        "age_ns": state.age_ns,
    }


def main():
    parser = argparse.ArgumentParser(description="Read FRS state without sending commands")
    parser.add_argument("--fixture", action="store_true",
                        help="use the repository's local JSON Lines fixture")
    args = parser.parse_args()
    if args.fixture:
        root = Path(__file__).resolve().parents[3]
        command = shlex.join([sys.executable, str(root / "tests/fixtures/platform_adapter.py")])
        connection = frs.connect(command=command)
    else:
        connection = frs.connect(socket_only=True)
    with connection as robot:
        print(json.dumps(inspect_robot(robot), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
