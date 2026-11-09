import argparse
import json

import frs


def read_sample(robot, topic, max_age_ns=100_000_000):
    sample = robot.topic.wait(topic, timeout=1.0)
    if not sample.available or not sample.fresh:
        raise RuntimeError(sample.frs.error or "topic is unavailable or stale")
    if not 0 <= sample.frs.age_ns <= max_age_ns:
        raise RuntimeError("topic exceeds this application's freshness limit")
    return sample


def main():
    parser = argparse.ArgumentParser(description="Read one fresh sample from a running FRS")
    parser.add_argument("topic", help="read name from frs topic list")
    args = parser.parse_args()
    with frs.connect(socket_only=True) as robot:
        sample = read_sample(robot, args.topic)
        print(json.dumps({"topic": sample.topic, "type": sample.message_type,
                          "sequence": sample.frs.sequence, "msg": sample.msg},
                         ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
