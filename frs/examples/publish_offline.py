import frs


class OfflineSink:
    def __init__(self):
        self.frames = []
        self.policy = None

    def publish(self, frame):
        self.frames.append(frame)

    def finish(self, policy, last_frame):
        self.policy = policy


def run_offline():
    sink = OfflineSink()
    with frs.Robot(sink, joint_count=3) as robot:
        command = frs.RobotJointCommand(position=[0.0, 0.0, 0.0],
                                        command_mode=frs.CommandMode.POSITION)
        robot.publish(command)
        robot.finish(frs.CompletionPolicy.SAFE_HOLD)
    return sink


if __name__ == "__main__":
    sink = run_offline()
    print(len(sink.frames), sink.policy.value)
