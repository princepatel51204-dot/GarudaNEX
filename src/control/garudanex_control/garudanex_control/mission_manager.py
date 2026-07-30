from enum import Enum


class MissionState(Enum):
    WAIT_CONNECTION = 0
    STREAM_SETPOINTS = 1
    ARM = 2
    TAKEOFF = 3
    HOVER = 4
    LAND = 5
    COMPLETE = 6


class MissionManager:

    def __init__(self, node, motion, state):
        self.node = node
        self.motion = motion
        self.state = state

        self.current_state = MissionState.WAIT_CONNECTION

        self.node.get_logger().info(
            "Mission Manager Initialized"
        )

    def update(self):
        pass