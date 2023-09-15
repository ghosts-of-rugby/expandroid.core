import json
from enum import Enum
import time

import rclpy
import rclpy.node

# from ament_index_python import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from expandroid_msgs.action import TrajectoryTracking
from expandroid_msgs.msg import (
    ExpandroidCommand,
    JoyAndApp,
    ExpandroidState,
    ExpandroidHandCommand,
)
from expandroid_commander.get_speed_command import (
    get_speed_command,
    modify_speed_command,
)


class CommandMode(Enum):
    BEFORE_START = -1
    SPEED_CTRL = 0
    TRAJECTORY_TRACKING = 1


class ExpandroidMainNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_commander")

        path_to_field_config = self.declare_parameter("path_to_field_config").value

        self._field_config = json.load(open(path_to_field_config, "r"))

        # subscribe joystick
        self.create_subscription(JoyAndApp, "joy_and_app", self.joy_callback, 10)
        # subscribe state
        self.create_subscription(
            ExpandroidState, "expandroid_state", self.state_callback, 10
        )
        # publish command
        self._speed_command = ExpandroidCommand()
        self.speed_command_publisher = self.create_publisher(
            ExpandroidCommand, "expandroid_speed_command", 10
        )
        self._command_mode = CommandMode.BEFORE_START

        # publish hand command
        self.hand_command_publisher = self.create_publisher(
            ExpandroidHandCommand, "expandroid_hand_command", 10
        )

        self._trajectory_tracking_action_client = ActionClient(
            self, TrajectoryTracking, "trajectory_tracking"
        )
        # timer
        self.timer = self.create_timer(0.1, self.commander_callback)

    def joy_callback(self, msg: JoyAndApp):
        self._recieved_msg = msg
        joy_msg = msg.joy
        if self._command_mode == CommandMode.BEFORE_START:
            self._command_mode = CommandMode.SPEED_CTRL
            self.get_logger().info("Start speed control")

        elif self._command_mode == CommandMode.TRAJECTORY_TRACKING:
            if joy_msg.buttons[2]:  # X button
                self.get_logger().info("Cancel trajectory tracking")
                self._goal_handle.cancel_goal_async()
                self._command_mode = CommandMode.SPEED_CTRL
                self.get_logger().info("Start speed control")

        elif self._command_mode == CommandMode.SPEED_CTRL:
            if msg.type.data == "pos":
                self._command_mode = CommandMode.TRAJECTORY_TRACKING
                self.get_logger().info("Start trajectory tracking")

                self._current_color = msg.color.data
                self._open_after_reaching: bool = self._field_config[
                    self._current_color
                ][str(msg.value)]["open_after_reaching"]

                # close hand before moving if y > 0
                if self._current_state.y_angle > 0.0:
                    hand_command = ExpandroidHandCommand()
                    hand_command.value = 1
                    hand_command.color.data = self._current_color
                    self.hand_command_publisher.publish(hand_command)
                    time.sleep(0.1)

                goal_msg = self.get_goal_msg(msg.color.data, msg.value)
                self._trajectory_tracking_action_client.wait_for_server()
                future = self._trajectory_tracking_action_client.send_goal_async(
                    goal_msg
                )
                future.add_done_callback(self.trajectory_tracing_goal_response_callback)
                self.get_logger().info("Goal sent")
                return

            self._speed_command = get_speed_command(joy_msg, self._speed_command)
            if hasattr(self, "_current_state"):
                self._speed_command = modify_speed_command(
                    self._current_state, self._speed_command
                )

    def trajectory_tracing_goal_response_callback(self, future: Future):
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        get_result_future: Future = self._goal_handle.get_result_async()

        get_result_future.add_done_callback(self.trajectory_tracing_get_result_callback)

    def trajectory_tracing_get_result_callback(self, future):
        # result = future.result().result
        self.get_logger().info("Goal result received")

        if self._open_after_reaching:
            hand_command = ExpandroidHandCommand()
            hand_command.value = 0
            hand_command.color.data = self._current_color
            self.hand_command_publisher.publish(hand_command)

        time.sleep(0.1)
        self._command_mode = CommandMode.SPEED_CTRL
        self.get_logger().info("Start speed control")

    def get_goal_msg(self, color: str, button_num: int):
        goal_msg = TrajectoryTracking.Goal()

        goal_msg.reference_angles.append(
            ExpandroidCommand(
                hand_command=self._current_state.hand_angle,
                x_command=self._current_state.x_angle,
                y_command=self._current_state.y_angle,
                z_command=0.0,
            )
        )

        if self._current_state.y_angle > 0.92:
            goal_msg.reference_angles.append(
                ExpandroidCommand(
                    hand_command=0.0,
                    x_command=self._current_state.x_angle,
                    y_command=self._current_state.y_angle,
                    z_command=0.0,
                )
            )

        for ref_pos in self._field_config[color][str(button_num)]["pos"]:
            goal_msg.reference_angles.append(
                ExpandroidCommand(
                    hand_command=ref_pos[0],
                    x_command=ref_pos[1],
                    y_command=ref_pos[2],
                    z_command=ref_pos[3],
                )
            )
        return goal_msg

    def commander_callback(self):
        if self._command_mode == CommandMode.SPEED_CTRL:
            self.speed_command_publisher.publish(self._speed_command)

    def state_callback(self, msg: ExpandroidState):
        self._current_state = msg


def main():
    rclpy.init()
    node = ExpandroidMainNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
