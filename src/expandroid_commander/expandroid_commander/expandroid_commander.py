import json
from enum import Enum

import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future

from expandroid_msgs.action import TrajectoryTracking
from expandroid_msgs.msg import ExpandroidCommand
from expandroid_msgs.msg import JoyAndApp


class CommandMode(Enum):
    SPEED_CTRL = 0
    TRAJECTORY_TRACKING = 1


class ExpandroidMainNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_commander")

        # get config/field.json from expandroid_commander
        self._field_config = json.load(
            open(
                get_package_share_directory("expandroid_commander")
                + "/config/field.json",
                "r",
            )
        )

        # subscribe joystick
        self.create_subscription(JoyAndApp, "joy_and_app", self.joy_callback, 10)
        # publish command
        self._speed_command = ExpandroidCommand()
        self.speed_command_publisher = self.create_publisher(
            ExpandroidCommand, "expandroid_speed_command", 10
        )
        self._command_mode = CommandMode.SPEED_CTRL

        self._trajectory_tracking_action_client = ActionClient(
            self, TrajectoryTracking, "trajectory_tracking"
        )
        # timer
        self.timer = self.create_timer(0.1, self.commander_callback)

    def joy_callback(self, msg: JoyAndApp):
        joy_msg = msg.joy

        if self._command_mode == CommandMode.TRAJECTORY_TRACKING:
            # self.get_logger().info("Trajectory tracking mode")

            if joy_msg.buttons[2]:  # X button
                self.get_logger().info("Cancel trajectory tracking")
                self._goal_handle.cancel_goal_async()
                self._command_mode = CommandMode.SPEED_CTRL

        elif self._command_mode == CommandMode.SPEED_CTRL:
            # self.get_logger().info("Speed control mode")

            if msg.color.data != "none":
                self._command_mode = CommandMode.TRAJECTORY_TRACKING
                self.get_logger().info("Start trajectory tracking")
                goal_msg = self.get_goal_msg(msg.color.data, msg.button_num)
                self._trajectory_tracking_action_client.wait_for_server()
                future = self._trajectory_tracking_action_client.send_goal_async(
                    goal_msg
                )
                future.add_done_callback(self.trajectory_tracing_goal_response_callback)
                self.get_logger().info("Goal sent")
                return

            self._command_mode = CommandMode.SPEED_CTRL

            if joy_msg.buttons[4]:
                self._speed_command.hand_command = 0.1
            elif joy_msg.buttons[5]:
                self._speed_command.hand_command = -0.1
            else:
                self._speed_command.hand_command = 0.0

            self._speed_command.x_command = joy_msg.axes[0] * 0.3
            self._speed_command.y_command = joy_msg.axes[1] * 0.3

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
        self._command_mode = CommandMode.SPEED_CTRL

    def get_goal_msg(self, color: str, button_num: int):
        goal_msg = TrajectoryTracking.Goal()
        for command in self._field_config[color][str(button_num)]:
            goal_msg.reference_angles.append(
                ExpandroidCommand(
                    hand_command=command[0],
                    x_command=command[1],
                    y_command=command[2],
                    z_command=command[3],
                )
            )
        return goal_msg

    def commander_callback(self):
        if self._command_mode == CommandMode.SPEED_CTRL:
            self.speed_command_publisher.publish(self._speed_command)


def main():
    rclpy.init()
    node = ExpandroidMainNode()
    rclpy.spin(node)
    rclpy.shutdown()
