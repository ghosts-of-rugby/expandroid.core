from enum import Enum

import rclpy
import rclpy.node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from sensor_msgs.msg import Joy

from expandroid_msgs.action import TrajectoryTracking
from expandroid_msgs.msg import ExpandroidCommand


class CommandMode(Enum):
    SPEED_CTRL = 0
    TRAJECTORY_TRACKING = 1


class ExpandroidMainNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_main")
        # subscribe joystick
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
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

    def joy_callback(self, msg: Joy):
        if self._command_mode == CommandMode.TRAJECTORY_TRACKING:
            self.get_logger().info("Trajectory tracking mode")

            if msg.buttons[2]:  # X button
                self.get_logger().info("Cancel trajectory tracking")
                self._goal_handle.cancel_goal_async()
                self._command_mode = CommandMode.SPEED_CTRL

        elif self._command_mode == CommandMode.SPEED_CTRL:
            self.get_logger().info("Speed control mode")

            if msg.buttons[0]:  # A button
                self._command_mode = CommandMode.TRAJECTORY_TRACKING
                self.get_logger().info("Start trajectory tracking")
                goal_msg = TrajectoryTracking.Goal()
                goal_msg.reference_angle.hand_command = 0.0
                goal_msg.reference_angle.x_command = 0.0
                self._trajectory_tracking_action_client.wait_for_server()
                future = self._trajectory_tracking_action_client.send_goal_async(
                    goal_msg
                )
                future.add_done_callback(self.trajectory_tracing_goal_response_callback)

                self.get_logger().info("Goal sent")
                return

            self._command_mode = CommandMode.SPEED_CTRL

            if msg.buttons[4]:
                self._speed_command.hand_command = 0.3
            elif msg.buttons[5]:
                self._speed_command.hand_command = -0.3
            else:
                self._speed_command.hand_command = 0.0
            self._speed_command.x_command = msg.axes[0] * 0.3

    def trajectory_tracing_goal_response_callback(self, future: Future):
        self._goal_handle: ClientGoalHandle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        get_result_future: Future = self._goal_handle.get_result_async()

        get_result_future.add_done_callback(
            self.trajectory_tracing_get_result_callback
        )

    def trajectory_tracing_get_result_callback(self, future):
        # result = future.result().result
        self.get_logger().info("Goal result received")
        self._command_mode = CommandMode.SPEED_CTRL

    def commander_callback(self):
        if self._command_mode == CommandMode.SPEED_CTRL:
            self.speed_command_publisher.publish(self._speed_command)
        # elif self._command_mode == CommandMode.TRAJECTORY_TRACKING:
        #     self.angle_command_publisher.publish(self._angle_command)


def main():
    rclpy.init()
    node = ExpandroidMainNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
