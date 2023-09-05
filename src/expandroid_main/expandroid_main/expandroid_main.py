import rclpy
import rclpy.node
from sensor_msgs.msg import Joy
from expandroid_msg.msg import ExpandroidCommand
from enum import Enum


class CommandMode(Enum):
    SPEED = 0
    ANGLE = 1


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
        self._angle_command = ExpandroidCommand()
        self.angle_command_publisher = self.create_publisher(
            ExpandroidCommand, "expandroid_angle_command", 10
        )

        self._command_mode = CommandMode.SPEED
        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joy_callback(self, msg: Joy):
        self._command_mode = CommandMode.ANGLE
        if msg.buttons[0]:
            self._angle_command.hand_command = 0.1
            self._angle_command.x_command = 0.0
            return
        elif msg.buttons[1]:
            self._angle_command.hand_command = -0.1
            self._angle_command.x_command = -0.5
            return
        elif msg.buttons[2]:
            self._angle_command.hand_command = 0.0
            self._angle_command.x_command = 0.5
            return

        self._command_mode = CommandMode.SPEED

        if msg.buttons[4]:
            self._speed_command.hand_command = 0.3
        elif msg.buttons[5]:
            self._speed_command.hand_command = -0.3
        else:
            self._speed_command.hand_command = 0.0
        self._speed_command.x_command = msg.axes[0] * 0.3

    def timer_callback(self):
        if self._command_mode == CommandMode.SPEED:
            self.speed_command_publisher.publish(self._speed_command)
        elif self._command_mode == CommandMode.ANGLE:
            self.angle_command_publisher.publish(self._angle_command)


def main():
    rclpy.init()
    node = ExpandroidMainNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
