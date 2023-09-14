import json

import rclpy
import rclpy.node

# from ament_index_python import get_package_share_directory
from expandroid_msgs.msg import ExpandroidCommand, JoyAndApp, ExpandroidState
from expandroid_commander.get_speed_command import (
    get_speed_command,
    modify_speed_command,
)


class ExpandroidParameterSetup(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_parameter_setup")

        self._path_to_field_config = self.declare_parameter(
            "path_to_field_config"
        ).value

        # subscribe joystick
        self.create_subscription(JoyAndApp, "joy_and_app", self.joy_callback, 10)
        self.create_subscription(
            ExpandroidState, "expandroid_state", self.state_callback, 10
        )
        # publish command
        self._speed_command = ExpandroidCommand()
        self.speed_command_publisher = self.create_publisher(
            ExpandroidCommand, "expandroid_speed_command", 10
        )
        # timer
        self.timer = self.create_timer(0.1, self.commander_callback)

    def joy_callback(self, msg: JoyAndApp):
        joy_msg = msg.joy

        if msg.type.data == "pos" and hasattr(self, "_current_state"):
            self._field_config = json.load(open(self._path_to_field_config, "r"))

            self.get_logger().info("color: {}".format(msg.color.data))
            self.get_logger().info("button_num: {}".format(msg.value))
            self.get_logger().info("state: {}".format(self._current_state))

            self._field_config[msg.color.data][str(msg.value)]["pos"].append(
                [
                    round(self._current_state.hand_angle, 3),
                    round(self._current_state.x_angle, 3),
                    round(self._current_state.y_angle, 3),
                    # round(self._current_state.z_angle, 3),
                    0.0,
                ]
            )
            json.dump(
                self._field_config,
                open(self._path_to_field_config, "w"),
            )

        self._speed_command = get_speed_command(joy_msg, self._speed_command)
        if hasattr(self, "_current_state"):
            self._speed_command = modify_speed_command(
                self._current_state, self._speed_command
            )

    def commander_callback(self):
        self.speed_command_publisher.publish(self._speed_command)

    def state_callback(self, msg: ExpandroidState):
        self._current_state = msg


def main():
    rclpy.init()
    node = ExpandroidParameterSetup()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
