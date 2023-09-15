from rclpy.node import Node
import rclpy
from expandroid_msgs.msg import JoyAndApp
from expandroid_msgs.msg import ExpandroidState, ExpandroidHandCommand
import json
import serial
from typing import Optional, List
import numpy as np
import time


OPEN = [135, 45, 135, 45, 135, 45]

CLOSE = [10, 150, 10, 150, 10, 150]


class ExpandroidHandNode(Node):
    def __init__(self):
        super().__init__("expandroid_hand_node")
        # subscribe joy_and_app
        self.create_subscription(JoyAndApp, "joy_and_app", self.joy_callback, 10)
        # subscribe expandroid_state
        self.create_subscription(
            ExpandroidState, "expandroid_state", self.state_callback, 10
        )
        # subscribe hand_command
        self.create_subscription(
            ExpandroidHandCommand, "expandroid_hand_command", self.hand_command_callback, 10
        )

        # open field config
        self._path_to_field_config = self.declare_parameter(
            "path_to_field_config"
        ).value
        self._field_config: dict = json.load(open(self._path_to_field_config, "r"))

        self._serial_port = serial.Serial("/dev/ttyUSB0", 9600)
        time.sleep(2)

        self.send_hand_command([1 for _ in range(6)])  # close hand

    def hand_command_callback(self, msg: ExpandroidHandCommand):
        self.get_logger().info("hand_command: {}".format(msg))
        if msg.value == 0:  # open
            if hasattr(self, "_current_state"):
                hand_command = self.get_hand_open_command_from_state(
                    msg.color.data, self._current_state
                )
            else:
                self.get_logger().error("current_state is not defined")

        elif msg.value == 1:  # close
            hand_command = [1 for _ in range(6)]

        elif msg.value == 2:  # open all
            hand_command = [0 for _ in range(6)]

        elif msg.value == 3:  # middle
            hand_command = [0.5 for _ in range(6)]
        self.get_logger().info("hand_command: {}".format(hand_command))
        self.send_hand_command(hand_command)

    def joy_callback(self, msg: JoyAndApp):
        if msg.type.data == "hand":
            if msg.value == 0:  # open
                if hasattr(self, "_current_state"):
                    hand_command = self.get_hand_open_command_from_state(
                        msg.color.data, self._current_state
                    )
                else:
                    self.get_logger().error("current_state is not defined")

            elif msg.value == 1:  # close
                hand_command = [1 for _ in range(6)]

            elif msg.value == 2:  # open all
                hand_command = [0 for _ in range(6)]

            elif msg.value == 3:  # middle
                hand_command = [0.5 for _ in range(6)]
            self.get_logger().info("hand_command: {}".format(hand_command))
            self.send_hand_command(hand_command)

    def state_callback(self, msg: ExpandroidState):
        self._current_state = msg

    def get_hand_open_command_from_state(
        self, color: str, current_state: ExpandroidState
    ) -> Optional[int]:
        min_difference = float("inf")  # 無限大の値で初期化
        best_hand = None

        for field_position_prop in self._field_config[color].values():
            if len(field_position_prop["pos"]) == 0:
                continue
            pos = field_position_prop["pos"][-1]
            hand_angle, x_angle, y_angle, z_angle = pos

            # Calculate the sum of absolute differences
            total_difference = (
                abs(hand_angle - current_state.hand_angle)
                + abs(x_angle - current_state.x_angle)
                + abs(y_angle - current_state.y_angle)
            )

            # Check if this is the minimum difference we've found so far
            if total_difference < min_difference:
                min_difference = total_difference
                best_hand = field_position_prop["hand"]

        return best_hand

    def send_hand_command(self, hand_command: List[float]):
        hand_command = np.array(hand_command)
        hand_open = np.array(OPEN)
        hand_close = np.array(CLOSE)
        hand_angle = hand_open + (hand_close - hand_open) * hand_command
        hand_angle = hand_angle.astype(int).tolist()
        self.get_logger().info("hand_angle: {}".format(hand_angle))
        angle_str = ",".join(map(str, hand_angle))
        self._serial_port.write((angle_str + "\n").encode("utf-8"))
        self.get_logger().info("Send hand command")


def main():
    rclpy.init()
    node = ExpandroidHandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
