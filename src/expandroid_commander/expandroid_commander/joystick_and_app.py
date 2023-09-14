import socket

import rclpy
import rclpy.node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from expandroid_msgs.msg import JoyAndApp

SERVER_PORT = 13579


class ExtendJoystickNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_commander")
        # create android socket
        self._android_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._android_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._android_socket.setblocking(False)
        self._android_socket.bind(("", SERVER_PORT))
        self.get_logger().info("Android socket created")

        # subscribe joystick
        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # publish command
        self.joy_and_app_publisher = self.create_publisher(JoyAndApp, "joy_and_app", 10)

    def joy_callback(self, msg: Joy):
        additional_button = [0] * 16
        pub_msg = JoyAndApp()
        pub_msg.joy = msg
        pub_msg.color = String(data="none")
        pub_msg.type = String(data="none")
        pub_msg.value = -1
        try:
            # Try to receive data
            data, address = self._android_socket.recvfrom(
                4096
            )  # This line may throw BlockingIOError
            self.get_logger().info(
                "Received data from Android: " + data.decode("utf-8")
            )
            color, command_type, val = data.decode("utf-8").split(",")
            pub_msg.color = String(data=color)
            pub_msg.type = String(data=command_type)
            pub_msg.value = int(val)

        except BlockingIOError:
            pass

        msg.buttons.extend(additional_button)

        self.joy_and_app_publisher.publish(pub_msg)


def main():
    rclpy.init()
    node = ExtendJoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
