import rclpy
import rclpy.node
from sensor_msgs.msg import Joy
import socket


SERVER_IP = "192.168.212.160"
SERVER_PORT = 13579


class ExtendJoystickNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("expandroid_commander")
        # create android socket
        self._android_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._android_socket.bind((SERVER_IP, SERVER_PORT))
        self._android_socket.setblocking(False)
        self.get_logger().info("Android socket created")

        # subscribe joystick
        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # publish command
        self.extended_joy_publisher = self.create_publisher(Joy, "extended_joy", 10)

    def joy_callback(self, msg: Joy):
        additional_button = [0] * 16

        try:
            # Try to receive data
            data, address = self._android_socket.recvfrom(
                4096
            )  # This line may throw BlockingIOError
            self.get_logger().info("Received data from Android: " + data.decode("utf-8"))
            additional_button[int(data.decode("utf-8")) - 1] = 1

        except BlockingIOError:
            pass

        msg.buttons.extend(additional_button)

        self.extended_joy_publisher.publish(msg)


def main():
    rclpy.init()
    node = ExtendJoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()
