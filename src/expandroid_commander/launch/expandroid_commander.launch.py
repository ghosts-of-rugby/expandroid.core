from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="expandroid_commander",
                executable="joystick_and_app",
                name="joystick_and_app",
                output="screen",
            ),
            Node(
                package="expandroid_commander",
                executable="expandroid_commander",
                name="expandroid_commander",
                output="screen",
            ),
        ]
    )
