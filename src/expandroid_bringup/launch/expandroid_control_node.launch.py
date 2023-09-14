from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="expandroid_controller",
                executable="expandroid_control_node",
                name="expandroid_control_node",
                output="screen",
            )
        ]
    )
