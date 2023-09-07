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
                executable="extend_joystick",
                name="extend_joystick",
                output="screen",
            ),
            Node(
                package="expandroid_commander",
                executable="expandroid_commander",
                name="expandroid_commander",
                # remappings=[
                #     ("~/joy", "extended_joy"),
                # ],
                output="screen",
            ),
        ]
    )
