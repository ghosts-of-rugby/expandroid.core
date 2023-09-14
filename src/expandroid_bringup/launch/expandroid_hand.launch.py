from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


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
                package="expandroid_hand",
                executable="expandroid_hand",
                name="expandroid_hand",
                parameters=[
                    {
                        "path_to_field_config": get_package_share_directory(
                            "expandroid_bringup"
                        )
                        + "/config/field.json"
                    }
                ],
                output="screen",
            ),
        ]
    )
