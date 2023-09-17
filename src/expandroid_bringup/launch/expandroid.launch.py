import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments if needed
            DeclareLaunchArgument(
                "inverse_y_axis",
                default_value="false",  # The default value is false, but you can change it as needed
                description="Set to true to invert the Y-axis.",
            ),
            # Include expandroid_control_node.launch.py
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), "/expandroid_control_node.launch.py"]
                ),
                # Pass launch arguments to the included launch file
                launch_arguments={
                    "inverse_y_axis": launch.substitutions.LaunchConfiguration(
                        "inverse_y_axis"
                    ),
                }.items(),
            ),
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
