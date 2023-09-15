from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "inverse_y_axis",
                default_value="false",  # デフォルト値は false ですが、必要に応じて変更してください
                description="Set to true to invert the Y-axis.",
            ),
            Node(
                package="expandroid_controller",
                executable="expandroid_control_node",
                name="expandroid_control_node",
                output="screen",
                parameters=[{"inverse_y_axis": LaunchConfiguration("inverse_y_axis")}],
            ),
        ]
    )
