from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("walking_zoo_runtime"), "params", "runtime.yaml"]
                ),
                description="Optional YAML parameters for walking_zoo_runtime_manager.",
            ),
            Node(
                package="walking_zoo_runtime",
                executable="walking_zoo_runtime_manager",
                name="walking_zoo_runtime_manager",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
