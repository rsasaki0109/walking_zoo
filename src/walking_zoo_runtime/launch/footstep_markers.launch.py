from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    step_count = LaunchConfiguration("step_count")
    lateral_shift = LaunchConfiguration("lateral_shift")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="TF frame the footstep markers are published in.",
            ),
            DeclareLaunchArgument(
                "step_count",
                default_value="6",
                description="Number of footsteps in the stub plan.",
            ),
            DeclareLaunchArgument(
                "lateral_shift",
                default_value="0.0",
                description="Sideways drift per step to preview a sidestep plan.",
            ),
            Node(
                package="walking_zoo_runtime",
                executable="footstep_marker_publisher",
                name="walking_zoo_footstep_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "frame_id": frame_id,
                        "step_count": step_count,
                        "lateral_shift": lateral_shift,
                    }
                ],
            ),
        ]
    )
