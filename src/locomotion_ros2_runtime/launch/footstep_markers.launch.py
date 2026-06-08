from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    step_count = LaunchConfiguration("step_count")
    lateral_shift = LaunchConfiguration("lateral_shift")
    costmap_topic = LaunchConfiguration("costmap_topic")
    elevation_topic = LaunchConfiguration("elevation_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="TF frame the footstep markers are published in "
                            "(overridden by the costmap frame when one is supplied).",
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
            DeclareLaunchArgument(
                "costmap_topic",
                default_value="",
                description="nav_msgs/OccupancyGrid costmap that drives keep-out "
                            "zones (empty disables the real terrain source).",
            ),
            DeclareLaunchArgument(
                "elevation_topic",
                default_value="",
                description="Optional nav_msgs/OccupancyGrid elevation grid that "
                            "drives step-up heights.",
            ),
            Node(
                package="locomotion_ros2_runtime",
                executable="footstep_marker_publisher",
                name="locomotion_ros2_footstep_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "frame_id": frame_id,
                        "step_count": step_count,
                        "lateral_shift": lateral_shift,
                        "costmap_topic": costmap_topic,
                        "elevation_topic": elevation_topic,
                    }
                ],
            ),
        ]
    )
