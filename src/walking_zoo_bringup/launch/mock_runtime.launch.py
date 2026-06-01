from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("walking_zoo_bringup"), "params", "mock_runtime.yaml"]
    )
    robot_profile = PathJoinSubstitution(
        [FindPackageShare("walking_zoo_bringup"), "params", "mock_robot_profile.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="walking_zoo_runtime",
                executable="walking_zoo_runtime_manager",
                name="walking_zoo_runtime_manager",
                output="screen",
                parameters=[params_file, {"robot_profile": robot_profile}],
            ),
            Node(
                package="walking_zoo_nav2",
                executable="cmd_vel_bridge",
                name="walking_zoo_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/walking_zoo/cmd_vel",
                        "frame_id": "base_link",
                    }
                ],
            ),
        ]
    )
