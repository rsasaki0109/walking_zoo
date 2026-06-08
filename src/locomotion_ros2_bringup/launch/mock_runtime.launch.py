from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("locomotion_ros2_bringup"), "params", "mock_runtime.yaml"]
    )
    robot_profile = PathJoinSubstitution(
        [FindPackageShare("locomotion_ros2_bringup"), "params", "mock_robot_profile.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="locomotion_ros2_runtime",
                executable="locomotion_ros2_runtime_manager",
                name="locomotion_ros2_runtime_manager",
                output="screen",
                parameters=[params_file, {"robot_profile": robot_profile}],
            ),
            Node(
                package="locomotion_ros2_nav2",
                executable="cmd_vel_bridge",
                name="locomotion_ros2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/locomotion_ros2/cmd_vel",
                        "frame_id": "base_link",
                    }
                ],
            ),
        ]
    )
