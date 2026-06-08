from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("locomotion_ros2_bringup"), "params", "mock_runtime.yaml"]
    )
    robot_profile = PathJoinSubstitution(
        [FindPackageShare("locomotion_ros2_bringup"), "params", "mock_robot_profile.yaml"]
    )

    menagerie_path = LaunchConfiguration("menagerie_path")
    output_dir = LaunchConfiguration("output_dir")
    fps = LaunchConfiguration("fps")
    gif_width = LaunchConfiguration("gif_width")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "menagerie_path",
                default_value="/tmp/locomotion_ros2_mujoco_menagerie",
                description="Path to a local google-deepmind/mujoco_menagerie checkout.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/locomotion_ros2_mujoco_g1_demo",
                description="Directory for latest.png and live.gif output frames.",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="12.0",
                description="Headless renderer frame rate.",
            ),
            DeclareLaunchArgument(
                "gif_width",
                default_value="360",
                description="Width of the lightweight live.gif output.",
            ),
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
            Node(
                package="locomotion_ros2_examples",
                executable="mujoco_g1_gait_demo.py",
                name="locomotion_ros2_mujoco_g1_gait_demo",
                output="screen",
                parameters=[
                    {
                        "menagerie_path": menagerie_path,
                        "output_dir": output_dir,
                        "fps": fps,
                        "gif_width": gif_width,
                    }
                ],
            ),
        ]
    )
