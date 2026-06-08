from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    menagerie_path = LaunchConfiguration("menagerie_path")
    output_dir = LaunchConfiguration("output_dir")
    fps = LaunchConfiguration("fps")
    gif_width = LaunchConfiguration("gif_width")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    step_duration_sec = LaunchConfiguration("step_duration_sec")
    include_estop = LaunchConfiguration("include_estop")

    demo_launch = PathJoinSubstitution(
        [
            FindPackageShare("locomotion_ros2_bringup"),
            "launch",
            "mujoco_g1_gait_demo.launch.py",
        ]
    )

    recorder_node = Node(
        package="locomotion_ros2_examples",
        executable="locomotion_ros2_demo_recorder.py",
        name="locomotion_ros2_demo_recorder",
        output="screen",
        parameters=[{"output_dir": output_dir}],
    )

    showcase_node = Node(
        package="locomotion_ros2_examples",
        executable="mujoco_g1_gait_showcase.py",
        name="locomotion_ros2_mujoco_g1_gait_showcase",
        output="screen",
        parameters=[
            {
                "start_delay_sec": start_delay_sec,
                "step_duration_sec": step_duration_sec,
                "loop": False,
                "include_estop": include_estop,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "menagerie_path",
                default_value="/tmp/locomotion_ros2_mujoco_menagerie",
                description="Path to a local google-deepmind/mujoco_menagerie checkout.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/locomotion_ros2_mujoco_g1_runtime_showcase",
                description="Directory for live.gif, latest.png, demo_trace.json, and demo_trace.md.",
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
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="4.0",
                description="Delay before the showcase starts, allowing subscriptions to connect.",
            ),
            DeclareLaunchArgument(
                "step_duration_sec",
                default_value="2.8",
                description="Duration of each gait in the showcase sequence.",
            ),
            DeclareLaunchArgument(
                "include_estop",
                default_value="true",
                description="End the showcase with the runtime e-stop gate.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments={
                    "menagerie_path": menagerie_path,
                    "output_dir": output_dir,
                    "fps": fps,
                    "gif_width": gif_width,
                }.items(),
            ),
            recorder_node,
            showcase_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=showcase_node,
                    on_exit=[Shutdown(reason="MuJoCo G1 runtime showcase complete")],
                )
            ),
        ]
    )
