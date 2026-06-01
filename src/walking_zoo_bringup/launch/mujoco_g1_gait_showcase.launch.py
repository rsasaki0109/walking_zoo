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
    loop = LaunchConfiguration("loop")
    include_estop = LaunchConfiguration("include_estop")

    demo_launch = PathJoinSubstitution(
        [
            FindPackageShare("walking_zoo_bringup"),
            "launch",
            "mujoco_g1_gait_demo.launch.py",
        ]
    )

    showcase_node = Node(
        package="walking_zoo_examples",
        executable="mujoco_g1_gait_showcase.py",
        name="walking_zoo_mujoco_g1_gait_showcase",
        output="screen",
        parameters=[
            {
                "start_delay_sec": start_delay_sec,
                "step_duration_sec": step_duration_sec,
                "loop": loop,
                "include_estop": include_estop,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "menagerie_path",
                default_value="/tmp/walking_zoo_mujoco_menagerie",
                description="Path to a local google-deepmind/mujoco_menagerie checkout.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/tmp/walking_zoo_mujoco_g1_showcase",
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
                "loop",
                default_value="false",
                description="Repeat the showcase sequence. E-stop is skipped when loop is true.",
            ),
            DeclareLaunchArgument(
                "include_estop",
                default_value="true",
                description="End the one-shot showcase with the runtime e-stop gate.",
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
            showcase_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=showcase_node,
                    on_exit=[Shutdown(reason="MuJoCo G1 gait showcase complete")],
                )
            ),
        ]
    )
