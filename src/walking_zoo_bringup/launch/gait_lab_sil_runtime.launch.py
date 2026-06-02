"""Bring up the walking_zoo runtime driving the gait_lab SIL adapter.

This wires the real runtime + safety pipeline to the reinforcement-learned
gait_lab ``rl-residual`` policy running in MuJoCo:

    runtime (GaitLabSilAdapter)  --cmd-->  gait_lab_sil_sim (MuJoCo G1 + RL gait)
                                 <-state--

Drive it by publishing to ``/cmd_vel`` (forward → the robot walks) and watch
``/walking_zoo/state``. The sim node needs a Python with MuJoCo + gait_lab's deps
(it is the only piece that does); point it at the gait_lab checkout with the
``gait_lab_path`` argument or WALKING_ZOO_GAIT_LAB_PATH.

    ros2 launch walking_zoo_bringup gait_lab_sil_runtime.launch.py \
        gait_lab_path:=/path/to/walking_zoo/experiments/gait_lab
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    menagerie_path = LaunchConfiguration("menagerie_path")
    gait_lab_path = LaunchConfiguration("gait_lab_path")
    controller = LaunchConfiguration("controller")

    sim_env = {
        "WALKING_ZOO_GAIT_LAB_PATH": gait_lab_path,
        "WALKING_ZOO_MENAGERIE_PATH": menagerie_path,
        "MUJOCO_GL": "egl",
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "menagerie_path", default_value="/tmp/walking_zoo_mujoco_menagerie",
            description="Local google-deepmind/mujoco_menagerie checkout (the G1 scene)."),
        DeclareLaunchArgument(
            "gait_lab_path", default_value="",
            description="Path to experiments/gait_lab (empty = auto-detect / "
                        "WALKING_ZOO_GAIT_LAB_PATH)."),
        DeclareLaunchArgument(
            "controller", default_value="rl-residual",
            description="gait_lab controller the sim runs (rl-residual, balanced-cpg, ...)."),

        # The runtime, loading the gait_lab SIL adapter and autostarting it.
        Node(
            package="walking_zoo_runtime",
            executable="walking_zoo_runtime_manager",
            name="walking_zoo_runtime_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "adapter_plugin": "walking_zoo_gait_lab_sil/GaitLabSilAdapter",
                "robot_model": "g1",
                "robot_family": "humanoid",
                "limits.max_linear_x": 0.4,
                "limits.max_linear_y": 0.2,
                "limits.max_angular_z": 0.5,
            }],
        ),
        # Route teleop/Nav2 /cmd_vel into the runtime's command input.
        Node(
            package="walking_zoo_nav2",
            executable="cmd_vel_bridge",
            name="walking_zoo_cmd_vel_bridge",
            output="screen",
            parameters=[{
                "input_topic": "/cmd_vel",
                "output_topic": "/walking_zoo/cmd_vel",
                "frame_id": "base_link",
            }],
        ),
        # The MuJoCo G1 + gait_lab policy behind the adapter (needs MuJoCo).
        Node(
            package="walking_zoo_examples",
            executable="gait_lab_sil_sim.py",
            name="gait_lab_sil_sim",
            output="screen",
            additional_env=sim_env,
            parameters=[{"controller": controller}],
        ),
    ])
