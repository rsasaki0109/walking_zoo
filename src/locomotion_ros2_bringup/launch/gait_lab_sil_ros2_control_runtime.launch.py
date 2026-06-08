"""gait_lab SIL runtime on the ros2_control-split path (B3 first rung).

Physics and joint-state bridging live in ``gait_lab_sil_sim``; the gait_lab policy
runs in ``gait_lab_sil_gait_controller`` and publishes ros2_control joint commands.
A ``joint_state_topic_hardware_interface`` + ``joint_state_broadcaster`` exposes
standard ``/joint_states`` for tools and future controllers.

Set ``use_ros2_control_forward:=true`` to route policy targets through the
``GaitLabSilJointForwardController`` plugin instead of direct joint_commands.

The legacy monolithic path remains the default
(``gait_lab_sil_runtime.launch.py``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory("locomotion_ros2_bringup")
    description_share = get_package_share_directory("locomotion_ros2_description")
    urdf_path = os.path.join(description_share, "urdf", "g1_sil_ros2_control.urdf")
    controllers_path = os.path.join(
        bringup_share, "config", "gait_lab_sil_ros2_control.yaml")
    with open(urdf_path, encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    menagerie_path = LaunchConfiguration("menagerie_path")
    gait_lab_path = LaunchConfiguration("gait_lab_path")
    controller = LaunchConfiguration("controller")
    use_ros2_control_forward = LaunchConfiguration("use_ros2_control_forward")

    sim_env = {
        "LOCOMOTION_ROS2_GAIT_LAB_PATH": gait_lab_path,
        "LOCOMOTION_ROS2_MENAGERIE_PATH": menagerie_path,
        "MUJOCO_GL": "egl",
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_path,
        ],
    )

    direct_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=UnlessCondition(use_ros2_control_forward),
    )

    forward_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "gait_lab_sil_gait_forward",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control_forward),
    )

    runtime_node = Node(
        package="locomotion_ros2_runtime",
        executable="locomotion_ros2_runtime_manager",
        name="locomotion_ros2_runtime_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "adapter_plugin": "locomotion_ros2_gait_lab_sil/GaitLabSilAdapter",
            "robot_model": "g1",
            "robot_family": "humanoid",
            "limits.max_linear_x": 0.4,
            "limits.max_linear_y": 0.2,
            "limits.max_angular_z": 0.5,
        }],
    )

    cmd_vel_bridge = Node(
        package="locomotion_ros2_nav2",
        executable="cmd_vel_bridge",
        name="locomotion_ros2_cmd_vel_bridge",
        output="screen",
        parameters=[{
            "input_topic": "/cmd_vel",
            "output_topic": "/locomotion_ros2/cmd_vel",
            "frame_id": "base_link",
        }],
    )

    sim_node = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_sim.py",
        name="gait_lab_sil_sim",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "ros2_control_split": True,
            "batch_substeps_per_command": False,
        }],
    )

    gait_controller_node = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_gait_controller.py",
        name="gait_lab_sil_gait_controller",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "use_ros2_control_forward": use_ros2_control_forward,
            "ros2_control_forward_topic": "/gait_lab_sil_gait_forward/commands",
        }],
    )

    delayed_spawner = TimerAction(
        period=2.0,
        actions=[direct_spawner, forward_spawner],
    )

    # Start sim + policy after ros2_control controllers are spawned so the
    # forward-controller path does not drop the first command burst.
    delayed_sim_stack = TimerAction(
        period=3.0,
        actions=[sim_node, gait_controller_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "menagerie_path", default_value="/tmp/locomotion_ros2_mujoco_menagerie"),
        DeclareLaunchArgument("gait_lab_path", default_value=""),
        DeclareLaunchArgument("controller", default_value="rl-residual"),
        DeclareLaunchArgument("use_ros2_control_forward", default_value="false"),
        robot_state_publisher,
        ros2_control_node,
        delayed_spawner,
        delayed_sim_stack,
        runtime_node,
        cmd_vel_bridge,
    ])
