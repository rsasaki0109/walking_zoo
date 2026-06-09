"""Full Nav2 autonomous navigation for the gait_lab SIL Unitree G1.

Capstone of the experiment -> product loop: the reinforcement-learned **steerable**
gait_lab gait, running in MuJoCo behind the real locomotion_ros2 runtime and
safety pipeline, driven to a goal by the complete Nav2 stack.

Set ``use_ros2_control_embedded:=true`` to run physics + C++ RL residual inference
on the ros2_control-split path (500 Hz lockstep nav config); ``false`` (default)
keeps the legacy monolithic sim node.

    ros2 launch locomotion_ros2_bringup gait_lab_sil_nav2.launch.py \
        gait_lab_path:=/path/to/locomotion_ros2/experiments/gait_lab

Then send a goal (RViz "Nav2 Goal", or ``tools/check_gait_lab_sil_nav2_nav_e2e.py``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_text(path: str) -> str:
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _default_gait_lab_path() -> str:
    env = os.environ.get("LOCOMOTION_ROS2_GAIT_LAB_PATH", "")
    if env:
        return env
    bringup_share = get_package_share_directory("locomotion_ros2_bringup")
    workspace_root = os.path.abspath(
        os.path.join(bringup_share, "..", "..", "..", ".."))
    return os.path.join(workspace_root, "experiments", "gait_lab")


def generate_launch_description():
    menagerie_path = LaunchConfiguration("menagerie_path")
    gait_lab_path = LaunchConfiguration("gait_lab_path")
    controller = LaunchConfiguration("controller")
    use_ros2_control_embedded = LaunchConfiguration("use_ros2_control_embedded")

    bringup_share = get_package_share_directory("locomotion_ros2_bringup")
    description_share = get_package_share_directory("locomotion_ros2_description")
    params_file = os.path.join(bringup_share, "params", "nav2_sil.yaml")
    map_yaml = os.path.join(bringup_share, "maps", "arena.yaml")
    urdf_forward_path = os.path.join(
        description_share, "urdf", "g1_sil_ros2_control_forward.urdf")
    controllers_embedded_path = os.path.join(
        bringup_share, "config", "gait_lab_sil_ros2_control_embedded_nav.yaml")
    robot_description_embedded = _load_text(urdf_forward_path)

    sim_env = {
        "LOCOMOTION_ROS2_GAIT_LAB_PATH": gait_lab_path,
        "LOCOMOTION_ROS2_MENAGERIE_PATH": menagerie_path,
        "LOCOMOTION_ROS2_GAIT_LAB_CONTROLLER": controller,
        "MUJOCO_GL": "egl",
    }

    nav2_nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[params_file, {"yaml_filename": map_yaml, "use_sim_time": False}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[params_file, {"use_sim_time": False}]),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[params_file, {"use_sim_time": False}]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params_file, {"use_sim_time": False}]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params_file, {"use_sim_time": False}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": False,
                 "autostart": True,
                 "node_names": [
                     "map_server", "planner_server", "controller_server",
                     "behavior_server", "bt_navigator",
                 ],
             }]),
    ]

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
            "input_stamped": True,
            "output_topic": "/locomotion_ros2/cmd_vel",
            "frame_id": "base_link",
            # SIL: do not gate Nav2 on brief gait balance oscillation.
            "require_ready": False,
            "legged.max_forward": 0.35,
            "legged.max_lateral": 0.05,
            "legged.max_yaw_rate": 0.35,
            "legged.yaw_deadband": 0.10,
            "legged.lateral_deadband": 0.05,
            "legged.turn_speed_coupling": 1.8,
            "legged.max_yaw_accel": 0.20,
        }],
    )

    sim_monolithic = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_sim.py",
        name="gait_lab_sil_sim",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "publish_odom": True,
            "steer_yaw_ramp_rate": 0.15,
        }],
        remappings=[("gait_lab_sil/odom", "/odom")],
        condition=UnlessCondition(use_ros2_control_embedded),
    )

    robot_state_publisher_embedded = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_embedded}],
        condition=IfCondition(use_ros2_control_embedded),
    )

    ros2_control_node_embedded = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description_embedded},
            controllers_embedded_path,
        ],
        additional_env=sim_env,
        condition=IfCondition(use_ros2_control_embedded),
    )

    embedded_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "gait_lab_sil_rl_residual",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control_embedded),
    )

    sim_embedded = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_sim.py",
        name="gait_lab_sil_sim",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "ros2_control_split": True,
            "batch_substeps_per_command": False,
            "substeps": 1,
            "publish_odom": True,
        }],
        remappings=[("gait_lab_sil/odom", "/odom")],
        condition=IfCondition(use_ros2_control_embedded),
    )

    gait_controller_embedded = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_gait_controller.py",
        name="gait_lab_sil_gait_controller",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "use_ros2_control_forward": False,
            "use_embedded_rl_policy": True,
            "substeps": 1,
            "steer_yaw_ramp_rate": 0.15,
        }],
        condition=IfCondition(use_ros2_control_embedded),
    )

    delayed_spawner = TimerAction(
        period=2.0,
        actions=[embedded_spawner],
    )

    delayed_sim_stack = TimerAction(
        period=3.0,
        actions=[sim_embedded, gait_controller_embedded],
    )

    # Start Nav2 after the SIL sim is publishing odom->base_link TF; otherwise
    # planner_server autostart times out and stays inactive.
    delayed_nav2 = TimerAction(
        period=7.0,
        actions=nav2_nodes,
    )

    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "menagerie_path", default_value="/tmp/locomotion_ros2_mujoco_menagerie",
            description="Local mujoco_menagerie checkout (the G1 scene)."),
        DeclareLaunchArgument(
            "gait_lab_path", default_value=_default_gait_lab_path(),
            description="Path to experiments/gait_lab (empty = auto-detect)."),
        DeclareLaunchArgument(
            "controller", default_value="rl-steerable",
            description="gait_lab controller (rl-steerable-footstep optional for tight turns)."),
        DeclareLaunchArgument(
            "use_ros2_control_embedded", default_value="false",
            description="ros2_control embedded RL path (true = C++ policy; false = monolithic sim)."),

        runtime_node,
        cmd_vel_bridge,
        sim_monolithic,
        robot_state_publisher_embedded,
        ros2_control_node_embedded,
        delayed_spawner,
        delayed_sim_stack,
        map_to_odom,
        delayed_nav2,
    ])
