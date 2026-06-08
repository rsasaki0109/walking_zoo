"""gait_lab SIL runtime on the ros2_control-split path (B3 first rung).

Physics and joint-state bridging live in ``gait_lab_sil_sim``; the gait_lab policy
runs in ``gait_lab_sil_gait_controller`` and publishes ros2_control joint commands.
A ``joint_state_topic_hardware_interface`` + ``joint_state_broadcaster`` exposes
standard ``/joint_states`` for tools and future controllers.

Launch modes:
  - default: direct ``joint_commands`` from the Python policy node
  - ``use_ros2_control_forward:=true``: ``GaitLabSilJointForwardController`` queue
  - ``use_embedded_rl_policy:=true``: C++ ``GaitLabSilRlResidualController`` inference

The legacy monolithic path remains the default
(``gait_lab_sil_runtime.launch.py``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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
    bringup_share = get_package_share_directory("locomotion_ros2_bringup")
    description_share = get_package_share_directory("locomotion_ros2_description")
    urdf_direct_path = os.path.join(
        description_share, "urdf", "g1_sil_ros2_control.urdf")
    urdf_forward_path = os.path.join(
        description_share, "urdf", "g1_sil_ros2_control_forward.urdf")
    controllers_direct_path = os.path.join(
        bringup_share, "config", "gait_lab_sil_ros2_control.yaml")
    controllers_forward_path = os.path.join(
        bringup_share, "config", "gait_lab_sil_ros2_control_forward.yaml")
    controllers_embedded_path = os.path.join(
        bringup_share, "config", "gait_lab_sil_ros2_control_embedded.yaml")
    robot_description_direct = _load_text(urdf_direct_path)
    robot_description_forward = _load_text(urdf_forward_path)

    menagerie_path = LaunchConfiguration("menagerie_path")
    gait_lab_path = LaunchConfiguration("gait_lab_path")
    controller = LaunchConfiguration("controller")
    use_ros2_control_forward = LaunchConfiguration("use_ros2_control_forward")
    use_embedded_rl_policy = LaunchConfiguration("use_embedded_rl_policy")

    direct_active = PythonExpression([
        "'", use_ros2_control_forward, "' != 'true' and '",
        use_embedded_rl_policy, "' != 'true'",
    ])
    split_active = PythonExpression([
        "'", use_ros2_control_forward, "' == 'true' or '",
        use_embedded_rl_policy, "' == 'true'",
    ])
    forward_only = PythonExpression([
        "'", use_ros2_control_forward, "' == 'true' and '",
        use_embedded_rl_policy, "' != 'true'",
    ])

    sim_env = {
        "LOCOMOTION_ROS2_GAIT_LAB_PATH": gait_lab_path,
        "LOCOMOTION_ROS2_MENAGERIE_PATH": menagerie_path,
        "LOCOMOTION_ROS2_GAIT_LAB_CONTROLLER": controller,
        "MUJOCO_GL": "egl",
    }

    robot_state_publisher_direct = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_direct}],
        condition=IfCondition(direct_active),
    )

    robot_state_publisher_split = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_forward}],
        condition=IfCondition(split_active),
    )

    ros2_control_node_direct = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description_direct},
            controllers_direct_path,
        ],
        condition=IfCondition(direct_active),
    )

    ros2_control_node_forward = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description_forward},
            controllers_forward_path,
        ],
        condition=IfCondition(forward_only),
    )

    ros2_control_node_embedded = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {"robot_description": robot_description_forward},
            controllers_embedded_path,
        ],
        additional_env=sim_env,
        condition=IfCondition(use_embedded_rl_policy),
    )

    direct_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(direct_active),
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
        condition=IfCondition(forward_only),
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
        condition=IfCondition(use_embedded_rl_policy),
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

    sim_node_direct = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_sim.py",
        name="gait_lab_sil_sim",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "ros2_control_split": True,
            "batch_substeps_per_command": False,
            "substeps": 10,
        }],
        condition=IfCondition(direct_active),
    )

    sim_node_forward = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_sim.py",
        name="gait_lab_sil_sim",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "ros2_control_split": True,
            "batch_substeps_per_command": False,
            "substeps": 10,
        }],
        condition=IfCondition(forward_only),
    )

    sim_node_embedded = Node(
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
        }],
        condition=IfCondition(use_embedded_rl_policy),
    )

    gait_controller_node_direct = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_gait_controller.py",
        name="gait_lab_sil_gait_controller",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "use_ros2_control_forward": False,
            "use_embedded_rl_policy": False,
            "substeps": 10,
        }],
        condition=IfCondition(direct_active),
    )

    gait_controller_node_forward = Node(
        package="locomotion_ros2_examples",
        executable="gait_lab_sil_gait_controller.py",
        name="gait_lab_sil_gait_controller",
        output="screen",
        additional_env=sim_env,
        parameters=[{
            "controller": controller,
            "use_ros2_control_forward": True,
            "use_embedded_rl_policy": False,
            "substeps": 10,
            "ros2_control_forward_topic": "/gait_lab_sil_gait_forward/commands",
        }],
        condition=IfCondition(forward_only),
    )

    gait_controller_node_embedded = Node(
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
        condition=IfCondition(use_embedded_rl_policy),
    )

    delayed_spawner = TimerAction(
        period=2.0,
        actions=[direct_spawner, forward_spawner, embedded_spawner],
    )

    delayed_sim_stack = TimerAction(
        period=3.0,
        actions=[
            sim_node_direct,
            sim_node_forward,
            sim_node_embedded,
            gait_controller_node_direct,
            gait_controller_node_forward,
            gait_controller_node_embedded,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "menagerie_path", default_value="/tmp/locomotion_ros2_mujoco_menagerie"),
        DeclareLaunchArgument("gait_lab_path", default_value=_default_gait_lab_path()),
        DeclareLaunchArgument("controller", default_value="rl-residual"),
        DeclareLaunchArgument("use_ros2_control_forward", default_value="false"),
        DeclareLaunchArgument("use_embedded_rl_policy", default_value="false"),
        robot_state_publisher_direct,
        robot_state_publisher_split,
        ros2_control_node_direct,
        ros2_control_node_forward,
        ros2_control_node_embedded,
        delayed_spawner,
        delayed_sim_stack,
        runtime_node,
        cmd_vel_bridge,
    ])
