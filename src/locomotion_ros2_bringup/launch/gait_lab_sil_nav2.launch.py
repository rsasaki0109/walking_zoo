"""Full Nav2 autonomous navigation for the gait_lab SIL Unitree G1.

This is the capstone of the experiment -> product loop: the reinforcement-learned
**steerable** gait_lab gait, running in MuJoCo behind the real locomotion_ros2 runtime
and safety pipeline, driven to a goal by the *complete* Nav2 stack — map server,
global planner (NavFn), local controller (Regulated Pure Pursuit), recovery
behaviours and behaviour-tree navigator.

How it fits together::

    Nav2 (planner + controller)  --/cmd_vel-->  cmd_vel_bridge (legged shaper)
                                                     |  /locomotion_ros2/cmd_vel
                                                     v
                                        runtime + safety pipeline
                                                     |
                                        GaitLabSilAdapter (C++ bridge)
                                                     |  command_velocity / control
                                                     v
                                 gait_lab_sil_sim.py  (MuJoCo G1 + rl-steerable)
                                     |  /odom + TF(odom->base_link) + WalkingState
                                     ^----- Nav2 localises off this (sim odometry)

Localisation is perfect in sim, so instead of AMCL the launch publishes a static
identity ``map -> odom`` transform; the MuJoCo base pose (published as ``/odom``
and the ``odom -> base_link`` TF by the sim) is the robot's true pose.

    ros2 launch locomotion_ros2_bringup gait_lab_sil_nav2.launch.py \
        gait_lab_path:=/path/to/locomotion_ros2/experiments/gait_lab

Then send a goal (RViz "Nav2 Goal", or ``tools/check_gait_lab_sil_nav2_nav_e2e.py``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    menagerie_path = LaunchConfiguration("menagerie_path")
    gait_lab_path = LaunchConfiguration("gait_lab_path")
    controller = LaunchConfiguration("controller")

    bringup_share = get_package_share_directory("locomotion_ros2_bringup")
    params_file = os.path.join(bringup_share, "params", "nav2_sil.yaml")
    map_yaml = os.path.join(bringup_share, "maps", "arena.yaml")

    sim_env = {
        "LOCOMOTION_ROS2_GAIT_LAB_PATH": gait_lab_path,
        "LOCOMOTION_ROS2_MENAGERIE_PATH": menagerie_path,
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

    return LaunchDescription([
        DeclareLaunchArgument(
            "menagerie_path", default_value="/tmp/locomotion_ros2_mujoco_menagerie",
            description="Local mujoco_menagerie checkout (the G1 scene)."),
        DeclareLaunchArgument(
            "gait_lab_path", default_value="",
            description="Path to experiments/gait_lab (empty = auto-detect)."),
        DeclareLaunchArgument(
            "controller", default_value="rl-steerable",
            description="gait_lab controller the sim runs (rl-steerable can turn)."),

        # --- The SIL robot, behind the real runtime + safety pipeline ---------
        Node(
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
        ),
        # Nav2's /cmd_vel -> legged-shaped, readiness-gated -> runtime input.
        Node(
            package="locomotion_ros2_nav2",
            executable="cmd_vel_bridge",
            name="locomotion_ros2_cmd_vel_bridge",
            output="screen",
            parameters=[{
                "input_topic": "/cmd_vel",
                "input_stamped": True,   # Nav2 Jazzy publishes TwistStamped cmd_vel
                "output_topic": "/locomotion_ros2/cmd_vel",
                "frame_id": "base_link",
                "legged.max_forward": 0.4,
                "legged.max_yaw_rate": 0.5,
            }],
        ),
        # MuJoCo G1 + the steerable RL gait. Remap its odom to the /odom Nav2 wants.
        Node(
            package="locomotion_ros2_examples",
            executable="gait_lab_sil_sim.py",
            name="gait_lab_sil_sim",
            output="screen",
            additional_env=sim_env,
            parameters=[{"controller": controller, "publish_odom": True}],
            remappings=[("gait_lab_sil/odom", "/odom")],
        ),

        # Perfect localisation in sim: static identity map -> odom.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),

        # --- The full Nav2 stack ---------------------------------------------
        *nav2_nodes,
    ])
