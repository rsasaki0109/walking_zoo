from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
            )
        ]
    )
