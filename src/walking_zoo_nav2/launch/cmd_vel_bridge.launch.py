from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="walking_zoo_nav2",
                executable="cmd_vel_bridge",
                name="walking_zoo_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/cmd_vel",
                        "output_topic": "/walking_zoo/cmd_vel",
                        "frame_id": "base_link",
                    }
                ],
            )
        ]
    )
