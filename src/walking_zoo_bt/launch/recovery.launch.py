from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tick_period = LaunchConfiguration("tick_period_sec")
    state_topic = LaunchConfiguration("state_topic")
    clear_fault_service = LaunchConfiguration("clear_fault_service")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "tick_period_sec",
                default_value="0.5",
                description="How often the recovery behavior tree is ticked.",
            ),
            DeclareLaunchArgument(
                "state_topic",
                default_value="/walking_zoo/state",
                description="WalkingState topic the recovery tree reads readiness from.",
            ),
            DeclareLaunchArgument(
                "clear_fault_service",
                default_value="/walking_zoo/clear_fault",
                description="Service the ClearWalkingFaultService BT node calls.",
            ),
            Node(
                package="walking_zoo_bt",
                executable="walking_zoo_bt_recovery_node",
                name="walking_zoo_bt_recovery",
                output="screen",
                parameters=[
                    {
                        "tick_period_sec": tick_period,
                        "state_topic": state_topic,
                        "clear_fault_service": clear_fault_service,
                    }
                ],
            ),
        ]
    )
