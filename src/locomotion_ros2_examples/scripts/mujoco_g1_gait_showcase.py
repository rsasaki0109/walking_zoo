#!/usr/bin/env python3
"""Drive the MuJoCo G1 demo through a repeatable gait showcase sequence."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from locomotion_ros2_msgs.msg import SemanticAction
from locomotion_ros2_msgs.srv import EmergencyStop


def parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class MujocoG1GaitShowcase(Node):
    def __init__(self):
        super().__init__("locomotion_ros2_mujoco_g1_gait_showcase")
        self.declare_parameter("semantic_topic", "/locomotion_ros2/semantic_action")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("estop_service", "/locomotion_ros2/estop")
        self.declare_parameter("source", "locomotion_ros2_gait_showcase")
        self.declare_parameter("start_delay_sec", 4.0)
        self.declare_parameter("step_duration_sec", 2.8)
        self.declare_parameter("loop", False)
        self.declare_parameter("include_estop", True)

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.estop_service_name = self.get_parameter("estop_service").value
        self.source = self.get_parameter("source").value
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.step_duration_sec = float(self.get_parameter("step_duration_sec").value)
        self.loop = parse_bool(self.get_parameter("loop").value)
        self.include_estop = parse_bool(self.get_parameter("include_estop").value)

        self.publisher = self.create_publisher(SemanticAction, self.semantic_topic, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.estop_client = self.create_client(EmergencyStop, self.estop_service_name)
        self.sequence = self.build_sequence()
        self.step_index = -1
        self.started = False
        self.finished = False
        self.last_step_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            f"waiting {self.start_delay_sec:.1f}s before gait showcase on {self.semantic_topic}"
        )

    def build_sequence(self):
        actions = [
            "walk_forward",
            "slow_careful_walk",
            "run_forward",
            "walk_backward",
            "sidestep_left",
            "sidestep_right",
            "turn_left",
            "turn_right",
            "stop",
            "fall_detected",
            "recovery_blocked",
        ]
        if self.include_estop and not self.loop:
            actions.append("estop")
        elif self.include_estop and self.loop:
            self.get_logger().warn(
                "loop:=true ignores include_estop:=true to keep the loop recoverable"
            )
        return actions

    def publish_action(self, action):
        msg = SemanticAction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.action = action
        msg.confidence = 1.0
        msg.tags = ["demo", "mujoco", "unitree_g1", "gait_showcase"]
        self.publisher.publish(msg)
        self.publish_cmd_vel(action)
        self.get_logger().info(f"showcase action -> {action}")
        if action == "estop":
            self.call_estop()

    def publish_cmd_vel(self, action):
        if action in ("estop", "fall_detected"):
            return
        msg = Twist()
        if action == "walk_forward":
            msg.linear.x = 0.22
        elif action == "slow_careful_walk":
            msg.linear.x = 0.10
        elif action == "run_forward":
            msg.linear.x = 0.45
        elif action == "walk_backward":
            msg.linear.x = -0.18
        elif action == "sidestep_left":
            msg.linear.y = 0.22
        elif action == "sidestep_right":
            msg.linear.y = -0.22
        elif action == "turn_left":
            msg.angular.z = 0.55
        elif action == "turn_right":
            msg.angular.z = -0.55
        elif action == "recovery_blocked":
            msg.linear.x = 0.22
        self.cmd_vel_publisher.publish(msg)

    def call_estop(self):
        service_ready = (
            self.estop_client.service_is_ready()
            or self.estop_client.wait_for_service(timeout_sec=0.2)
        )
        if not service_ready:
            self.get_logger().warn(
                f"{self.estop_service_name} unavailable; semantic estop was still published"
            )
            return

        request = EmergencyStop.Request()
        request.stop = True
        request.reason = "MuJoCo G1 gait showcase"
        future = self.estop_client.call_async(request)
        future.add_done_callback(self.on_estop_response)

    def on_estop_response(self, future):
        result = future.result()
        if result is None:
            self.get_logger().warn("estop service returned no response")
            return
        self.get_logger().info(
            f"estop service response: success={result.success} active={result.estop_active}"
        )

    def on_timer(self):
        now = self.get_clock().now()
        if not self.started:
            age = (now - self.last_step_time).nanoseconds / 1e9
            if age < self.start_delay_sec:
                return
            self.started = True
            self.step_index = 0
            self.last_step_time = now
            self.publish_action(self.sequence[self.step_index])
            return

        age = (now - self.last_step_time).nanoseconds / 1e9
        if age < self.step_duration_sec:
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            if self.loop:
                self.step_index = 0
            else:
                self.get_logger().info("gait showcase complete")
                self.finished = True
                self.timer.cancel()
                return

        self.last_step_time = now
        self.publish_action(self.sequence[self.step_index])


def main():
    rclpy.init()
    node = MujocoG1GaitShowcase()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
