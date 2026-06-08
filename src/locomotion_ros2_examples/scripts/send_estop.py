#!/usr/bin/env python3
import rclpy
from locomotion_ros2_msgs.srv import EmergencyStop


def main():
    rclpy.init()
    node = rclpy.create_node("locomotion_ros2_send_estop")
    client = node.create_client(EmergencyStop, "/locomotion_ros2/estop")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/locomotion_ros2/estop service not available")
    request = EmergencyStop.Request()
    request.stop = True
    request.reason = "example estop"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is None:
        raise RuntimeError("estop service call failed")
    node.get_logger().info(future.result().status_text)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
