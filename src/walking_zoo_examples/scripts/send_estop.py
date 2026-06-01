#!/usr/bin/env python3
import rclpy
from walking_zoo_msgs.srv import EmergencyStop


def main():
    rclpy.init()
    node = rclpy.create_node("walking_zoo_send_estop")
    client = node.create_client(EmergencyStop, "/walking_zoo/estop")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("/walking_zoo/estop service not available")
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
