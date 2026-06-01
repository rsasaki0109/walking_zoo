#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node("walking_zoo_send_mock_cmd_vel")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.1
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
