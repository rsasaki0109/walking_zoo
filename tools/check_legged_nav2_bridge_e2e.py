#!/usr/bin/env python3
"""End-to-end check for the legged-aware Nav2 cmd_vel bridge.

Launches the mock runtime and the cmd_vel bridge, publishes a raw Nav2 Twist
that is over the legged envelope (too-fast forward while turning hard), and
confirms the bridge republishes a shaped TwistStamped (clamped forward speed,
turn/forward coupling applied). Then e-stops the runtime and confirms the bridge
holds (publishes zero) because the robot is no longer ready. This exercises the
LeggedVelocityShaper and the readiness gate against the real runtime state.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import suppress


def terminate_process_group(process):
    if process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def collect_latest(rclpy, node, pub, twist, seconds):
    """Publish `twist` on `pub` for `seconds` and return the last shaped output."""
    from geometry_msgs.msg import TwistStamped

    latest = {}
    sub = node.create_subscription(
        TwistStamped, "/locomotion_ros2/cmd_vel", lambda m: latest.__setitem__("msg", m), 10)
    deadline = time.time() + seconds
    while time.time() < deadline:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.destroy_subscription(sub)
    return latest.get("msg")


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "58")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    launch_log = tempfile.NamedTemporaryFile(
        mode="w+", prefix="locomotion_ros2_nav2_e2e_", suffix=".log", delete=False)
    launch = subprocess.Popen(
        ["ros2", "launch", "locomotion_ros2_bringup", "mock_runtime.launch.py"],
        env=env, stdout=launch_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    bridge = subprocess.Popen(
        ["ros2", "run", "locomotion_ros2_nav2", "cmd_vel_bridge", "--ros-args",
         "-p", "input_topic:=/cmd_vel", "-p", "output_topic:=/locomotion_ros2/cmd_vel",
         "-p", "legged_aware:=true", "-p", "require_ready:=true",
         "-p", "legged.max_forward:=0.6", "-p", "legged.turn_speed_coupling:=0.7",
         "-p", "legged.max_yaw_rate:=0.8"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from locomotion_ros2_msgs.srv import EmergencyStop

        rclpy.init(args=None)
        node = rclpy.create_node("locomotion_ros2_nav2_bridge_e2e")
        raw_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        time.sleep(2.0)  # let runtime activate and bridge connect

        # Over-envelope command: 2.0 m/s forward while spinning at full yaw rate.
        raw = Twist()
        raw.linear.x = 2.0
        raw.angular.z = 0.8
        shaped = collect_latest(rclpy, node, raw_pub, raw, 3.0)
        if shaped is None:
            print("no shaped command received from bridge", file=sys.stderr)
            return 1
        print(f"shaped while ready: vx={shaped.twist.linear.x:.3f} vyaw={shaped.twist.angular.z:.3f}")
        if shaped.twist.linear.x > 0.6 + 1e-6:
            print("forward speed not clamped to legged envelope", file=sys.stderr)
            return 1
        # Forward must be cut below the clamp by turn coupling (0.6 -> ~0.18).
        if shaped.twist.linear.x >= 0.6 - 1e-6:
            print("turn/forward coupling not applied", file=sys.stderr)
            return 1
        if shaped.twist.linear.x <= 0.0:
            print("forward speed fully suppressed unexpectedly", file=sys.stderr)
            return 1

        # E-stop the runtime; the bridge must then hold (publish zero).
        estop_cli = node.create_client(EmergencyStop, "/locomotion_ros2/estop")
        if not estop_cli.wait_for_service(timeout_sec=10.0):
            print("no estop service", file=sys.stderr)
            return 1
        req = EmergencyStop.Request()
        req.stop = True
        fut = estop_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        time.sleep(1.0)  # let the new state propagate to the bridge

        held = collect_latest(rclpy, node, raw_pub, raw, 3.0)
        if held is None:
            print("no command received after estop", file=sys.stderr)
            return 1
        print(f"shaped while e-stopped: vx={held.twist.linear.x:.3f} vyaw={held.twist.angular.z:.3f}")
        if abs(held.twist.linear.x) > 1e-6 or abs(held.twist.angular.z) > 1e-6:
            print("bridge did NOT hold while robot not ready", file=sys.stderr)
            return 1

        print("legged nav2 bridge E2E passed: shaped while ready, held while e-stopped")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"legged nav2 bridge E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(bridge)
        terminate_process_group(launch)
        launch_log.flush()
        if exit_code != 0:
            launch_log.seek(0)
            sys.stderr.write(launch_log.read())
        with suppress(OSError):
            Path(launch_log.name).unlink()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
