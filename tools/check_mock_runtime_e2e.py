#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import suppress


def wait_until(node, predicate, timeout_sec, spin_once):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        spin_once(node)
    return False


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


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "42")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    launch_log = tempfile.NamedTemporaryFile(
        mode="w+",
        prefix="walking_zoo_mock_runtime_",
        suffix=".log",
        delete=False,
    )
    launch = subprocess.Popen(
        ["ros2", "launch", "walking_zoo_bringup", "mock_runtime.launch.py"],
        env=env,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from walking_zoo_msgs.msg import WalkingState
        from walking_zoo_msgs.srv import EmergencyStop

        rclpy.init(args=None)
        node = rclpy.create_node("walking_zoo_mock_runtime_e2e_check")
        states = []

        node.create_subscription(
            WalkingState,
            "/walking_zoo/state",
            lambda msg: states.append(msg),
            10,
        )
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        estop_client = node.create_client(EmergencyStop, "/walking_zoo/estop")

        def spin_once(active_node):
            rclpy.spin_once(active_node, timeout_sec=0.1)

        if not wait_until(node, lambda: launch.poll() is None, 3.0, spin_once):
            print("mock runtime launch exited early", file=sys.stderr)
            return 1

        if not wait_until(node, lambda: cmd_pub.get_subscription_count() > 0, 15.0, spin_once):
            print("/cmd_vel bridge subscription did not appear", file=sys.stderr)
            return 1

        if not wait_until(
            node,
            lambda: bool(states) and states[-1].adapter_connected,
            15.0,
            spin_once,
        ):
            print("/walking_zoo/state did not report adapter_connected", file=sys.stderr)
            return 1

        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.1
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            cmd_pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.1)
            if states and states[-1].locomotion_state == WalkingState.STATE_WALKING:
                break

        if not states or states[-1].locomotion_state != WalkingState.STATE_WALKING:
            print("runtime did not transition to WALKING after /cmd_vel", file=sys.stderr)
            return 1

        if not estop_client.wait_for_service(timeout_sec=5.0):
            print("/walking_zoo/estop service did not appear", file=sys.stderr)
            return 1

        request = EmergencyStop.Request()
        request.stop = True
        request.reason = "e2e check"
        future = estop_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None or not future.result().estop_active:
            print("estop service did not activate estop", file=sys.stderr)
            return 1

        if not wait_until(
            node,
            lambda: bool(states) and states[-1].locomotion_state == WalkingState.STATE_ESTOPPED,
            5.0,
            spin_once,
        ):
            print("runtime did not transition to ESTOPPED", file=sys.stderr)
            return 1

        print("mock runtime E2E passed: STANDING/WALKING/ESTOPPED path verified")
        exit_code = 0
        return 0
    finally:
        with suppress(Exception):
            if node is not None:
                node.destroy_node()
        with suppress(Exception):
            if rclpy is not None:
                rclpy.shutdown()
        terminate_process_group(launch)
        launch_log.close()
        if exit_code != 0:
            print(f"mock runtime launch log: {launch_log.name}", file=sys.stderr)
            with suppress(OSError):
                print(Path(launch_log.name).read_text(), file=sys.stderr)
        else:
            with suppress(OSError):
                Path(launch_log.name).unlink()


if __name__ == "__main__":
    raise SystemExit(main())
