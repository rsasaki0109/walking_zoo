#!/usr/bin/env python3
"""End-to-end check for the runtime ExecuteBodyPose action.

Launches the mock runtime, sends a body-pose goal within the safe tilt band
(expecting success with feedback), then sends a goal whose requested tilt is far
past the fall threshold (expecting the runtime fall-aware safety gate to reject
it). This exercises the real runtime action server, the SafetyPipeline body-pose
gate (FallDetector), and the mock adapter together.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
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


def make_pose(roll, pitch, duration=0.6):
    from locomotion_ros2_msgs.msg import BodyPoseCommand

    cmd = BodyPoseCommand()
    cmd.roll = float(roll)
    cmd.pitch = float(pitch)
    cmd.duration_sec = float(duration)
    cmd.source = "e2e_check"
    return cmd


def send_pose(rclpy, node, ActionClient, ExecuteBodyPose, command):
    client = ActionClient(node, ExecuteBodyPose, "/locomotion_ros2/execute_body_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        return None, 0

    feedback_count = [0]

    def on_feedback(_msg):
        feedback_count[0] += 1

    goal = ExecuteBodyPose.Goal()
    goal.command = command
    goal.source = "e2e_check"

    goal_future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return "rejected", 0

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
    wrapped = result_future.result()
    if wrapped is None:
        return None, 0
    return wrapped.result, feedback_count[0]


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "43")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    launch_log = tempfile.NamedTemporaryFile(
        mode="w+",
        prefix="locomotion_ros2_bodypose_e2e_",
        suffix=".log",
        delete=False,
    )
    launch = subprocess.Popen(
        ["ros2", "launch", "locomotion_ros2_bringup", "mock_runtime.launch.py"],
        env=env,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        import rclpy
        from rclpy.action import ActionClient
        from locomotion_ros2_msgs.action import ExecuteBodyPose

        rclpy.init(args=None)
        node = rclpy.create_node("locomotion_ros2_bodypose_e2e_check")

        safe = make_pose(0.1, 0.1)
        result, feedbacks = send_pose(rclpy, node, ActionClient, ExecuteBodyPose, safe)
        if result is None or result == "rejected":
            print("safe body pose was not executed", file=sys.stderr)
            return 1
        if not result.success:
            print(f"safe body pose failed: {result.status_text}", file=sys.stderr)
            return 1
        if feedbacks < 1:
            print("expected at least one feedback for safe body pose", file=sys.stderr)
            return 1
        print(f"safe body pose succeeded with {feedbacks} feedbacks: {result.status_text}")

        # Combined tilt sqrt(0.6^2 + 0.6^2) ~= 0.85 rad, past the 0.70 fall band.
        unsafe = make_pose(0.6, 0.6)
        bad_result, _ = send_pose(rclpy, node, ActionClient, ExecuteBodyPose, unsafe)
        if bad_result is None:
            print("unsafe body pose produced no result", file=sys.stderr)
            return 1
        if bad_result != "rejected" and bad_result.success:
            print("unsafe (fall-band) body pose was NOT rejected", file=sys.stderr)
            return 1
        reason = bad_result if isinstance(bad_result, str) else bad_result.status_text
        print(f"unsafe body pose correctly rejected: {reason}")

        print("body pose action E2E passed: safe executed, fall-band rejected")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"body pose action E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
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
