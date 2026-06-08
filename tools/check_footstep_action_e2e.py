#!/usr/bin/env python3
"""End-to-end check for the runtime ExecuteFootstepPlan action.

Launches the mock runtime, sends a feasible footstep plan (expecting success with
per-step feedback), then sends an obviously infeasible plan (expecting the
runtime feasibility gate to reject it). This exercises the real runtime action
server, the StepFeasibilityChecker, and the mock adapter together.
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


def make_plan(footsteps):
    from geometry_msgs.msg import Pose
    from locomotion_ros2_msgs.msg import Footstep, FootstepPlan

    plan = FootstepPlan()
    plan.frame_id = "base_link"
    plan.planner_id = "e2e_check"
    for leg, x, y in footsteps:
        step = Footstep()
        step.leg_name = leg
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        step.pose = pose
        step.swing_height = 0.05
        step.duration = 0.2
        plan.footsteps.append(step)
    return plan


def send_plan(rclpy, node, ActionClient, ExecuteFootstepPlan, plan):
    client = ActionClient(node, ExecuteFootstepPlan, "/locomotion_ros2/execute_footstep_plan")
    if not client.wait_for_server(timeout_sec=10.0):
        return None, 0

    feedback_steps = []

    def on_feedback(msg):
        feedback_steps.append(msg.feedback.completed_steps)

    goal = ExecuteFootstepPlan.Goal()
    goal.plan = plan
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
    return wrapped.result, max(feedback_steps) if feedback_steps else 0


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
        prefix="locomotion_ros2_footstep_e2e_",
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
        from locomotion_ros2_msgs.action import ExecuteFootstepPlan

        rclpy.init(args=None)
        node = rclpy.create_node("locomotion_ros2_footstep_e2e_check")

        feasible = make_plan([
            ("left", 0.10, 0.16),
            ("right", 0.20, -0.16),
            ("left", 0.30, 0.16),
            ("right", 0.40, -0.16),
        ])
        result, steps = send_plan(rclpy, node, ActionClient, ExecuteFootstepPlan, feasible)
        if result is None or result == "rejected":
            print("feasible footstep plan was not executed", file=sys.stderr)
            return 1
        if not result.success:
            print(f"feasible plan failed: {result.status_text}", file=sys.stderr)
            return 1
        if steps < 4:
            print(f"expected feedback up to 4 steps, saw {steps}", file=sys.stderr)
            return 1
        print(f"feasible plan succeeded with {steps} step feedbacks: {result.status_text}")

        infeasible = make_plan([
            ("left", 0.0, 0.16),
            ("right", 2.0, -0.16),  # 2 m stride: must be rejected
        ])
        bad_result, _ = send_plan(rclpy, node, ActionClient, ExecuteFootstepPlan, infeasible)
        if bad_result is None:
            print("infeasible plan produced no result", file=sys.stderr)
            return 1
        if bad_result != "rejected" and bad_result.success:
            print("infeasible footstep plan was NOT rejected", file=sys.stderr)
            return 1
        reason = bad_result if isinstance(bad_result, str) else bad_result.status_text
        print(f"infeasible plan correctly rejected: {reason}")

        print("footstep action E2E passed: feasible executed, infeasible rejected")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"footstep action E2E error: {error}", file=sys.stderr)
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
