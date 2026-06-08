#!/usr/bin/env python3
"""End-to-end check for the gait_lab SIL ros2_control-split path.

Brings up ``gait_lab_sil_ros2_control_runtime.launch.py`` and confirms:
  - ``/joint_states`` is published with the 12 G1 leg joints
  - the runtime still drives walking through the split sim + gait controller
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import suppress

REPO = Path(__file__).resolve().parents[1]


def terminate_process_group(process):
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "58")
    env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    launch = subprocess.Popen(
        [
            "ros2", "launch", "locomotion_ros2_bringup",
            "gait_lab_sil_ros2_control_runtime.launch.py",
            "controller:=rl-residual",
        ],
        env=env,
        preexec_fn=os.setsid,
    )

    node = None
    rclpy = None
    exit_code = 1
    try:
        import rclpy
        from rclpy.action import ActionClient
        from locomotion_ros2_msgs.action import ExecuteVelocity
        from locomotion_ros2_msgs.msg import WalkingState
        from sensor_msgs.msg import JointState

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_ros2_control_e2e")
        latest = {}
        joint_names = set()

        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: latest.__setitem__("state", m), 10)
        node.create_subscription(
            JointState, "/joint_states",
            lambda m: joint_names.update(m.name), 10)

        deadline = time.time() + 50.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if (s and s.active_adapter.endswith("GaitLabSilAdapter")
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and s.adapter_connected
                    and len(joint_names) >= 12):
                break

        if len(joint_names) < 12:
            print(
                f"expected >=12 /joint_states names, saw {len(joint_names)}",
                file=sys.stderr,
            )
            return 1
        print(f"ros2_control joint_states ok: {len(joint_names)} joints")

        s = latest.get("state")
        if s is None or not s.adapter_connected:
            print("SIL adapter never connected on ros2_control path", file=sys.stderr)
            return 1

        client = ActionClient(node, ExecuteVelocity, "/locomotion_ros2/execute_velocity")
        if not client.wait_for_server(timeout_sec=12.0):
            print("no execute_velocity server", file=sys.stderr)
            return 1

        # Let the split sim + gait controller exchange a few state/command cycles.
        settle_deadline = time.time() + 3.0
        while time.time() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        goal = ExecuteVelocity.Goal()
        goal.command.twist.linear.x = 0.3
        goal.duration_sec = 3.0
        observed = {"walking": False, "fell": False}

        def on_feedback(msg):
            st = msg.feedback.state
            if st.locomotion_state == WalkingState.STATE_WALKING:
                observed["walking"] = True
            if st.is_fallen:
                observed["fell"] = True

        goal_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=12.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("velocity goal rejected", file=sys.stderr)
            return 1
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=18.0)
        result = result_future.result().result
        print(
            f"velocity result: success={result.success} "
            f"walking={observed['walking']} fell={observed['fell']}"
        )
        final = latest.get("state")
        final_fallen = bool(final and final.is_fallen)
        if result.success and observed["walking"] and not final_fallen:
            print("gait_lab SIL ros2_control E2E passed")
            exit_code = 0
        else:
            print(
                f"gait_lab SIL ros2_control E2E failed "
                f"(final_fallen={final_fallen})",
                file=sys.stderr,
            )
    except Exception as error:  # noqa: BLE001
        print(f"gait_lab SIL ros2_control E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(launch)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
