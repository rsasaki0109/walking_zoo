#!/usr/bin/env python3
"""End-to-end check for loading the Unitree SDK2 adapter in the real runtime.

Launches the runtime manager configured to load the
`walking_zoo_unitree_sdk2/UnitreeSdk2Adapter` plugin, waits for it to autostart
into the active (balance-stand) state, then drives an ExecuteVelocity goal and
confirms the adapter's locomotion FSM reports WALKING. This exercises pluginlib
discovery, the adapter lifecycle, and the G1 command translation through the
real runtime (software-in-the-loop; no vendor SDK or hardware required).
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
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


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "55")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    runtime = subprocess.Popen(
        [
            "ros2", "run", "walking_zoo_runtime", "walking_zoo_runtime_manager",
            "--ros-args",
            "-p", "autostart:=true",
            "-p", "adapter_plugin:=walking_zoo_unitree_sdk2/UnitreeSdk2Adapter",
            "-p", "robot_model:=g1",
            "-p", "robot_family:=humanoid",
            "-p", "limits.max_linear_x:=0.6",
            "-p", "limits.max_linear_y:=0.4",
            "-p", "limits.max_angular_z:=0.8",
        ],
        env=env,
        preexec_fn=os.setsid,
    )

    try:
        import rclpy
        from rclpy.action import ActionClient
        from walking_zoo_msgs.action import ExecuteVelocity
        from walking_zoo_msgs.msg import WalkingState

        rclpy.init(args=None)
        node = rclpy.create_node("walking_zoo_unitree_adapter_e2e")

        latest = {}
        node.create_subscription(
            WalkingState, "/walking_zoo/state", lambda m: latest.__setitem__("state", m), 10)

        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            state = latest.get("state")
            if (state and state.active_adapter.endswith("UnitreeSdk2Adapter")
                    and state.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE):
                break

        state = latest.get("state")
        if state is None:
            print("no runtime state received", file=sys.stderr)
            return 1
        if not state.active_adapter.endswith("UnitreeSdk2Adapter"):
            print(f"unitree adapter not loaded (got {state.active_adapter})", file=sys.stderr)
            return 1
        print(f"unitree adapter active: model={state.active_robot_model} "
              f"loco_state={state.locomotion_state}")

        client = ActionClient(node, ExecuteVelocity, "/walking_zoo/execute_velocity")
        if not client.wait_for_server(timeout_sec=10.0):
            print("no execute_velocity server", file=sys.stderr)
            return 1

        goal = ExecuteVelocity.Goal()
        goal.command.twist.linear.x = 0.3
        goal.duration_sec = 0.5
        saw_walking = {"value": False}

        def on_feedback(msg):
            if msg.feedback.state.locomotion_state == WalkingState.STATE_WALKING:
                saw_walking["value"] = True

        goal_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("velocity goal rejected", file=sys.stderr)
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        result = result_future.result().result
        print(f"velocity result: success={result.success} text='{result.status_text}' "
              f"saw_walking={saw_walking['value']}")

        if result.success and saw_walking["value"]:
            print("unitree adapter E2E passed: pluginlib load, active, walking via real runtime")
            exit_code = 0
        else:
            print("unitree adapter E2E failed: did not observe walking", file=sys.stderr)
    except Exception as error:  # noqa: BLE001
        print(f"unitree adapter E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(runtime)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
