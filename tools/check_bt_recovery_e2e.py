#!/usr/bin/env python3
"""End-to-end check for the live BehaviorTree.CPP recovery node.

Brings up the mock runtime, estops it and releases the estop (which leaves a
residual fault: the runtime reports STATE_ESTOPPED, not ready), then starts the
locomotion_ros2_bt_recovery_node and verifies the behavior tree drives the robot
back to a ready STANDING state by actually calling /locomotion_ros2/clear_fault.

Phase 1 (no recovery node) asserts the runtime does NOT self-recover, so the
recovery seen in phase 2 is genuinely the BT's doing -- not the runtime.
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
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def latest_state(rclpy, node, WalkingState, timeout=5.0):
    received = []
    sub = node.create_subscription(
        WalkingState, "/locomotion_ros2/state", lambda msg: received.append(msg), 10)
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return received[-1] if received else None


def is_ready(state, WalkingState):
    return (
        state is not None
        and state.adapter_connected
        and state.is_balanced
        and not state.is_fallen
        and not state.estop_active
        and state.locomotion_state in (
            WalkingState.STATE_STANDING, WalkingState.STATE_IDLE)
    )


def call_estop(rclpy, node, EmergencyStop, stop):
    client = node.create_client(EmergencyStop, "/locomotion_ros2/estop")
    if not client.wait_for_service(timeout_sec=10.0):
        return False
    request = EmergencyStop.Request()
    request.stop = stop
    request.reason = "bt recovery e2e"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result() is not None


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "48")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    recovery = None
    exit_code = 1
    runtime_log = tempfile.NamedTemporaryFile(
        mode="w+", prefix="locomotion_ros2_bt_recovery_e2e_", suffix=".log", delete=False)
    recovery_log = tempfile.NamedTemporaryFile(
        mode="w+", prefix="locomotion_ros2_bt_recovery_node_", suffix=".log", delete=False)
    runtime = subprocess.Popen(
        ["ros2", "launch", "locomotion_ros2_bringup", "mock_runtime.launch.py"],
        env=env, stdout=runtime_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    try:
        import rclpy
        from locomotion_ros2_msgs.msg import WalkingState
        from locomotion_ros2_msgs.srv import EmergencyStop

        rclpy.init(args=None)
        node = rclpy.create_node("locomotion_ros2_bt_recovery_e2e_check")

        # Wait for the runtime to come up ready.
        deadline = time.time() + 20.0
        state = None
        while time.time() < deadline:
            state = latest_state(rclpy, node, WalkingState, timeout=1.0)
            if is_ready(state, WalkingState):
                break
        if not is_ready(state, WalkingState):
            print("runtime never reached a ready state", file=sys.stderr)
            return 1
        print("runtime is up and ready")

        # Estop, then release. The runtime keeps STATE_ESTOPPED (residual fault).
        if not call_estop(rclpy, node, EmergencyStop, True):
            print("estop engage failed", file=sys.stderr)
            return 1
        time.sleep(0.5)
        if not call_estop(rclpy, node, EmergencyStop, False):
            print("estop release failed", file=sys.stderr)
            return 1
        time.sleep(1.0)

        # Phase 1: no recovery node -> the runtime must NOT self-recover.
        state = latest_state(rclpy, node, WalkingState, timeout=2.0)
        if is_ready(state, WalkingState):
            print("runtime self-recovered without the BT; test not discriminating",
                  file=sys.stderr)
            return 1
        print(f"phase 1 ok: still not ready after estop release "
              f"(locomotion_state={state.locomotion_state})")

        # Phase 2: start the recovery node and expect the BT to clear the fault.
        recovery = subprocess.Popen(
            ["ros2", "run", "locomotion_ros2_bt", "locomotion_ros2_bt_recovery_node",
             "--ros-args", "-p", "tick_period_sec:=0.3"],
            env=env, stdout=recovery_log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        recovered = False
        deadline = time.time() + 15.0
        while time.time() < deadline:
            state = latest_state(rclpy, node, WalkingState, timeout=1.0)
            if is_ready(state, WalkingState):
                recovered = True
                break
        if not recovered:
            print("BT recovery node did not restore readiness", file=sys.stderr)
            return 1

        print(f"phase 2 ok: BT recovery restored readiness "
              f"(locomotion_state={state.locomotion_state})")
        print("BT recovery E2E passed: runtime stayed faulted alone, BT cleared it")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"BT recovery E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(recovery)
        terminate_process_group(runtime)
        for log in (runtime_log, recovery_log):
            log.flush()
        if exit_code != 0:
            for label, log in (("runtime", runtime_log), ("recovery", recovery_log)):
                log.seek(0)
                sys.stderr.write(f"--- {label} log ---\n{log.read()}")
        for log in (runtime_log, recovery_log):
            with suppress(OSError):
                Path(log.name).unlink()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
