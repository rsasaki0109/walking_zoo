#!/usr/bin/env python3
"""End-to-end check for the gait_lab SIL adapter in the real runtime.

Brings up the runtime manager configured to load
`walking_zoo_gait_lab_sil/GaitLabSilAdapter` together with the companion MuJoCo
sim node (`gait_lab_sil_sim.py`, running the reinforcement-learned `rl-residual`
gait). It then drives an ExecuteVelocity goal through the runtime + safety
pipeline and confirms the simulated robot walks (WalkingState reports WALKING,
not fallen, with the adapter connected to the sim).

This exercises the whole bridge: pluginlib discovery of the SIL adapter, the
adapter forwarding the safety-filtered velocity to the sim, the MuJoCo G1 walking
under the learned policy, and the simulated state flowing back into the runtime.

Requires a Python with both rclpy and mujoco (gait_lab's deps). Run it with such
an interpreter, ROS + the workspace sourced; the sim subprocess reuses
sys.executable by default (override with GAIT_LAB_SIL_SIM_PYTHON). Point at the
gait_lab checkout with WALKING_ZOO_GAIT_LAB_PATH and a menagerie G1 with
WALKING_ZOO_MENAGERIE_PATH.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import suppress

REPO = Path(__file__).resolve().parents[1]
SIM_SCRIPT = REPO / "src" / "walking_zoo_examples" / "scripts" / "gait_lab_sil_sim.py"


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


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "57")
    env.setdefault("WALKING_ZOO_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    sim_python = env.get("GAIT_LAB_SIL_SIM_PYTHON", sys.executable)

    runtime = subprocess.Popen(
        [
            "ros2", "run", "walking_zoo_runtime", "walking_zoo_runtime_manager",
            "--ros-args",
            "-p", "autostart:=true",
            "-p", "adapter_plugin:=walking_zoo_gait_lab_sil/GaitLabSilAdapter",
            "-p", "robot_model:=g1",
            "-p", "robot_family:=humanoid",
            "-p", "limits.max_linear_x:=0.4",
        ],
        env=env,
        preexec_fn=os.setsid,
    )
    sim = subprocess.Popen(
        [
            sim_python, str(SIM_SCRIPT),
            "--ros-args", "-p", "controller:=rl-residual",
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
        from walking_zoo_msgs.action import ExecuteVelocity
        from walking_zoo_msgs.msg import WalkingState

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_e2e")
        latest = {}
        node.create_subscription(
            WalkingState, "/walking_zoo/state",
            lambda m: latest.__setitem__("state", m), 10)

        # Wait for the adapter to be active AND connected to the MuJoCo sim
        # (MuJoCo + the G1 model load takes a few seconds).
        deadline = time.time() + 45.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if (s and s.active_adapter.endswith("GaitLabSilAdapter")
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and s.adapter_connected):
                break
        s = latest.get("state")
        if s is None or not s.active_adapter.endswith("GaitLabSilAdapter"):
            print(f"SIL adapter not loaded (got {s and s.active_adapter})", file=sys.stderr)
            return 1
        if not s.adapter_connected:
            print("SIL adapter never connected to the MuJoCo sim node", file=sys.stderr)
            return 1
        print(f"SIL adapter active and connected to sim: model={s.active_robot_model}")

        client = ActionClient(node, ExecuteVelocity, "/walking_zoo/execute_velocity")
        if not client.wait_for_server(timeout_sec=10.0):
            print("no execute_velocity server", file=sys.stderr)
            return 1

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
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("velocity goal rejected", file=sys.stderr)
            return 1
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
        result = result_future.result().result
        print(f"velocity result: success={result.success} "
              f"walking={observed['walking']} fell={observed['fell']} "
              f"text='{result.status_text}'")

        if result.success and observed["walking"] and not observed["fell"]:
            print("gait_lab SIL E2E passed: RL gait walks the full command via the "
                  "real runtime + safety pipeline, no fall")
            exit_code = 0
        else:
            print("gait_lab SIL E2E failed: did not observe sustained walking",
                  file=sys.stderr)
    except Exception as error:  # noqa: BLE001
        print(f"gait_lab SIL E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(sim)
        terminate_process_group(runtime)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
