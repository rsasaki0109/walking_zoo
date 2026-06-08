#!/usr/bin/env python3
"""End-to-end check: a Nav2-style /cmd_vel drives the gait_lab SIL robot.

Where ``check_gait_lab_sil_e2e.py`` drives the SIL adapter through the
ExecuteVelocity *action*, this exercises the **Nav2 velocity path**: it brings up
the runtime (gait_lab SIL adapter) + the MuJoCo sim + the legged ``cmd_vel_bridge``
(the same shaper + readiness gate Nav2 uses), then publishes a plain
``/cmd_vel`` Twist stream — exactly what Nav2's controller server emits. The
bridge shapes it to the legged envelope and forwards it (only while the robot is
ready) to the runtime, which filters it through the safety pipeline and into the
SIL adapter. Success = the reinforcement-learned gait walks under Nav2-style
velocity commands without falling.

Run with an interpreter that has both rclpy and mujoco (gait_lab's deps), ROS +
the workspace sourced. See ``check_gait_lab_sil_e2e.py`` for the env vars.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import suppress

REPO = Path(__file__).resolve().parents[1]
SIM_SCRIPT = REPO / "src" / "locomotion_ros2_examples" / "scripts" / "gait_lab_sil_sim.py"


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
    env.setdefault("ROS_DOMAIN_ID", "63")
    env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    sim_python = env.get("GAIT_LAB_SIL_SIM_PYTHON", sys.executable)

    procs = []
    procs.append(subprocess.Popen(
        ["ros2", "run", "locomotion_ros2_runtime", "locomotion_ros2_runtime_manager",
         "--ros-args", "-p", "autostart:=true",
         "-p", "adapter_plugin:=locomotion_ros2_gait_lab_sil/GaitLabSilAdapter",
         "-p", "robot_model:=g1", "-p", "robot_family:=humanoid",
         "-p", "limits.max_linear_x:=0.4"],
        env=env, preexec_fn=os.setsid))
    procs.append(subprocess.Popen(
        [sim_python, str(SIM_SCRIPT), "--ros-args", "-p", "controller:=rl-residual"],
        env=env, preexec_fn=os.setsid))
    # The legged Nav2 bridge: /cmd_vel (Twist) -> /locomotion_ros2/cmd_vel (shaped).
    procs.append(subprocess.Popen(
        ["ros2", "run", "locomotion_ros2_nav2", "cmd_vel_bridge"],
        env=env, preexec_fn=os.setsid))

    node = None
    rclpy = None
    exit_code = 1
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from locomotion_ros2_msgs.msg import WalkingState

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_nav2_e2e")
        latest = {}
        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: latest.__setitem__("state", m), 10)
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        # Wait for the SIL adapter to be active + connected to the sim.
        deadline = time.time() + 45.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if (s and s.active_adapter.endswith("GaitLabSilAdapter")
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and s.adapter_connected):
                break
        s = latest.get("state")
        if s is None or not s.adapter_connected:
            print("SIL adapter/sim not ready", file=sys.stderr)
            return 1
        print("SIL adapter active and connected; streaming Nav2-style /cmd_vel")

        # Stream a plain Twist (as Nav2's controller does) for a few seconds.
        twist = Twist()
        twist.linear.x = 0.3
        observed = {"walking": False, "fell": False}
        end = time.time() + 4.0
        while time.time() < end:
            cmd_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.1)
            st = latest.get("state")
            if st:
                if st.locomotion_state == WalkingState.STATE_WALKING:
                    observed["walking"] = True
                if st.is_fallen:
                    observed["fell"] = True

        print(f"nav2-path result: walking={observed['walking']} fell={observed['fell']}")
        if observed["walking"] and not observed["fell"]:
            print("gait_lab SIL Nav2 E2E passed: RL gait walks under Nav2-style "
                  "/cmd_vel via the legged shaper + safety pipeline, no fall")
            exit_code = 0
        else:
            print("gait_lab SIL Nav2 E2E failed: did not observe sustained walking",
                  file=sys.stderr)
    except Exception as error:  # noqa: BLE001
        print(f"gait_lab SIL Nav2 E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        for p in reversed(procs):
            terminate_process_group(p)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
