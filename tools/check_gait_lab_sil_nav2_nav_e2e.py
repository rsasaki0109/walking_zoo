#!/usr/bin/env python3
"""End-to-end check: the FULL Nav2 stack autonomously walks the SIL G1 to a goal.

Where ``check_gait_lab_sil_nav2_e2e.py`` only exercises the raw /cmd_vel path,
this brings up the *complete* Nav2 stack via ``gait_lab_sil_nav2.launch.py`` —
map server, NavFn planner, Regulated Pure Pursuit controller, behaviour tree,
recovery behaviours — driving the reinforcement-learned **steerable** gait_lab
gait in MuJoCo behind the runtime + safety pipeline. It sends one
``NavigateToPose`` goal and checks the robot actually gets near it.

Success = the robot's odometry reaches within ``--tolerance`` of the goal (the
planner planned, the controller drove the legged gait there, and it did not fall
on the way). Run with an interpreter that has rclpy + mujoco, ROS + workspace
sourced.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from contextlib import suppress
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def terminate(process):
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal-x", type=float, default=2.5)
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--tolerance", type=float, default=0.6)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--controller", default="rl-steerable",
                    help="gait_lab controller the SIL sim runs")
    args = ap.parse_args()

    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "65")
    env.setdefault("WALKING_ZOO_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    # The sim node is launched via its `#!/usr/bin/env python3` shebang, so make a
    # mujoco-capable interpreter win on PATH (the launch's other nodes are C++).
    sim_python = env.get("GAIT_LAB_SIL_SIM_PYTHON", "")
    if sim_python:
        env["PATH"] = os.path.dirname(sim_python) + os.pathsep + env.get("PATH", "")
    launch = subprocess.Popen(
        ["ros2", "launch", "walking_zoo_bringup", "gait_lab_sil_nav2.launch.py",
         f"controller:={args.controller}"],
        env=env, preexec_fn=os.setsid)

    node = None
    rclpy = None
    exit_code = 1
    try:
        import rclpy
        from rclpy.action import ActionClient
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import NavigateToPose
        from walking_zoo_msgs.msg import WalkingState

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_nav_e2e")
        latest = {}
        node.create_subscription(Odometry, "/odom",
                                 lambda m: latest.__setitem__("odom", m), 10)
        node.create_subscription(
            WalkingState, "/walking_zoo/state",
            lambda m: latest.__setitem__("state", m), 10)

        # Wait for the SIL robot to be active and Nav2's action server to be up.
        nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        deadline = time.time() + 90.0
        ready = False
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if (s and s.adapter_connected
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and nav_client.server_is_ready()):
                ready = True
                break
        if not ready:
            print("SIL robot or Nav2 action server not ready in time", file=sys.stderr)
            return 1
        print("SIL robot active and Nav2 up; sending NavigateToPose goal "
              f"({args.goal_x}, {args.goal_y})")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.w = 1.0
        nav_client.send_goal_async(goal)

        best = float("inf")
        fell = False
        reached = False
        end = time.time() + args.timeout
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            od = latest.get("odom")
            st = latest.get("state")
            if st and st.is_fallen:
                fell = True
            if od is not None:
                dx = od.pose.pose.position.x - args.goal_x
                dy = od.pose.pose.position.y - args.goal_y
                dist = math.hypot(dx, dy)
                best = min(best, dist)
                if dist <= args.tolerance:
                    reached = True
                    break
        print(f"nav result: reached={reached} closest={best:.2f}m "
              f"tolerance={args.tolerance}m fell={fell}")
        if reached and not fell:
            print("gait_lab SIL full-Nav2 E2E passed: the stack planned and walked "
                  "the steerable RL gait to the goal")
            exit_code = 0
        else:
            print("gait_lab SIL full-Nav2 E2E failed", file=sys.stderr)
    except Exception as error:  # noqa: BLE001
        print(f"gait_lab SIL full-Nav2 E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate(launch)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
