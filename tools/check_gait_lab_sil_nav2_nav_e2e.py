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
    ap.add_argument("--goal-x", type=float, default=2.0)
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--tolerance", type=float, default=0.8)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--attempts", type=int, default=1,
                    help="extra NavigateToPose tries when the first is rejected")
    ap.add_argument("--controller", default="rl-steerable",
                    help="gait_lab controller (try rl-steerable-footstep for tight turns)")
    ap.add_argument(
        "--embedded",
        action="store_true",
        help="use ros2_control embedded C++ RL (default: monolithic sim)",
    )
    args = ap.parse_args()

    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", str(60 + (int(time.time()) % 20)))
    env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    # The sim node is launched via its `#!/usr/bin/env python3` shebang, so make a
    # mujoco-capable interpreter win on PATH (the launch's other nodes are C++).
    sim_python = env.get("GAIT_LAB_SIL_SIM_PYTHON", "")
    if sim_python:
        env["PATH"] = os.path.dirname(sim_python) + os.pathsep + env.get("PATH", "")
    launch_args = [
        "ros2", "launch", "locomotion_ros2_bringup", "gait_lab_sil_nav2.launch.py",
        f"controller:={args.controller}",
    ]
    if not args.embedded:
        launch_args.append("use_ros2_control_embedded:=false")
    launch = subprocess.Popen(launch_args, env=env, preexec_fn=os.setsid)

    node = None
    rclpy = None
    exit_code = 1
    try:
        import rclpy
        from rclpy.action import ActionClient
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import FollowPath, NavigateToPose
        from locomotion_ros2_msgs.msg import WalkingState
        from tf2_ros import Buffer, TransformListener

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_nav_e2e")
        latest = {}
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
        node.create_subscription(Odometry, "/odom",
                                 lambda m: latest.__setitem__("odom", m), 10)
        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: latest.__setitem__("state", m), 10)

        def tf_ready() -> bool:
            try:
                tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                return True
            except Exception:  # noqa: BLE001
                return False

        from lifecycle_msgs.srv import GetState

        nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        follow_client = ActionClient(node, FollowPath, "follow_path")
        lifecycle_clients = {
            name: node.create_client(GetState, f"/{name}/get_state")
            for name in (
                "planner_server", "controller_server", "bt_navigator",
            )
        }

        def lifecycle_active(name: str) -> bool:
            client = lifecycle_clients[name]
            if not client.service_is_ready():
                return False
            req = GetState.Request()
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
            result = fut.result()
            return result is not None and result.current_state.id == 3

        deadline = time.time() + (120.0 if not args.embedded else 150.0)
        ready = False
        nav2_active_cached = False
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if not nav2_active_cached:
                nav2_active_cached = all(
                    lifecycle_active(n) for n in lifecycle_clients)
            if (s and s.adapter_connected
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and nav_client.server_is_ready()
                    and follow_client.server_is_ready()
                    and nav2_active_cached
                    and tf_ready()
                    and latest.get("odom") is not None):
                ready = True
                break
        if not ready:
            print("SIL robot, TF map->base_link, or Nav2 not ready in time",
                  file=sys.stderr)
            return 1
        # Lifecycle activate finishes before action servers accept goals.
        settle_end = time.time() + 5.0
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.2)
        stack = "ros2_control_embedded" if args.embedded else "monolithic"
        print(f"SIL robot active ({stack}) and Nav2 up")

        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        def prime_gait(seconds: float = 2.0, speed: float = 0.22):
            twist = Twist()
            twist.linear.x = speed
            prime_end = time.time() + seconds
            while time.time() < prime_end:
                cmd_pub.publish(twist)
                rclpy.spin_once(node, timeout_sec=0.1)

        def distance_to_goal() -> float:
            od = latest.get("odom")
            if od is None:
                return float("inf")
            dx = od.pose.pose.position.x - args.goal_x
            dy = od.pose.pose.position.y - args.goal_y
            return math.hypot(dx, dy)

        prime_gait()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.pose.position.x = args.goal_x
        nav_goal.pose.pose.position.y = args.goal_y
        nav_goal.pose.pose.orientation.w = 1.0

        goal_handle = None
        for attempt in range(1, args.attempts + 2):
            nav_goal.pose.header.stamp = node.get_clock().now().to_msg()
            print(f"NavigateToPose attempt {attempt} -> ({args.goal_x}, {args.goal_y})")
            goal_future = nav_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(node, goal_future, timeout_sec=15.0)
            goal_handle = goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break
            time.sleep(1.0)
        if goal_handle is None or not goal_handle.accepted:
            print("NavigateToPose goal rejected", file=sys.stderr)
            return 1

        best = float("inf")
        max_lateral = 0.0
        fell_before_reach = False
        reached = False
        nav_timeout = args.timeout * (1.25 if args.embedded else 1.0)
        end = time.time() + nav_timeout
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            st = latest.get("state")
            if st and st.is_fallen and not reached:
                fell_before_reach = True
            od = latest.get("odom")
            if od is not None:
                max_lateral = max(max_lateral, abs(od.pose.pose.position.y))
            dist = distance_to_goal()
            best = min(best, dist)
            if dist <= args.tolerance:
                reached = True
                break

        print(f"nav result: reached={reached} closest={best:.2f}m "
              f"max_lateral={max_lateral:.2f}m tolerance={args.tolerance}m "
              f"fell_before_reach={fell_before_reach}")
        if reached and not fell_before_reach:
            print(f"gait_lab SIL full-Nav2 E2E passed ({stack}): the stack planned "
                  "and walked the steerable RL gait to the goal")
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
