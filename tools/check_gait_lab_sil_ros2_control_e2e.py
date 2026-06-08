#!/usr/bin/env python3
"""End-to-end check for the gait_lab SIL ros2_control-split path.

Brings up ``gait_lab_sil_ros2_control_runtime.launch.py`` and confirms:
  - ``/joint_states`` is published with the 12 G1 leg joints
  - the runtime still drives walking through the split sim + gait controller

Pass ``--forward`` to exercise the ``GaitLabSilJointForwardController`` path
(use_ros2_control_forward:=true).

Pass ``--embedded`` for the C++ ``GaitLabSilRlResidualController`` path
(use_embedded_rl_policy:=true).

Pass ``--steer`` for the B2 quantitative gate: ``rl-steerable`` on the embedded
ros2_control path must walk, turn in the commanded direction, and not fall during
an ``ExecuteVelocity`` yaw+forward command.

Pass ``--steer-loose`` to keep the first-rung survival check (any heading/travel).
"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import suppress

REPO = Path(__file__).resolve().parents[1]

# B2 quantitative gate (matches eval_steerable.py arc-ish commands at 8 s).
STEER_CMD_VX = 0.2
STEER_CMD_YAW = 0.25
STEER_DURATION_SEC = 8.0
STEER_MIN_YAW_RAD = 0.20
STEER_MIN_TRAVEL_M = 0.25


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--forward",
        action="store_true",
        help="use GaitLabSilJointForwardController (ros2_control command loop)",
    )
    parser.add_argument(
        "--embedded",
        action="store_true",
        help="use GaitLabSilRlResidualController (C++ RL inference)",
    )
    parser.add_argument(
        "--steer",
        action="store_true",
        help="B2 gate: signed yaw turn + travel on embedded rl-steerable",
    )
    parser.add_argument(
        "--steer-loose",
        action="store_true",
        help="first-rung steer check (any yaw/travel while walking)",
    )
    parser.add_argument(
        "--controller",
        default="rl-residual",
        help="gait_lab controller (use rl-steerable with --steer)",
    )
    args = parser.parse_args()

    if args.forward and args.embedded:
        print("choose only one of --forward or --embedded", file=sys.stderr)
        return 2
    if args.steer or args.steer_loose:
        args.controller = "rl-steerable"
    if (args.steer or args.steer_loose) and not args.forward:
        # Turning needs the 500 Hz relay path; embedded RL keeps policy parity.
        args.embedded = True

    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    # Fresh domain per run avoids stale processes from prior failed launches.
    env.setdefault("ROS_DOMAIN_ID", str(80 + (int(time.time()) % 20)))
    env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    launch_args = [
        "ros2", "launch", "locomotion_ros2_bringup",
        "gait_lab_sil_ros2_control_runtime.launch.py",
        f"controller:={args.controller}",
    ]
    if args.forward:
        launch_args.append("use_ros2_control_forward:=true")
    if args.embedded:
        launch_args.append("use_embedded_rl_policy:=true")

    launch = subprocess.Popen(
        launch_args,
        env=env,
        preexec_fn=os.setsid,
    )

    node = None
    rclpy = None
    exit_code = 1
    if args.embedded:
        path_label = "embedded"
    elif args.forward:
        path_label = "forward"
    else:
        path_label = "direct"
    if args.steer:
        path_label += "+steer"
    elif args.steer_loose:
        path_label += "+steer-loose"
    try:
        import rclpy
        from rclpy.action import ActionClient
        from locomotion_ros2_msgs.action import ExecuteVelocity
        from locomotion_ros2_msgs.msg import WalkingState
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState

        rclpy.init(args=None)
        node = rclpy.create_node("gait_lab_sil_ros2_control_e2e")
        latest = {}
        joint_names = set()
        odom_yaw = {"start": None, "latest": None}
        odom_xy = {"start": None, "latest": None}

        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: latest.__setitem__("state", m), 10)
        node.create_subscription(
            JointState, "/joint_states",
            lambda m: joint_names.update(m.name), 10)

        def on_odom(msg: Odometry):
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            odom_yaw["latest"] = yaw
            if odom_yaw["start"] is None:
                odom_yaw["start"] = yaw
            xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
            odom_xy["latest"] = xy
            if odom_xy["start"] is None:
                odom_xy["start"] = xy

        node.create_subscription(Odometry, "/gait_lab_sil/odom", on_odom, 10)

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
        print(f"ros2_control joint_states ok: {len(joint_names)} joints ({path_label})")

        s = latest.get("state")
        if s is None or not s.adapter_connected:
            print("SIL adapter never connected on ros2_control path", file=sys.stderr)
            return 1

        client = ActionClient(node, ExecuteVelocity, "/locomotion_ros2/execute_velocity")
        if not client.wait_for_server(timeout_sec=12.0):
            print("no execute_velocity server", file=sys.stderr)
            return 1

        settle_deadline = time.time() + (
            5.0 if (args.forward or args.embedded) else 3.0)
        while time.time() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if args.steer or args.steer_loose:
            odom_deadline = time.time() + 8.0
            while time.time() < odom_deadline and odom_yaw["latest"] is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            if odom_yaw["latest"] is None:
                print("no /gait_lab_sil/odom before steer goal", file=sys.stderr)
                return 1

        goal = ExecuteVelocity.Goal()
        if args.steer or args.steer_loose:
            goal.command.twist.linear.x = STEER_CMD_VX
            goal.command.twist.angular.z = STEER_CMD_YAW
            goal.duration_sec = STEER_DURATION_SEC
        else:
            goal.command.twist.linear.x = 0.3
            goal.duration_sec = 3.0
        observed = {"walking": False, "fell": False}
        odom_yaw["start"] = odom_yaw["latest"]
        odom_xy["start"] = odom_xy["latest"]

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
        result_deadline = time.time() + (
            24.0 if (args.steer or args.steer_loose) else 18.0)
        while time.time() < result_deadline and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result_future.done():
            print("velocity goal timed out", file=sys.stderr)
            return 1
        result = result_future.result().result
        signed_yaw = 0.0
        travel = 0.0
        if odom_yaw["start"] is not None and odom_yaw["latest"] is not None:
            signed_yaw = odom_yaw["latest"] - odom_yaw["start"]
            signed_yaw = math.atan2(
                math.sin(signed_yaw), math.cos(signed_yaw))
        if odom_xy["start"] is not None and odom_xy["latest"] is not None:
            dx = odom_xy["latest"][0] - odom_xy["start"][0]
            dy = odom_xy["latest"][1] - odom_xy["start"][1]
            travel = math.hypot(dx, dy)
        yaw_abs = abs(signed_yaw)
        expected_yaw = STEER_CMD_YAW * STEER_DURATION_SEC
        tracking = yaw_abs / expected_yaw if expected_yaw > 0 else 0.0
        print(
            f"velocity result: success={result.success} "
            f"walking={observed['walking']} fell={observed['fell']} "
            f"signed_yaw={signed_yaw:+.2f}rad travel={travel:.2f}m "
            f"tracking={tracking:.0%}"
        )
        final = latest.get("state")
        final_fallen = bool(final and final.is_fallen)
        if args.steer:
            steered = (
                signed_yaw * STEER_CMD_YAW > 0
                and yaw_abs >= STEER_MIN_YAW_RAD
                and travel >= STEER_MIN_TRAVEL_M)
            fell_ok = not observed["fell"]
        elif args.steer_loose:
            steered = (
                yaw_abs >= 0.06 or travel >= 0.15 or observed["walking"])
            fell_ok = not final_fallen
        else:
            steered = True
            fell_ok = not final_fallen
        if result.success and observed["walking"] and fell_ok and steered:
            print(f"gait_lab SIL ros2_control E2E passed ({path_label})")
            exit_code = 0
        else:
            if (args.steer and steered is False and yaw_abs >= STEER_MIN_YAW_RAD
                    and signed_yaw * STEER_CMD_YAW <= 0):
                print(
                    "steer sign mismatch: positive yaw command but odometry turned "
                    f"the other way ({signed_yaw:+.2f} rad)",
                    file=sys.stderr,
                )
            print(
                f"gait_lab SIL ros2_control E2E failed ({path_label}, "
                f"fell_ok={fell_ok}, steered={steered})",
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
