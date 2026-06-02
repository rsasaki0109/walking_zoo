#!/usr/bin/env python3
"""End-to-end check for terrain-aware footstep planning.

Launches the real footstep_marker_publisher node with a keep-out zone over the
first foothold and a curb under the mid-stride feet, then subscribes to the
published /walking_zoo/footstep_plan and asserts the planner actually reacted to
the terrain: a foot nudged clear of the keep-out band and feet raised onto the
curb with extra swing. Exercises the TerrainModel + FootstepPlanner through the
live ROS node, not just unit logic.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import suppress

# Terrain handed to the node (must match the unit-test geometry).
KEEP_OUT = [0.0, 0.10, 0.2, 0.22]          # min_x, min_y, max_x, max_y
CURB = [0.25, -1.0, 0.55, 1.0, 0.12]       # + height
CURB_HEIGHT = CURB[4]


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


def in_keep_out(x, y):
    return KEEP_OUT[0] <= x <= KEEP_OUT[2] and KEEP_OUT[1] <= y <= KEEP_OUT[3]


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "47")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    node_log = tempfile.NamedTemporaryFile(
        mode="w+", prefix="walking_zoo_terrain_e2e_", suffix=".log", delete=False)
    launch = subprocess.Popen(
        [
            "ros2", "run", "walking_zoo_runtime", "footstep_marker_publisher",
            "--ros-args",
            "-p", "step_count:=6",
            "-p", f"no_step_zone:={KEEP_OUT}",
            "-p", f"curb_box:={CURB}",
        ],
        env=env,
        stdout=node_log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        import rclpy
        from walking_zoo_msgs.msg import FootstepPlan

        rclpy.init(args=None)
        node = rclpy.create_node("walking_zoo_terrain_e2e_check")

        received = []
        node.create_subscription(
            FootstepPlan, "/walking_zoo/footstep_plan",
            lambda msg: received.append(msg), 10)

        deadline = time.time() + 15.0
        while time.time() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.5)

        if not received:
            print("no footstep plan received", file=sys.stderr)
            return 1

        plan = received[-1]
        if plan.planner_id != "walking_zoo_terrain_planner":
            print(f"unexpected planner_id: {plan.planner_id}", file=sys.stderr)
            return 1

        feet = [(f.pose.position.x, f.pose.position.y, f.pose.position.z, f.swing_height)
                for f in plan.footsteps]

        # 1) No foot may land inside the keep-out zone.
        inside = [(x, y) for (x, y, _z, _s) in feet if in_keep_out(x, y)]
        if inside:
            print(f"feet still inside keep-out zone: {inside}", file=sys.stderr)
            return 1

        # 2) At least one foot must have been nudged: a foot at the first
        #    foothold's x (~0.1) but pushed clear of the band on y.
        nudged = [(x, y) for (x, y, _z, _s) in feet if abs(x - 0.1) < 1e-6 and abs(y) > 0.22]
        if not nudged:
            print(f"no foot was nudged clear of the keep-out zone: {feet}", file=sys.stderr)
            return 1

        # 3) Mid-stride feet must be raised onto the curb.
        on_curb = [(x, z) for (x, _y, z, _s) in feet if z > CURB_HEIGHT - 1e-3]
        if not on_curb:
            print(f"no foot raised onto the curb: {feet}", file=sys.stderr)
            return 1

        # 4) The step-up foot must have extra swing to clear the rise.
        step_up_swing = [s for (x, _y, z, s) in feet if z > CURB_HEIGHT - 1e-3 and s > 0.12]
        if not step_up_swing:
            print(f"no raised swing for the step-up: {feet}", file=sys.stderr)
            return 1

        print(
            "footstep terrain E2E passed: "
            f"{len(nudged)} foot nudged clear, {len(on_curb)} feet on curb "
            f"(z={CURB_HEIGHT}), step-up swing={max(step_up_swing):.3f}")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"footstep terrain E2E error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(launch)
        node_log.flush()
        if exit_code != 0:
            node_log.seek(0)
            sys.stderr.write(node_log.read())
        with suppress(OSError):
            Path(node_log.name).unlink()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
