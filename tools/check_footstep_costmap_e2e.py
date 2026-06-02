#!/usr/bin/env python3
"""End-to-end check: footstep planning driven by a real OccupancyGrid costmap.

Publishes a synthetic Nav2-style ``nav_msgs/OccupancyGrid`` (latched) with a
lethal patch over the first foothold, then launches the real
``footstep_marker_publisher`` subscribed to that costmap topic. It subscribes to
the published ``/walking_zoo/footstep_plan`` and asserts the planner reacted to
the *map data* (not hand-authored boxes): the foot over the lethal patch is
nudged clear and no placed foot lands in a blocked cell. This exercises the
OccupancyGrid -> TerrainGrid -> FootstepPlanner path through a live ROS node.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import suppress

# Lethal patch in the costmap frame (metres): a box around the first foothold
# (~0.10, +0.16) that leaves higher |y| free so a lateral nudge can succeed.
BLOCK = (0.05, 0.10, 0.20, 0.22)  # min_x, min_y, max_x, max_y

GRID_ORIGIN = (-0.2, -0.6)
GRID_RES = 0.05
GRID_W = 28  # 1.4 m of x
GRID_H = 24  # 1.2 m of y


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


def in_block(x, y):
    return BLOCK[0] <= x <= BLOCK[2] and BLOCK[1] <= y <= BLOCK[3]


def build_costmap(OccupancyGrid):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = GRID_W
    grid.info.height = GRID_H
    grid.info.resolution = GRID_RES
    grid.info.origin.position.x = GRID_ORIGIN[0]
    grid.info.origin.position.y = GRID_ORIGIN[1]
    grid.info.origin.orientation.w = 1.0
    data = []
    for row in range(GRID_H):
        cy = GRID_ORIGIN[1] + (row + 0.5) * GRID_RES
        for col in range(GRID_W):
            cx = GRID_ORIGIN[0] + (col + 0.5) * GRID_RES
            data.append(100 if in_block(cx, cy) else 0)
    grid.data = data
    return grid


def main() -> int:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "46")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    node = None
    rclpy = None
    exit_code = 1
    node_log = tempfile.NamedTemporaryFile(
        mode="w+", prefix="walking_zoo_costmap_e2e_", suffix=".log", delete=False)
    launch = subprocess.Popen(
        [
            "ros2", "run", "walking_zoo_runtime", "footstep_marker_publisher",
            "--ros-args",
            "-p", "step_count:=6",
            "-p", "costmap_topic:=/walking_zoo/terrain_costmap",
        ],
        env=env,
        stdout=node_log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    try:
        import rclpy
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        from nav_msgs.msg import OccupancyGrid
        from walking_zoo_msgs.msg import FootstepPlan

        rclpy.init(args=None)
        node = rclpy.create_node("walking_zoo_costmap_e2e_check")

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        costmap_pub = node.create_publisher(
            OccupancyGrid, "/walking_zoo/terrain_costmap", latched)
        costmap_pub.publish(build_costmap(OccupancyGrid))

        received = []
        node.create_subscription(
            FootstepPlan, "/walking_zoo/footstep_plan",
            lambda msg: received.append(msg), 10)

        # Keep republishing for a moment so a late-joining node still latches it.
        deadline = time.time() + 15.0
        while time.time() < deadline:
            costmap_pub.publish(build_costmap(OccupancyGrid))
            rclpy.spin_once(node, timeout_sec=0.3)
            if received and received[-1].planner_id == "walking_zoo_terrain_planner":
                break

        if not received:
            print("no footstep plan received", file=sys.stderr)
            return 1

        plan = received[-1]
        if plan.planner_id != "walking_zoo_terrain_planner":
            print(f"planner did not consume terrain (planner_id={plan.planner_id})",
                  file=sys.stderr)
            return 1

        feet = [(f.pose.position.x, f.pose.position.y) for f in plan.footsteps]

        inside = [(x, y) for (x, y) in feet if in_block(x, y)]
        if inside:
            print(f"feet still inside the lethal costmap patch: {inside}", file=sys.stderr)
            return 1

        nudged = [(x, y) for (x, y) in feet if abs(x - 0.10) < 1e-6 and abs(y) > 0.22]
        if not nudged:
            print(f"no foot was nudged clear of the costmap patch: {feet}", file=sys.stderr)
            return 1

        print(
            "footstep costmap E2E passed: planner consumed a live OccupancyGrid, "
            f"{len(nudged)} foot nudged clear of the lethal patch, "
            f"0 feet inside the patch")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"footstep costmap E2E error: {error}", file=sys.stderr)
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
