#!/usr/bin/env python3
"""Compare gait_lab controllers *through the real runtime* — the honest benchmark
reproduced on the product path.

`experiments/gait_lab/run_compare.py` rolls every controller out on the bare
MuJoCo harness. This tool does the same comparison through
locomotion_ros2: for each controller it brings up the runtime manager loading
`locomotion_ros2_gait_lab_sil/GaitLabSilAdapter` plus the companion MuJoCo sim
(`gait_lab_sil_sim.py controller:=<name>`), drives one ExecuteVelocity command
through the runtime + safety pipeline, and scores the result from what the
runtime publishes — `/locomotion_ros2/state` (reached WALKING? fell?) and the
sim's genuine base odometry `/gait_lab_sil/odom` (forward distance, min base
height, time-to-fall). A bad gait still actually falls over here; the point is
that it does so *behind the real runtime*, not just in the lab.

It prints an honest table and writes JSON + Markdown evidence, the same way the
runtime showcase records a trace. Controllers whose deps or trained weights are
absent (the sim node refuses to start / never connects) are reported as
``skipped``, not failures.

Requires a Python with rclpy + mujoco (gait_lab's deps), ROS + the workspace
sourced. Point at the gait_lab checkout with LOCOMOTION_ROS2_GAIT_LAB_PATH and a
menagerie G1 with LOCOMOTION_ROS2_MENAGERIE_PATH. The sim subprocess reuses
sys.executable (override with GAIT_LAB_SIL_SIM_PYTHON).

    python3 tools/check_gait_lab_sil_compare.py
    python3 tools/check_gait_lab_sil_compare.py --controllers rl-residual dcm-walk \
        --speed 0.3 --horizon 5 --out /tmp/cmp
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import suppress

REPO = Path(__file__).resolve().parents[1]
SIM_SCRIPT = REPO / "src" / "locomotion_ros2_examples" / "scripts" / "gait_lab_sil_sim.py"

# The honest benchmark set, in roughly increasing sophistication. Mirrors
# gait_lab/run_compare.py; every one is numpy-only at inference except
# zmp-preview (needs scipy) — absent deps are reported as skipped, not failed.
DEFAULT_CONTROLLERS = [
    "stand-hold",
    "open-loop-cpg",
    "balanced-cpg",
    "capture-point",
    "dcm-walk",
    "zmp-preview",
    "rl-residual",
]


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


def run_one(rclpy, controller, env, sim_python, speed, horizon, connect_timeout,
            push_speed=0.0, push_at=0.0, push_dir="forward"):
    """Bring up runtime + sim for one controller, drive a velocity command, and
    return a result dict. ``status`` is ok / FELL / stand / recovered / skipped.

    With ``push_speed > 0`` a base-velocity shove is injected ``push_at`` seconds
    after the command starts, in ``push_dir`` relative to the robot heading
    (forward/back/left/right) — push-recovery benchmarking through the runtime."""
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Vector3
    from locomotion_ros2_msgs.action import ExecuteVelocity
    from locomotion_ros2_msgs.msg import WalkingState
    from nav_msgs.msg import Odometry

    runtime = subprocess.Popen(
        [
            "ros2", "run", "locomotion_ros2_runtime", "locomotion_ros2_runtime_manager",
            "--ros-args",
            "-p", "autostart:=true",
            "-p", "adapter_plugin:=locomotion_ros2_gait_lab_sil/GaitLabSilAdapter",
            "-p", "robot_model:=g1",
            "-p", "robot_family:=humanoid",
            "-p", f"limits.max_linear_x:={max(speed, 0.4)}",
        ],
        env=env,
        preexec_fn=os.setsid,
    )
    sim = subprocess.Popen(
        [sim_python, str(SIM_SCRIPT), "--ros-args", "-p", f"controller:={controller}"],
        env=env,
        preexec_fn=os.setsid,
    )

    node = None
    result = {"controller": controller, "status": "skipped", "walked": False,
              "fell": False, "survival_sec": 0.0, "forward_m": 0.0,
              "min_height_m": None, "note": ""}
    try:
        node = rclpy.create_node("gait_lab_sil_compare")
        state = {"msg": None}
        # Track the planar base pose + heading so "forward" is measured along the
        # robot's body-forward axis (like gait_lab), not world x — the menagerie
        # G1 does not start facing world +x, so a body-forward walk shows up as
        # motion in -x world unless we project onto the initial heading.
        odom = {"x": None, "y": None, "yaw": None, "zmin": None}
        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: state.__setitem__("msg", m), 10)

        def on_odom(m):
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            odom["x"], odom["y"] = p.x, p.y
            odom["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            odom["zmin"] = p.z if odom["zmin"] is None else min(odom["zmin"], p.z)
        node.create_subscription(Odometry, "/gait_lab_sil/odom", on_odom, 10)

        # Wait for the adapter to be active AND connected to the MuJoCo sim.
        deadline = time.time() + connect_timeout
        connected = False
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = state["msg"]
            if (s and s.active_adapter.endswith("GaitLabSilAdapter")
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and s.adapter_connected):
                connected = True
                break
        if not connected:
            result["note"] = "sim never connected (missing deps/weights or load timeout)"
            return result

        client = ActionClient(node, ExecuteVelocity, "/locomotion_ros2/execute_velocity")
        if not client.wait_for_server(timeout_sec=10.0):
            result["note"] = "no execute_velocity server"
            return result

        goal = ExecuteVelocity.Goal()
        goal.command.twist.linear.x = float(speed)
        goal.duration_sec = float(horizon)
        obs = {"walked": False, "fell": False, "t_fell": None}

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=10.0)
        gh = goal_future.result()
        if gh is None or not gh.accepted:
            result["note"] = "velocity goal rejected"
            return result

        start = {"x": odom["x"], "y": odom["y"], "yaw": odom["yaw"]}  # latch start pose
        push_pub = node.create_publisher(Vector3, "/gait_lab_sil/push", 10)
        pushed = push_speed <= 0.0  # nothing to do if no shove requested
        t0 = time.time()
        result_future = gh.get_result_async()
        # Spin through the whole horizon collecting state, then a little slack.
        while time.time() - t0 < horizon + 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time() - t0
            if not pushed and now >= push_at:
                # Shove relative to the robot's heading, in the odom/world frame.
                yaw = start["yaw"] or 0.0
                fwd = (math.cos(yaw), math.sin(yaw))
                left = (-math.sin(yaw), math.cos(yaw))
                vec = {"forward": fwd, "back": (-fwd[0], -fwd[1]),
                       "left": left, "right": (-left[0], -left[1])}[push_dir]
                kick = Vector3(x=push_speed * vec[0], y=push_speed * vec[1], z=0.0)
                push_pub.publish(kick)
                pushed = True
            s = state["msg"]
            if s is not None:
                if s.locomotion_state == WalkingState.STATE_WALKING:
                    obs["walked"] = True
                if s.is_fallen and obs["t_fell"] is None:
                    obs["fell"] = True
                    obs["t_fell"] = now
            if result_future.done():
                break

        forward = 0.0
        if start["x"] is not None and odom["x"] is not None:
            dx, dy = odom["x"] - start["x"], odom["y"] - start["y"]
            yaw = start["yaw"] or 0.0
            forward = dx * math.cos(yaw) + dy * math.sin(yaw)  # body-forward axis
        survival = obs["t_fell"] if obs["fell"] else float(horizon)
        if obs["fell"]:
            status = "FELL"
        elif push_speed > 0.0:
            status = "recovered"  # took a shove and still reached the horizon
        elif obs["walked"] and abs(forward) > 0.03:
            status = "ok"
        else:
            status = "stand"
        result.update({
            "status": status, "walked": obs["walked"], "fell": obs["fell"],
            "survival_sec": round(survival, 2), "forward_m": round(forward, 3),
            "min_height_m": None if odom["zmin"] is None else round(odom["zmin"], 3),
            "push_speed": push_speed, "push_dir": push_dir if push_speed > 0 else "",
        })
        return result
    finally:
        if node is not None:
            node.destroy_node()
        terminate_process_group(sim)
        terminate_process_group(runtime)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controllers", nargs="+", default=DEFAULT_CONTROLLERS,
                    help="gait_lab controllers to compare through the runtime.")
    ap.add_argument("--speed", type=float, default=0.3, help="forward m/s command.")
    ap.add_argument("--horizon", type=float, default=5.0, help="command seconds.")
    ap.add_argument("--connect-timeout", type=float, default=45.0,
                    help="seconds to wait for the sim to load + connect.")
    ap.add_argument("--push-speed", type=float, default=0.0,
                    help="mid-walk base-velocity shove (m/s); 0 = no shove.")
    ap.add_argument("--push-at", type=float, default=1.0,
                    help="seconds into the command to apply the shove.")
    ap.add_argument("--push-dir", default="forward",
                    choices=["forward", "back", "left", "right"],
                    help="shove direction relative to the robot heading.")
    ap.add_argument("--domain-base", type=int, default=57,
                    help="ROS_DOMAIN_ID base; each controller uses base+index.")
    ap.add_argument("--out", default="/tmp/locomotion_ros2_gait_lab_sil_compare",
                    help="output dir for compare.json / compare.md.")
    args = ap.parse_args()

    base_env = os.environ.copy()
    base_env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    base_env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH",
                        str(REPO / "experiments" / "gait_lab"))
    base_env.setdefault("MUJOCO_GL", "egl")
    sim_python = base_env.get("GAIT_LAB_SIL_SIM_PYTHON", sys.executable)

    import rclpy

    rows = []
    for i, controller in enumerate(args.controllers):
        env = base_env.copy()
        # Fresh domain per run so stale DDS discovery from the prior run cannot
        # leak in (domain 0 is congested on some hosts; isolate each rollout).
        env["ROS_DOMAIN_ID"] = str(args.domain_base + i)
        os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
        os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
        print(f"[{i+1}/{len(args.controllers)}] {controller}: bringing up runtime + sim "
              f"(domain {env['ROS_DOMAIN_ID']})...", flush=True)
        rclpy.init(args=None)
        try:
            row = run_one(rclpy, controller, env, sim_python, args.speed,
                          args.horizon, args.connect_timeout,
                          push_speed=args.push_speed, push_at=args.push_at,
                          push_dir=args.push_dir)
        except Exception as error:  # noqa: BLE001
            row = {"controller": controller, "status": "skipped",
                   "note": f"error: {error}", "walked": False, "fell": False,
                   "survival_sec": 0.0, "forward_m": 0.0, "min_height_m": None}
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        rows.append(row)
        print(f"    -> {row['status']:6s} survive={row['survival_sec']:.2f}s "
              f"fwd={row['forward_m']:+.3f}m minH="
              f"{row['min_height_m'] if row['min_height_m'] is not None else '?'} "
              f"{row['note']}", flush=True)
        time.sleep(2.0)  # let DDS/ports settle before the next domain

    push_desc = (f", {args.push_speed} m/s {args.push_dir} shove @ {args.push_at}s"
                 if args.push_speed > 0 else "")
    # Honest table.
    print()
    print("gait_lab controllers through the locomotion_ros2 runtime "
          f"(cmd {args.speed} m/s, horizon {args.horizon}s{push_desc}):")
    print(f"{'controller':<16}{'forward':>10}{'survival':>11}{'minH':>8}  status")
    for r in rows:
        mh = "    ?" if r["min_height_m"] is None else f"{r['min_height_m']:.2f}m"
        print(f"{r['controller']:<16}{r['forward_m']:>+9.3f}m"
              f"{r['survival_sec']:>9.2f}s {mh:>8}  [{r['status']}]")

    scored = [r for r in rows if r["status"] in ("ok", "FELL", "stand")]
    if scored:
        farthest = max(scored, key=lambda r: r["forward_m"])
        survivors = [r for r in scored if r["status"] in ("ok", "stand")]
        walks = [r for r in survivors if r["walked"] and abs(r["forward_m"]) > 0.03]
        print(f"\nfarthest through the runtime: {farthest['controller']} "
              f"({farthest['forward_m']:+.3f} m)")
        if walks:
            best = max(walks, key=lambda r: r["forward_m"])
            print(f"survives & walks via the runtime: {best['controller']} "
                  f"({best['forward_m']:+.3f} m over {best['survival_sec']:.2f}s)")
        else:
            print("survives & walks via the runtime: none in this set")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = {"schema": "locomotion_ros2.gait_lab_sil_compare.v1",
            "command_speed_mps": args.speed, "horizon_sec": args.horizon,
            "push_speed_mps": args.push_speed, "push_dir": args.push_dir,
            "push_at_sec": args.push_at,
            "path": "cmd_vel -> runtime -> safety -> GaitLabSilAdapter -> MuJoCo G1",
            "results": rows}
    (out_dir / "compare.json").write_text(json.dumps(meta, indent=2))
    md = ["# gait_lab controllers through the locomotion_ros2 runtime", "",
          f"Command `{args.speed} m/s` forward, horizon `{args.horizon}s`{push_desc}, "
          "scored from `/locomotion_ros2/state` and the sim's base odometry — every "
          "gait driven through the real runtime + safety pipeline.", "",
          "| controller | forward | survival | min height | status |",
          "|---|---|---|---|---|"]
    for r in rows:
        mh = "?" if r["min_height_m"] is None else f"{r['min_height_m']:.2f} m"
        md.append(f"| `{r['controller']}` | {r['forward_m']:+.3f} m | "
                  f"{r['survival_sec']:.2f} s | {mh} | {r['status']} |")
    (out_dir / "compare.md").write_text("\n".join(md) + "\n")
    print(f"\nwrote {out_dir/'compare.json'} and {out_dir/'compare.md'}")

    # Exit non-zero only if nothing scored at all (environment unusable).
    return 0 if scored else 1


if __name__ == "__main__":
    raise SystemExit(main())
