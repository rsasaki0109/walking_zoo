#!/usr/bin/env python3
"""Drive the live ROS gait_lab SIL robot through a command sequence for the
filmstrip hero asset.

Unlike ``experiments/gait_lab/render_rl_walk.py`` (which renders a gait in the
offline harness), this captures the robot *as actually driven through ROS*: it
brings up the runtime + safety pipeline + the steerable SIL sim (with frame
capture on), then publishes a ``/cmd_vel`` sequence — walk straight, arc left,
straight, arc right — exactly as a teleop or Nav2 controller would. The sim node
renders each control tick and writes a rolling ``filmstrip.png`` of the recent
motion, so the resulting asset shows the *runtime-driven* robot turning, not a
harness rollout.

    # from the workspace, ROS + workspace sourced, interpreter with rclpy+mujoco
    python3 tools/capture_sil_live.py --out docs/assets/readme/sil_live_steer.png

Needs an interpreter with both rclpy and mujoco. Set GAIT_LAB_SIL_SIM_PYTHON if
the sim needs a different interpreter than this one.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from contextlib import suppress
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SIM_SCRIPT = REPO / "src" / "locomotion_ros2_examples" / "scripts" / "gait_lab_sil_sim.py"


def terminate(process):
    if process is None or process.poll() is not None:
        return
    with suppress(ProcessLookupError):
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=6.0)
    except subprocess.TimeoutExpired:
        with suppress(ProcessLookupError):
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


# (linear.x, angular.z, seconds) — the trajectory the filmstrip should show.
SEQUENCE = [
    (0.18, 0.0, 4.0),    # walk straight
    (0.12, 0.4, 4.0),    # arc left
    (0.18, 0.0, 3.0),    # straight again
    (0.12, -0.4, 4.0),   # arc right
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "docs/assets/readme/sil_live_steer.png"))
    ap.add_argument("--controller", default="rl-steerable")
    ap.add_argument("--frames-dir", default="/tmp/sil_live_frames")
    args = ap.parse_args()

    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env.setdefault("ROS_DOMAIN_ID", "64")
    env.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(REPO / "experiments" / "gait_lab"))
    env.setdefault("MUJOCO_GL", "egl")
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    sim_python = env.get("GAIT_LAB_SIL_SIM_PYTHON", sys.executable)

    frames_dir = args.frames_dir
    if os.path.isdir(frames_dir):
        shutil.rmtree(frames_dir, ignore_errors=True)

    procs = []
    procs.append(subprocess.Popen(
        ["ros2", "run", "locomotion_ros2_runtime", "locomotion_ros2_runtime_manager",
         "--ros-args", "-p", "autostart:=true",
         "-p", "adapter_plugin:=locomotion_ros2_gait_lab_sil/GaitLabSilAdapter",
         "-p", "robot_model:=g1", "-p", "robot_family:=humanoid",
         "-p", "limits.max_linear_x:=0.4", "-p", "limits.max_angular_z:=0.5"],
        env=env, preexec_fn=os.setsid))
    procs.append(subprocess.Popen(
        [sim_python, str(SIM_SCRIPT), "--ros-args",
         "-p", f"controller:={args.controller}",
         "-p", "render:=true", "-p", f"save_frames_dir:={frames_dir}",
         "-p", "frame_stride:=2"],
        env=env, preexec_fn=os.setsid))
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
        node = rclpy.create_node("sil_live_capture")
        latest = {}
        node.create_subscription(
            WalkingState, "/locomotion_ros2/state",
            lambda m: latest.__setitem__("state", m), 10)
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        deadline = time.time() + 60.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            s = latest.get("state")
            if (s and s.active_adapter.endswith("GaitLabSilAdapter")
                    and s.lifecycle_state == WalkingState.LIFECYCLE_ACTIVE
                    and s.adapter_connected):
                break
        if not (latest.get("state") and latest["state"].adapter_connected):
            print("SIL adapter/sim not ready", file=sys.stderr)
            return 1
        print("SIL active; driving the command sequence (straight / arc / straight / arc)")

        fell = False
        for vx, wz, secs in SEQUENCE:
            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            end = time.time() + secs
            while time.time() < end:
                cmd_pub.publish(twist)
                rclpy.spin_once(node, timeout_sec=0.05)
                st = latest.get("state")
                if st and st.is_fallen:
                    fell = True
            print(f"  drove vx={vx} wz={wz} for {secs}s  fell={fell}")
        # Let the sim flush a final filmstrip.
        time.sleep(1.0)

        strip = Path(frames_dir) / "filmstrip.png"
        if strip.exists():
            os.makedirs(Path(args.out).parent, exist_ok=True)
            shutil.copyfile(strip, args.out)
            print(f"wrote live filmstrip -> {args.out}  (fell={fell})")
            exit_code = 0 if not fell else 2
        else:
            print(f"no filmstrip at {strip} (render/save may be unavailable)",
                  file=sys.stderr)
    except Exception as error:  # noqa: BLE001
        print(f"capture error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        for p in reversed(procs):
            terminate(p)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
