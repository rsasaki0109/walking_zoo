#!/usr/bin/env python3
"""Capture multiple LeRobot episodes from live locomotion_ros2 showcase runs.

This drives the real mock runtime (the same launch the showcase uses) through a
sequence of distinct command episodes, records each one with the live
``locomotion_ros2_demo_recorder``, and exports all episodes into a single LeRobot
v2.1 dataset via ``locomotion_ros2_lerobot_export``. Unlike the synthetic exporter
checks, the traces here come from real ROS topics flowing through the cmd_vel
bridge, runtime, safety pipeline, and adapter.

Each episode publishes a distinct semantic action (so the dataset gets multiple
task labels) and a matching ``/cmd_vel`` command stream. One runtime stays up for
the whole run; a fresh recorder process records each episode in turn.

Multi-process DDS needs an uncongested domain in this sandbox, so the default
``ROS_DOMAIN_ID`` is set away from 0.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import suppress

_REPO = Path(__file__).resolve().parent.parent
_EXPORT_PATH = (
    _REPO / "src" / "locomotion_ros2_examples" / "scripts" / "locomotion_ros2_lerobot_export.py")

# (semantic action, twist linear_x, linear_y, angular_z) per episode. The list
# cycles if more episodes are requested than entries.
EPISODE_SCRIPTS = [
    ("walk_forward", 0.30, 0.0, 0.0),
    ("turn_left", 0.0, 0.0, 0.40),
    ("sidestep_left", 0.0, 0.25, 0.0),
    ("run_forward", 0.50, 0.0, 0.0),
]


def load_exporter():
    spec = importlib.util.spec_from_file_location("locomotion_ros2_lerobot_export", _EXPORT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def wait_until_ready(rclpy, node, WalkingState, timeout):
    latest = {}
    sub = node.create_subscription(
        WalkingState, "/locomotion_ros2/state", lambda m: latest.__setitem__("state", m), 10)
    deadline = time.time() + timeout
    ready = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        state = latest.get("state")
        if state is not None and state.adapter_connected and state.locomotion_state in (
                WalkingState.STATE_STANDING, WalkingState.STATE_IDLE):
            ready = True
            break
    node.destroy_subscription(sub)
    return ready


def record_episode(env, rclpy, node, Twist, SemanticAction, out_dir, script, duration):
    action, lx, ly, az = script
    recorder = subprocess.Popen(
        ["ros2", "run", "locomotion_ros2_examples", "locomotion_ros2_demo_recorder.py",
         "--ros-args",
         "-p", f"output_dir:={out_dir}",
         "-p", "write_period_sec:=0.3"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    action_pub = node.create_publisher(SemanticAction, "/locomotion_ros2/semantic_action", 10)

    # Let the recorder's subscriptions match before driving.
    warmup = time.time() + 2.0
    while time.time() < warmup:
        rclpy.spin_once(node, timeout_sec=0.1)

    semantic = SemanticAction()
    semantic.source = "capture"
    semantic.action = action
    semantic.confidence = 1.0
    for _ in range(3):
        action_pub.publish(semantic)
        rclpy.spin_once(node, timeout_sec=0.05)

    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.angular.z = az
    drive_deadline = time.time() + duration
    while time.time() < drive_deadline:
        cmd_pub.publish(twist)
        action_pub.publish(semantic)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    # Settle back to zero so the trace ends cleanly.
    stop = Twist()
    settle = time.time() + 0.6
    while time.time() < settle:
        cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    node.destroy_publisher(cmd_pub)
    node.destroy_publisher(action_pub)
    terminate_process_group(recorder)

    trace_path = Path(out_dir) / "demo_trace.json"
    if not trace_path.is_file():
        raise RuntimeError(f"recorder produced no trace at {trace_path}")
    return json.loads(trace_path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=3, help="number of episodes to capture")
    parser.add_argument("--out", type=Path, required=True, help="output LeRobot dataset dir")
    parser.add_argument("--fps", type=float, default=10.0, help="resampling rate (default 10)")
    parser.add_argument(
        "--episode-duration", type=float, default=3.0, help="seconds of driving per episode")
    parser.add_argument("--domain", type=int, default=57, help="ROS_DOMAIN_ID to use")
    args = parser.parse_args()

    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    env["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["RMW_IMPLEMENTATION"] = env["RMW_IMPLEMENTATION"]
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    exporter = load_exporter()

    node = None
    rclpy = None
    exit_code = 1
    runtime = subprocess.Popen(
        ["ros2", "launch", "locomotion_ros2_bringup", "mock_runtime.launch.py"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

    work_dir = tempfile.mkdtemp(prefix="locomotion_ros2_capture_")
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from locomotion_ros2_msgs.msg import SemanticAction, WalkingState

        rclpy.init(args=None)
        node = rclpy.create_node("locomotion_ros2_lerobot_capture")

        if not wait_until_ready(rclpy, node, WalkingState, timeout=25.0):
            print("runtime never became ready", file=sys.stderr)
            return 1
        print("runtime ready; capturing episodes")

        traces = []
        for episode in range(args.episodes):
            script = EPISODE_SCRIPTS[episode % len(EPISODE_SCRIPTS)]
            out_dir = Path(work_dir) / f"ep_{episode:03d}"
            out_dir.mkdir(parents=True, exist_ok=True)
            trace = record_episode(
                env, rclpy, node, Twist, SemanticAction, out_dir, script, args.episode_duration)
            n_events = len(trace.get("events", []))
            if n_events == 0 or float(trace.get("duration_sec", 0.0)) <= 0.0:
                print(f"episode {episode} produced an empty trace", file=sys.stderr)
                return 1
            print(f"  episode {episode}: action={script[0]} events={n_events} "
                  f"duration={trace['duration_sec']:.2f}s")
            traces.append(trace)

        summary = exporter.write_episodes_dataset(traces, args.out, fps=args.fps)
        # Light structural sanity on the produced dataset.
        info = json.loads((Path(args.out) / "meta" / "info.json").read_text())
        if info["total_episodes"] != args.episodes:
            print(f"dataset total_episodes {info['total_episodes']} != {args.episodes}",
                  file=sys.stderr)
            return 1

        print(f"captured LeRobot dataset: {summary['episodes']} live episode(s), "
              f"{summary['frames']} frames @ {summary['fps']} fps "
              f"({summary['data_format']}) -> {summary['out_dir']}")
        print(f"tasks: {', '.join(summary['tasks'])}")
        print("Confirm HuggingFace compatibility with: "
              "python3 tools/check_lerobot_hf_load.py")
        exit_code = 0
    except Exception as error:  # noqa: BLE001
        print(f"capture error: {error}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        terminate_process_group(runtime)
        with suppress(OSError):
            for child in Path(work_dir).rglob("*"):
                with suppress(OSError):
                    child.unlink()
            Path(work_dir).rmdir()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
