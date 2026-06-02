#!/usr/bin/env python3
"""Export a walking_zoo runtime trace to a LeRobot-format dataset.

This converts a ``walking_zoo.demo_trace.v1`` JSON trace (produced by
``walking_zoo_demo_recorder.py``) into a `LeRobot
<https://github.com/huggingface/lerobot>`_ dataset directory so walking_zoo
runs can feed imitation-learning pipelines.

The change-triggered event trace is resampled to a fixed-rate frame timeline.
Each frame pairs the *action* (the Nav2/teleop velocity command published on
``/cmd_vel``) with the *observation* (the runtime's executed velocity and
locomotion state). The output layout mirrors LeRobot v2.1:

    <out>/meta/info.json
    <out>/meta/tasks.jsonl
    <out>/meta/episodes.jsonl
    <out>/meta/stats.json
    <out>/data/chunk-000/episode_000000.parquet   (.jsonl fallback if no pyarrow)

The trace -> dataset logic is pure Python (no ROS) so it is unit tested.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

LEROBOT_CODEBASE_VERSION = "v2.1"
CHUNK_SIZE = 1000

# Inverse of the recorder's WalkingState id->name map, so we can turn the trace's
# state strings back into the stable numeric ids LeRobot stores.
STATE_NAME_TO_ID = {
    "UNKNOWN": 0,
    "IDLE": 1,
    "STANDING": 2,
    "WALKING": 3,
    "TURNING": 4,
    "BODY_POSE_CONTROL": 5,
    "EXECUTING_FOOTSTEPS": 6,
    "STOPPING": 7,
    "SITTING": 8,
    "FALLEN": 9,
    "FAULT": 10,
    "ESTOPPED": 11,
}

OBSERVATION_NAMES = [
    "exec_linear_x",
    "exec_linear_y",
    "exec_angular_z",
    "locomotion_state",
    "locomotion_mode",
    "estop_active",
]
ACTION_NAMES = ["cmd_linear_x", "cmd_linear_y", "cmd_angular_z"]


class TraceFormatError(ValueError):
    """Raised when the input trace is not a recognised walking_zoo trace."""


def load_trace(path: Path) -> dict:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    schema = payload.get("schema", "")
    if not schema.startswith("walking_zoo.demo_trace"):
        raise TraceFormatError(f"unexpected trace schema: {schema!r}")
    return payload


def _running_state():
    return {
        "exec": [0.0, 0.0, 0.0],   # executed velocity (observation)
        "cmd": [0.0, 0.0, 0.0],    # commanded velocity (action)
        "loc_state": 0.0,
        "loc_mode": 0.0,
        "estop": 0.0,
    }


def _apply_event(state: dict, event: dict) -> None:
    topic = event.get("topic", "")
    data = event.get("data", {}) or {}
    if topic == "/cmd_vel":
        state["cmd"] = [
            float(data.get("linear_x", 0.0)),
            float(data.get("linear_y", 0.0)),
            float(data.get("angular_z", 0.0)),
        ]
    elif topic == "/walking_zoo/cmd_vel":
        state["exec"] = [
            float(data.get("linear_x", 0.0)),
            float(data.get("linear_y", 0.0)),
            float(data.get("angular_z", 0.0)),
        ]
    elif topic == "/walking_zoo/state":
        name = str(data.get("state", "UNKNOWN"))
        state["loc_state"] = float(STATE_NAME_TO_ID.get(name, 0))
        state["loc_mode"] = float(data.get("mode", 0) or 0)
        state["estop"] = 1.0 if data.get("estop_active") else 0.0


def _observation(state: dict) -> list:
    return [
        state["exec"][0], state["exec"][1], state["exec"][2],
        state["loc_state"], state["loc_mode"], state["estop"],
    ]


def derive_task(trace: dict) -> str:
    """Pick a human-readable task label from semantic actions, if present."""
    counts: dict[str, int] = {}
    for event in trace.get("events", []):
        if event.get("topic") == "/walking_zoo/semantic_action":
            action = str((event.get("data") or {}).get("action", "")).strip()
            if action:
                counts[action] = counts.get(action, 0) + 1
    if counts:
        top = max(counts, key=counts.get)
        return f"walking_zoo semantic action: {top}"
    return "walking_zoo teleop/nav velocity control"


def build_frames(trace: dict, fps: float) -> list:
    """Resample the event trace into fixed-rate (action, observation) frames."""
    if fps <= 0:
        raise ValueError("fps must be positive")
    events = sorted(
        (e for e in trace.get("events", []) if "t_sec" in e),
        key=lambda e: float(e["t_sec"]),
    )
    duration = float(trace.get("duration_sec", 0.0))
    if events:
        duration = max(duration, float(events[-1]["t_sec"]))
    if duration <= 0.0:
        raise TraceFormatError("trace has no positive duration; nothing to export")

    dt = 1.0 / fps
    state = _running_state()
    frames = []
    cursor = 0
    n_ticks = int(round(duration * fps)) + 1
    for frame_index in range(n_ticks):
        t = frame_index * dt
        while cursor < len(events) and float(events[cursor]["t_sec"]) <= t + 1e-9:
            _apply_event(state, events[cursor])
            cursor += 1
        frames.append({
            "observation.state": _observation(state),
            "action": list(state["cmd"]),
            "timestamp": round(t, 6),
            "frame_index": frame_index,
            "episode_index": 0,
            "index": frame_index,
            "task_index": 0,
            "next.done": frame_index == n_ticks - 1,
        })
    return frames


def compute_stats(frames: list) -> dict:
    """Per-feature min/max/mean/std/count over vector and scalar features."""
    def stats_for(vectors: list) -> dict:
        if not vectors:
            return {"min": [], "max": [], "mean": [], "std": [], "count": [0]}
        dim = len(vectors[0])
        n = len(vectors)
        mins = [float("inf")] * dim
        maxs = [float("-inf")] * dim
        sums = [0.0] * dim
        sqsums = [0.0] * dim
        for vec in vectors:
            for i, v in enumerate(vec):
                v = float(v)
                mins[i] = min(mins[i], v)
                maxs[i] = max(maxs[i], v)
                sums[i] += v
                sqsums[i] += v * v
        mean = [s / n for s in sums]
        std = [max(0.0, (sqsums[i] / n) - mean[i] * mean[i]) ** 0.5 for i in range(dim)]
        return {"min": mins, "max": maxs, "mean": mean, "std": std, "count": [n]}

    out = {
        "observation.state": stats_for([f["observation.state"] for f in frames]),
        "action": stats_for([f["action"] for f in frames]),
    }
    for scalar in ("timestamp", "frame_index", "episode_index", "index", "task_index"):
        out[scalar] = stats_for([[float(f[scalar])] for f in frames])
    return out


def _features_schema() -> dict:
    return {
        "observation.state": {
            "dtype": "float32", "shape": [len(OBSERVATION_NAMES)], "names": OBSERVATION_NAMES},
        "action": {"dtype": "float32", "shape": [len(ACTION_NAMES)], "names": ACTION_NAMES},
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
        "next.done": {"dtype": "bool", "shape": [1], "names": None},
    }


def _write_jsonl(path: Path, rows: list) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def write_dataset(trace: dict, out_dir: Path, fps: float = 10.0) -> dict:
    """Write a LeRobot dataset for `trace` under `out_dir`; return a summary."""
    out_dir = Path(out_dir)
    frames = build_frames(trace, fps)
    task = derive_task(trace)
    robot_type = "walking_zoo"
    latest_state = (trace.get("latest") or {}).get("walking_state") or {}
    if latest_state.get("robot"):
        robot_type = str(latest_state["robot"])

    meta_dir = out_dir / "meta"
    data_dir = out_dir / "data" / "chunk-000"
    meta_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    data_format, data_path_template = _write_episode(data_dir, frames)

    info = {
        "codebase_version": LEROBOT_CODEBASE_VERSION,
        "robot_type": robot_type,
        "total_episodes": 1,
        "total_frames": len(frames),
        "total_tasks": 1,
        "total_videos": 0,
        "total_chunks": 1,
        "chunks_size": CHUNK_SIZE,
        "fps": fps,
        "splits": {"train": "0:1"},
        "data_path": data_path_template,
        "data_format": data_format,
        "video_path": None,
        "features": _features_schema(),
        "source": {
            "schema": trace.get("schema"),
            "generated_by": trace.get("generated_by"),
            "duration_sec": trace.get("duration_sec"),
        },
    }
    (meta_dir / "info.json").write_text(
        json.dumps(info, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_jsonl(meta_dir / "tasks.jsonl", [{"task_index": 0, "task": task}])
    _write_jsonl(
        meta_dir / "episodes.jsonl",
        [{"episode_index": 0, "tasks": [task], "length": len(frames)}])
    (meta_dir / "stats.json").write_text(
        json.dumps(compute_stats(frames), indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return {
        "out_dir": str(out_dir),
        "frames": len(frames),
        "fps": fps,
        "task": task,
        "data_format": data_format,
    }


def _write_episode(data_dir: Path, frames: list) -> tuple:
    """Write episode 0; return (data_format, info data_path template)."""
    try:
        import pandas as pd  # noqa: WPS433 (optional dependency)

        frame = pd.DataFrame(frames)
        frame.to_parquet(data_dir / "episode_000000.parquet", index=False)
        return "parquet", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
    except Exception:  # noqa: BLE001 - fall back to a dependency-free format
        _write_jsonl(data_dir / "episode_000000.jsonl", frames)
        return "jsonl", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.jsonl"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="path to a walking_zoo demo_trace.json")
    parser.add_argument(
        "--out", type=Path, required=True, help="output LeRobot dataset directory")
    parser.add_argument("--fps", type=float, default=10.0, help="resampling rate (default 10)")
    args = parser.parse_args()

    trace = load_trace(args.trace)
    summary = write_dataset(trace, args.out, fps=args.fps)
    print(
        f"wrote LeRobot dataset: {summary['frames']} frames @ {summary['fps']} fps "
        f"({summary['data_format']}) -> {summary['out_dir']}")
    print(f"task: {summary['task']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
