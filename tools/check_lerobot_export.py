#!/usr/bin/env python3
"""Validate the walking_zoo -> LeRobot dataset exporter end to end.

Runs the exporter on a synthetic runtime trace and asserts the produced dataset
has the LeRobot layout, a consistent frame count, and a loadable episode file.
No ROS or DDS required, so this is safe to run in CI.
"""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile

_REPO = Path(__file__).resolve().parent.parent
_MODULE_PATH = (
    _REPO / "src" / "walking_zoo_examples" / "scripts" / "walking_zoo_lerobot_export.py")


def load_exporter():
    spec = importlib.util.spec_from_file_location("walking_zoo_lerobot_export", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def synthetic_trace():
    return {
        "schema": "walking_zoo.demo_trace.v1",
        "generated_by": "walking_zoo_demo_recorder",
        "duration_sec": 3.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.5, "topic": "/cmd_vel", "summary": "",
             "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.2}},
            {"t_sec": 0.5, "topic": "/walking_zoo/cmd_vel", "summary": "",
             "data": {"linear_x": 0.18, "linear_y": 0.0, "angular_z": 0.2}},
            {"t_sec": 0.6, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "WALKING", "mode": 3, "estop_active": False}},
            {"t_sec": 2.0, "topic": "/walking_zoo/semantic_action", "summary": "",
             "data": {"action": "walk_forward"}},
            {"t_sec": 2.5, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "ESTOPPED", "mode": 1, "estop_active": True}},
        ],
    }


def main() -> int:
    exporter = load_exporter()
    fps = 10.0
    expected_frames = int(round(3.0 * fps)) + 1

    with tempfile.TemporaryDirectory(prefix="walking_zoo_lerobot_") as tmp:
        out = Path(tmp) / "dataset"
        summary = exporter.write_dataset(synthetic_trace(), out, fps=fps)

        if summary["frames"] != expected_frames:
            print(f"frame count {summary['frames']} != {expected_frames}", file=sys.stderr)
            return 1

        for rel in ("meta/info.json", "meta/tasks.jsonl", "meta/episodes.jsonl", "meta/stats.json"):
            if not (out / rel).is_file():
                print(f"missing dataset file: {rel}", file=sys.stderr)
                return 1

        info = json.loads((out / "meta" / "info.json").read_text())
        if info["features"]["observation.state"]["shape"] != [6]:
            print("observation.state shape mismatch", file=sys.stderr)
            return 1
        if info["features"]["action"]["shape"] != [3]:
            print("action shape mismatch", file=sys.stderr)
            return 1
        if info["total_frames"] != expected_frames:
            print("info total_frames mismatch", file=sys.stderr)
            return 1

        episodes = list((out / "data" / "chunk-000").glob("episode_000000.*"))
        if len(episodes) != 1:
            print(f"expected one episode file, found {episodes}", file=sys.stderr)
            return 1
        episode = episodes[0]

        if episode.suffix == ".parquet":
            import pandas as pd

            frame = pd.read_parquet(episode)
            if len(frame) != expected_frames:
                print("parquet row count mismatch", file=sys.stderr)
                return 1
            required = {"observation.state", "action", "timestamp", "next.done"}
            if not required.issubset(frame.columns):
                print(f"parquet missing columns: {required - set(frame.columns)}", file=sys.stderr)
                return 1
            if not bool(frame["next.done"].iloc[-1]):
                print("last frame next.done not set", file=sys.stderr)
                return 1
            print(f"lerobot export check passed: {len(frame)} frames in parquet, "
                  f"task={summary['task']!r}")
        else:
            rows = episode.read_text().strip().splitlines()
            if len(rows) != expected_frames:
                print("jsonl row count mismatch", file=sys.stderr)
                return 1
            print(f"lerobot export check passed: {len(rows)} frames in jsonl, "
                  f"task={summary['task']!r}")

        # Multi-episode round trip: two traces -> two episodes, aggregated meta.
        multi_out = Path(tmp) / "multi"
        multi = exporter.write_episodes_dataset(
            [synthetic_trace(), synthetic_trace()], multi_out, fps=fps)
        if multi["episodes"] != 2:
            print(f"multi episode count {multi['episodes']} != 2", file=sys.stderr)
            return 1
        if multi["frames"] != 2 * expected_frames:
            print(f"multi frame count {multi['frames']} != {2 * expected_frames}",
                  file=sys.stderr)
            return 1
        multi_info = json.loads((multi_out / "meta" / "info.json").read_text())
        if multi_info["total_episodes"] != 2 or multi_info["splits"]["train"] != "0:2":
            print("multi info episodes/splits mismatch", file=sys.stderr)
            return 1
        chunk = multi_out / "data" / "chunk-000"
        for ep in ("episode_000000", "episode_000001"):
            if not list(chunk.glob(f"{ep}.*")):
                print(f"missing multi episode file: {ep}", file=sys.stderr)
                return 1
        print(f"lerobot multi-episode check passed: {multi['episodes']} episodes, "
              f"{multi['frames']} frames, {len(multi['tasks'])} task(s)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
