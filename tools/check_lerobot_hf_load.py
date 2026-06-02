#!/usr/bin/env python3
"""Confirm a walking_zoo LeRobot export loads with HuggingFace ``datasets``.

The exporter writes a LeRobot v2.1 layout; this check proves that layout is
actually consumable by the HuggingFace ``datasets.load_dataset`` reader (the
common entry point for LeRobot datasets, and the one that does not require the
full ``lerobot`` package). It builds a small multi-episode dataset from synthetic
traces, then:

  - loads the parquet episode files via ``load_dataset("parquet", ...)`` and
    checks the row count, columns, and feature shapes against ``meta/info.json``;
  - loads ``meta/tasks.jsonl`` and ``meta/episodes.jsonl`` via
    ``load_dataset("json", ...)`` and checks their row counts.

If ``datasets`` (or pyarrow, so the export is parquet) is unavailable the check
prints SKIP and succeeds, so it stays CI-safe.
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


def walk_trace():
    return {
        "schema": "walking_zoo.demo_trace.v1",
        "generated_by": "walking_zoo_demo_recorder",
        "duration_sec": 2.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.5, "topic": "/cmd_vel", "summary": "",
             "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.5, "topic": "/walking_zoo/cmd_vel", "summary": "",
             "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.6, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "WALKING", "mode": 3, "estop_active": False}},
            {"t_sec": 1.5, "topic": "/walking_zoo/semantic_action", "summary": "",
             "data": {"action": "walk_forward"}},
        ],
    }


def turn_trace():
    return {
        "schema": "walking_zoo.demo_trace.v1",
        "generated_by": "walking_zoo_demo_recorder",
        "duration_sec": 1.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/walking_zoo/state", "summary": "",
             "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.3, "topic": "/cmd_vel", "summary": "",
             "data": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.4}},
            {"t_sec": 0.4, "topic": "/walking_zoo/semantic_action", "summary": "",
             "data": {"action": "turn_left"}},
        ],
    }


def main() -> int:
    if importlib.util.find_spec("datasets") is None:
        print("SKIP: HuggingFace 'datasets' not installed; cannot check load_dataset compat")
        return 0
    if importlib.util.find_spec("pyarrow") is None:
        print("SKIP: 'pyarrow' not installed; export would be jsonl, not LeRobot parquet")
        return 0

    from datasets import load_dataset

    exporter = load_exporter()
    fps = 10.0

    with tempfile.TemporaryDirectory(prefix="walking_zoo_hf_") as tmp:
        out = Path(tmp) / "dataset"
        summary = exporter.write_episodes_dataset([walk_trace(), turn_trace()], out, fps=fps)
        if summary["data_format"] != "parquet":
            print(f"SKIP: exporter wrote {summary['data_format']}, not parquet", file=sys.stderr)
            return 0

        info = json.loads((out / "meta" / "info.json").read_text())

        episode_files = sorted(str(p) for p in (out / "data").rglob("episode_*.parquet"))
        if len(episode_files) != info["total_episodes"]:
            print(f"episode file count {len(episode_files)} != {info['total_episodes']}",
                  file=sys.stderr)
            return 1

        # The core compatibility claim: HuggingFace datasets reads the episodes.
        ds = load_dataset("parquet", data_files=episode_files, split="train")

        if ds.num_rows != info["total_frames"]:
            print(f"loaded {ds.num_rows} rows != info total_frames {info['total_frames']}",
                  file=sys.stderr)
            return 1

        expected_columns = set(info["features"].keys())
        got_columns = set(ds.column_names)
        if got_columns != expected_columns:
            print(f"column mismatch: missing={expected_columns - got_columns} "
                  f"extra={got_columns - expected_columns}", file=sys.stderr)
            return 1

        row = ds[0]
        if len(row["observation.state"]) != len(info["features"]["observation.state"]["names"]):
            print("observation.state width mismatch on load", file=sys.stderr)
            return 1
        if len(row["action"]) != len(info["features"]["action"]["names"]):
            print("action width mismatch on load", file=sys.stderr)
            return 1
        if not isinstance(ds[-1]["next.done"], bool):
            print("next.done did not round-trip as bool", file=sys.stderr)
            return 1
        # episode_index column distinguishes the two episodes.
        if set(ds.unique("episode_index")) != set(range(info["total_episodes"])):
            print("episode_index values do not cover all episodes", file=sys.stderr)
            return 1

        # The metadata tables are HF-loadable too.
        tasks = load_dataset(
            "json", data_files=str(out / "meta" / "tasks.jsonl"), split="train")
        if tasks.num_rows != info["total_tasks"]:
            print(f"tasks rows {tasks.num_rows} != total_tasks {info['total_tasks']}",
                  file=sys.stderr)
            return 1
        episodes = load_dataset(
            "json", data_files=str(out / "meta" / "episodes.jsonl"), split="train")
        if episodes.num_rows != info["total_episodes"]:
            print(f"episodes rows {episodes.num_rows} != total_episodes "
                  f"{info['total_episodes']}", file=sys.stderr)
            return 1

        print(f"lerobot HuggingFace load_dataset check passed: loaded {ds.num_rows} frames "
              f"across {info['total_episodes']} episodes, {len(expected_columns)} columns, "
              f"{tasks.num_rows} task(s)")
        print(f"  columns: {sorted(got_columns)}")
        print(f"  observation.state width={len(row['observation.state'])}, "
              f"action width={len(row['action'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
