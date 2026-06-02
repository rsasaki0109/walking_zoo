"""HuggingFace ``datasets`` compatibility tests for the LeRobot exporter.

These confirm the exported LeRobot v2.1 layout is actually consumable by
``datasets.load_dataset`` (the common LeRobot entry point that does not require
the full ``lerobot`` package). They are skipped automatically when ``datasets``
or ``pyarrow`` are not installed, so they stay CI-safe.
"""

import importlib.util
import json
from pathlib import Path

import pytest

datasets = pytest.importorskip("datasets")
pytest.importorskip("pyarrow")

_MODULE_PATH = Path(__file__).resolve().parent.parent / "scripts" / "walking_zoo_lerobot_export.py"
_spec = importlib.util.spec_from_file_location("walking_zoo_lerobot_export", _MODULE_PATH)
exporter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(exporter)


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


@pytest.fixture()
def exported_dataset(tmp_path):
    summary = exporter.write_episodes_dataset(
        [walk_trace(), turn_trace()], tmp_path, fps=10.0)
    if summary["data_format"] != "parquet":
        pytest.skip("exporter fell back to jsonl; parquet path unavailable")
    info = json.loads((tmp_path / "meta" / "info.json").read_text())
    return tmp_path, info


def test_parquet_episodes_load_with_huggingface(exported_dataset):
    out, info = exported_dataset
    files = sorted(str(p) for p in (out / "data").rglob("episode_*.parquet"))
    assert len(files) == info["total_episodes"]

    ds = datasets.load_dataset("parquet", data_files=files, split="train")
    assert ds.num_rows == info["total_frames"]
    assert set(ds.column_names) == set(info["features"].keys())
    assert set(ds.unique("episode_index")) == set(range(info["total_episodes"]))


def test_loaded_features_have_expected_widths(exported_dataset):
    out, info = exported_dataset
    files = sorted(str(p) for p in (out / "data").rglob("episode_*.parquet"))
    ds = datasets.load_dataset("parquet", data_files=files, split="train")

    row = ds[0]
    assert len(row["observation.state"]) == len(info["features"]["observation.state"]["names"])
    assert len(row["action"]) == len(info["features"]["action"]["names"])
    assert isinstance(ds[-1]["next.done"], bool)


def test_metadata_tables_load_with_huggingface(exported_dataset):
    out, info = exported_dataset
    tasks = datasets.load_dataset(
        "json", data_files=str(out / "meta" / "tasks.jsonl"), split="train")
    assert tasks.num_rows == info["total_tasks"]

    episodes = datasets.load_dataset(
        "json", data_files=str(out / "meta" / "episodes.jsonl"), split="train")
    assert episodes.num_rows == info["total_episodes"]
