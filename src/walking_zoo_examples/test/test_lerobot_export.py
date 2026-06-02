"""Unit tests for the walking_zoo -> LeRobot dataset exporter.

Pure-Python (no ROS); exercises the trace resampling, schema, and on-disk
LeRobot layout against a synthetic demo trace.
"""

import importlib.util
import json
from pathlib import Path

import pytest

# Load the exporter module directly from the scripts directory.
_MODULE_PATH = Path(__file__).resolve().parent.parent / "scripts" / "walking_zoo_lerobot_export.py"
_spec = importlib.util.spec_from_file_location("walking_zoo_lerobot_export", _MODULE_PATH)
exporter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(exporter)


def make_trace():
    return {
        "schema": "walking_zoo.demo_trace.v1",
        "generated_by": "walking_zoo_demo_recorder",
        "duration_sec": 2.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/walking_zoo/state",
             "summary": "", "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.5, "topic": "/cmd_vel",
             "summary": "", "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.5, "topic": "/walking_zoo/cmd_vel",
             "summary": "", "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.6, "topic": "/walking_zoo/state",
             "summary": "", "data": {"state": "WALKING", "mode": 3, "estop_active": False}},
            {"t_sec": 1.5, "topic": "/walking_zoo/semantic_action",
             "summary": "", "data": {"action": "walk_forward"}},
            {"t_sec": 1.8, "topic": "/walking_zoo/state",
             "summary": "", "data": {"state": "ESTOPPED", "mode": 1, "estop_active": True}},
        ],
    }


def test_build_frames_resamples_to_fps():
    frames = exporter.build_frames(make_trace(), fps=10.0)
    # duration 2.0 s at 10 fps -> 21 frames (inclusive of t=0).
    assert len(frames) == 21
    assert frames[0]["timestamp"] == 0.0
    assert frames[-1]["next.done"] is True
    assert all(f["next.done"] is False for f in frames[:-1])


def test_frames_carry_action_and_observation():
    frames = exporter.build_frames(make_trace(), fps=10.0)
    # Before the command at t=0.5 the action is zero.
    assert frames[0]["action"] == [0.0, 0.0, 0.0]
    # After t=0.5 the commanded forward velocity is present.
    after_cmd = frames[6]
    assert after_cmd["action"][0] == pytest.approx(0.3)
    # Observation carries executed velocity and locomotion state id.
    assert after_cmd["observation.state"][0] == pytest.approx(0.3)
    # By the last frame the robot is e-stopped (state id 11, estop flag 1).
    assert frames[-1]["observation.state"][3] == pytest.approx(11.0)
    assert frames[-1]["observation.state"][5] == pytest.approx(1.0)


def test_derive_task_prefers_semantic_action():
    assert "walk_forward" in exporter.derive_task(make_trace())


def test_write_dataset_produces_lerobot_layout(tmp_path):
    summary = exporter.write_dataset(make_trace(), tmp_path, fps=10.0)
    assert summary["frames"] == 21

    info = json.loads((tmp_path / "meta" / "info.json").read_text())
    assert info["codebase_version"] == exporter.LEROBOT_CODEBASE_VERSION
    assert info["robot_type"] == "g1"
    assert info["total_frames"] == 21
    assert info["fps"] == 10.0
    assert info["features"]["observation.state"]["shape"] == [6]
    assert info["features"]["action"]["shape"] == [3]

    tasks = (tmp_path / "meta" / "tasks.jsonl").read_text().strip().splitlines()
    assert len(tasks) == 1
    assert json.loads(tasks[0])["task_index"] == 0

    episodes = (tmp_path / "meta" / "episodes.jsonl").read_text().strip().splitlines()
    assert json.loads(episodes[0])["length"] == 21

    stats = json.loads((tmp_path / "meta" / "stats.json").read_text())
    assert stats["action"]["count"] == [21]
    assert len(stats["observation.state"]["mean"]) == 6

    # The episode data file exists in whichever format was written.
    chunk = tmp_path / "data" / "chunk-000"
    written = list(chunk.glob("episode_000000.*"))
    assert len(written) == 1


def test_load_trace_rejects_unknown_schema(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema": "something.else", "events": []}))
    with pytest.raises(exporter.TraceFormatError):
        exporter.load_trace(bad)


def test_build_frames_rejects_empty_trace():
    with pytest.raises(exporter.TraceFormatError):
        exporter.build_frames({"schema": "walking_zoo.demo_trace.v1", "events": []}, fps=10.0)
