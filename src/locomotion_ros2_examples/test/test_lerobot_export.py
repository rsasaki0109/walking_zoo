"""Unit tests for the locomotion_ros2 -> LeRobot dataset exporter.

Pure-Python (no ROS); exercises the trace resampling, schema, and on-disk
LeRobot layout against a synthetic demo trace.
"""

import importlib.util
import json
from pathlib import Path

import pytest

# Load the exporter module directly from the scripts directory.
_MODULE_PATH = Path(__file__).resolve().parent.parent / "scripts" / "locomotion_ros2_lerobot_export.py"
_spec = importlib.util.spec_from_file_location("locomotion_ros2_lerobot_export", _MODULE_PATH)
exporter = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(exporter)


def make_trace():
    return {
        "schema": "locomotion_ros2.demo_trace.v1",
        "generated_by": "locomotion_ros2_demo_recorder",
        "duration_sec": 2.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/locomotion_ros2/state",
             "summary": "", "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.5, "topic": "/cmd_vel",
             "summary": "", "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.5, "topic": "/locomotion_ros2/cmd_vel",
             "summary": "", "data": {"linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.0}},
            {"t_sec": 0.6, "topic": "/locomotion_ros2/state",
             "summary": "", "data": {"state": "WALKING", "mode": 3, "estop_active": False}},
            {"t_sec": 1.5, "topic": "/locomotion_ros2/semantic_action",
             "summary": "", "data": {"action": "walk_forward"}},
            {"t_sec": 1.8, "topic": "/locomotion_ros2/state",
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


def make_turn_trace():
    """A second, shorter trace with a different semantic action."""
    return {
        "schema": "locomotion_ros2.demo_trace.v1",
        "generated_by": "locomotion_ros2_demo_recorder",
        "duration_sec": 1.0,
        "latest": {"walking_state": {"robot": "g1"}},
        "events": [
            {"t_sec": 0.0, "topic": "/locomotion_ros2/state",
             "summary": "", "data": {"state": "STANDING", "mode": 2, "estop_active": False}},
            {"t_sec": 0.3, "topic": "/cmd_vel",
             "summary": "", "data": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.4}},
            {"t_sec": 0.4, "topic": "/locomotion_ros2/state",
             "summary": "", "data": {"state": "TURNING", "mode": 3, "estop_active": False}},
            {"t_sec": 0.6, "topic": "/locomotion_ros2/semantic_action",
             "summary": "", "data": {"action": "turn_right"}},
        ],
    }


def test_build_frames_offsets_index_and_labels_episode():
    frames = exporter.build_frames(
        make_turn_trace(), fps=10.0, episode_index=2, task_index=1, global_offset=100)
    assert frames[0]["index"] == 100
    assert frames[0]["frame_index"] == 0
    assert all(f["episode_index"] == 2 for f in frames)
    assert all(f["task_index"] == 1 for f in frames)


def test_write_episodes_dataset_aggregates_two_episodes(tmp_path):
    traces = [make_trace(), make_turn_trace()]
    summary = exporter.write_episodes_dataset(traces, tmp_path, fps=10.0)
    assert summary["episodes"] == 2
    assert summary["frames"] == 21 + 11  # 2.0s and 1.0s at 10 fps, inclusive of t=0
    assert len(summary["tasks"]) == 2  # walk_forward and turn_right are distinct

    info = json.loads((tmp_path / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 2
    assert info["total_frames"] == 32
    assert info["total_tasks"] == 2
    assert info["splits"]["train"] == "0:2"

    episodes = (tmp_path / "meta" / "episodes.jsonl").read_text().strip().splitlines()
    assert len(episodes) == 2
    assert json.loads(episodes[0])["episode_index"] == 0
    assert json.loads(episodes[0])["length"] == 21
    assert json.loads(episodes[1])["episode_index"] == 1
    assert json.loads(episodes[1])["length"] == 11

    tasks = (tmp_path / "meta" / "tasks.jsonl").read_text().strip().splitlines()
    assert len(tasks) == 2

    # Stats cover every frame across both episodes.
    stats = json.loads((tmp_path / "meta" / "stats.json").read_text())
    assert stats["action"]["count"] == [32]

    # One episode file per trace.
    chunk = tmp_path / "data" / "chunk-000"
    assert (chunk / "episode_000000.parquet").exists() or (chunk / "episode_000000.jsonl").exists()
    assert (chunk / "episode_000001.parquet").exists() or (chunk / "episode_000001.jsonl").exists()


def test_write_episodes_dedupes_identical_tasks(tmp_path):
    # Two traces with the same semantic action share one task table entry.
    summary = exporter.write_episodes_dataset([make_trace(), make_trace()], tmp_path, fps=10.0)
    assert summary["episodes"] == 2
    assert len(summary["tasks"]) == 1
    info = json.loads((tmp_path / "meta" / "info.json").read_text())
    assert info["total_tasks"] == 1
    assert info["total_episodes"] == 2


def test_write_episodes_rejects_empty_list(tmp_path):
    with pytest.raises(exporter.TraceFormatError):
        exporter.write_episodes_dataset([], tmp_path, fps=10.0)


def test_load_trace_rejects_unknown_schema(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema": "something.else", "events": []}))
    with pytest.raises(exporter.TraceFormatError):
        exporter.load_trace(bad)


def test_build_frames_rejects_empty_trace():
    with pytest.raises(exporter.TraceFormatError):
        exporter.build_frames({"schema": "locomotion_ros2.demo_trace.v1", "events": []}, fps=10.0)
