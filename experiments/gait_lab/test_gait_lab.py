"""Tests for the gait_lab physics testbed.

Run with the MuJoCo virtualenv, e.g.

    /path/to/.venv-mujoco/bin/python -m pytest experiments/gait_lab/test_gait_lab.py

The whole module is skipped if mujoco or the menagerie G1 model is unavailable,
so it never breaks a mujoco-free environment.
"""

from __future__ import annotations

import pytest

mujoco = pytest.importorskip("mujoco")

from gait_lab import (  # noqa: E402
    CONTROLLERS,
    BalancedCPG,
    CapturePointWalk,
    OptimizedCapturePoint,
    ZMPPreviewWalk,
    Command,
    GaitHarness,
    G1Model,
    OpenLoopCPG,
    StandHold,
    rollout,
)


@pytest.fixture(scope="module")
def model():
    try:
        return G1Model()
    except FileNotFoundError as exc:
        pytest.skip(str(exc))


def test_model_loads_with_actuators(model):
    assert model.nu == 29
    # Key leg actuators resolve to valid ctrl indices.
    for name in ("left_knee_joint", "right_ankle_pitch_joint", "left_hip_roll_joint"):
        assert 0 <= model.actuator(name) < model.nu
    assert model.stand_targets.shape == (model.nu,)


def test_stand_hold_survives_full_horizon(model):
    m = rollout(model, StandHold(), horizon=4.0)
    assert not m.fell
    assert m.survival_time == pytest.approx(4.0)
    # Standing still: essentially no travel.
    assert abs(m.forward_distance) < 0.05
    assert m.min_base_height > 0.6


def test_open_loop_cpg_topples_quickly(model):
    m = rollout(model, OpenLoopCPG(), horizon=6.0)
    # The honest baseline: open-loop stepping falls early.
    assert m.fell
    assert m.survival_time < 2.0


def test_balanced_cpg_beats_open_loop(model):
    balanced = rollout(model, BalancedCPG(), horizon=6.0)
    naive = rollout(model, OpenLoopCPG(), horizon=6.0)
    # Feedback + weight-shift survives clearly longer...
    assert balanced.survival_time > naive.survival_time + 1.0
    # ...and actually makes forward progress.
    assert balanced.forward_distance > 0.1


def test_leg_ik_reaches_foot_target(model):
    model.reset()
    stand_foot = model.foot_pos("left")
    target = stand_foot + [0.12, 0.0, 0.05]  # forward + up: a reachable swing pose
    angles = model.solve_leg_ik("left", target)
    assert angles.shape == (6,)
    # Apply the solution on a scratch and confirm the foot got close.
    import numpy as np

    s = model._ik_scratch
    s.qpos[:] = model.data.qpos
    s.qpos[model._leg_qadr["left"]] = angles
    mujoco.mj_kinematics(model.model, s)
    reached = s.site_xpos[model._foot_site["left"]]
    assert np.linalg.norm(reached - target) < 0.02


def test_capture_point_walks_farthest(model):
    results = {c.name: rollout(model, c, horizon=6.0) for c in CONTROLLERS()}
    cp = results["capture-point"]
    # The model-based IK walker travels farther than every other algorithm...
    assert cp.forward_distance > results["balanced-cpg"].forward_distance
    assert cp.forward_distance > 0.4
    # ...and tracks a straighter line than the naive open-loop stepper.
    assert cp.lateral_drift < results["open-loop-cpg"].lateral_drift


def test_optimized_gait_beats_hand_tuned(model):
    hand = rollout(model, CapturePointWalk(), horizon=6.0)
    opt = rollout(model, OptimizedCapturePoint(), horizon=6.0)
    # Same algorithm/interface, parameters from CEM optimisation instead of by
    # hand: it should walk markedly farther (it roughly doubled the distance).
    assert opt.forward_distance > hand.forward_distance * 1.3


def test_zmp_preview_control_tracks_and_leads():
    # Offline check of the preview servo (needs scipy, not mujoco).
    pytest.importorskip("scipy")
    import numpy as np

    from gait_lab.zmp_preview import PreviewAxis, design_preview

    dt, horizon = 0.01, 200
    gains = design_preview(dt, 0.70, horizon)
    n = 400
    ref = np.array(
        [0.1 if (int((k * dt) // 0.5)) % 2 == 0 else -0.1 for k in range(n + horizon)]
    )
    ax = PreviewAxis(gains)
    com = np.array([ax.step(ref[k:k + horizon]) for k in range(n)])
    # The induced ZMP tracks the reference...
    zmp = com - (0.70 / 9.81) * np.gradient(np.gradient(com, dt), dt)
    assert np.abs(zmp[80:] - ref[80:n]).mean() < 0.03
    # ...and the CoM *leads* the step: it heads toward the next foot before the
    # reference switches (the whole point of preview).
    switch = int(0.5 / dt)
    assert com[switch - 15] < com[switch - 30]


def test_zmp_preview_walks_and_balances(model):
    pytest.importorskip("scipy")
    zmp = rollout(model, ZMPPreviewWalk(), horizon=8.0)
    cap = rollout(model, CapturePointWalk(), horizon=8.0)
    # The preview walker makes real forward progress...
    assert zmp.forward_distance > 0.3
    # ...and is more balanced than the reactive capture-point walker.
    assert zmp.survival_time > cap.survival_time


def test_optimizer_objectives_reward_their_axis():
    # Pure-function check (no physics): each objective should prefer the gait that
    # is good on its own axis. A far-but-falls gait vs. a sustained-but-slower one.
    from types import SimpleNamespace

    from optimize import OBJECTIVES

    horizon = 8.0
    far_faller = SimpleNamespace(forward_distance=1.2, survival_time=1.3)
    sustained = SimpleNamespace(forward_distance=0.8, survival_time=8.0)
    # 'distance' rewards raw ground covered -> prefers the far faller.
    assert OBJECTIVES["distance"](far_faller, horizon) > OBJECTIVES["distance"](
        sustained, horizon
    )
    # 'balanced' rewards distance sustained without falling -> prefers the
    # sustained walker (the gap-closing axis).
    assert OBJECTIVES["balanced"](sustained, horizon) > OBJECTIVES["balanced"](
        far_faller, horizon
    )


def test_metrics_are_finite_and_serializable(model):
    m = rollout(model, BalancedCPG(), horizon=3.0)
    d = m.as_dict()
    assert set(d) >= {"forward_distance", "survival_time", "fell", "mean_speed"}
    for k in ("forward_distance", "survival_time", "mean_speed", "lateral_drift"):
        assert d[k] == d[k]  # not NaN
    assert isinstance(d["fell"], bool)


def test_results_are_deterministic(model):
    a = rollout(model, BalancedCPG(), horizon=5.0)
    b = rollout(model, BalancedCPG(), horizon=5.0)
    assert a.forward_distance == pytest.approx(b.forward_distance)
    assert a.survival_time == pytest.approx(b.survival_time)


def test_harness_render_produces_frames(model):
    pytest.importorskip("mujoco")
    harness = GaitHarness(model, horizon=1.0)
    try:
        metrics, frames = harness.rollout(
            StandHold(), cmd=Command(), render=True, width=160, height=120, fps=20
        )
    except Exception as exc:  # no GL backend available in this environment
        pytest.skip(f"rendering unavailable: {exc}")
    assert frames, "expected rendered frames"
    assert frames[0].shape == (120, 160, 3)
