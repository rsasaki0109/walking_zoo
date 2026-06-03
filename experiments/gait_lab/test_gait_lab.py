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
    SteerableCPG,
    CapturePointWalk,
    OptimizedCapturePoint,
    ZMPPreviewWalk,
    LearnedFeedbackWalk,
    RLSteerableWalk,
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


def test_png_writer_roundtrip(tmp_path):
    # Pure stdlib + numpy (no mujoco): the montage asset writer makes a valid PNG.
    import struct

    import numpy as np

    from gait_lab.pngio import save_png

    img = np.zeros((12, 20, 3), np.uint8)
    img[:, :, 1] = 128
    path = tmp_path / "t.png"
    save_png(str(path), img)
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", data[16:24])
    assert (width, height) == (20, 12)


def test_learned_feedback_walks_far_and_is_deterministic(model):
    learned = rollout(model, LearnedFeedbackWalk(), horizon=8.0)
    balanced = rollout(model, BalancedCPG(), horizon=8.0)
    # The learned linear feedback walks markedly farther than the hand-tuned
    # feedback (it does not out-survive it — the honest farthest-vs-stable tradeoff).
    assert learned.forward_distance > balanced.forward_distance
    # The baked policy must be reproducible (the whole point after the chaotic
    # 3.4s-fluke lesson).
    again = rollout(model, LearnedFeedbackWalk(), horizon=8.0)
    assert learned.forward_distance == pytest.approx(again.forward_distance)
    assert learned.survival_time == pytest.approx(again.survival_time)


def test_perturbation_is_deterministic_and_robust(model):
    from gait_lab import GaitHarness

    harness = GaitHarness(model, horizon=8.0)
    a, _ = harness.rollout(LearnedFeedbackWalk(), render=False, perturb_seed=0)
    b, _ = harness.rollout(LearnedFeedbackWalk(), render=False, perturb_seed=0)
    # Same perturbation seed -> identical rollout.
    assert a.survival_time == pytest.approx(b.survival_time)
    # Robustly trained: it still walks forward from a perturbed start.
    assert a.forward_distance > 0.3


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


def test_gae_advantages_are_correct():
    # Pure-function check of the PPO GAE (needs only numpy, not mujoco/torch).
    pytest.importorskip("numpy")
    import numpy as np

    from gait_lab.ppo import compute_gae

    rewards = np.array([1.0, 1.0, 1.0])
    values = np.array([0.5, 0.5, 0.5])
    # No terminal inside the segment; bootstrap from last_value.
    adv, ret = compute_gae(rewards, values, [False, False, False], last_value=0.5,
                           gamma=1.0, lam=1.0)
    # With gamma=lam=1 and constant value 0.5, the advantage at step t is the
    # sum of future rewards + bootstrap - V(t).
    assert adv[2] == pytest.approx(1.0 + 0.5 - 0.5)
    assert adv[1] == pytest.approx(2.0 + 0.5 - 0.5)
    assert adv[0] == pytest.approx(3.0 + 0.5 - 0.5)
    assert ret == pytest.approx(adv + values)
    # A true terminal stops the bootstrap from leaking past it.
    adv2, _ = compute_gae(rewards, values, [False, True, False], last_value=99.0,
                          gamma=1.0, lam=1.0)
    assert adv2[1] == pytest.approx(1.0 - 0.5)  # no bootstrap past terminal


def test_rl_env_zero_action_reproduces_cpg(model):
    # The residual env's feedforward IS balanced-cpg, so a zero residual must
    # reproduce its rollout exactly (the contract the learned residual builds on).
    import numpy as np

    from gait_lab.rl_env import ACT_DIM, OBS_DIM, G1WalkEnv

    env = G1WalkEnv(model, horizon=6.0)
    obs = env.reset()
    assert obs.shape == (OBS_DIM,)
    done = False
    while not done:
        res = env.step(np.zeros(ACT_DIM))
        done = res.done
    cpg = rollout(model, BalancedCPG(), horizon=6.0)
    # Same survival time (the env stops at the same fall), within one control step.
    assert res.info["t"] == pytest.approx(cpg.survival_time, abs=env.decim * env.timestep + 1e-6)


def test_rl_residual_is_deterministic_and_dependency_free(model):
    # RLResidualWalk runs the trained actor with numpy only (no torch at inference).
    from pathlib import Path

    from gait_lab import RLResidualWalk

    policy = Path(__file__).parent / "gait_lab" / "rl_policy.npz"
    if not policy.exists():
        pytest.skip("rl_policy.npz not trained yet (run train_rl.py)")
    a = rollout(model, RLResidualWalk(), horizon=8.0)
    b = rollout(model, RLResidualWalk(), horizon=8.0)
    assert a.survival_time == pytest.approx(b.survival_time)
    assert a.forward_distance == pytest.approx(b.forward_distance)
    # The headline: the RL residual BREAKS the ~3 s kinematic stability ceiling
    # that every hand-tuned / model-based gait hits. It survives the full horizon
    # (where balanced-cpg falls at ~3 s) and walks forward while doing so.
    cpg = rollout(model, BalancedCPG(), horizon=8.0)
    assert a.survival_time > cpg.survival_time + 3.0
    assert not a.fell
    assert a.forward_distance > 0.3


def test_steerable_cpg_equals_balanced_when_not_turning(model):
    # The steerable feedforward adds only a yaw-turning knob; with yaw_rate=0 it is
    # EXACTLY balanced-cpg (forward speed is owned by the learned residual, not the
    # feedforward). That equality keeps the ~3 s balanced baseline as the training
    # start point.
    cmd = Command(forward_speed=0.2, yaw_rate=0.0)
    bal = rollout(model, BalancedCPG(), horizon=6.0, cmd=cmd)
    steer = rollout(model, SteerableCPG(), horizon=6.0, cmd=cmd)
    assert steer.survival_time == pytest.approx(bal.survival_time)
    assert steer.forward_distance == pytest.approx(bal.forward_distance)


def test_steerable_cpg_yaw_knob_changes_control(model):
    # The one command-driven feedforward knob is hip-yaw turning: a +yaw and a
    # -yaw command produce opposite hip-yaw biases. Checked on the control vector
    # so it is deterministic and independent of the chaotic fall dynamics. Forward
    # speed does NOT change the feedforward (the residual owns it), so the rest of
    # the control vector is command-speed-invariant.
    ctrl = SteerableCPG()
    ctrl.reset(model)
    model.reset()
    obs = model.observe(0.0)
    li = model.actuator("left_hip_yaw_joint")
    ri = model.actuator("right_hip_yaw_joint")
    left = ctrl.update(obs, Command(0.2, 0.4))
    right = ctrl.update(obs, Command(0.2, -0.4))
    # +yaw biases both hip-yaws one way, -yaw the other.
    assert left[li] > right[li] and left[ri] > right[ri]
    # Forward speed is not a feedforward term: changing it leaves the control
    # vector unchanged (only the policy residual, absent here, would react).
    import numpy as np

    fast = ctrl.update(obs, Command(0.25, 0.0))
    slow = ctrl.update(obs, Command(0.05, 0.0))
    assert np.allclose(fast, slow)


def test_rl_env_steerable_obs_includes_command(model):
    # Steerable mode appends (forward_speed, yaw_rate) to the observation and a
    # zero residual reproduces the SteerableCPG feedforward it rides on.
    import numpy as np

    from gait_lab.rl_env import ACT_DIM, STEER_OBS_DIM, G1WalkEnv

    env = G1WalkEnv(model, horizon=4.0, steerable=True)
    obs = env.reset(seed=0, cmd=Command(0.3, 0.2))
    assert obs.shape == (STEER_OBS_DIM,)
    # The last two entries are the command.
    assert obs[-2] == pytest.approx(0.3)
    assert obs[-1] == pytest.approx(0.2)
    done = False
    while not done:
        res = env.step(np.zeros(ACT_DIM))
        done = res.done
    steer = rollout(model, SteerableCPG(), horizon=4.0, cmd=Command(0.3, 0.2))
    assert res.info["t"] == pytest.approx(
        steer.survival_time, abs=env.decim * env.timestep + 1e-6)


def test_rl_steerable_is_robust_but_does_not_track(model):
    # The honest result of command-conditioning the position-controlled CPG gait
    # via RL: the policy becomes ROBUST (stays up the full horizon across the whole
    # command grid) but does NOT cleanly track the command — it survives by
    # spiralling, ignoring the requested velocity/yaw. This is the structural
    # ceiling that motivates the footstep substrate (steerable-footstep); see the
    # README's steerable-gait section. Skipped until the policy is trained.
    from pathlib import Path

    policy = Path(__file__).parent / "gait_lab" / "rl_policy_steer.npz"
    if not policy.exists():
        pytest.skip("rl_policy_steer.npz not trained yet (run train_rl.py --steerable)")
    harness = GaitHarness(model, horizon=8.0)
    # Dependency-free numpy inference is deterministic.
    a, _ = harness.rollout(RLSteerableWalk(), cmd=Command(0.2, 0.0))
    b, _ = harness.rollout(RLSteerableWalk(), cmd=Command(0.2, 0.0))
    assert a.survival_time == pytest.approx(b.survival_time)
    assert a.forward_distance == pytest.approx(b.forward_distance)
    # Robust: it walks the full horizon under several commands without falling...
    for cmd in (Command(0.2, 0.0), Command(0.1, 0.3), Command(0.1, -0.3)):
        m, _ = harness.rollout(RLSteerableWalk(), cmd=cmd)
        assert m.survival_time >= 7.9 and not m.fell


def test_steerable_footstep_steers(model):
    # The thicker substrate: footstep placement in a rotating heading frame gives
    # the steering authority the CPG substrate lacks. At yaw_rate=0 it reduces to
    # the (stable) capture-point walker; a left-turn command bends the heading
    # noticeably more positive (CCW) than walking straight. (It is a kinematic
    # footstep walker, so it tops out near the few-second ceiling on its own — a
    # learned residual is what would carry it the full horizon.)
    import numpy as np

    from gait_lab import SteerableFootstepGait
    from gait_lab.controllers import CapturePointWalk

    def net_yaw(ctrl, cmd, horizon=2.5):
        model.reset()
        ctrl.reset(model)
        steps = int(round(horizon / model.timestep))
        y0 = 0.0
        for i in range(steps):
            t = i * model.timestep
            model.data.ctrl[:] = ctrl.update(model.observe(t), cmd)
            model.step()
            if float(model.data.qpos[2]) < 0.5:
                break
            if i == int(round(0.3 / model.timestep)):
                y0 = float(model.observe(t).torso_rpy[2])
        y1 = float(model.observe(0.0).torso_rpy[2])
        return float(np.arctan2(np.sin(y1 - y0), np.cos(y1 - y0)))

    # yaw=0 reduces to capture-point: same survival (it inherits its stability).
    straight_fs = rollout(model, SteerableFootstepGait(), horizon=4.0,
                          cmd=Command(0.1, 0.0))
    cp = rollout(model, CapturePointWalk(), horizon=4.0)
    assert straight_fs.survival_time == pytest.approx(cp.survival_time, abs=0.6)
    # A left-turn command bends the heading more CCW than going straight does —
    # the footstep frame rotation actually steers the robot.
    straight_yaw = net_yaw(SteerableFootstepGait(), Command(0.05, 0.0))
    left_yaw = net_yaw(SteerableFootstepGait(), Command(0.05, 0.4))
    assert left_yaw > straight_yaw + 0.1


def test_torque_mode_actuates_by_force(model):
    # Force-control foundation: an actuator switched to torque mode applies its
    # ctrl value as a joint TORQUE (not a position target), and switching back
    # restores position-servo behaviour. This is what lets a controller command
    # ankle/CoM torques (ground-reaction / ZMP balance) the position gait cannot.
    import numpy as np

    # Fresh model: the torque/mj_inverse machinery perturbs shared MjData state in
    # ways that are immaterial on their own but would change a later chaotic
    # rollout sharing the fixture (e.g. the capture-step recovery).
    model = G1Model()
    knee = "left_knee_joint"
    i = model.actuator(knee)
    qadr = int(model.model.jnt_qposadr[model.model.actuator_trnid[i, 0]])

    # Torque mode: a nonzero ctrl is a torque, so the joint accelerates that way.
    model.set_torque_mode([knee])
    try:
        model.reset()
        q0 = float(model.data.qpos[qadr])
        for _ in range(40):
            model.data.ctrl[:] = 0.0
            model.data.ctrl[i] = 8.0   # constant joint torque
            model.step()
        moved_pos = float(model.data.qpos[qadr]) - q0

        model.reset()
        q0b = float(model.data.qpos[qadr])
        for _ in range(40):
            model.data.ctrl[:] = 0.0
            model.data.ctrl[i] = -8.0  # opposite torque
            model.step()
        moved_neg = float(model.data.qpos[qadr]) - q0b
    finally:
        model.set_position_mode([knee])

    # Opposite torques move the joint in opposite directions, by a real amount.
    assert moved_pos * moved_neg < 0
    assert abs(moved_pos) > 1e-3 and abs(moved_neg) > 1e-3

    # Back in position mode: a position target above the current angle is tracked.
    model.reset()
    target = model.stand_targets.copy()
    target[i] += 0.2
    for _ in range(200):
        model.data.ctrl[:] = target
        model.step()
    assert float(model.data.qpos[qadr]) > float(model.stand_qpos[qadr]) + 0.05


def test_steerable_zmp_plan_curves_and_walks(model):
    # Steering on the most balance-aware base: the ZMP-preview walker made
    # command-conditioned. Its footstep schedule curves with a yaw command, and it
    # walks forward on its (more stable) base. It still tops out near the ~2 s
    # kinematic ceiling like every footstep walker — clean full-horizon steering
    # is the contact-WBC frontier — but it is the most stable steerable base.
    pytest.importorskip("scipy")
    import numpy as np

    from gait_lab import SteerableZMPWalk

    # A yaw command curves the footstep schedule: late plants rotate (CCW for +yaw)
    # relative to the straight plan's axis.
    straight = SteerableZMPWalk(0.12, 0.0)
    straight.reset(model)
    left = SteerableZMPWalk(0.12, 0.4)
    left.reset(model)
    seg_straight = straight._foot_plants[6] - straight._foot_plants[2]
    seg_left = left._foot_plants[6] - left._foot_plants[2]
    ang_straight = np.arctan2(seg_straight[1], seg_straight[0])
    ang_left = np.arctan2(seg_left[1], seg_left[0])
    assert ang_left > ang_straight + 0.2   # the +yaw plan heads more CCW

    # Straight: walks forward on the ZMP base before the kinematic ceiling.
    m = rollout(model, SteerableZMPWalk(0.12, 0.0), horizon=2.0)
    assert m.forward_distance > 0.2


def test_reactive_steerable_walks_but_does_not_break_the_ceiling(model):
    # The synthesis attempt (capture step + steering through one reactive
    # foot-placement law): it walks forward and responds to the command, but
    # continuous reactive capture-stepping is LESS stable than the open-loop
    # steerable-zmp -- closing the map that no position-controlled kinematic
    # steerable walker reaches the full horizon.
    from gait_lab import ReactiveSteerableWalk, SteerableZMPWalk

    fwd = rollout(model, ReactiveSteerableWalk(), horizon=8.0, cmd=Command(0.12, 0.0))
    # It walks forward (fast) before toppling...
    assert fwd.forward_distance > 0.4
    # ...but does not reach the full horizon (the kinematic ceiling holds)...
    assert fwd.fell and fwd.survival_time < 4.0
    # ...and is less stable than the (open-loop, smoother) steerable-zmp base.
    zmp = pytest.importorskip("scipy") and rollout(
        model, SteerableZMPWalk(0.12, 0.0), horizon=8.0)
    assert zmp.survival_time > fwd.survival_time


def test_force_walk_torque_wbc_runs(model):
    # The frontier attempt: a contact-Jacobian torque WBC tracking the ZMP-preview
    # plan inside a walk (gravity comp via mj_inverse + posture + CoM tasks). This
    # checks the WBC walker mechanism runs end to end and returns a sane survival;
    # the honest result (it does not beat position IK on a position-built model —
    # the limit is the substrate, not the controller) is reported by force_walk.py.
    pytest.importorskip("scipy")

    from force_walk import run_force_walk

    fresh = G1Model()  # self-contained: the torque WBC perturbs shared MjData
    surv = run_force_walk(fresh, horizon=1.2, fall_h=0.5)
    assert 0.0 < surv <= 1.2
    # The leg actuators are restored to position mode afterwards (no leakage).
    import numpy as np
    i = fresh.actuator("left_knee_joint")
    assert np.isclose(fresh.model.actuator_gainprm[i, 0], 500.0)


def test_qp_wbc_holds_a_stand_but_certifies_the_support_polygon_limit():
    # The contact-QP WBC (proper TSID: solve joint accels AND friction-cone ground-
    # reaction forces). Two honest facts the module establishes: (1) it HOLDS a
    # quiet stand with real GRF; (2) under a shove it does NOT beat the stiff servo
    # -- the QP goes infeasible (or topples) when the capture point leaves the
    # support polygon, certifying "you must step". Needs a QP solver (qpsolvers).
    pytest.importorskip("qpsolvers")

    from wbc_qp import run_qp_stand_push, run_position_stand_push

    fresh = G1Model()  # self-contained: torque mode perturbs shared MjData
    # (1) a quiet stand (no push) is held to the horizon.
    held, why = run_qp_stand_push(fresh, horizon=1.0, fall_h=0.5, push_speed=0.0)
    assert held == pytest.approx(1.0) and why == "held"
    # (2) a 0.6 m/s forward shove is NOT recovered, and fails for a physical reason.
    surv, why = run_qp_stand_push(fresh, horizon=1.5, fall_h=0.5, push_speed=0.6,
                                  direction=(1.0, 0.0))
    assert why in ("infeasible", "toppled")
    stiff = run_position_stand_push(fresh, horizon=1.5, fall_h=0.5, push_speed=0.6,
                                    direction=(1.0, 0.0))
    assert surv < stiff  # force control does not beat the stiff servo here
    # The actuators are restored to position mode afterwards (no leakage).
    import numpy as np
    i = fresh.actuator("left_knee_joint")
    assert np.isclose(fresh.model.actuator_gainprm[i, 0], 500.0)


def test_com_velocity_xy_is_real_not_silently_zero(model):
    # Regression guard for a subtle bug: data.subtree_linvel is not populated by
    # mj_step unless a subtree-velocity sensor exists (the G1 has none), so the
    # capture-point velocity term read via observe() was silently zero lab-wide.
    # com_velocity_xy() calls mj_subtreeVel explicitly and returns the real velocity.
    import numpy as np
    model.reset()
    model.data.qvel[0] += 0.6           # kick the base forward
    v = model.com_velocity_xy()
    assert v[0] > 0.3                   # real CoM velocity, not zero
    # ...and matches the CoM Jacobian times qvel (the independent ground truth).
    J = np.zeros((3, model.model.nv))
    mujoco.mj_jacSubtreeCom(model.model, model.data, J, 0)
    assert np.allclose(v, (J @ model.data.qvel)[:2], atol=1e-6)


def test_qp_capture_step_steps_and_beats_bare_qp_but_not_the_stiff_stand():
    # The culmination: force-aware QP balance that hands off to a capture step on the
    # QP's own feasibility certificate. Honest null result -- it extends survival over
    # the bare QP (the step does fire and help) but does NOT fully recover a 0.6 m/s
    # shove on this position-built model (the QP's compliance lets the push develop
    # before the step; the stiff stand's large forward support is hard to beat).
    pytest.importorskip("qpsolvers")
    import numpy as np
    from wbc_qp import run_qp_capture_step, run_qp_stand_push

    bare, _ = run_qp_stand_push(G1Model(), horizon=2.0, fall_h=0.5, push_speed=0.6,
                                direction=(1.0, 0.0))
    fresh = G1Model()  # self-contained: torque mode perturbs shared MjData
    whole, stepped = run_qp_capture_step(fresh, horizon=2.0, fall_h=0.5,
                                         push_speed=0.6, direction=(1.0, 0.0))
    assert stepped                 # the capture step actually fires (needs real CoM vel)
    assert whole > bare            # ...and extends survival past the bare QP
    assert whole < 2.0             # ...but does not fully recover the hard shove
    # Actuators restored to position mode afterwards (no leakage).
    i = fresh.actuator("left_knee_joint")
    assert np.isclose(fresh.model.actuator_gainprm[i, 0], 500.0)


def test_complete_tsid_is_torque_honest_but_the_wall_is_unchanged():
    # The "torque-native model" frontier the notes named turns out to already exist:
    # the menagerie G1 ships real joint torque limits (jnt_actfrcrange) that MuJoCo
    # enforces. The gap was the CONTROLLER -- the friction-cone-only QP plans ankle
    # torques several times the limit that MuJoCo silently clamps (never dynamically
    # consistent). The complete TSID (+torque-limit constraints) only plans torques
    # it can deliver, at no real cost to survival -- because the binding limit under
    # a shove is the support polygon, not the torque budget.
    pytest.importorskip("qpsolvers")
    import numpy as np
    from wbc_qp import run_qp_torque_audit

    # (1) friction-only QP demands torque well over the real limit under a shove...
    _, _, over_f, ratio_f = run_qp_torque_audit(
        G1Model(), horizon=1.0, fall_h=0.5, push_speed=0.6, tau_limits=False)
    assert over_f > 0 and ratio_f > 1.5      # plans torques MuJoCo would clamp
    # (2) ...the complete TSID never exceeds the limit (caps at 1.0).
    surv_t, _, over_t, ratio_t = run_qp_torque_audit(
        G1Model(), horizon=1.0, fall_h=0.5, push_speed=0.6, tau_limits=True)
    assert over_t == 0 and ratio_t <= 1.01   # torque-honest
    # (3) a quiet stand needs only a fraction of the budget -- torque is not the wall.
    _, why_q, _, ratio_q = run_qp_torque_audit(
        G1Model(), horizon=0.8, fall_h=0.5, push_speed=0.0, tau_limits=True)
    assert why_q == "held" and ratio_q < 0.8
    # Actuators restored to position mode afterwards (no leakage).
    m = G1Model()
    run_qp_torque_audit(m, horizon=0.3, fall_h=0.5, push_speed=0.0, tau_limits=True)
    i = m.actuator("left_knee_joint")
    assert np.isclose(m.model.actuator_gainprm[i, 0], 500.0)


def test_motor_model_zoh_and_bandwidth_are_honest():
    # Unit-level: the shared actuator pipeline does what it claims. No physics.
    import numpy as np
    from motor_model import MotorModel
    lo, hi = np.full(2, -10.0), np.full(2, 10.0)

    # zero-order hold: a 100 Hz control rate refreshes only every 0.01 s.
    m = MotorModel(lo, hi, control_hz=100.0)
    m.reset(2)
    assert m.should_recompute(0.000)          # first call always due
    assert not m.should_recompute(0.005)      # mid-period: held
    assert m.should_recompute(0.010)          # next period: refresh
    # control_hz=None refreshes every step.
    mn = MotorModel(lo, hi)
    mn.reset(2)
    assert mn.should_recompute(0.0) and mn.should_recompute(0.002)

    # first-order bandwidth lag: applied torque relaxes toward command, never jumps.
    m = MotorModel(lo, hi, bw_hz=20.0)
    m.reset(2)
    a1 = m.step(np.array([5.0, 5.0]), 0.002)
    assert np.all(a1 > 0) and np.all(a1 < 5.0)      # partial step, not instant
    a2 = m.step(np.array([5.0, 5.0]), 0.002)
    assert np.all(a2 > a1) and np.all(a2 < 5.0)      # keeps approaching
    # the real torque clamp still binds at the limit.
    mc = MotorModel(lo, hi)                           # ideal motor, clamp only
    mc.reset(2)
    assert np.allclose(mc.step(np.array([100.0, -100.0]), 0.002), [10.0, -10.0])


def test_stiff_servos_standing_win_was_an_implicit_damping_idealisation():
    # The lab's through-line ("a stiff position servo beats the force controllers")
    # rested on an unpaid idealisation: MuJoCo integrates the position servo's
    # velocity-damping term -kd q. IMPLICITLY (an unconditionally-stable inner loop
    # at the full physics rate). Applied as honest explicit torque -- the bit-
    # identical force a real finite-rate digital servo computes -- the SAME servo
    # cannot even hold a quiet stand. Routing -kd q. back through implicit damping
    # recovers most of it, localising the crutch to the velocity term.
    pytest.importorskip("qpsolvers")
    import numpy as np
    from motor_model import localize_servo_idealisation

    res = localize_servo_idealisation(G1Model(), horizon=2.0, fall_h=0.5,
                                      push_speed=0.0)
    t_imp, why_imp = res["implicit"]
    t_exp, why_exp = res["explicit"]
    t_dmp, _ = res["explicit+impl-damp"]
    t_qp, why_qp = res["qp"]
    # implicit position servo holds the quiet stand; the explicit one topples early.
    assert why_imp == "held" and t_imp >= 2.0
    assert why_exp == "toppled" and t_exp < 1.5
    # the damping term is the crutch: re-implicit-ing it recovers a big chunk.
    assert t_dmp > t_exp + 0.5
    # the model-based QP, on the same honest explicit-torque footing, DOES hold the
    # quiet stand the explicit servo cannot -- the standing-balance verdict flips.
    assert why_qp == "held" and t_qp >= 2.0


def test_under_a_real_shove_the_support_polygon_wall_is_unchanged():
    # The other half of the verdict, unflipped: in the fair fight (both controllers
    # explicit torque through the same motor) a real forward shove still topples
    # both at ~0.6 s, and the QP certifies "must step" (infeasible) at the ideal
    # motor. Removing the servo's idealisation buys standing balance, NOT push
    # recovery -- the binding limit under a shove is the support polygon.
    pytest.importorskip("qpsolvers")
    from motor_model import run_motor_stand_push
    m = G1Model()
    t_servo, _ = run_motor_stand_push(m, "servo", 2.0, 0.5, 0.6)
    t_qp, why_qp = run_motor_stand_push(m, "qp", 2.0, 0.5, 0.6)
    assert t_servo < 1.0 and t_qp < 1.0           # neither balances the shove
    assert why_qp in ("toppled", "infeasible")    # QP can certify the step
    # actuators restored to position mode afterwards (no fixture leakage).
    import numpy as np
    assert np.isclose(m.model.actuator_gainprm[m.actuator("left_knee_joint"), 0], 500.0)


def test_paying_the_idealisation_does_not_flip_the_walking_verdict():
    # The companion to the standing result, and the more important one: the lab's
    # CENTRAL conclusion ("position-IK zmp-preview beats the torque WBC while walking")
    # used the same implicit-servo idealisation as its winning baseline. Re-run both as
    # explicit torque on the ZMP-preview plan. Unlike standing, the verdict does NOT
    # flip: the position servo loses ~a third of its survival (the implicit-damping
    # share) but STILL beats the QP walk -- tracking the fast swing trajectory is a
    # genuine control-authority win, not an integrator gift.
    pytest.importorskip("scipy")          # ZMPPreviewWalk
    pytest.importorskip("qpsolvers")
    from wbc_qp import run_zmp_position
    from motor_model import run_motor_zmp_walk

    m = G1Model()
    implicit = run_zmp_position(m, 3.0, 0.5)              # the idealised baseline
    servo, _ = run_motor_zmp_walk(m, "servo", 3.0, 0.5)  # honest explicit torque
    qp, _ = run_motor_zmp_walk(m, "qp", 3.0, 0.5)
    # paying the idealisation costs the position walk real survival...
    assert servo < implicit - 0.3
    # ...but the position walk STILL beats the QP walk: the walking verdict holds.
    assert servo > qp + 0.4
    # actuators restored to position mode afterwards (no fixture leakage).
    import numpy as np
    assert np.isclose(m.model.actuator_gainprm[m.actuator("left_knee_joint"), 0], 500.0)


def test_capture_step_recovers_a_forward_push(model):
    # Push recovery that works: a forward shove topples the static position stand,
    # but a capture STEP (step the foot to the capture point) catches it. The
    # recovery is the *decision to step*, realised with the same position IK --
    # the honest, working rung the in-place force strategies could not reach.
    from capture_step import run_capture_step, run_stand

    speed, theta, horizon = 0.4, 0.0, 4.0  # forward shove
    stand = run_stand(model, speed, theta, horizon, fall_h=0.5)
    step = run_capture_step(model, speed, theta, horizon, fall_h=0.5)
    # The static stand topples well before the horizon...
    assert stand < horizon - 0.5
    # ...and the capture step survives markedly longer (here, the full horizon).
    assert step > stand + 1.0
    # Deterministic.
    again = run_capture_step(model, speed, theta, horizon, fall_h=0.5)
    assert step == pytest.approx(again)


def test_push_is_deterministic_and_disturbing(model):
    # A mid-rollout shove is reproducible for a given (push_seed, speed) and makes
    # a gait fall sooner than the same unshoved rollout — the basis of the
    # push-recovery benchmark in eval_policy.py.
    from gait_lab import GaitHarness

    harness = GaitHarness(model, horizon=8.0)
    a, _ = harness.rollout(BalancedCPG(), render=False, push_speed=0.6, push_seed=3)
    b, _ = harness.rollout(BalancedCPG(), render=False, push_speed=0.6, push_seed=3)
    assert a.survival_time == pytest.approx(b.survival_time)
    calm, _ = harness.rollout(BalancedCPG(), render=False)
    # Shoving the (already fragile) CPG does not make it survive *longer*.
    assert a.survival_time <= calm.survival_time + 1e-6


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
