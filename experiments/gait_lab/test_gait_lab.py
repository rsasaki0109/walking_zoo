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
    CaptureStepRecovery,
    DCMWalk,
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


def test_dcm_step_adjustment_reacts_to_forward_velocity_error(model):
    # The reactive heart of the DCM walker, tested directly (no rollout): the next
    # foothold is the nominal plan plus k*(v_com - v_nominal)/omega, the divergent-
    # mode step adjustment. At nominal velocity the correction vanishes (the foot
    # lands on the nominal plan); a forward velocity *error* moves the foothold
    # further forward to catch the faster CoM. That single law is what reacts to a
    # shove during a rollout.
    import numpy as np

    from gait_lab import DCMWalk, Observation

    model.reset()             # known physics state (the shared fixture may be stale)
    dcm = DCMWalk()
    dcm.reset(model)
    nominal_v = dcm.step_length / dcm.step_duration
    common = dict(t=0.2, base_height=0.7, base_pos_xy=np.zeros(2),
                  base_lin_vel=np.zeros(3), torso_rpy=np.zeros(3),
                  torso_ang_vel=np.zeros(3), com_xy=np.zeros(2), com_z=0.70)
    on_plan = Observation(com_vel_xy=np.array([nominal_v, 0.0]), **common)
    too_fast = Observation(com_vel_xy=np.array([nominal_v + 0.4, 0.0]), **common)
    u_on = dcm._plan_foothold(on_plan, t_remaining=0.2)
    u_fast = dcm._plan_foothold(too_fast, t_remaining=0.2)
    # nominal velocity -> zero correction -> foot lands on the nominal plan.
    assert u_on[0] == pytest.approx(dcm.base0[0] + dcm.step_length, abs=1e-6)
    # a forward velocity error steps the foot meaningfully further forward.
    assert u_fast[0] > u_on[0] + 0.10


def test_dcm_walk_walks_well_but_its_closed_loop_does_not_break_the_ceiling(model):
    # DCMWalk is a legitimate new model-based baseline: a nominal footstep plan plus
    # a continuous DCM step adjustment. It walks farther and straighter than the
    # hand-tuned capture-point. But the honest result -- and the lab's recurring one
    # -- is that on this *position-controlled* G1 (no CoP authority within a step)
    # the closed loop buys no survival: every footstep walker topples from its own
    # dynamics in ~1-1.3 s, and the *open-loop* zmp-preview actually outlives it. The
    # DCM's theoretical robustness needs force-aware control to materialise.
    pytest.importorskip("scipy")          # ZMPPreviewWalk's Riccati solve
    dcm = rollout(model, DCMWalk(), horizon=6.0)
    cp = rollout(model, CapturePointWalk(), horizon=6.0)
    zmp = rollout(model, ZMPPreviewWalk(), horizon=6.0)
    assert "dcm-walk" in [c.name for c in CONTROLLERS()]   # registered in the zoo
    assert dcm.forward_distance > 0.4
    assert dcm.forward_distance > cp.forward_distance       # beats hand-tuned capture-point
    assert dcm.lateral_drift < 0.25                         # and walks roughly straight
    # the honest negative: the closed loop does not out-survive the open-loop preview.
    assert dcm.survival_time < zmp.survival_time
    # deterministic (a baked baseline, not a chaotic fluke).
    again = rollout(model, DCMWalk(), horizon=6.0)
    assert dcm.forward_distance == pytest.approx(again.forward_distance)
    assert dcm.survival_time == pytest.approx(again.survival_time)


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


def test_capture_step_controller_promotes_the_standalone_faithfully(model):
    # The capture step, promoted from a standalone script to a first-class
    # GaitController so it runs behind the interface (and thus through the SIL
    # runtime). It is the actionable conclusion of the contact-QP WBC's "you must
    # step" certificate. This pins two things: it is registered in the zoo, and the
    # promotion is *behaviour-preserving* -- the controller reproduces the validated
    # standalone run_capture_step exactly, and recovers a shove the static stand
    # topples to (here via the capture-aware ankle feedback; the step itself fires
    # for larger excursions, below).
    import numpy as np
    from capture_step import run_capture_step, run_stand

    assert "capture-step" in [c.name for c in CONTROLLERS()]   # registered

    def rollout_with_push(controller, speed, theta, horizon=4.0, fall_h=0.5):
        model.reset()
        controller.reset(model)
        d = model.data
        d.qvel[0] += speed * np.cos(theta)        # shove at t=0
        d.qvel[1] += speed * np.sin(theta)
        for i in range(int(round(horizon / model.timestep))):
            obs = model.observe(i * model.timestep)
            d.ctrl[:] = controller.update(obs, cmd=Command())
            model.step()
            if float(d.qpos[2]) < fall_h:
                return i * model.timestep
        return horizon

    horizon = 4.0
    # (1) Behaviour-preserving: the GaitController reproduces the standalone exactly.
    for speed, theta in ((0.4, 0.0), (0.6, 0.0), (0.6, np.pi / 4)):
        ctrl_surv = rollout_with_push(CaptureStepRecovery(), speed, theta, horizon)
        standalone = run_capture_step(model, speed, theta, horizon, fall_h=0.5)
        assert ctrl_surv == pytest.approx(standalone)

    # (2) Recovers a 0.4 m/s forward shove the static stand topples to.
    capture = rollout_with_push(CaptureStepRecovery(), 0.4, 0.0, horizon)
    stand = run_stand(model, 0.4, 0.0, horizon, fall_h=0.5)
    assert stand < horizon - 0.5          # the static stand topples...
    assert capture > stand + 1.0          # ...the capture-step controller does not

    # (3) The step machinery actually fires under a large enough excursion (a hard
    # shove drives the capture point past the support, exactly the QP's "must step").
    hard = CaptureStepRecovery()
    rollout_with_push(hard, 0.8, 0.0, horizon)
    assert hard.has_stepped


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


# --- push-robustness frontier (push_frontier.py) ---------------------------------

def test_push_frontier_search_and_area_are_exact():
    # Pure, no physics: the frontier's two primitives must be exact. The binary
    # search finds the largest surviving magnitude of a monotone survival function;
    # polygon_area is the true polar area (a unit "diamond" of radius 1 at the four
    # cardinals encloses area 2), and is monotone in the radii.
    import math

    from push_frontier import max_survivable, polygon_area

    survives_below = lambda model, v, th, H, fall: (v <= 0.63, "")
    r = max_survivable(survives_below, None, 0.0, 1.0, 1.0, hi=1.5, tol=0.01)
    assert r == pytest.approx(0.63, abs=0.02)
    # never survives even the smallest shove -> radius 0; always survives -> the cap.
    assert max_survivable(lambda *a: (False, ""), None, 0.0, 1, 1, hi=1.5) == 0.0
    assert max_survivable(lambda *a: (True, ""), None, 0.0, 1, 1, hi=1.5) == 1.5

    cardinals = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    assert polygon_area([1, 1, 1, 1], cardinals) == pytest.approx(2.0)
    assert polygon_area([2, 2, 2, 2], cardinals) == pytest.approx(8.0)  # ~r^2
    assert polygon_area([1, 0.5, 1, 0.5], cardinals) < polygon_area([1, 1, 1, 1],
                                                                    cardinals)


def test_capture_step_widens_the_forward_push_frontier_but_not_the_backward(model):
    # The showdown's thesis, quantified into a hard number. Map the max base-velocity
    # shove (m/s) each controller survives, by direction. Where stepping helps -- a
    # forward-diagonal shove -- the capture step survives a MUCH bigger shove than the
    # stiff in-place stand. But the backward direction is the shared worst for both,
    # and stepping does NOT meaningfully widen it: an honest, directional result, not
    # a blanket "stepping always wins".
    import math

    from push_frontier import FRONTIER_CONTROLLERS, max_survivable

    stiff = FRONTIER_CONTROLLERS["stiff-stand"][0]
    capture = FRONTIER_CONTROLLERS["capture-step"][0]
    H, hi, tol = 2.5, 1.0, 0.05

    fwd_diag = math.radians(45.0)
    s_fwd = max_survivable(stiff, model, fwd_diag, H, 0.5, hi=hi, tol=tol)
    c_fwd = max_survivable(capture, model, fwd_diag, H, 0.5, hi=hi, tol=tol)
    assert c_fwd > s_fwd + 0.15            # stepping buys a much bigger forward shove

    back = math.radians(180.0)
    s_back = max_survivable(stiff, model, back, H, 0.5, hi=hi, tol=tol)
    c_back = max_survivable(capture, model, back, H, 0.5, hi=hi, tol=tol)
    assert abs(c_back - s_back) < 0.12     # backward is the shared worst; no real gain
    # and the backward frontier really is the weak one (well under the forward gain).
    assert s_back < s_fwd and c_back < c_fwd


def test_contact_qp_push_frontier_collapses_to_a_must_step_certificate():
    # The contact-QP WBC holds a quiet stand, but its in-place push frontier collapses
    # to ~0: under any non-trivial shove it returns INFEASIBLE -- the QP itself
    # certifying "no friction-cone ground force can arrest this, you must step". So its
    # robustness polygon is a degenerate point at the origin. That is not the QP
    # failing; it is the QP telling you to do exactly what the capture step does.
    pytest.importorskip("qpsolvers")
    from push_frontier import FRONTIER_CONTROLLERS, max_survivable
    from wbc_qp import run_qp_stand_push

    model = G1Model()
    # holds the quiet (zero-shove) stand for the full horizon...
    t, reason = run_qp_stand_push(model, 2.5, 0.5, 0.0, direction=(1.0, 0.0))
    assert reason == "held" and t == pytest.approx(2.5)
    # ...but the largest shove it survives *in place* is essentially nothing.
    qp = FRONTIER_CONTROLLERS["contact-qp"][0]
    r = max_survivable(qp, model, 0.0, 2.5, 0.5, hi=1.0, tol=0.05)
    assert r <= 0.1


def test_detail_time_parses_the_survival_time_from_every_adapter_format():
    # The survival curve reads time-to-fall out of each adapter's detail string;
    # the formats differ ("1.31s", "1.31s/held", "1.31s/stepped") but all lead with
    # the float, so the parser must be format-agnostic.
    from push_frontier import _detail_time

    assert _detail_time("3.00s") == pytest.approx(3.0)
    assert _detail_time("0.59s/infeasible") == pytest.approx(0.59)
    assert _detail_time("1.31s/stepped") == pytest.approx(1.31)


def test_survival_curve_separates_recovering_from_merely_delaying():
    # The binary frontier scores both the contact-QP and the force+step synthesis at
    # r=0 -- it asks only "did you reach the full horizon?". The survival-TIME curve
    # un-flattens that: an immediate capture step RECOVERS a small shove (rides the
    # ceiling), while feeding the QP's must-step certificate to a late step only
    # DELAYS the fall -- but measurably longer than the bare QP that just certifies
    # infeasible. This is the value the polygon's r=0 hid.
    pytest.importorskip("qpsolvers")
    from push_frontier import survival_curve

    model = G1Model()
    H = 1.5
    curve = survival_curve(
        model, theta=0.0, mags=[0.0, 0.1], H=H, fall=0.5,
        controllers=["capture-step", "contact-qp", "qp-capture-step"], log=lambda *a: None)

    # the immediate capture step recovers the 0.1 m/s forward shove (reaches H)...
    assert curve["capture-step"]["ceiling_mag"] >= 0.1
    # ...while neither QP-based controller recovers ANY nonzero shove.
    assert curve["contact-qp"]["ceiling_mag"] == 0.0
    assert curve["qp-capture-step"]["ceiling_mag"] == 0.0
    # but the force+step synthesis DELAYS the fall markedly longer than the bare QP
    # (the late capture step buys time the in-place QP cannot) -- the real, measurable
    # value the binary frontier collapsed to a flat zero.
    t_qp = curve["contact-qp"]["times"][1]            # at 0.1 m/s
    t_qp_step = curve["qp-capture-step"]["times"][1]
    assert t_qp_step > t_qp + 0.3
    assert t_qp_step < H            # ...yet still does not recover the horizon


# --- fall-time theory: the ~1s collapse predicted from 1/omega (fall_time_theory.py)

def test_capturability_predicts_the_frontier_anisotropy_from_geometry():
    # v* = d*omega (the largest in-place-recoverable kick) is set by the support
    # margin. The lab's measured push frontier is anisotropic for exactly this
    # reason: the lateral/forward margins are wide, the backward heel margin narrow.
    # Pure geometry from the settled stand -- assert the ordering and the timescale.
    import numpy as np
    from fall_time_theory import lipm_capturability

    cap = lipm_capturability(G1Model())
    # the LIPM clock 1/omega = sqrt(z/g) is ~0.27s for this CoM height...
    assert cap["tau"] == pytest.approx(0.266, abs=0.03)
    assert cap["omega"] == pytest.approx(1.0 / cap["tau"], rel=1e-3)
    # ...and the capturability kick ranks lateral > forward > backward (wide stance,
    # long foot, narrow heel) -- the frontier's measured shape.
    v = cap["vstar"]
    assert v["lat"] > v["fwd"] > v["back"]
    # backward is the worst case and v*=d*omega lands near the measured ~0.20 m/s.
    assert v["back"] == pytest.approx(0.20, abs=0.04)


def test_free_inverted_pendulum_topple_is_the_two_over_omega_floor_and_a_lower_bound():
    # Claim 2: once balance is lost the fall clock is 1/omega, not the controller. The
    # free inverted-pendulum topple from upright is ~2/omega (~0.53s) and is a LOWER
    # bound on the measured stiff-stand fall (the servo can only delay within the
    # leg-length budget, never escape it).
    from fall_time_theory import (lipm_capturability, fall_angle,
                                  free_ip_fall_time, measure_stiff_fall)

    model = G1Model()
    cap = lipm_capturability(model)
    omega, tau = cap["omega"], cap["tau"]
    phi_fall = fall_angle(cap["pelvis0"], 0.5)
    # the free-IP topple shortens monotonically as the kick grows (more momentum =
    # faster fall) -- the fall is governed by the divergence, not a fixed duration.
    t_soft = free_ip_fall_time(omega, 0.3, phi_fall)
    t_hard = free_ip_fall_time(omega, 1.3, phi_fall)
    assert t_hard < t_soft
    # a hard kick topples sooner in the free model than the robot measures, because
    # the real stiff servo fights the fall the whole way down: free-IP is a strict
    # LOWER bound on the measured fall.
    (_, t_meas), = measure_stiff_fall(model, [1.3])
    assert t_hard < t_meas
    # and the measured hard-kick fall asymptotes to ~2/omega (the leg-length topple
    # floor) -- the universal ~1s ceiling is a few of these clocks, controller-free.
    assert t_meas == pytest.approx(2 * tau, abs=0.15)


# --- terrain frontier: capturability on a slope (terrain_frontier.py) -------------
# Tilting gravity by alpha is equivalent to walking a slope; tests the flat-ground
# capturability theory in a regime it was not fitted to.

def test_terrain_critical_slope_is_torque_limited_and_stepping_extends_it():
    # The static geometry says the stand should self-hold to arctan(d_fwd/z) ~ 9.6 deg.
    # It actually lets go near ~5 deg: the binding limit is the ankle torque that
    # cannot drive the CoP to the toe to hold the downhill lean -- the fall-time
    # theory's forward asymmetry, now on terrain. And a capture step extends it.
    from terrain_frontier import critical_slope, geometric_critical_slope

    model = G1Model()
    geo, _ = geometric_critical_slope(model)
    assert geo == pytest.approx(9.56, abs=1.0)
    stand = critical_slope(model, "stand", H=2.0)
    step = critical_slope(model, "capture-step", H=2.0)
    # the stand is torque-limited: it gives up well below the geometric bound...
    assert 2.5 < stand < geo - 2.0
    # ...and stepping pushes the critical slope higher (the recovery is the step).
    assert step >= stand + 0.3


def test_slope_biases_the_capturability_frontier_uphill():
    # Tilting gravity downhill biases the inverted pendulum downhill, so the uphill
    # support margin grows and the downhill one shrinks: on a slope the recoverable
    # kick is larger uphill than downhill, reversing the flat-ground forward>backward
    # order. This is v* = d*omega's predicted shift, measured.
    from terrain_frontier import max_kick, DIRS

    model = G1Model()
    down = max_kick(model, "stand", 3.0, DIRS["downhill"], H=2.0)
    up = max_kick(model, "stand", 3.0, DIRS["uphill"], H=2.0)
    assert up > down + 0.03


# --- adaptive step duration on restricted footholds (adaptive_step.py) ------------
# A from-paper port of "Adaptive Step Duration for Accurate Foot Placement"
# (arXiv:2403.17136, 2024), which ships no public code.

def test_adaptive_step_planner_recovers_a_clean_nominal_gait():
    # Sanity for the discrete DCM step-to-step planner: started on its periodic orbit
    # with no foothold constraints, it should reproduce a clean nominal gait --
    # durations near nominal, footsteps marching forward by ~step_length, stance side
    # alternating. (If this drifts, the bilinear solve or the DCM map is wrong.)
    pytest.importorskip("scipy")
    import numpy as np

    from adaptive_step import GaitParams, _limit_cycle_dcm, plan_steps

    par = GaitParams()
    p0 = np.array([0.0, -par.half_width])
    xi0 = _limit_cycle_dcm(par, p0[1], 1.0)
    plan = plan_steps(xi0, p0, par=par, n_steps=4, swing_side=1.0)
    assert plan.ok
    assert np.all(plan.T > 0.35) and np.all(plan.T < 0.65)         # near nominal 0.45
    assert np.all(np.diff(plan.u[:, 0]) > 0.08)                    # marches forward
    assert 0.30 < plan.u[-1, 0] - plan.u[0, 0] < 0.45             # ~3 * step_length
    assert plan.u[0, 1] > 0 > plan.u[1, 1] and plan.u[2, 1] > 0 > plan.u[3, 1]  # alternates


def test_adaptive_step_timing_keeps_dcm_viable_where_fixed_cadence_diverges():
    # The paper's core claim, quantified. On *irregular* stepping stones (long/short
    # forward gaps), both adaptive and fixed timing can hit the footholds -- but a
    # fixed cadence forced over uneven gaps lets the DCM run away (it would topple),
    # while adapting the step *duration* keeps the DCM viable. The viability error must
    # be dramatically lower for adaptive timing.
    pytest.importorskip("scipy")
    from adaptive_step import GaitParams, compare_timing_on_stones

    _, _, _, s = compare_timing_on_stones(GaitParams(), n=5)
    assert s["adaptive_viab_mean"] < 1.5            # adaptive stays viable
    assert s["fixed_viab_mean"] > 5.0               # fixed cadence diverges
    assert s["fixed_viab_mean"] > 8.0 * s["adaptive_viab_mean"]   # a large, clear gap
    # the adaptive plan genuinely varies the step duration (it is not secretly fixed).
    import numpy as np
    assert np.ptp(s["adaptive_T"]) > 0.1


def test_adaptive_walk_realises_the_plan_but_hits_the_position_control_ceiling(model):
    # The closed-loop realisation on the G1, reported honestly. Seeded on its periodic
    # orbit it walks forward via the planned footsteps -- but, like every position-
    # controlled footstep walker in this lab, it topples from its own dynamics in ~1 s
    # (no within-step CoP authority). The planner's viability advantage is real; it
    # cannot be cashed without force control. Deterministic baseline, not a fluke.
    pytest.importorskip("scipy")
    from adaptive_step import run_adaptive_walk

    a = run_adaptive_walk(model, horizon=6.0)
    assert a["forward"] > 0.2          # it does walk forward off the mark...
    assert a["fell"] and a["survival"] < 2.0    # ...but topples at the ceiling
    b = run_adaptive_walk(model, horizon=6.0)
    assert a["survival"] == pytest.approx(b["survival"])   # deterministic
