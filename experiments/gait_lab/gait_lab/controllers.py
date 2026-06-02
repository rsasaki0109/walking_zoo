"""Gait-generation algorithms behind one ``GaitController`` interface.

Each controller maps ``(time, observation, command) -> ctrl[nu]`` where ctrl is
the position-actuator target vector for the G1. They share the same robot and
the same metrics, so the comparison is apples-to-apples: the *only* thing that
changes between rows in the report is the algorithm.

Add a new algorithm by subclassing ``GaitController`` and appending it to
``CONTROLLERS``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import G1Model, LEG_JOINTS, Observation

_GRAVITY = 9.81


@dataclass
class Command:
    """What we ask the gait to do. Kept minimal for now."""

    forward_speed: float = 0.4   # desired forward walking speed (m/s)


class GaitController:
    """Base class: turn an observation into position-actuator targets."""

    name: str = "base"

    def reset(self, model: G1Model) -> None:
        """Cache anything derived from the model (called once per rollout)."""
        self.model = model
        self.stand = model.stand_targets.copy()

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:  # pragma: no cover
        raise NotImplementedError

    # -- helpers shared by leg-driving controllers -------------------------
    def _leg(self, ctrl: np.ndarray, name: str, value: float) -> None:
        if self.model.has_actuator(name):
            ctrl[self.model.actuator(name)] += value


class StandHold(GaitController):
    """Baseline: hold the standing keyframe. Should never fall, never advance."""

    name = "stand-hold"

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        return self.stand.copy()


class OpenLoopCPG(GaitController):
    """Naive open-loop central pattern generator.

    A fixed sinusoid on hip-pitch/knee/ankle, left and right in anti-phase. No
    feedback whatsoever — the honest baseline that shows why a humanoid needs
    balance control (it topples within ~1 s).
    """

    name = "open-loop-cpg"
    frequency = 1.4   # Hz
    hip_amp = 0.35
    knee_amp = 0.50
    ankle_amp = 0.20

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        ctrl = self.stand.copy()
        phase = 2.0 * np.pi * self.frequency * obs.t
        for side, offset in (("left", 0.0), ("right", np.pi)):
            s = np.sin(phase + offset)
            swing = max(0.0, s)
            self._leg(ctrl, f"{side}_hip_pitch_joint", self.hip_amp * s)
            self._leg(ctrl, f"{side}_knee_joint", self.knee_amp * swing)
            self._leg(ctrl, f"{side}_ankle_pitch_joint", -self.ankle_amp * swing)
        return ctrl


@dataclass
class _Gains:
    # Torso attitude -> ankle restoring offset. NB the ankle is high-authority
    # (a 0.15 rad offset topples the G1), so these are deliberately small and the
    # SIGN counters the lean: a negative ankle offset pitches the torso forward,
    # so we feed +kp*pitch back to push it upright.
    pitch_kp: float = 0.15
    pitch_kd: float = 0.05
    roll_kp: float = 0.30
    roll_kd: float = 0.05


class BalancedCPG(GaitController):
    """CPG stepping plus lateral weight-shift and torso-attitude feedback.

    The naive CPG fails for two reasons: it never shifts weight onto the stance
    foot (so the swing leg can't unload without toppling sideways) and it has no
    way to arrest a forward/backward lean. This controller fixes both:

    * a lateral "rock" sinusoid on both hip-roll joints, in anti-phase with the
      step, shifts the centre of mass onto the stance foot each step;
    * torso roll/pitch (and their rates) feed back into the ankles to keep the
      body upright.

    With the same robot it survives ~3x longer than :class:`OpenLoopCPG` and
    makes net forward progress. It is *not* a robustly-walking controller —
    that is the gap this testbed exists to measure, and the natural place to
    drop in a learned or optimisation-based gait behind the same interface.
    """

    name = "balanced-cpg"
    frequency = 0.8
    hip_amp = 0.25
    knee_amp = 0.50
    ankle_amp = 0.12
    lateral_amp = 0.10   # hip-roll weight-shift amplitude (rad)

    def __init__(self, gains: _Gains | None = None):
        self.gains = gains or _Gains()

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        ctrl = self.stand.copy()
        g = self.gains
        roll, pitch, _ = obs.torso_rpy
        roll_rate, pitch_rate = obs.torso_ang_vel[0], obs.torso_ang_vel[1]

        ankle_pitch_fix = g.pitch_kp * pitch + g.pitch_kd * pitch_rate
        ankle_roll_fix = g.roll_kp * roll + g.roll_kd * roll_rate

        phase = 2.0 * np.pi * self.frequency * obs.t
        # Lateral weight-shift: both hips roll together (anti-phase to the step)
        # to load the stance foot before the swing foot lifts.
        rock = self.lateral_amp * np.sin(phase + np.pi)
        self._leg(ctrl, "left_hip_roll_joint", rock)
        self._leg(ctrl, "right_hip_roll_joint", rock)
        self._leg(ctrl, "left_ankle_roll_joint", ankle_roll_fix)
        self._leg(ctrl, "right_ankle_roll_joint", ankle_roll_fix)

        for side, offset in (("left", 0.0), ("right", np.pi)):
            s = np.sin(phase + offset)
            swing = max(0.0, s)
            self._leg(ctrl, f"{side}_hip_pitch_joint", self.hip_amp * s)
            self._leg(ctrl, f"{side}_knee_joint", self.knee_amp * swing)
            self._leg(
                ctrl, f"{side}_ankle_pitch_joint", -self.ankle_amp * swing + ankle_pitch_fix
            )
        return ctrl


class CapturePointWalk(GaitController):
    """Model-based footstep walking: capture-point foot placement + leg IK.

    A different *class* of algorithm from the CPGs above. Instead of a fixed
    joint pattern, it reasons about where to put the next foot and then solves
    inverse kinematics to get there:

    * The robot is modelled as a linear inverted pendulum, ``omega = sqrt(g/z)``.
    * At each foot strike it commits a footstep for the swing leg at the
      *instantaneous capture point* ``xi = x_com + v_com / omega`` (laterally, to
      catch the sideways fall) plus a forward term that regulates walking speed.
    * The swing foot arcs to that target (a lift profile) while the stance foot
      is held at its planted world position; both are realised by per-leg
      Jacobian IK (:meth:`G1Model.solve_leg_ik`).
    * A light ankle attitude feedback is layered on the IK solution to keep the
      torso upright.

    On the same robot it walks the *farthest* of all the algorithms here — but
    it is *less* durable than :class:`BalancedCPG`: kinematic footstep placement
    commits to long strides without true dynamic (ZMP/force) balance, so it
    eventually topples. That tradeoff — farthest walker vs. most stable — is the
    headline result the testbed is built to surface, and the motivation for a
    learned or optimisation-based gait behind this same interface.
    """

    name = "capture-point"
    step_duration = 0.45     # s per step (~ a LIPM lateral half-period)
    forward_speed = 0.15     # forward foot-placement bias (m/s)
    capture_x = 1.0          # how strongly forward capture point regulates speed
    capture_y = 0.4          # lateral capture-point weight
    nominal_width = 0.9      # lateral nominal-stance weight
    step_lift = 0.05         # swing-foot apex above ground (m)
    ankle_pitch_kp = 0.15
    ankle_roll_kp = 0.30
    ankle_kd = 0.05

    # Search space an optimiser may explore (name -> (low, high)). These are the
    # hand-tuned constants above; the optimised gait simply overrides them.
    TUNABLES = {
        "step_duration": (0.30, 0.60),
        "forward_speed": (0.0, 0.40),
        "capture_x": (0.30, 1.50),
        "capture_y": (0.0, 0.80),
        "nominal_width": (0.40, 1.10),
        "step_lift": (0.03, 0.10),
        "ankle_pitch_kp": (0.0, 0.40),
        "ankle_roll_kp": (0.0, 0.60),
    }

    def __init__(self, params: dict | None = None):
        # Override the hand-tuned class defaults with instance params (the hook
        # the optimiser uses; a plain CapturePointWalk() keeps the hand defaults).
        if params:
            for key, value in params.items():
                if key not in self.TUNABLES:
                    raise KeyError(f"unknown tunable {key!r}")
                setattr(self, key, float(value))

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        self.ground = float(model.foot_pos("left")[2])
        self.planted = {f: model.foot_pos(f) for f in ("left", "right")}
        self.stance = "right"
        self.swing = "left"
        self.t_step_start = 0.0
        self.swing_from = self.planted["left"].copy()
        self.plant_target = self._plan_footstep(model.observe(0.0))

    def _plan_footstep(self, obs: Observation) -> np.ndarray:
        omega = np.sqrt(_GRAVITY / max(obs.com_z, 0.3))
        xi_y = obs.com_xy[1] + obs.com_vel_xy[1] / omega
        nominal_y = 0.119 if self.swing == "left" else -0.119
        target_x = (
            obs.com_xy[0]
            + self.capture_x * obs.com_vel_xy[0] / omega
            + self.forward_speed * self.step_duration
        )
        target_y = self.capture_y * xi_y + self.nominal_width * nominal_y
        return np.array([target_x, target_y, self.ground])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        model = self.model
        phase = (obs.t - self.t_step_start) / self.step_duration
        if phase >= 1.0:
            # Foot strike: plant the swing foot, swap stance/swing, replan.
            self.planted[self.swing] = model.foot_pos(self.swing)
            self.stance, self.swing = self.swing, self.stance
            self.t_step_start = obs.t
            phase = 0.0
            self.swing_from = model.foot_pos(self.swing).copy()
            self.plant_target = self._plan_footstep(obs)

        ph = float(np.clip(phase, 0.0, 1.0))
        swing_xy = (1.0 - ph) * self.swing_from[:2] + ph * self.plant_target[:2]
        swing_z = self.ground + self.step_lift * np.sin(np.pi * ph)
        swing_target = np.array([swing_xy[0], swing_xy[1], swing_z])

        roll, pitch, _ = obs.torso_rpy
        roll_rate, pitch_rate = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        ankle_pitch_fix = self.ankle_pitch_kp * pitch + self.ankle_kd * pitch_rate
        ankle_roll_fix = self.ankle_roll_kp * roll + self.ankle_kd * roll_rate

        ctrl = self.stand.copy()
        for foot, target in ((self.stance, self.planted[self.stance]),
                             (self.swing, swing_target)):
            angles = model.solve_leg_ik(foot, target)
            for joint, value in zip(LEG_JOINTS[foot], angles):
                ctrl[model.actuator(joint)] = value
            # Attitude feedback layered on top of the IK solution.
            ctrl[model.actuator(f"{foot}_ankle_pitch_joint")] += ankle_pitch_fix
            ctrl[model.actuator(f"{foot}_ankle_roll_joint")] += ankle_roll_fix
        return ctrl


# Parameters discovered by `optimize.py` (Cross-Entropy Method, seed 0, 10
# generations of 18) maximising distance walked over a 6 s horizon. The optimiser
# warm-started from the hand-tuned CapturePointWalk defaults and roughly doubled
# the distance (0.61 m -> 1.27 m). Reproduce with:
#   python3 optimize.py --iters 10 --pop 18 --seed 0
OPTIMIZED_CAPTURE_POINT_PARAMS = {
    "step_duration": 0.4275,
    "forward_speed": 0.0755,
    "capture_x": 0.4356,
    "capture_y": 0.3345,
    "nominal_width": 0.9126,
    "step_lift": 0.0464,
    "ankle_pitch_kp": 0.3676,
    "ankle_roll_kp": 0.1673,
}


class OptimizedCapturePoint(CapturePointWalk):
    """CapturePointWalk with parameters found by optimisation, not by hand.

    Same algorithm and same interface as :class:`CapturePointWalk` — only the
    constants differ, and they came from `optimize.py` (Cross-Entropy Method over
    physics rollouts) rather than hand-tuning. It walks markedly farther than the
    hand-tuned version, the testbed's concrete answer to "does an optimisation-
    based gait beat hand-tuning?" A learned policy would plug in the same way:
    swap the parameter source (an optimiser, or a trained network) behind this
    identical `GaitController` surface.
    """

    name = "optimized-cp"

    def __init__(self):
        super().__init__(OPTIMIZED_CAPTURE_POINT_PARAMS)


class ZMPPreviewWalk(GaitController):
    """The most principled model-based walker here: ZMP preview control + IK.

    Where ``capture-point`` reacts one footstep at a time, this plans a whole
    walk up front and tracks it:

    1. Lay down a footstep schedule that marches forward, alternating feet at a
       nominal stance width.
    2. Turn that into a Zero-Moment-Point reference (where the support foot is)
       and run **Kajita preview control** (:mod:`gait_lab.zmp_preview`) to get a
       smooth centre-of-mass trajectory whose induced ZMP tracks the reference
       and *leads* it — the CoM sways over the next stance foot before the step.
    3. Each control tick, place both feet relative to that planned CoM via leg
       IK (so commanding the feet drives the pelvis along the planned sway), with
       light ankle attitude feedback on top.

    Needs SciPy (for the Riccati solve behind the preview gains). It produces the
    smoothest, most balance-aware motion of the model-based controllers.
    """

    name = "zmp-preview"
    plan_dt = 0.01
    preview_horizon = 200    # 2.0 s of ZMP look-ahead
    com_height = 0.70
    step_length = 0.10       # forward advance per footstep (m)
    step_duration = 0.55     # single-support time per step (s)
    double_support = 0.90    # initial settle in double support (s)
    nominal_y = 0.119        # half stance width (m)
    step_lift = 0.05         # swing-foot apex (m)
    plan_seconds = 14.0      # precomputed plan length (>= any rollout)
    ankle_pitch_kp = 0.20
    ankle_roll_kp = 0.20
    ankle_kd = 0.05

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        from .zmp_preview import PreviewAxis, design_preview

        left0 = model.foot_pos("left")
        right0 = model.foot_pos("right")
        self.ground = float(left0[2])
        self.base0 = model.data.qpos[0:2].copy()
        self._init_foot = {"left": left0[:2].copy(), "right": right0[:2].copy()}

        # Footstep schedule: stance foot per single-support step s (right on even
        # s, left on odd), marching forward by step_length each step.
        self._foot_plants = []
        sx = float(self.base0[0])
        for s in range(int(self.plan_seconds / self.step_duration) + 4):
            y = -self.nominal_y if s % 2 == 0 else self.nominal_y
            self._foot_plants.append(np.array([sx, float(self.base0[1]) + y]))
            sx += self.step_length

        # ZMP reference (centre during the initial double support, then the
        # support foot) and the preview-tracked CoM trajectory.
        n = int(self.plan_seconds / self.plan_dt)
        N = self.preview_horizon
        zmp = np.zeros((n + N, 2))
        for k in range(n + N):
            t = k * self.plan_dt
            if t >= self.double_support:
                s = int((t - self.double_support) // self.step_duration)
                zmp[k] = self._foot_plants[min(s, len(self._foot_plants) - 1)]
            else:
                zmp[k] = self.base0
        gains = design_preview(self.plan_dt, self.com_height, N)
        ax, ay = PreviewAxis(gains), PreviewAxis(gains)
        ax.reset(float(self.base0[0]))
        ay.reset(float(self.base0[1]))
        self._com = np.zeros((n, 2))
        for k in range(n):
            self._com[k, 0] = ax.step(zmp[k:k + N, 0])
            self._com[k, 1] = ay.step(zmp[k:k + N, 1])
        self._n = n

    def _foot_world(self, foot: str, t: float) -> np.ndarray:
        """Planned world position (x, y, z) of a foot at plan time ``t``."""
        if t < self.double_support:
            xy = self._init_foot[foot]
            return np.array([xy[0], xy[1], self.ground])
        s = int((t - self.double_support) // self.step_duration)
        phase = ((t - self.double_support) % self.step_duration) / self.step_duration
        stance_parity = s % 2                      # 0 -> right is stance
        foot_parity = 0 if foot == "right" else 1
        if foot_parity == stance_parity:
            xy = self._foot_plants[min(s, len(self._foot_plants) - 1)]
            return np.array([xy[0], xy[1], self.ground])
        # Swing foot: from its previous plant to its next plant, with a lift arc.
        prev = self._init_foot[foot] if s == 0 else self._foot_plants[s - 1]
        nxt = self._foot_plants[min(s + 1, len(self._foot_plants) - 1)]
        xy = (1.0 - phase) * np.asarray(prev) + phase * np.asarray(nxt)
        z = self.ground + self.step_lift * np.sin(np.pi * phase)
        return np.array([xy[0], xy[1], z])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        model = self.model
        k = min(int(obs.t / self.plan_dt), self._n - 1)
        com_plan = self._com[k]
        base_now = model.data.qpos[0:2]

        roll, pitch, _ = obs.torso_rpy
        roll_rate, pitch_rate = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        ankle_pitch_fix = self.ankle_pitch_kp * pitch + self.ankle_kd * pitch_rate
        ankle_roll_fix = self.ankle_roll_kp * roll + self.ankle_kd * roll_rate

        ctrl = self.stand.copy()
        for foot in ("left", "right"):
            fw = self._foot_world(foot, obs.t)
            # Place the foot relative to the *planned* CoM: commanding the feet
            # this way drives the actual pelvis along the planned sway. Vertical
            # stays absolute (reach for the ground / lift height).
            target = np.array([
                base_now[0] + (fw[0] - com_plan[0]),
                base_now[1] + (fw[1] - com_plan[1]),
                fw[2],
            ])
            angles = model.solve_leg_ik(foot, target)
            for joint, value in zip(LEG_JOINTS[foot], angles):
                ctrl[model.actuator(joint)] = value
            ctrl[model.actuator(f"{foot}_ankle_pitch_joint")] += ankle_pitch_fix
            ctrl[model.actuator(f"{foot}_ankle_roll_joint")] += ankle_roll_fix
        return ctrl


# Linear feedback policy (4 outputs x 6 observations, row-major) found by
# `train_policy.py`. Trained ROBUSTLY: each candidate is scored on the WORST of
# several perturbed initial states, because a falling humanoid is chaotic and a
# naive single-rollout search overfits to fragile flukes (an early run found a
# "3.4 s" policy that collapsed to 1.8 s under mere 4-decimal weight rounding —
# see the README). Robustly trained, this learned feedback walks much farther than
# the hand-tuned balanced-cpg (~0.74 m vs 0.27 m) at ~2.0 s survival: learning the
# feedback buys distance, but does NOT break the gait class's balance ceiling.
LEARNED_FEEDBACK_WEIGHTS = [
    -0.0080, 0.4121, -0.1297, 0.0634, -0.1680, 0.1113,   # -> ankle_pitch residual
    0.1905, -0.7327, 0.0276, 0.1250, -0.0741, 0.6008,    # -> ankle_roll residual
    -0.2176, -0.0297, -0.0539, -0.0049, -0.0214, 0.7609, # -> hip_pitch residual
    -0.0065, 0.2130, 0.0163, -0.1260, -0.3664, -0.1725,  # -> hip_roll residual
]


class LearnedFeedbackWalk(GaitController):
    """CPG feedforward + a *learned* linear feedback policy.

    The feedforward rhythm (the leg sinusoids and lateral rock) is a CPG like
    :class:`BalancedCPG`'s. The difference is the balance feedback: instead of
    hand-tuned ankle gains, a linear policy ``residual = W @ observation`` maps the
    full torso/CoM state to ankle and hip corrections, with ``W`` *trained* by the
    Cross-Entropy Method (`train_policy.py`) against physics rollouts.

    The honest answer it gives to "can a learned closed-loop feedback beat
    hand-designed feedback?": it walks much farther than ``balanced-cpg`` (~0.74 m
    vs 0.27 m) but does *not* out-survive it — the same farthest-vs-stable
    tradeoff, now learned rather than hand-tuned. Learning buys distance, not a
    broken balance ceiling. (And it must be trained *robustly* — see
    ``train_policy.py`` and the README on the chaotic-overfit trap.) A neural
    policy is the same shape with more capacity, behind this same interface.
    """

    name = "learned-feedback"
    frequency = 0.8
    hip_amp = 0.25
    knee_amp = 0.45
    ankle_amp = 0.16
    lateral_amp = 0.10
    OBS_DIM = 6
    OUT_DIM = 4

    def __init__(self, weights: list | None = None):
        w = weights if weights is not None else LEARNED_FEEDBACK_WEIGHTS
        self.W = np.asarray(w, dtype=float).reshape(self.OUT_DIM, self.OBS_DIM)

    def _observe(self, obs: Observation) -> np.ndarray:
        return np.array([
            obs.torso_rpy[0], obs.torso_rpy[1],
            obs.torso_ang_vel[0], obs.torso_ang_vel[1],
            obs.com_vel_xy[0], obs.com_vel_xy[1],
        ])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        # Learned feedback: [ankle_pitch, ankle_roll, hip_pitch, hip_roll].
        ap, ar, hp, hr = self.W @ self._observe(obs)

        ctrl = self.stand.copy()
        phase = 2.0 * np.pi * self.frequency * obs.t
        rock = self.lateral_amp * np.sin(phase + np.pi)
        self._leg(ctrl, "left_hip_roll_joint", rock + hr)
        self._leg(ctrl, "right_hip_roll_joint", rock + hr)
        self._leg(ctrl, "left_ankle_roll_joint", ar)
        self._leg(ctrl, "right_ankle_roll_joint", ar)
        for side, offset in (("left", 0.0), ("right", np.pi)):
            s = np.sin(phase + offset)
            swing = max(0.0, s)
            self._leg(ctrl, f"{side}_hip_pitch_joint", self.hip_amp * s + hp)
            self._leg(ctrl, f"{side}_knee_joint", self.knee_amp * swing)
            self._leg(ctrl, f"{side}_ankle_pitch_joint", -self.ankle_amp * swing + ap)
        return ctrl


# Registry. ``run_compare`` iterates this; tests assert each invariant.
def CONTROLLERS() -> list[GaitController]:
    return [
        StandHold(),
        OpenLoopCPG(),
        BalancedCPG(),
        CapturePointWalk(),
        OptimizedCapturePoint(),
        ZMPPreviewWalk(),
        LearnedFeedbackWalk(),
    ]
