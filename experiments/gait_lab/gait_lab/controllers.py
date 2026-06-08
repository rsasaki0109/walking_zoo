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
    """What we ask the gait to do.

    ``forward_speed`` is the desired sagittal walking speed; ``yaw_rate`` is the
    desired turning rate (rad/s, +ve = turn left/CCW). Most gaits here ignore
    ``yaw_rate`` (they only ever walked straight); :class:`SteerableCPG` and
    :class:`RLSteerableWalk` are the ones that act on it.
    """

    forward_speed: float = 0.4   # desired forward walking speed (m/s)
    yaw_rate: float = 0.0        # desired turning rate (rad/s, +ve = CCW)


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


class SteerableCPG(BalancedCPG):
    """:class:`BalancedCPG` with feedforward knobs for *velocity* and *turning*.

    The plain ``balanced-cpg`` ignores its :class:`Command` entirely — it walks
    one fixed rhythm and whatever forward drift falls out is incidental. That is
    fine for the ceiling study, but a robot Nav2 can actually drive has to track
    a commanded forward speed *and* turn. This subclass adds three command-driven
    feedforward terms on top of the same rhythm and weight-shift:

    * **turning** — a hip-yaw bias proportional to ``yaw_rate`` on both legs
      steers the stance, so each step lands rotated. This is the one genuinely new
      feedforward knob; it is the capability Nav2 needs that the straight gait
      lacked.

    Forward speed is deliberately NOT a feedforward knob. Hand-built lean/stride
    "speed" terms on this position-controlled CPG were unreliable (a forward lean
    measured as *backward* drift, and they toppled the gait faster) — exactly the
    fragility this lab exists to expose. So the forward part is left identical to
    :class:`BalancedCPG` and the *learned residual* owns forward speed, driven by
    a linear velocity-tracking reward. The command ``(forward_speed, yaw_rate)``
    is in the policy observation, so a single network learns to push harder or
    softer to track the requested speed while the yaw knob handles turning.

    Calibrated so that at ``yaw_rate=0`` it is EXACTLY :class:`BalancedCPG`, which
    keeps the ~3 s balanced baseline as the training start point. On its own it is
    the steerable *feedforward*, not a walker.
    """

    name = "steerable-cpg"
    yaw_gain = 0.25        # hip-yaw bias (rad) per (rad/s) of commanded yaw_rate

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        ctrl = self.stand.copy()
        g = self.gains
        roll, pitch, _ = obs.torso_rpy
        roll_rate, pitch_rate = obs.torso_ang_vel[0], obs.torso_ang_vel[1]

        ankle_pitch_fix = g.pitch_kp * pitch + g.pitch_kd * pitch_rate
        ankle_roll_fix = g.roll_kp * roll + g.roll_kd * roll_rate

        # The only command-driven feedforward knob: a hip-yaw turning bias.
        yaw_bias = self.yaw_gain * cmd.yaw_rate

        phase = 2.0 * np.pi * self.frequency * obs.t
        rock = self.lateral_amp * np.sin(phase + np.pi)
        self._leg(ctrl, "left_hip_roll_joint", rock)
        self._leg(ctrl, "right_hip_roll_joint", rock)
        self._leg(ctrl, "left_ankle_roll_joint", ankle_roll_fix)
        self._leg(ctrl, "right_ankle_roll_joint", ankle_roll_fix)
        # Both hip-yaw joints biased the same way rotates the whole stance, so
        # each step lands rotated — a coarse but monotonic turn knob.
        self._leg(ctrl, "left_hip_yaw_joint", yaw_bias)
        self._leg(ctrl, "right_hip_yaw_joint", yaw_bias)

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


class CaptureStepRecovery(GaitController):
    """Push recovery that works: hold a stand, and take a capture STEP when shoved.

    This is the controller form of the lab's one genuinely-working push-recovery
    move (``capture_step.py``), and the *actionable conclusion* of the contact-QP
    WBC (``wbc_qp.py``). The QP holds a quiet stand with real friction-cone ground
    forces but goes **infeasible** the instant the capture point leaves the support
    polygon — it certifies "no in-place force recovers this; you must STEP." This is
    the controller that steps.

    It holds the standing keyframe until the **capture point**
    ``xi = com + com_vel / omega`` (Pratt's capturability — the point you must step
    to to come to rest) leaves the support, then swings the foot on the falling side
    to the (reach-clamped) capture point via leg IK while the stance foot holds, and
    settles over the new support. Repeated triggers give N-step capturability. It is
    realised with the *same position-controlled IK* the footstep walkers use — the
    win is the decision to step, not a new actuator — so it drops in behind the
    ``GaitController`` interface and runs through the SIL runtime where
    ``capture_step.run_capture_step`` was only a standalone script.

    Honest scope: it does **not** walk forward (it ignores ``forward_speed``); it is
    a *recovery* controller, the reactive-footstep substrate a real steerable/force-
    aware gait would build on. Through the runtime + push pipeline it is the one gait
    that recovers a shove the in-place stands and the open-loop walkers fall to —
    while a strong-enough shove still topples it (capturability is finite: past
    ``v* = d*omega`` set by the support margin, even one step cannot catch the fall).
    """

    name = "capture-step"
    trigger = 0.18        # capture-point excursion (m) past which a step fires
    step_time = 0.34      # swing duration (s)
    max_reach = 0.45      # foothold clamp from the stance foot (m)
    step_lift = 0.05      # swing-foot apex above ground (m)
    refractory_time = 0.30  # settle time before another step may fire (s)
    ankle_kp = 0.20
    ankle_kd = 0.05

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        self.ground = float(model.foot_pos("left")[2])
        self.planted = {f: model.foot_pos(f).copy() for f in ("left", "right")}
        self.stepping = False
        self.has_stepped = False
        self.t_step0 = 0.0
        self.refractory = 0.0
        self.swing = self.stance = None
        self.swing_from = self.swing_to = None

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        model = self.model
        com = obs.com_xy
        # observe()'s com_vel_xy reads the unpopulated subtree_linvel and is silently
        # zero on this model; com_velocity_xy() computes the real J_com @ qvel.
        com_vel = model.com_velocity_xy()
        omega = np.sqrt(_GRAVITY / max(obs.com_z, 0.3))
        xi = com + com_vel / omega                 # capture point

        ctrl = self.stand.copy()
        roll, pitch, _ = obs.torso_rpy
        roll_rate, pitch_rate = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        ankle_pitch_fix = self.ankle_kp * pitch + self.ankle_kd * pitch_rate
        ankle_roll_fix = self.ankle_kp * roll + self.ankle_kd * roll_rate

        if (not self.stepping and obs.t >= self.refractory
                and np.linalg.norm(xi - com) > self.trigger):
            # Trigger a (re)step: swing the falling-side foot toward the capture
            # point. Keep re-triggering until xi is back inside the support.
            fall_dir = xi - com
            self.swing = "left" if fall_dir[1] >= 0 else "right"
            self.stance = "right" if self.swing == "left" else "left"
            self.swing_from = self.planted[self.swing].copy()
            stance_xy = self.planted[self.stance][:2]
            tgt = xi.copy()
            v = tgt - stance_xy
            r = float(np.linalg.norm(v))
            if r > self.max_reach:
                tgt = stance_xy + v / r * self.max_reach
            self.swing_to = np.array([tgt[0], tgt[1], self.ground])
            self.stepping = True
            self.t_step0 = obs.t

        if self.stepping:
            ph = float(np.clip((obs.t - self.t_step0) / self.step_time, 0.0, 1.0))
            sx = (1.0 - ph) * self.swing_from[:2] + ph * self.swing_to[:2]
            sz = self.ground + self.step_lift * np.sin(np.pi * ph)
            feet = {self.stance: self.planted[self.stance],
                    self.swing: np.array([sx[0], sx[1], sz])}
            if ph >= 1.0:
                self.planted[self.swing] = self.swing_to.copy()  # commit foothold
                self.stepping = False
                self.has_stepped = True
                self.refractory = obs.t + self.refractory_time
        else:
            feet = {f: self.planted[f] for f in ("left", "right")}

        # Before the first step, hold the (stable) nominal stand. Once stepping has
        # begun, hold the planted/swinging feet via IK so the pose stays continuous
        # with the new footholds instead of snapping back to the stand.
        if self.stepping or self.has_stepped:
            for foot, target in feet.items():
                angles = model.solve_leg_ik(foot, target)
                for joint, value in zip(LEG_JOINTS[foot], angles):
                    ctrl[model.actuator(joint)] = value
        for side in ("left", "right"):
            ctrl[model.actuator(f"{side}_ankle_pitch_joint")] += ankle_pitch_fix
            ctrl[model.actuator(f"{side}_ankle_roll_joint")] += ankle_roll_fix
        return ctrl


class DCMWalk(GaitController):
    """Continuous **DCM** (divergent-component-of-motion) step adjustment — the
    modern textbook reactive walker (Englsberger et al. 2015; Khadiv et al. 2016),
    and the closed-loop model-based baseline the other two leave out.

    ``capture-point`` reasons about the foothold only *at each foot strike*;
    ``zmp-preview`` plans the whole CoM trajectory *open-loop* and never feeds the
    measured state back. This fills the gap between them: a nominal forward footstep
    plan (so it bootstraps and marches straight, like the preview) **plus** a DCM
    correction recomputed **every control tick** (so it reacts to disturbance, like
    the capture step). The next foothold is

        u = u_nominal + k_dcm * (v_com - v_nominal) / omega   (lateral term clamped)

    where ``u_nominal`` marches forward at the nominal stance width and the second
    term is the DCM's *divergent* component — a foot placement proportional to the
    velocity error, the unstable mode you must step on. A shove changes ``v_com``,
    which immediately moves the foothold to catch it, with no separate recovery mode.

    Honest note on what's realizable here. The full DCM law places the foot at the
    DCM *propagated to the end of the step*, ``xi_eos = p + (xi - p) e^{omega T}``,
    and relies on the CoP tracking a reference DCM *within* the step to stay on its
    periodic orbit. On this position-controlled G1 there is no CoP authority — the
    only handle is where the feet go — so that full propagation diverges from a cold
    stand (``e^{omega T} ~ 5``) and cannot bootstrap. The velocity-error form above is
    the realizable projection: the same divergent-mode foot placement, anchored to a
    nominal plan instead of an un-trackable propagated reference. Whether that closed
    loop actually buys distance or push-robustness over the open-loop preview on this
    robot is exactly what the benchmark and the push frontier measure — and the
    honest answer is "comparable to walk, with its edge (if any) under disturbance".
    """

    name = "dcm-walk"
    step_duration = 0.45
    step_length = 0.10        # forward advance per step (m)
    com_height = 0.70
    nominal_half_width = 0.119  # half stance width (m)
    step_lift = 0.05
    dcm_gain = 1.3            # foot-placement gain on the DCM velocity error
    lateral_clamp = 0.10     # cap on the lateral DCM correction (m)
    max_reach = 0.50         # foothold clamp from the stance foot (m)
    ankle_pitch_kp = 0.20
    ankle_roll_kp = 0.30
    ankle_kd = 0.05

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        self.ground = float(model.foot_pos("left")[2])
        self.planted = {f: model.foot_pos(f).copy() for f in ("left", "right")}
        self.stance = "right"
        self.swing = "left"
        self.t_step_start = 0.0
        self.swing_from = self.planted["left"].copy()
        self.base0 = model.data.qpos[0:2].copy()
        self.step_idx = 0
        self.plant_target = self.planted["left"].copy()

    def _omega(self, obs: Observation) -> float:
        return float(np.sqrt(_GRAVITY / max(obs.com_z, 0.3)))

    def _plan_foothold(self, obs: Observation, t_remaining: float) -> np.ndarray:
        """DCM step-adjustment: nominal forward plan + clamped velocity-error term."""
        omega = self._omega(obs)
        p = self.planted[self.stance][:2]                      # current CoP ~ stance
        nominal_y = self.nominal_half_width if self.swing == "left" \
            else -self.nominal_half_width
        u_nom = np.array([self.base0[0] + (self.step_idx + 1) * self.step_length,
                          self.base0[1] + nominal_y])
        v_nom = np.array([self.step_length / self.step_duration, 0.0])
        # The DCM divergent component: place the foot ahead of where the velocity
        # error is carrying the CoM. This is what reacts to a shove.
        corr = self.dcm_gain * (obs.com_vel_xy - v_nom) / omega
        corr[1] = float(np.clip(corr[1], -self.lateral_clamp, self.lateral_clamp))
        u = u_nom + corr
        # Clamp the foothold to a reachable radius from the stance foot.
        v = u - p
        r = float(np.linalg.norm(v))
        if r > self.max_reach:
            u = p + v / r * self.max_reach
        return np.array([u[0], u[1], self.ground])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        model = self.model
        phase = (obs.t - self.t_step_start) / self.step_duration
        if phase >= 1.0:
            self.planted[self.swing] = model.foot_pos(self.swing).copy()
            self.stance, self.swing = self.swing, self.stance
            self.t_step_start = obs.t
            phase = 0.0
            self.swing_from = model.foot_pos(self.swing).copy()
            self.step_idx += 1

        ph = float(np.clip(phase, 0.0, 1.0))
        t_remaining = (1.0 - ph) * self.step_duration
        # Recompute the foothold from the live DCM *every tick* — the reactive heart.
        self.plant_target = self._plan_foothold(obs, t_remaining)
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
            ctrl[model.actuator(f"{foot}_ankle_pitch_joint")] += ankle_pitch_fix
            ctrl[model.actuator(f"{foot}_ankle_roll_joint")] += ankle_roll_fix
        return ctrl


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


class SteerableFootstepGait(CapturePointWalk):
    """A *footstep-based* steerable walker — the thicker substrate for turning.

    The position-controlled CPG gaits in this lab cannot be made to steer: a small
    leg residual on a fixed leg sinusoid has no lever on *where the feet land*, so
    command-conditioned RL on that substrate collapses to a robust-but-spiralling
    gait that ignores the yaw command (see ``train_rl.py --steerable`` and the
    README). Turning a biped is fundamentally about *foot placement*: to turn, you
    step around a rotating heading.

    So this reuses :class:`CapturePointWalk`'s capture-point foot placement (the
    most *stable* of the kinematic footstep walkers) but resolves it in a
    **commanded, rotating heading frame**: forward/lateral are along/across the
    current heading ``theta`` instead of world axes, the commanded ``forward_speed``
    sets the fore step, and ``theta`` advances by ``yaw_rate * step_duration`` every
    step. At ``yaw_rate=0`` it is *exactly* ``capture-point`` (so it inherits its
    stability); a non-zero yaw command rotates the footstep frame, curving the
    steps so the robot actually turns to track the command — the lever the CPG
    substrate lacked. Like every kinematic footstep walker here it is not
    bullet-proof on its own (it tops out near the same few-second ceiling); a
    learned residual on top is what carries it the full horizon. But it *steers*.
    """

    name = "steerable-footstep"

    def reset(self, model: G1Model) -> None:
        # Defaults must exist before super().reset(), which calls _plan_footstep.
        self.theta = float(model.observe(0.0).torso_rpy[2])
        self._cmd = Command()
        super().reset(model)

    def _plan_footstep(self, obs: Observation) -> np.ndarray:
        # CapturePointWalk's placement, but with forward/lateral along/across the
        # heading theta. Projecting onto the orthonormal heading basis and back
        # makes this reduce EXACTLY to the parent at theta=0.
        omega = np.sqrt(_GRAVITY / max(obs.com_z, 0.3))
        c, s = np.cos(self.theta), np.sin(self.theta)
        fwd_dir = np.array([c, s])
        lat_dir = np.array([-s, c])
        com = obs.com_xy
        v = obs.com_vel_xy
        com_fwd, com_lat = float(com @ fwd_dir), float(com @ lat_dir)
        v_fwd, v_lat = float(v @ fwd_dir), float(v @ lat_dir)
        nominal = 0.119 if self.swing == "left" else -0.119
        target_fwd = (com_fwd + self.capture_x * v_fwd / omega
                      + self._cmd.forward_speed * self.step_duration)
        xi_lat = com_lat + v_lat / omega
        target_lat = self.capture_y * xi_lat + self.nominal_width * nominal
        target_xy = target_fwd * fwd_dir + target_lat * lat_dir
        return np.array([target_xy[0], target_xy[1], self.ground])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        # Stash the command for _plan_footstep and rotate the heading on a strike.
        self._cmd = cmd
        if (obs.t - self.t_step_start) / self.step_duration >= 1.0:
            self.theta += float(cmd.yaw_rate) * self.step_duration
        return super().update(obs, cmd)


class ReactiveSteerableWalk(GaitController):
    """The synthesis attempt: reactive-footstep steering — and its honest verdict.

    The capstone idea, unifying the two things that each half-work: the capture
    step (``capture_step.py``) demonstrably *recovers* a biped by placing the foot
    at the **capture point** ``xi = com + com_vel/omega``, and footstep placement
    *steers*. So this steps rhythmically with every footstep target set to the
    capture point (balance) **plus** a commanded forward offset along a heading that
    rotates with the commanded yaw rate (steering) — balance and steering through
    one reactive foot-placement lever, realised with the same position IK.

    Honest verdict: it does NOT break the ceiling. It walks fast and responds to
    the command, but *continuous* reactive capture-stepping is **less** stable than
    the open-loop ``steerable-zmp`` — it tops out around ~1.2 s. The capture step
    wins as a *recovery from a stand* (catch one shove and settle), but stepping to
    the capture point every step, while also pushing forward and turning, gives the
    CoM no settled support to recover onto — the very reactivity that rescues a
    stand destabilises a walk. This closes the map: across CPG-residual,
    capture-point, ZMP-preview, and this reactive synthesis, no position-controlled
    kinematic steerable walker reaches the full horizon. The genuine path is a
    contact-constrained torque WBC that regulates the moving CoM/ZMP while walking
    (foundation in ``force_balance.py``) or an RL policy that can stabilise such a
    base — both beyond position-controlled kinematics, which is the lab's point.
    """

    name = "reactive-steerable"
    step_duration = 0.34
    nominal_width = 0.12      # half stance width (m)
    step_lift = 0.05
    capture_x = 1.0           # fore/aft capture weight (balance)
    capture_y = 1.0           # lateral capture weight (balance)
    max_step = 0.40           # reach clamp from the stance foot (m)
    ankle_pitch_kp = 0.20
    ankle_roll_kp = 0.25
    ankle_kd = 0.05

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        self.ground = float(model.foot_pos("left")[2])
        self.theta = float(model.observe(0.0).torso_rpy[2])
        self.stance, self.swing = "right", "left"
        self.t0 = 0.0
        self.planted = {f: model.foot_pos(f).copy() for f in ("left", "right")}
        self.swing_from = self.planted["left"].copy()
        self.swing_to = self.planted["left"].copy()
        self._first = True

    def _lateral_sign(self, foot: str) -> float:
        return 1.0 if foot == "left" else -1.0

    def _plan_step(self, obs: Observation, cmd: Command) -> np.ndarray:
        omega = np.sqrt(_GRAVITY / max(obs.com_z, 0.3))
        c, s = np.cos(self.theta), np.sin(self.theta)
        fwd_dir = np.array([c, s])
        lat_dir = np.array([-s, c])
        v = obs.com_vel_xy
        # Capture point in the heading frame: react to where the body is going.
        cap_fwd = self.capture_x * float(v @ fwd_dir) / omega
        cap_lat = self.capture_y * float(v @ lat_dir) / omega
        # Steering offsets: a commanded forward step + the swing-side stance width.
        cmd_fwd = float(np.clip(cmd.forward_speed * self.step_duration, -0.16, 0.16))
        lat = self._lateral_sign(self.swing) * self.nominal_width
        target = (obs.com_xy
                  + fwd_dir * (cap_fwd + cmd_fwd)
                  + lat_dir * (cap_lat + lat))
        # Clamp the reach from the stance foot so the IK target stays feasible.
        stance_xy = self.planted[self.stance][:2]
        d = target - stance_xy
        r = float(np.linalg.norm(d))
        if r > self.max_step:
            target = stance_xy + d / r * self.max_step
        return np.array([target[0], target[1], self.ground])

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        model = self.model
        if self._first or (obs.t - self.t0) >= self.step_duration:
            if not self._first:
                self.planted[self.swing] = model.foot_pos(self.swing).copy()
                self.stance, self.swing = self.swing, self.stance
                self.theta += float(cmd.yaw_rate) * self.step_duration
            self._first = False
            self.t0 = obs.t
            self.swing_from = model.foot_pos(self.swing).copy()
            self.swing_to = self._plan_step(obs, cmd)

        ph = float(np.clip((obs.t - self.t0) / self.step_duration, 0.0, 1.0))
        sx = (1.0 - ph) * self.swing_from[:2] + ph * self.swing_to[:2]
        sz = self.ground + self.step_lift * np.sin(np.pi * ph)
        swing_target = np.array([sx[0], sx[1], sz])

        roll, pitch, _ = obs.torso_rpy
        rr, pr = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        ap_fix = self.ankle_pitch_kp * pitch + self.ankle_kd * pr
        ar_fix = self.ankle_roll_kp * roll + self.ankle_kd * rr

        ctrl = self.stand.copy()
        for foot, target in ((self.stance, self.planted[self.stance]),
                             (self.swing, swing_target)):
            angles = model.solve_leg_ik(foot, target)
            for joint, value in zip(LEG_JOINTS[foot], angles):
                ctrl[model.actuator(joint)] = value
            ctrl[model.actuator(f"{foot}_ankle_pitch_joint")] += ap_fix
            ctrl[model.actuator(f"{foot}_ankle_roll_joint")] += ar_fix
        return ctrl


class SteerableZMPWalk(ZMPPreviewWalk):
    """A *steerable* ZMP-preview walker — steering on the most balance-aware base.

    The kinematic footstep walkers (:class:`CapturePointWalk`,
    :class:`SteerableFootstepGait`) steer but topple near a ~2 s ceiling because
    they only *place feet* — they never control the CoM trajectory. :class:`ZMPPreviewWalk`
    is the most stable model-based gait here precisely because it does: Kajita
    preview control gives a smooth CoM trajectory whose induced ZMP leads the
    support foot. This subclass keeps that dynamic balance and makes the footstep
    schedule **curve**: each step advances by ``forward_speed * step_duration``
    along a heading ``theta`` that rotates by ``yaw_rate * step_duration``, with the
    plant offset half a stance-width across the heading. The preview-tracked CoM
    then sways along the *curved* path, so the robot walks an arc while staying
    balanced — steering on a base that does not just fall over at ~2 s.

    The command is fixed at construction (the plan is precomputed, like the
    parent); a reactive re-planning version is the natural next step.
    """

    name = "steerable-zmp"

    def __init__(self, forward_speed: float = 0.12, yaw_rate: float = 0.0):
        self._fs = float(forward_speed)
        self._yr = float(yaw_rate)

    def reset(self, model: G1Model) -> None:
        GaitController.reset(self, model)
        from .zmp_preview import PreviewAxis, design_preview

        left0 = model.foot_pos("left")
        right0 = model.foot_pos("right")
        self.ground = float(left0[2])
        self.base0 = model.data.qpos[0:2].copy()
        self._init_foot = {"left": left0[:2].copy(), "right": right0[:2].copy()}

        # Curved footstep schedule: a centreline that advances along a rotating
        # heading, each plant offset half a stance-width across it. theta rotates
        # by yaw_rate*step_duration per step, so the steps — and the walk — curve.
        adv = float(np.clip(self._fs * self.step_duration, 0.0, 0.16))
        theta = float(model.observe(0.0).torso_rpy[2])
        pos = self.base0.astype(float).copy()
        self._foot_plants = []
        for s in range(int(self.plan_seconds / self.step_duration) + 4):
            fwd = np.array([np.cos(theta), np.sin(theta)])
            perp = np.array([-np.sin(theta), np.cos(theta)])
            y_sign = -1.0 if s % 2 == 0 else 1.0     # right on even, left on odd
            self._foot_plants.append(pos + perp * (y_sign * self.nominal_y))
            pos = pos + fwd * adv
            theta += self._yr * self.step_duration

        # ZMP reference (centre during the initial double support, then the support
        # foot) and the preview-tracked CoM trajectory — identical to the parent.
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


class RLResidualWalk(GaitController):
    """A reinforcement-learned residual on top of the CPG rhythm.

    This is the answer to the ceiling that ``stability_ceiling.py`` measures: a
    hand-tuned / model-based position-controlled gait tops out near ~3 s because
    reactive position feedback cannot arrest the lateral inverted-pendulum fall.
    Here a neural policy (a two-hidden-layer MLP) is *trained by PPO*
    (:mod:`gait_lab.ppo`, ``train_rl.py``) to output a small position-target
    residual on the 12 leg actuators each control tick, on top of the same
    :class:`BalancedCPG` rhythm the linear ``learned-feedback`` gait uses.

    Same ``GaitController`` interface, same feedforward — the only thing that
    changed from ``learned-feedback`` is the policy's *capacity* (a nonlinear
    network, learned with full RL credit assignment over whole episodes, rather
    than a linear map fit by CEM). Whether that capacity is enough to break the
    structural balance ceiling — to actually walk the full horizon — is the
    headline question this controller exists to settle; see the README.

    Inference is dependency-free (numpy only): the trained actor is exported to
    ``rl_policy.npz`` (weights + observation normaliser) by ``train_rl.py``. Run
    that first; without the file this controller raises ``FileNotFoundError``.
    """

    name = "rl-residual"

    def __init__(self, policy_path: str | None = None):
        from pathlib import Path

        self.policy_path = Path(policy_path) if policy_path else (
            Path(__file__).parent / "rl_policy.npz"
        )
        self._loaded = False

    def _load(self) -> None:
        if not self.policy_path.exists():
            raise FileNotFoundError(
                f"RL policy not found at {self.policy_path}. Train it first:\n"
                "  python3 train_rl.py --iters 400 --steps 4096"
            )
        d = np.load(self.policy_path)
        n = int(d["n_layers"][0])
        self._W = [d[f"W{i}"] for i in range(n)]
        self._b = [d[f"b{i}"] for i in range(n)]
        self._obs_mean = d["obs_mean"]
        self._obs_std = d["obs_std"]
        self._loaded = True

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        from .model import LEG_ACTUATORS
        from .rl_env import DEFAULT_ACTION_SCALE, DEFAULT_CONTROL_HZ

        if not self._loaded:
            self._load()
        self.action_scale = DEFAULT_ACTION_SCALE
        # The policy was trained at DEFAULT_CONTROL_HZ with the residual held
        # across the intervening sim steps. The harness calls update() every sim
        # step, so we must decimate identically — recompute the residual every
        # `decim` steps and hold it — or the policy runs at the wrong control
        # rate and the behaviour diverges from training.
        self._decim = max(1, int(round((1.0 / DEFAULT_CONTROL_HZ) / model.timestep)))
        self._k = 0
        self._residual = np.zeros(len(LEG_ACTUATORS))
        self.cpg = BalancedCPG()
        self.cpg.reset(model)
        m = model.model
        self._leg_ctrl = np.array([model.actuator(nm) for nm in LEG_ACTUATORS])
        self._leg_qadr = np.array(
            [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._leg_dofadr = np.array(
            [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._stand_leg = model.stand_qpos[self._leg_qadr].copy()
        self._freq = self.cpg.frequency

    def _policy(self, x: np.ndarray) -> np.ndarray:
        """Deterministic actor: normalise, MLP (tanh), return the mean action."""
        h = (x - self._obs_mean) / self._obs_std
        for i, (W, b) in enumerate(zip(self._W, self._b)):
            h = W @ h + b
            if i < len(self._W) - 1:
                h = np.tanh(h)
        return h

    def feedforward_and_observation(self, obs: Observation, cmd: Command | None = None):
        """CPG feedforward and policy observation for embedded (C++) RL inference."""
        from .rl_env import observe_policy

        cmd = cmd or Command()
        ctrl = self.cpg.update(obs, cmd)
        refresh = (self._k % self._decim) == 0
        x = observe_policy(
            self.model, obs, self._leg_qadr, self._leg_dofadr,
            self._stand_leg, self._freq, obs.t,
        ) if refresh else None
        self._k += 1
        return ctrl, x, refresh

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        from .rl_env import observe_policy

        ctrl = self.cpg.update(obs, cmd)   # BalancedCPG feedforward, every sim step
        if self._k % self._decim == 0:     # recompute the residual at the control rate
            x = observe_policy(
                self.model, obs, self._leg_qadr, self._leg_dofadr,
                self._stand_leg, self._freq, obs.t,
            )
            self._residual = self.action_scale * np.clip(self._policy(x), -1.0, 1.0)
        self._k += 1
        ctrl[self._leg_ctrl] += self._residual
        return ctrl


class RLSteerableWalk(RLResidualWalk):
    """A *command-conditioned* residual gait: tracks forward speed + yaw rate.

    Where :class:`RLResidualWalk` learned one fixed-rhythm straight walk, this
    rides the command-aware :class:`SteerableCPG` feedforward and feeds the
    commanded ``(forward_speed, yaw_rate)`` into the policy observation, so a
    single network modulates its residual to *track a velocity command* while
    staying balanced. It is what lets Nav2 actually drive the SIL robot — change
    the command and the gait changes, instead of only ever walking straight.

    Same dependency-free numpy inference and the same ``GaitController``
    interface; only the feedforward (steerable), the observation (+2 command
    dims), and the trained weights (``rl_policy_steer.npz``) differ. Honest
    caveat: tracking a position-controlled humanoid's velocity off a coarse CPG
    is hard, so it tracks *approximately*, not to a tight tolerance — see the
    README's steerable-gait section and ``eval_steerable.py``.
    """

    name = "rl-steerable"

    def __init__(self, policy_path: str | None = None):
        from pathlib import Path

        self.policy_path = Path(policy_path) if policy_path else (
            Path(__file__).parent / "rl_policy_steer.npz"
        )
        self._loaded = False

    def reset(self, model: G1Model) -> None:
        # Same setup as the parent, but on the steerable feedforward.
        GaitController.reset(self, model)
        from .model import LEG_ACTUATORS
        from .rl_env import DEFAULT_ACTION_SCALE, DEFAULT_CONTROL_HZ

        if not self._loaded:
            self._load()
        self.action_scale = DEFAULT_ACTION_SCALE
        self._decim = max(1, int(round((1.0 / DEFAULT_CONTROL_HZ) / model.timestep)))
        self._k = 0
        self._residual = np.zeros(len(LEG_ACTUATORS))
        self.cpg = SteerableCPG()
        self.cpg.reset(model)
        m = model.model
        self._leg_ctrl = np.array([model.actuator(nm) for nm in LEG_ACTUATORS])
        self._leg_qadr = np.array(
            [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._leg_dofadr = np.array(
            [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._stand_leg = model.stand_qpos[self._leg_qadr].copy()
        self._freq = self.cpg.frequency

    def feedforward_and_observation(self, obs: Observation, cmd: Command | None = None):
        from .rl_env import observe_policy

        cmd = cmd or Command()
        ctrl = self.cpg.update(obs, cmd)
        refresh = (self._k % self._decim) == 0
        x = observe_policy(
            self.model, obs, self._leg_qadr, self._leg_dofadr,
            self._stand_leg, self._freq, obs.t, cmd=cmd,
        ) if refresh else None
        self._k += 1
        return ctrl, x, refresh

    def update(self, obs: Observation, cmd: Command) -> np.ndarray:
        from .rl_env import observe_policy

        ctrl = self.cpg.update(obs, cmd)   # SteerableCPG feedforward, every sim step
        if self._k % self._decim == 0:     # recompute the residual at the control rate
            x = observe_policy(
                self.model, obs, self._leg_qadr, self._leg_dofadr,
                self._stand_leg, self._freq, obs.t, cmd=cmd,
            )
            self._residual = self.action_scale * np.clip(self._policy(x), -1.0, 1.0)
        self._k += 1
        ctrl[self._leg_ctrl] += self._residual
        return ctrl


class RLSteerableFootstepWalk(RLSteerableWalk):
    """The learned residual on the *footstep* substrate — the steerable gait that
    actually turns.

    Same command-conditioned residual idea as :class:`RLSteerableWalk`, but riding
    :class:`SteerableFootstepGait` instead of the CPG. The footstep base supplies
    real steering authority (foot placement in a rotating heading frame); the
    learned residual supplies the balance the kinematic footstep walker lacks, so
    together they walk a velocity/yaw command the full horizon. Loads
    ``rl_policy_steer_fs.npz`` (train with ``train_rl.py --steerable --footstep``).
    """

    name = "rl-steerable-footstep"

    def __init__(self, policy_path: str | None = None):
        from pathlib import Path

        self.policy_path = Path(policy_path) if policy_path else (
            Path(__file__).parent / "rl_policy_steer_fs.npz"
        )
        self._loaded = False

    def reset(self, model: G1Model) -> None:
        super().reset(model)
        # Swap the CPG feedforward for the steering footstep substrate.
        self.cpg = SteerableFootstepGait()
        self.cpg.reset(model)
        self._freq = 1.0 / self.cpg.step_duration


# Registry. ``run_compare`` iterates this; tests assert each invariant.
def CONTROLLERS() -> list[GaitController]:
    return [
        StandHold(),
        OpenLoopCPG(),
        BalancedCPG(),
        CapturePointWalk(),
        OptimizedCapturePoint(),
        CaptureStepRecovery(),
        DCMWalk(),
        ZMPPreviewWalk(),
        LearnedFeedbackWalk(),
        RLResidualWalk(),
        RLSteerableWalk(),
        RLSteerableFootstepWalk(),
    ]
