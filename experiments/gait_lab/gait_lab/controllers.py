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


# Registry. ``run_compare`` iterates this; tests assert each invariant.
def CONTROLLERS() -> list[GaitController]:
    return [StandHold(), OpenLoopCPG(), BalancedCPG(), CapturePointWalk()]
