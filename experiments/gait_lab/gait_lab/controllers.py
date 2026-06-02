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

from .model import G1Model, Observation


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


# Registry. ``run_compare`` iterates this; tests assert balanced beats open-loop.
def CONTROLLERS() -> list[GaitController]:
    return [StandHold(), OpenLoopCPG(), BalancedCPG()]
