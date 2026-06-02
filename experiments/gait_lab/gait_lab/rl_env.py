"""A lightweight RL environment for *residual* gait learning on the G1.

The model-based and hand-tuned gaits in this lab all hit the same wall: a
position-controlled humanoid topples laterally within ~2-3 s of single support,
and no amount of gain tuning pushes past it (see ``stability_ceiling.py``). That
ceiling is a property of *reactive, hand-designed* feedback — the natural place
to ask whether a *learned* closed-loop policy can do better.

This env frames that question as residual reinforcement learning:

* the **feedforward** is :class:`~gait_lab.controllers.BalancedCPG` — it supplies
  the walking rhythm (the leg sinusoids and the lateral weight-shift), so the
  policy never has to discover *that* a humanoid walks by stepping;
* the **policy** outputs a small position-target *residual* on the 12 leg
  actuators each control tick, on top of that rhythm. All it has to learn is the
  stabilising correction the hand-tuned ankle/hip feedback could not provide.

Keeping the rhythm fixed and learning only the residual is what makes this
trainable on a single CPU MuJoCo instance in minutes rather than the millions of
env-steps a from-scratch humanoid policy needs.

The env is deliberately framework-free (no gymnasium dependency): ``reset`` /
``step`` return plain numpy, so the self-contained PPO in ``ppo.py`` and the
inference-time :class:`~gait_lab.controllers.RLResidualWalk` share one code path.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controllers import BalancedCPG, Command
from .model import G1Model, LEG_ACTUATORS

# Observation layout (also consumed by RLResidualWalk at inference time):
#   roll, pitch                         (2)  torso attitude
#   ang_vel x, y, z                     (3)  torso angular velocity
#   com_vel x, y                        (2)  whole-body CoM velocity
#   base_height                         (1)
#   phase sin, cos                      (2)  where we are in the CPG cycle
#   leg qpos - stand                    (12) leg joint angles (residual from stand)
#   leg qvel                            (12) leg joint velocities
OBS_DIM = 34
ACT_DIM = 12  # residual on the 12 leg actuators (LEG_ACTUATORS order)
DEFAULT_ACTION_SCALE = 0.25  # max residual (rad) per leg joint; shared with RLResidualWalk
DEFAULT_CONTROL_HZ = 50.0     # policy control rate; RLResidualWalk decimates to match


def observe_policy(model: G1Model, obs, leg_qadr, leg_dofadr, stand_leg, freq, t):
    """Build the policy observation vector. Shared by training and inference."""
    phase = 2.0 * np.pi * freq * t
    legq = model.data.qpos[leg_qadr] - stand_leg
    legv = model.data.qvel[leg_dofadr]
    return np.concatenate([
        obs.torso_rpy[:2],
        obs.torso_ang_vel,
        obs.com_vel_xy,
        [obs.base_height],
        [np.sin(phase), np.cos(phase)],
        legq,
        legv,
    ]).astype(np.float64)


@dataclass
class StepResult:
    obs: np.ndarray
    reward: float
    done: bool
    info: dict


class G1WalkEnv:
    """Residual-gait env: BalancedCPG feedforward + a learned leg-residual policy."""

    def __init__(
        self,
        model: G1Model | None = None,
        *,
        horizon: float = 8.0,
        control_hz: float = DEFAULT_CONTROL_HZ,
        target_speed: float = 0.4,
        action_scale: float = DEFAULT_ACTION_SCALE,
        fall_height: float = 0.5,
        perturb_scale: float = 0.0,
        push_interval: float = 0.0,   # mean s between mid-episode shoves (0 = off)
        push_speed: float = 0.0,      # velocity kick magnitude per shove (m/s)
    ):
        self.model = model or G1Model()
        self.horizon = horizon
        self.target_speed = target_speed
        self.action_scale = action_scale
        self.fall_height = fall_height
        self.perturb_scale = perturb_scale
        self.push_interval = push_interval
        self.push_speed = push_speed
        self.timestep = self.model.timestep
        self.decim = max(1, int(round((1.0 / control_hz) / self.timestep)))
        self.max_control_steps = int(round(horizon / (self.decim * self.timestep)))

        self.cpg = BalancedCPG()
        self.cmd = Command(forward_speed=target_speed)
        # Resolve the 12 leg actuator ctrl indices + their qpos/dof addresses once.
        self._leg_ctrl = np.array([self.model.actuator(n) for n in LEG_ACTUATORS])
        m = self.model.model
        self._leg_qadr = np.array(
            [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._leg_dofadr = np.array(
            [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in self._leg_ctrl]
        )
        self._stand_leg = self.model.stand_qpos[self._leg_qadr].copy()
        self._freq = self.cpg.frequency

    # -- gym-ish API -------------------------------------------------------
    def reset(self, seed: int | None = None) -> np.ndarray:
        self.model.reset()
        if seed is not None and self.perturb_scale > 0.0:
            self.model.perturb(seed, self.perturb_scale)
        self.cpg.reset(self.model)
        self._k = 0
        self._t = 0.0
        # Schedule mid-episode shoves (push-recovery training). Seed-derived so a
        # given episode's disturbances are reproducible.
        self._push_rng = np.random.default_rng((seed or 0) + 90001)
        self._next_push = self._schedule_push(self._t)
        return self._obs()

    def _schedule_push(self, t: float) -> float:
        if self.push_interval <= 0.0 or self.push_speed <= 0.0:
            return float("inf")
        # Exponential gaps around the mean interval, with a short initial grace.
        return t + 0.5 + float(self._push_rng.exponential(self.push_interval))

    def _obs(self) -> np.ndarray:
        o = self.model.observe(self._t)
        return observe_policy(
            self.model, o, self._leg_qadr, self._leg_dofadr,
            self._stand_leg, self._freq, self._t,
        )

    def step(self, action: np.ndarray) -> StepResult:
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        residual = self.action_scale * action
        # Mid-episode shove: an external velocity kick the policy must recover
        # from (only when push training is enabled).
        if self._t >= self._next_push:
            self.model.push(self._push_rng, self.push_speed)
            self._next_push = self._schedule_push(self._t)
        # Hold the residual across the decimation block; the CPG feedforward is
        # recomputed every sim step so the rhythm stays smooth.
        for _ in range(self.decim):
            obs = self.model.observe(self._t)
            ctrl = self.cpg.update(obs, self.cmd)
            ctrl[self._leg_ctrl] += residual
            self.model.data.ctrl[:] = ctrl
            self.model.step()
            self._t += self.timestep
        self._k += 1

        o = self.model.observe(self._t)
        roll, pitch = o.torso_rpy[0], o.torso_rpy[1]
        vx, vy = o.com_vel_xy
        fell = o.base_height < self.fall_height
        done = fell or self._k >= self.max_control_steps

        # Reward = stay up AND walk forward. The alive bonus breaks the lateral
        # ceiling; the forward term (gated on uprightness, so the policy can't
        # lunge-and-topple) is strong and SYMMETRIC — drifting backward is
        # penalised as much as walking forward is rewarded, so the policy cannot
        # cheaply "survive in place" by shuffling backward. Capped at the target
        # speed so it does not gain from sprinting into a fall.
        tilt = roll * roll + pitch * pitch
        upright_factor = max(0.0, 1.0 - 4.0 * tilt)
        alive = 1.0
        r_fwd = 1.0 * float(np.clip(vx, -self.target_speed, self.target_speed)) * upright_factor
        r_upright = -2.0 * tilt
        r_lateral = -0.5 * abs(vy)
        r_ctrl = -0.005 * float(action @ action)
        reward = alive + r_fwd + r_upright + r_lateral + r_ctrl
        if fell:
            reward -= 5.0  # strong one-off penalty for toppling early

        info = {"fell": bool(fell), "t": self._t, "vx": float(vx)}
        return StepResult(self._obs(), float(reward), bool(done), info)
