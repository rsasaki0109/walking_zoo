#!/usr/bin/env python3
"""Adaptive **step duration + placement** for walking on restricted footholds —
a faithful, dependency-light implementation of a 2024 method that ships no code.

Paper: "Adaptive Step Duration for Accurate Foot Placement: Achieving Robust
Bipedal Locomotion on Terrains with Restricted Footholds" (arXiv:2403.17136, 2024).
It has no public implementation (the paper uses the commercial FORCES PRO solver),
so this is a from-scratch port onto the gait_lab G1.

The idea, and why step *timing* matters. A fixed-cadence walker can only choose
*where* to put the next foot; if the only legal footholds (stepping stones) are
spaced so that the nominal stride can't reach the next stone while keeping the
Divergent Component of Motion (DCM) viable, it must also choose *when* to land —
a longer step lets the DCM diverge further and reach a far stone; a shorter step
catches a near one. This plans both, every step, over a short receding horizon.

Model — the discrete step-to-step DCM map (LIP, CoM height ``zc``, ``lam=sqrt(g/zc)``):
during a step the Centre-of-Pressure sits at the stance foot ``p``, and the DCM
``xi = com + com_vel/lam`` diverges as ``xi(T) = p + (xi(0) - p) e^{lam T}``. So with
``sigma_k = e^{lam T_k}`` the end-of-step DCM and the next stance foot ``u_k`` give

    xi_k = p_{k-1} + (xi_{k-1} - p_{k-1}) * sigma_k          (p_{k-1} = u_{k-1})

The planner picks ``{T_k, u_k}`` over an ``N``-step horizon to minimise

    sum_k  beta_k [ a_z ||(xi_k - u_k) - b_nom_k||^2          (keep the DCM offset
                  + a_sig (T_k - T_nom)^2                       periodic = viable)
                  + a_u  ||u_k - u_nom_k||^2 ]                  (stay near the plan)

subject to ``T_min <= T_k <= T_max`` and ``u_k`` inside the next stepping stone.
The ``sigma_k * xi_{k-1}`` coupling is bilinear for ``N >= 2``, so this solves the
small nonlinear program with SciPy SLSQP (commodity, already a lab dependency) rather
than FORCES PRO. ``N = 1`` is a convex QP — used for the reactive push-recovery mode.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

_GRAVITY = 9.81


@dataclass
class StepPlan:
    """One receding-horizon solve: the foot placements and durations to execute."""
    u: np.ndarray            # (N, 2) planned touch-down foot positions
    T: np.ndarray            # (N,)   planned step durations (s)
    xi: np.ndarray           # (N, 2) predicted end-of-step DCM
    cost: float
    ok: bool                 # solver reported success


@dataclass
class GaitParams:
    com_height: float = 0.70
    T_nom: float = 0.45              # nominal step duration (s)
    T_min: float = 0.25
    T_max: float = 0.70
    step_length: float = 0.12        # nominal forward advance per step (m)
    half_width: float = 0.119        # nominal half stance width (m)
    step_lift: float = 0.05          # swing-foot apex above ground (m)
    a_z: float = 1.0e2               # DCM-viability weight
    a_sig: float = 1.0               # timing-deviation weight
    a_u: float = 1.0e2               # foot-deviation weight
    beta_decay: float = 0.6          # per-step preview discount (beta_k = decay^(k-1))

    def lam(self) -> float:
        return float(np.sqrt(_GRAVITY / max(self.com_height, 0.3)))


def _b_nominal(par: GaitParams, swing_side: float) -> np.ndarray:
    """Periodic DCM offset (xi_end - u) a nominal step should hold, per axis.

    Forward: ``L/(e^{lam T}-1)`` (the constant offset of a periodic LIP stride).
    Lateral: ``-side * w/(e^{lam T}+1)`` (alternating, inboard)."""
    e = np.exp(par.lam() * par.T_nom)
    bx = par.step_length / (e - 1.0)
    by = -swing_side * par.half_width / (e + 1.0)
    return np.array([bx, by])


def plan_steps(xi0, p0, *, par: GaitParams, n_steps: int,
               stones=None, u_nom=None, swing_side=1.0,
               t_init=None) -> StepPlan:
    """Plan ``n_steps`` of (foot location, duration) from the measured DCM ``xi0``
    and current stance foot ``p0``.

    ``stones[k]`` (optional) = ``(center_xy, half_xy)`` box the k-th foot must land in
    (a restricted foothold). ``u_nom[k]`` = nominal k-th foothold (defaults to a
    forward march at alternating stance width). ``swing_side`` = +1 if the first swing
    foot is the robot's left (+y), else -1; it alternates each step.
    """
    from scipy.optimize import minimize

    xi0 = np.asarray(xi0, float)
    p0 = np.asarray(p0, float)
    lam = par.lam()

    sides = [swing_side * (-1.0) ** k for k in range(n_steps)]
    if u_nom is None:
        u_nom = []
        x = float(p0[0])
        for k in range(n_steps):
            x += par.step_length
            u_nom.append(np.array([x, sides[k] * par.half_width]))
    u_nom = [np.asarray(u, float) for u in u_nom]
    b_nom = [_b_nominal(par, s) for s in sides]
    beta = [par.beta_decay ** k for k in range(n_steps)]

    def unpack(z):
        T = z[:n_steps]
        u = z[n_steps:].reshape(n_steps, 2)
        return T, u

    def rollout(T, u):
        xis, xi, p = [], xi0.copy(), p0.copy()
        for k in range(n_steps):
            xi = p + (xi - p) * np.exp(lam * T[k])
            xis.append(xi.copy())
            p = u[k]
        return xis

    def cost(z):
        T, u = unpack(z)
        xis = rollout(T, u)
        c = 0.0
        for k in range(n_steps):
            off = (xis[k] - u[k]) - b_nom[k]
            c += beta[k] * (par.a_z * off @ off
                            + par.a_sig * (T[k] - par.T_nom) ** 2
                            + par.a_u * (u[k] - u_nom[k]) @ (u[k] - u_nom[k]))
        return c

    # bounds: step duration box; feet inside their stone (or loose around nominal).
    bounds = [(par.T_min, par.T_max)] * n_steps
    for k in range(n_steps):
        if stones is not None and stones[k] is not None:
            c0, h = np.asarray(stones[k][0], float), np.asarray(stones[k][1], float)
            bounds += [(c0[0] - h[0], c0[0] + h[0]), (c0[1] - h[1], c0[1] + h[1])]
        else:
            bounds += [(u_nom[k][0] - 0.35, u_nom[k][0] + 0.35),
                       (u_nom[k][1] - 0.25, u_nom[k][1] + 0.25)]

    z0 = np.concatenate([
        np.full(n_steps, par.T_nom) if t_init is None else np.asarray(t_init, float),
        np.concatenate([(np.asarray(stones[k][0], float) if stones is not None
                         and stones[k] is not None else u_nom[k])
                        for k in range(n_steps)]),
    ])
    res = minimize(cost, z0, method="SLSQP", bounds=bounds,
                   options={"maxiter": 80, "ftol": 1e-9})
    T, u = unpack(res.x)
    return StepPlan(u=u, T=T, xi=np.array(rollout(T, u)),
                    cost=float(res.fun), ok=bool(res.success))


def _limit_cycle_dcm(par: GaitParams, stance_y: float, swing_side: float):
    """The periodic-orbit DCM at the start of a step: forward offset ``L/(e-1)``,
    lateral offset inboard of the stance foot. Used to seed a clean gait."""
    e = np.exp(par.lam() * par.T_nom)
    bx = par.step_length / (e - 1.0)
    by = (par.half_width * 2.0) / (e + 1.0)        # magnitude; inboard from stance
    return np.array([bx, stance_y + by * (1.0 if stance_y < 0 else -1.0)])


def run_adaptive_walk(model, *, stones=None, horizon=6.0, fall_h=0.5,
                      par: GaitParams | None = None, n_horizon=2,
                      fixed_timing=False, seed_velocity=True, record=None):
    """Walk the G1 with the adaptive step-duration planner, realised by leg IK.

    Each step: measure the DCM, solve ``plan_steps`` (optionally constrained to the
    next stepping stones), and swing the swing foot to the planned foothold over the
    *planned* (adapted) duration while the stance foot holds. ``fixed_timing=True``
    pins the duration to nominal (the baseline that can hit stones but loses DCM
    viability). Returns a dict of metrics. If ``record`` is a list, appends the
    per-step plan for inspection/plotting.

    The gait is seeded on its periodic orbit (``seed_velocity``) so the DCM template
    is valid from the first step — the cold-start a position-controlled biped can't
    bootstrap from a dead stand (see the DCM walker's null result).
    """
    from gait_lab.model import LEG_JOINTS
    par = par or GaitParams()
    if fixed_timing:
        par = GaitParams(**{**par.__dict__, "T_min": par.T_nom, "T_max": par.T_nom})
    lam = par.lam()
    model.reset()
    d = model.data
    ground = float(model.foot_pos("left")[2])
    planted = {f: model.foot_pos(f).copy() for f in ("left", "right")}
    stance, swing = "right", "left"
    swing_side = 1.0                                  # left swings first (+y)

    if seed_velocity:
        # Seed the periodic-orbit CoM velocity so the DCM template is valid at step 0.
        xi0 = _limit_cycle_dcm(par, planted[stance][1], swing_side)
        com0 = model.observe(0.0).com_xy
        d.qvel[0] += lam * (xi0[0] - com0[0])
        d.qvel[1] += lam * (xi0[1] - com0[1])

    stand = model.stand_targets.copy()
    stone_idx = 0
    t_step0 = 0.0
    swing_from = planted[swing].copy()
    # initial plan
    def make_plan(t):
        com = model.observe(t)
        xi = com.com_xy + model.com_velocity_xy() / lam
        ahead = None
        if stones is not None:
            ahead = [stones[stone_idx + k] if stone_idx + k < len(stones) else None
                     for k in range(n_horizon)]
        return plan_steps(xi, planted[stance][:2], par=par, n_steps=n_horizon,
                          stones=ahead, swing_side=swing_side)

    plan = make_plan(0.0)
    step_T = float(plan.T[0])
    plant_target = np.array([plan.u[0, 0], plan.u[0, 1], ground])

    steps = int(round(horizon / model.timestep))
    hit_errs, min_h = [], float("inf")
    ankle_kp, ankle_kd = 0.20, 0.05
    for i in range(steps):
        t = i * model.timestep
        phase = (t - t_step0) / step_T
        if phase >= 1.0:
            planted[swing] = model.foot_pos(swing).copy()
            if stones is not None and stone_idx < len(stones):
                hit_errs.append(float(np.linalg.norm(
                    planted[swing][:2] - np.asarray(stones[stone_idx][0]))))
            stone_idx += 1
            stance, swing = swing, stance
            swing_side = -swing_side
            t_step0 = t
            phase = 0.0
            swing_from = model.foot_pos(swing).copy()
            plan = make_plan(t)
            if record is not None:
                record.append(plan)
            step_T = float(plan.T[0])
            plant_target = np.array([plan.u[0, 0], plan.u[0, 1], ground])

        ph = float(np.clip(phase, 0.0, 1.0))
        swing_xy = (1.0 - ph) * swing_from[:2] + ph * plant_target[:2]
        swing_z = ground + par.step_lift * np.sin(np.pi * ph)
        swing_target = np.array([swing_xy[0], swing_xy[1], swing_z])

        obs = model.observe(t)
        roll, pitch, _ = obs.torso_rpy
        rr, pr = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        ap_fix = ankle_kp * pitch + ankle_kd * pr
        ar_fix = ankle_kp * roll + ankle_kd * rr

        ctrl = stand.copy()
        for foot, target in ((stance, planted[stance]), (swing, swing_target)):
            angles = model.solve_leg_ik(foot, target)
            for joint, value in zip(LEG_JOINTS[foot], angles):
                ctrl[model.actuator(joint)] = value
            ctrl[model.actuator(f"{foot}_ankle_pitch_joint")] += ap_fix
            ctrl[model.actuator(f"{foot}_ankle_roll_joint")] += ar_fix
        d.ctrl[:] = ctrl
        model.step()
        h = float(d.qpos[2])
        min_h = min(min_h, h)
        if h < fall_h:
            return {"survival": t, "fell": True, "forward": float(d.qpos[0]),
                    "stone_errs": hit_errs, "min_h": min_h}
    return {"survival": horizon, "fell": False, "forward": float(d.qpos[0]),
            "stone_errs": hit_errs, "min_h": min_h}


def irregular_stones(par: GaitParams, gaps=(0.20, 0.08, 0.20, 0.08, 0.20),
                     half=0.025):
    """A line of restricted footholds at *irregular* forward gaps — long/short
    alternating, so a single fixed stride+cadence cannot reach them all while
    keeping the DCM viable. Feet alternate ``+half_width`` / ``-half_width``."""
    xs = np.cumsum(gaps)
    return [(np.array([xs[k], par.half_width * (1.0 if k % 2 == 0 else -1.0)]),
             np.array([half, half])) for k in range(len(gaps))]


def viability_errors(plan: StepPlan, par: GaitParams, swing_side=1.0):
    """Per-step DCM viability error: how far the end-of-step DCM offset ``xi-u``
    strays from the periodic nominal ``b_nom``. Large = the gait is diverging."""
    sides = [swing_side * (-1.0) ** k for k in range(len(plan.T))]
    out = []
    for k in range(len(plan.T)):
        off = (plan.xi[k] - plan.u[k]) - _b_nominal(par, sides[k])
        out.append(float(np.linalg.norm(off)))
    return out


def compare_timing_on_stones(par: GaitParams | None = None, n=5, **stone_kw):
    """Plan the same irregular stepping stones with adaptive vs fixed step timing.
    Returns ``(stones, adaptive_plan, fixed_plan, summary_dict)``."""
    par = par or GaitParams()
    stones = irregular_stones(par, **stone_kw)[:n]
    p0 = np.array([0.0, -par.half_width])
    xi0 = _limit_cycle_dcm(par, p0[1], 1.0)
    fixed_par = GaitParams(**{**par.__dict__, "T_min": par.T_nom, "T_max": par.T_nom})
    adaptive = plan_steps(xi0, p0, par=par, n_steps=n, stones=stones, swing_side=1.0)
    fixed = plan_steps(xi0, p0, par=fixed_par, n_steps=n, stones=stones, swing_side=1.0)
    va, vf = viability_errors(adaptive, par), viability_errors(fixed, par)
    summary = {
        "adaptive_viab_mean": float(np.mean(va)), "adaptive_viab_max": float(np.max(va)),
        "fixed_viab_mean": float(np.mean(vf)), "fixed_viab_max": float(np.max(vf)),
        "adaptive_T": adaptive.T.tolist(), "fixed_T": fixed.T.tolist(),
    }
    return stones, adaptive, fixed, summary


if __name__ == "__main__":
    par = GaitParams()
    stones, adaptive, fixed, s = compare_timing_on_stones(par)
    print("adaptive step duration on irregular stepping stones "
          "(arXiv:2403.17136, no public code)\n")
    print(f"  {'stone x':>8}  {'adaptive T':>11} {'fixed T':>9}")
    for k in range(len(stones)):
        print(f"  {stones[k][0][0]:8.2f}  {adaptive.T[k]:10.3f}s {fixed.T[k]:8.3f}s")
    print(f"\n  DCM viability error (lower = stays balanced):")
    print(f"    adaptive timing: mean {s['adaptive_viab_mean']:.3f}  "
          f"max {s['adaptive_viab_max']:.3f}")
    print(f"    fixed timing:    mean {s['fixed_viab_mean']:.3f}  "
          f"max {s['fixed_viab_max']:.3f}  "
          f"({s['fixed_viab_mean'] / max(s['adaptive_viab_mean'], 1e-9):.1f}x worse)")
    print("\n  Both hit the stones; only adaptive timing keeps the DCM viable. "
          "Fixed\n  cadence must over/under-shoot the irregular gaps and the DCM "
          "diverges.")
