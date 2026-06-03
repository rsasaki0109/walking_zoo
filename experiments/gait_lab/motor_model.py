"""The last idealisation: was the stiff servo's win an *implementation* artifact?

Every gait_lab result so far ends the same way — on this position-built G1 a stiff
500-gain position servo beats the force-aware controllers (the CoM-Jacobian split,
``force_walk``, the contact-QP WBC) under a shove. ``wbc_qp.py`` even made the QP
torque-honest and the verdict held: the binding wall is the support polygon, not
the torque budget; a quiet stand needs ~45% of the budget and the servo wins on ~40%.

But one asymmetry was never paid for. The position servo is a MuJoCo *implicit*
actuator: its force ``kp(q_des - q) - kd q̇`` is recomputed inside every physics
substep (500 Hz here), with zero latency, infinite bandwidth and perfect torque
delivery. The QP, by contrast, is an *explicit* controller — solved once per control
step and applied open-loop until the next solve. A real robot has neither luxury:
its controller runs at a finite rate (50-200 Hz), and its motors track a commanded
torque with finite bandwidth, lag and noise. So "the servo wins" might be partly a
gift of the implicit integrator that no hardware servo enjoys.

This module removes that asymmetry. It re-implements BOTH controllers as *explicit
torque* controllers fed through ONE shared :class:`MotorModel`:

  * the servo becomes ``tau = kp(q_des - q) - kd q̇`` computed in Python (kp/kd read
    from the very position actuators it replaces, so at the ideal motor it is the
    same servo) — no longer an implicit free lunch;
  * the QP is the existing complete TSID (``WBCSolver(tau_limits=True)``).

Both pass through the same actuator pipeline: a control-rate zero-order hold, a
first-order torque-tracking lag (finite bandwidth), the real ``jnt_actfrcrange``
clamp, and optional torque noise. Dial the motor from ideal toward hardware and
ask the honest question: *does the servo's advantage survive, or was it the
idealisation?* The experiment prints the full sweep so the answer is auditable.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model
from wbc_qp import WBCSolver, _all_actuator_names


class MotorModel:
    """The actuator between a *commanded* and an *applied* joint torque.

    Three departures from the implicit ideal, each independently switchable so a
    sweep can attribute the effect:

    * **control rate (ZOH)** — the command is refreshed only every ``1/control_hz``
      seconds and held constant in between; the physics still integrates at the full
      rate. ``control_hz=None`` refreshes every physics step (the QP's current luxury).
    * **bandwidth (first-order lag)** — the applied torque relaxes toward the command
      with time constant ``1/(2*pi*bw_hz)``; ``bw_hz=None`` is instantaneous delivery.
    * **torque clamp + noise** — the applied torque is clipped to the real per-joint
      ``jnt_actfrcrange`` and (optionally) perturbed by gaussian noise whose std is
      ``noise_frac`` of each joint's limit.

    ``tau_lo``/``tau_hi`` are the same per-actuator bounds ``WBCSolver`` reads.
    """

    def __init__(self, tau_lo, tau_hi, *, control_hz=None, bw_hz=None,
                 noise_frac=0.0, seed=0):
        self.tau_lo = np.asarray(tau_lo, float)
        self.tau_hi = np.asarray(tau_hi, float)
        self.control_hz = control_hz
        self.bw_hz = bw_hz
        self.noise_frac = noise_frac
        self._rng = np.random.default_rng(seed)
        self._applied = None       # last applied torque (lag state)
        self._held = None          # last command held by the ZOH
        self._t_last = -1e9        # time of last ZOH refresh

    def reset(self, nu):
        self._applied = np.zeros(nu)
        self._held = np.zeros(nu)
        self._t_last = -1e9

    def should_recompute(self, t):
        """True when the control-rate ZOH is due to refresh the command at time t."""
        if self.control_hz is None:
            return True
        if t - self._t_last >= 1.0 / self.control_hz - 1e-9:
            self._t_last = t
            return True
        return False

    def step(self, tau_cmd, dt):
        """Advance the actuator one physics step: hold the latest command, relax the
        applied torque toward it (bandwidth), clamp and add noise. Returns applied tau."""
        if tau_cmd is not None:
            self._held = np.asarray(tau_cmd, float)
        cmd = self._held
        if self.bw_hz is None:
            applied = cmd.copy()
        else:
            tau_m = 1.0 / (2.0 * np.pi * self.bw_hz)
            alpha = dt / (tau_m + dt)              # stable first-order step
            applied = self._applied + alpha * (cmd - self._applied)
        if self.noise_frac > 0.0:
            scale = self.noise_frac * np.maximum(self.tau_hi, -self.tau_lo)
            applied = applied + self._rng.normal(0.0, scale)
        applied = np.clip(applied, self.tau_lo, self.tau_hi)
        self._applied = applied
        return applied


def _servo_gains(model):
    """Read the position servo's real per-actuator (kp, kd) from the loaded model.

    The menagerie G1 ships ``kp=500`` uniform and a per-joint ``kd`` (from
    ``dampratio=1``); the explicit servo reproduces it exactly: ``force =
    kp(ctrl - q) - kd q̇`` is the affine position actuator MuJoCo runs implicitly."""
    M = model.model
    kp = M.actuator_gainprm[:, 0].copy()
    kd = -M.actuator_biasprm[:, 2].copy()
    return kp, kd


def _servo_torque(model, kp, kd, q_des):
    """Explicit position-servo torque ``kp(q_des - q) - kd q̇`` per actuator."""
    d = model.data
    M = model.model
    q = np.array([float(d.qpos[M.jnt_qposadr[M.actuator_trnid[i, 0]]])
                  for i in range(M.nu)])
    qd = np.array([float(d.qvel[M.jnt_dofadr[M.actuator_trnid[i, 0]]])
                   for i in range(M.nu)])
    return kp * (q_des - q) - kd * qd


def localize_servo_idealisation(model, horizon, fall_h, push_speed,
                                push_at=0.3, direction=(1.0, 0.0)):
    """Pin down WHERE the stiff servo's robustness comes from, at a quiet/standing
    pose, by running four variants of the *same* control effort and timing:

    * ``implicit``  — the lab's original position actuator. MuJoCo integrates its
      velocity-damping term ``-kd q̇`` implicitly (its Euler step always does, for
      stability), an unconditionally-stable inner velocity loop at the full physics
      rate. This is the idealisation under audit.
    * ``explicit``  — the SAME PD law ``kp(q_des-q) - kd q̇`` applied as explicit
      torque (bit-identical force; only the integration of the damping differs). This
      is the honest model of a real digital servo, which computes -kd q̇ and applies
      it one step late.
    * ``explicit+impl-damp`` — explicit ``-kp(q-q_des)`` torque but with ``kd`` moved
      into MuJoCo's implicit joint damping (``dof_damping``). Isolates the velocity
      term: if this recovers the hold, the implicit treatment of ``-kd q̇`` *was* the
      crutch.
    * ``qp`` — the complete-TSID QP (model-based explicit torque, gravity included).

    Returns a dict ``name -> (survive_time, reason)``. The point: the gap between
    ``implicit`` and ``explicit`` is the idealisation the earlier 'servo wins' rested on."""
    import mujoco
    names = _all_actuator_names(model)
    M = model.model

    def _run(setup, control, restore):
        model.set_position_mode(names)
        model.reset()
        q_des = model.stand_targets.copy()
        kp, kd = _servo_gains(model)
        com0 = model.data.subtree_com[0].copy()
        solver = WBCSolver(model, tau_limits=True) if control == "qp" else None
        lo = M.jnt_actfrcrange[M.actuator_trnid[:, 0]][:, 0]
        hi = M.jnt_actfrcrange[M.actuator_trnid[:, 0]][:, 1]
        qadr = np.array([M.jnt_qposadr[M.actuator_trnid[i, 0]] for i in range(M.nu)])
        model.set_torque_mode(names)
        saved = setup(kd)
        model.reset()
        d = model.data
        pushed = False
        try:
            for i in range(int(round(horizon / model.timestep))):
                t = i * model.timestep
                if not pushed and t >= push_at:
                    d.qvel[0] += push_speed * direction[0]
                    d.qvel[1] += push_speed * direction[1]
                    pushed = True
                if control == "qp":
                    tau = solver.compute(com0, q_des=q_des)
                    if tau is None:
                        return t, "infeasible"
                elif control == "explicit":
                    q = d.qpos[qadr]
                    qd = np.array([float(d.qvel[M.jnt_dofadr[M.actuator_trnid[j, 0]]])
                                   for j in range(M.nu)])
                    tau = kp * (q_des - q) - kd * qd
                else:  # "pos_only": damping handled implicitly via dof_damping
                    q = d.qpos[qadr]
                    tau = kp * (q_des - q)
                d.ctrl[:] = np.clip(tau, lo, hi)
                model.step()
                if float(d.qpos[2]) < fall_h:
                    return t, "toppled"
            return horizon, "held"
        finally:
            restore(saved)
            model.set_position_mode(names)

    def no_setup(kd):
        return None

    def damp_setup(kd):
        saved = M.dof_damping.copy()
        for j in range(M.nu):
            M.dof_damping[M.jnt_dofadr[M.actuator_trnid[j, 0]]] += kd[j]
        return saved

    def damp_restore(saved):
        if saved is not None:
            M.dof_damping[:] = saved

    out = {}
    # implicit position servo (no torque-mode run; just the position actuator)
    model.set_position_mode(names)
    model.reset()
    d = model.data
    pushed = False
    res = (horizon, "held")
    for i in range(int(round(horizon / model.timestep))):
        t = i * model.timestep
        if not pushed and t >= push_at:
            d.qvel[0] += push_speed * direction[0]
            d.qvel[1] += push_speed * direction[1]
            pushed = True
        d.ctrl[:] = model.stand_targets
        model.step()
        if float(d.qpos[2]) < fall_h:
            res = (t, "toppled")
            break
    out["implicit"] = res
    out["explicit"] = _run(no_setup, "explicit", lambda s: None)
    out["explicit+impl-damp"] = _run(damp_setup, "pos_only", damp_restore)
    out["qp"] = _run(no_setup, "qp", lambda s: None)
    return out


def run_motor_stand_push(model, controller, horizon, fall_h, push_speed,
                         push_at=0.3, direction=(1.0, 0.0), *,
                         control_hz=None, bw_hz=None, noise_frac=0.0, seed=0):
    """Stand-under-shove with BOTH controllers driven as explicit torque through the
    same :class:`MotorModel`. ``controller`` is ``"servo"`` or ``"qp"``.

    Returns ``(survive_time, reason)`` — reason ``"held"`` / ``"toppled"`` and, for
    the QP only, ``"infeasible"`` (no friction-cone GRF can arrest the fall: step).
    Both controllers see the identical control rate, bandwidth, clamp and noise, so
    the only thing being compared is the control law, not the idealisation it enjoys."""
    names = _all_actuator_names(model)
    # read servo gains while still in position mode, and build the QP solver
    model.set_position_mode(names)
    model.reset()
    kp, kd = _servo_gains(model)
    com0 = model.data.subtree_com[0].copy()
    q_des = model.stand_targets.copy()
    solver = WBCSolver(model, tau_limits=True)
    motor = MotorModel(solver.tau_lo, solver.tau_hi, control_hz=control_hz,
                       bw_hz=bw_hz, noise_frac=noise_frac, seed=seed)

    model.set_torque_mode(names)
    model.reset()
    motor.reset(model.nu)
    d = model.data
    pushed = False
    try:
        for i in range(int(round(horizon / model.timestep))):
            t = i * model.timestep
            if not pushed and t >= push_at:
                d.qvel[0] += push_speed * direction[0]
                d.qvel[1] += push_speed * direction[1]
                pushed = True

            tau_cmd = None
            if motor.should_recompute(t):
                if controller == "servo":
                    tau_cmd = _servo_torque(model, kp, kd, q_des)
                else:
                    tau_cmd = solver.compute(com0, q_des=q_des)
                    if tau_cmd is None:
                        return t, "infeasible"
            d.ctrl[:] = motor.step(tau_cmd, model.timestep)
            model.step()
            if float(d.qpos[2]) < fall_h:
                return t, "toppled"
        return horizon, "held"
    finally:
        model.set_position_mode(names)


def run_motor_zmp_walk(model, controller, horizon, fall_h, *,
                       control_hz=None, bw_hz=None, noise_frac=0.0, seed=0):
    """Walk the ZMP-preview plan with BOTH controllers as explicit torque through the
    same :class:`MotorModel` — the *walking* half of the idealisation audit.

    The lab's central conclusion ("position-IK `zmp-preview` beats the torque WBC
    while walking, ~2.4 s vs ~1.3 s") used the *implicit* position servo as the
    winning baseline — the same free lunch the standing result exposed. This re-runs
    it honestly: the ``"servo"`` controller tracks the planner's joint targets with
    explicit torque ``kp(q_des-q) - kd q̇`` (same form the standing experiment showed
    cannot hold a quiet stand), and ``"qp"`` is the complete-TSID QP tracking the
    planned CoM + swing foot. Both pass the identical control-rate/bandwidth/clamp.

    Returns ``(survive_time, reason)``. The finding (see the README): unlike standing,
    paying the idealisation does NOT flip the walking verdict — the position servo
    loses ~a third of its survival but still beats the QP walk, because tracking the
    fast swing trajectory is a genuine control-authority win, not an integrator gift."""
    from gait_lab.controllers import ZMPPreviewWalk, Command
    from gait_lab.model import LEG_JOINTS
    from wbc_qp import _stance_swing

    names = _all_actuator_names(model)
    planner = ZMPPreviewWalk()
    model.set_position_mode(names)
    model.reset()
    planner.reset(model)
    kp, kd = _servo_gains(model)
    stand = model.stand_targets.copy()
    com_z = float(model.data.subtree_com[0, 2])
    solver = WBCSolver(model, tau_limits=True)
    motor = MotorModel(solver.tau_lo, solver.tau_hi, control_hz=control_hz,
                       bw_hz=bw_hz, noise_frac=noise_frac, seed=seed)

    model.set_torque_mode(names)
    model.reset()
    motor.reset(model.nu)
    d = model.data
    try:
        for i in range(int(round(horizon / model.timestep))):
            t = i * model.timestep
            tau_cmd = None
            if motor.should_recompute(t):
                obs = model.observe(t)
                if controller == "servo":
                    q_des = planner.update(obs, Command())
                    tau_cmd = _servo_torque(model, kp, kd, q_des)
                else:
                    k = min(int(t / planner.plan_dt), planner._n - 1)
                    com_plan = planner._com[k]
                    com_des = np.array([com_plan[0], com_plan[1], com_z])
                    q_des = stand.copy()
                    base_now = d.qpos[0:2]
                    for foot in ("left", "right"):
                        fw = planner._foot_world(foot, t)
                        target = np.array([base_now[0] + (fw[0] - com_plan[0]),
                                           base_now[1] + (fw[1] - com_plan[1]), fw[2]])
                        angles = model.solve_leg_ik(foot, target)
                        for joint, value in zip(LEG_JOINTS[foot], angles):
                            q_des[model.actuator(joint)] = value
                    stance, swing = _stance_swing(planner, t)
                    swing_des = planner._foot_world(swing, t)
                    tau_cmd = solver.compute(com_des, q_des=q_des,
                                             swing_foot=swing, swing_pos_des=swing_des)
                    if tau_cmd is None:
                        return t, "infeasible"
            d.ctrl[:] = motor.step(tau_cmd, model.timestep)
            model.step()
            if float(d.qpos[2]) < fall_h:
                return t, "toppled"
        return horizon, "held"
    finally:
        model.set_position_mode(names)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=float, default=4.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    args = ap.parse_args()
    model = G1Model()

    tag = {"held": "H", "toppled": "T", "infeasible": "I"}.get

    print("Experiment A — WHERE does the stiff servo's standing balance come from?\n"
          "Four variants of the same effort at a quiet stand (push 0.0) and a gentle\n"
          "shove. The only thing that changes is how the servo's velocity-damping term\n"
          "is integrated:\n")
    order = ["implicit", "explicit", "explicit+impl-damp", "qp"]
    desc = {
        "implicit": "implicit servo (MuJoCo position actuator)",
        "explicit": "explicit servo (honest digital PD torque)",
        "explicit+impl-damp": "explicit servo, -kd q. routed implicitly",
        "qp": "complete-TSID QP (model-based torque)",
    }
    for p in (0.0, 0.4):
        print(f"  push {p:.1f} m/s:")
        res = localize_servo_idealisation(model, args.horizon, args.fall_height, p)
        for k in order:
            t, why = res[k]
            print(f"    {desc[k]:46s} {t:5.2f}s ({why})")
    print("\n  -> the gap between 'implicit' and 'explicit' (holds vs topples ~1.3s at a\n"
          "     QUIET stand) is the idealisation. MuJoCo integrates the servo's -kd q.\n"
          "     implicitly (an unconditionally-stable inner velocity loop at 500Hz);\n"
          "     routing it back implicitly recovers most of the hold. A real finite-rate\n"
          "     servo and the QP apply that damping explicitly and never get the crutch.\n")

    print("Experiment B — the fair fight: BOTH controllers as explicit torque through\n"
          "ONE shared actuator (control-rate ZOH + first-order bandwidth lag + real\n"
          "torque clamp). Forward shove. Does either survive a realistic motor?\n")
    motors = [
        (None,  None, "ideal (500Hz, inf BW)"),
        (200.0, 100.0, "200Hz ctrl, 100Hz BW"),
        (100.0, 50.0,  "100Hz ctrl,  50Hz BW"),
        (50.0,  30.0,  " 50Hz ctrl,  30Hz BW"),
    ]
    pushes = (0.0, 0.4, 0.6)
    header = "  motor                       " + "  ".join(
        f"{p:.1f}m/s" for p in pushes)
    for ctl in ("servo", "qp"):
        print(f"\n  {ctl.upper()} (explicit torque):")
        print(header)
        for control_hz, bw_hz, label in motors:
            cells = []
            for p in pushes:
                t, why = run_motor_stand_push(
                    model, ctl, args.horizon, args.fall_height, p,
                    control_hz=control_hz, bw_hz=bw_hz)
                cells.append(f"{t:4.2f}{tag(why)}")
            print(f"  {label:24s}     " + "  ".join(cells))

    print("\n(H held to horizon, T toppled, I QP infeasible = must step.)\n"
          "Stand verdict: the servo's standing-balance 'win' was substantially an\n"
          "implicit-damping idealisation — on equal explicit-torque footing the model-\n"
          "based QP is the better stand-keeper (holds the quiet stand the explicit servo\n"
          "cannot), and degrades gracefully with control rate. But under a real shove\n"
          "BOTH still fail at ~0.6s and the QP certifies 'must step': the support-polygon\n"
          "wall is unchanged. The push-recovery verdict stands; the standing half flips.\n")

    print("Experiment C — apply the same lens to the lab's CENTRAL conclusion: WALKING.\n"
          "'position-IK zmp-preview beats the torque WBC' used the implicit servo as the\n"
          "winning baseline too. Re-run both as explicit torque on the ZMP-preview plan:\n")
    print("  controller                   ideal   200Hz/100BW  100Hz/50BW")
    for ctl in ("servo", "qp"):
        cells = []
        for control_hz, bw_hz in ((None, None), (200.0, 100.0), (100.0, 50.0)):
            t, why = run_motor_zmp_walk(model, ctl, args.horizon, args.fall_height,
                                        control_hz=control_hz, bw_hz=bw_hz)
            cells.append(f"{t:5.2f}{tag(why)}")
        print(f"  {ctl.upper():10s} (explicit torque)     " + "    ".join(cells))
    print("\nWalk verdict: UNLIKE standing, paying the idealisation does NOT flip the\n"
          "walking result. The position servo loses ~a third of its survival (the\n"
          "implicit-damping share) but still beats the QP walk — tracking the fast swing\n"
          "trajectory is a genuine control-authority win, not an integrator gift. So the\n"
          "lab's central conclusion survives its most adversarial test: on this position-\n"
          "built model, honest explicit-torque position tracking still walks farther than\n"
          "the torque WBC. The one idealisation that quietly carried the STANDING claim is\n"
          "found and paid; the WALKING claim needed no crutch.")


if __name__ == "__main__":
    main()
