"""Force control, step one: an ankle-torque balance the position gait cannot do.

The whole substrate ladder (see the README's *Steering* section and the push
-recovery negative result) lands on the same wall: a *position*-controlled
humanoid cannot modulate the **ground-reaction force / ZMP**. Its ankles hold a
fixed commanded angle, so under a disturbance they cannot push back — the ankle
is the body's primary balance actuator and position control gives it away.

This switches leg joints to **torque** mode (:meth:`G1Model.set_torque_mode`) and
pits two force strategies against the stiff position stand under the same shove:
an **ankle strategy** (ankle torque opposing the torso lean) and a **whole-body
CoM** controller (every leg joint in torque mode, a restoring CoM force mapped to
joint torques through the CoM Jacobian, gravity-compensated each step via
``mj_inverse``).

    python3 force_balance.py --speeds 0.3 0.5 0.7

It establishes the *foundation* (torque actuation in gait_lab) and is honest about
how far it gets: **neither** naive force strategy beats the stiff position stand
for a *standing* shove. Two instructive reasons — stiffness is itself resistance
(a 500-gain ankle just resists), and without the **contact** Jacobian the
unconstrained CoM Jacobian barely couples leg torque to CoM motion (the CoM gain
has almost no effect). The force-control payoff is *dynamic*: a contact-constrained
whole-body controller (force-distribution QP) plus a capture **step** — catching a
big shove by stepping, which position control cannot decide to do. That is the
real next rung; this script is its foothold and honest floor.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model

ANKLES = ["left_ankle_pitch_joint", "left_ankle_roll_joint",
          "right_ankle_pitch_joint", "right_ankle_roll_joint"]

from gait_lab.model import LEG_ACTUATORS  # noqa: E402


def _push(model: G1Model, speed: float, theta: float) -> None:
    model.data.qvel[0] += speed * np.cos(theta)
    model.data.qvel[1] += speed * np.sin(theta)


def run_wbc_com(model: G1Model, speed: float, theta: float,
                horizon: float, fall_h: float,
                kp_com=900.0, kd_com=160.0,
                post_kp=160.0, post_kd=8.0) -> float:
    """Whole-body CoM balance: every leg joint in TORQUE mode, torques computed to
    drive the *centre of mass* back over the feet — the force-aware strategy the
    ankle-only one could not do.

    Each step it forms a desired horizontal force on the CoM,
    ``F = kp*(com_ref - com) - kd*com_vel``, and maps it to leg-joint torques with
    the CoM Jacobian transpose ``tau = J_com[:, leg_dofs]^T @ F`` — distributing
    the restoring effort across ankle + knee + hip the way a person catches a
    shove with the whole leg, not just the ankle. A gravity-comp feedforward (the
    static stand-holding torque) keeps the legs from sagging."""
    import mujoco

    M = model.model
    leg_acts = [model.actuator(n) for n in LEG_ACTUATORS]
    leg_dofs = [int(M.jnt_dofadr[M.actuator_trnid[a, 0]]) for a in leg_acts]
    leg_qadr = [int(M.jnt_qposadr[M.actuator_trnid[a, 0]]) for a in leg_acts]
    stand_q = np.array([float(model.stand_qpos[q]) for q in leg_qadr])

    model.set_position_mode(LEG_ACTUATORS)
    model.reset()
    stand = model.stand_targets.copy()
    for _ in range(int(round(0.5 / model.timestep))):
        model.data.ctrl[:] = stand
        model.step()
    com_ref = model.data.subtree_com[0, :2].copy()

    model.set_torque_mode(LEG_ACTUATORS)
    model.reset()
    _push(model, speed, theta)
    d = model.data
    jacp = np.zeros((3, M.nv))
    try:
        for i in range(int(round(horizon / model.timestep))):
            # Contact-consistent gravity/dynamics compensation: the generalized
            # force that holds the *current* configuration static (qacc=0) given
            # the actual contacts. This is what a fixed stand feedforward cannot
            # do once the robot moves — it is why the naive ff sagged.
            mujoco.mj_forward(M, d)
            d.qacc[:] = 0.0
            mujoco.mj_inverse(M, d)
            grav = d.qfrc_inverse.copy()
            mujoco.mj_jacSubtreeCom(M, d, jacp, 0)
            com = d.subtree_com[0, :2]
            com_vel = d.subtree_linvel[0, :2] if hasattr(d, "subtree_linvel") \
                else d.qvel[:2]
            F = np.zeros(3)
            F[:2] = kp_com * (com_ref - com) - kd_com * np.asarray(com_vel)
            tau_com = jacp[:, leg_dofs].T @ F
            # Posture task (regularises the null space so the legs do not collapse
            # while the CoM task balances) + gravity-comp feedforward.
            q = np.array([float(d.qpos[qa]) for qa in leg_qadr])
            qd = np.array([float(d.qvel[dv]) for dv in leg_dofs])
            tau_post = post_kp * (stand_q - q) - post_kd * qd
            ctrl = stand.copy()
            for k, a in enumerate(leg_acts):
                ctrl[a] = float(grav[leg_dofs[k]]) + float(tau_post[k]) + float(tau_com[k])
            d.ctrl[:] = ctrl
            model.step()
            if float(d.qpos[2]) < fall_h:
                return i * model.timestep
        return horizon
    finally:
        model.set_position_mode(LEG_ACTUATORS)


def run_position(model: G1Model, speed: float, theta: float,
                 horizon: float, fall_h: float) -> float:
    """Position-controlled stand under a shove: ankles hold their angle."""
    model.set_position_mode(ANKLES)
    model.reset()
    stand = model.stand_targets.copy()
    _push(model, speed, theta)
    steps = int(round(horizon / model.timestep))
    for i in range(steps):
        model.data.ctrl[:] = stand
        model.step()
        if float(model.data.qpos[2]) < fall_h:
            return i * model.timestep
    return horizon


def run_torque_ankle(model: G1Model, speed: float, theta: float,
                     horizon: float, fall_h: float,
                     kp_pitch=260.0, kd_pitch=22.0,
                     kp_roll=300.0, kd_roll=24.0,
                     hold_kp=140.0, hold_kd=6.0) -> float:
    """Stand with the ankles in TORQUE mode running an ankle-strategy balance: the
    ankle torque is a *compliant* posture hold (so the foot can roll) PLUS a term
    that opposes the torso lean to drive the centre of pressure under the falling
    CoM. Every other joint still holds the stand by position control."""
    ap = model.actuator("left_ankle_pitch_joint")
    ar = model.actuator("left_ankle_roll_joint")
    ap_r = model.actuator("right_ankle_pitch_joint")
    ar_r = model.actuator("right_ankle_roll_joint")

    # Gravity-comp feedforward: settle the stand under position control and read
    # the static holding torque each ankle needs (switching to torque mode would
    # otherwise drop it and the ankles would sag). The balance torque rides on top.
    model.set_position_mode(ANKLES)
    model.reset()
    stand = model.stand_targets.copy()
    for _ in range(int(round(0.5 / model.timestep))):
        model.data.ctrl[:] = stand
        model.step()
    ff = model.data.actuator_force.copy()

    # qpos / qvel addresses for each ankle joint (for the compliant hold term).
    M = model.model
    qadr = {a: int(M.jnt_qposadr[M.actuator_trnid[a, 0]]) for a in (ap, ar, ap_r, ar_r)}
    vadr = {a: int(M.jnt_dofadr[M.actuator_trnid[a, 0]]) for a in (ap, ar, ap_r, ar_r)}
    stand_q = {a: float(model.stand_qpos[qadr[a]]) for a in qadr}

    model.set_torque_mode(ANKLES)
    model.reset()
    _push(model, speed, theta)
    steps = int(round(horizon / model.timestep))
    d = model.data

    def hold(a):  # gravity-comp ff + compliant posture hold (as torque)
        return (ff[a] + hold_kp * (stand_q[a] - float(d.qpos[qadr[a]]))
                - hold_kd * float(d.qvel[vadr[a]]))

    try:
        for i in range(steps):
            obs = model.observe(i * model.timestep)
            roll, pitch, _ = obs.torso_rpy
            rr, pr = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
            # Ankle strategy: torque opposes the lean (and its rate). A forward
            # lean (+pitch) commands an ankle torque that rotates the body back.
            tau_pitch = -(kp_pitch * pitch + kd_pitch * pr)
            tau_roll = -(kp_roll * roll + kd_roll * rr)
            ctrl = stand.copy()
            ctrl[ap] = hold(ap) + tau_pitch
            ctrl[ap_r] = hold(ap_r) + tau_pitch
            ctrl[ar] = hold(ar) + tau_roll
            ctrl[ar_r] = hold(ar_r) + tau_roll
            d.ctrl[:] = ctrl
            model.step()
            if float(d.qpos[2]) < fall_h:
                return i * model.timestep
        return horizon
    finally:
        model.set_position_mode(ANKLES)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="*", default=[0.3, 0.5, 0.7, 0.9])
    ap.add_argument("--trials", type=int, default=4, help="push directions per speed")
    ap.add_argument("--horizon", type=float, default=4.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    args = ap.parse_args()

    model = G1Model()
    print("balance under a shove — three actuation strategies on the same robot:\n"
          "  position : every joint a stiff position servo (the gait_lab default)\n"
          "  ankle    : ankles in torque mode running an ankle strategy\n"
          "  wbc-com  : every leg joint in torque mode, restoring force mapped\n"
          "             through the CoM Jacobian (gravity comp via mj_inverse)\n")
    print(f"  {'shove':>7} | {'position':>10} | {'ankle':>10} | {'wbc-com':>10}")
    for speed in args.speeds:
        pos, tor, wbc = [], [], []
        for k in range(args.trials):
            theta = 2.0 * np.pi * k / args.trials
            pos.append(run_position(model, speed, theta, args.horizon, args.fall_height))
            tor.append(run_torque_ankle(model, speed, theta, args.horizon, args.fall_height))
            wbc.append(run_wbc_com(model, speed, theta, args.horizon, args.fall_height))
        print(f"  {speed:4.1f} m/s | {np.mean(pos):7.2f}s  | {np.mean(tor):7.2f}s  "
              f"| {np.mean(wbc):7.2f}s")
    print("\n  Honest read: torque actuation works (the legs are now force, not\n"
          "  angle), but neither a naive ankle strategy NOR a whole-body CoM-Jacobian\n"
          "  controller beats a stiff position stand for a *standing* shove. Two\n"
          "  reasons, both instructive: (1) stiffness IS resistance — a 500-gain\n"
          "  ankle simply resists a push; (2) without the CONTACT Jacobian the\n"
          "  unconstrained CoM Jacobian barely couples leg torque to CoM motion\n"
          "  (the com gain has ~no effect). The real payoff is dynamic: a proper\n"
          "  contact-constrained WBC (force-distribution QP) plus a capture STEP —\n"
          "  catching a big shove by stepping, which position control cannot decide\n"
          "  to do. That is the next rung; this is its foundation and honest floor.")


if __name__ == "__main__":
    main()
