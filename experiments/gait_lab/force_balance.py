"""Force control, step one: an ankle-torque balance the position gait cannot do.

The whole substrate ladder (see the README's *Steering* section and the push
-recovery negative result) lands on the same wall: a *position*-controlled
humanoid cannot modulate the **ground-reaction force / ZMP**. Its ankles hold a
fixed commanded angle, so under a disturbance they cannot push back — the ankle
is the body's primary balance actuator and position control gives it away.

This is the foundational counter-demonstration: switch *only the ankles* to
**torque** mode (:meth:`G1Model.set_torque_mode`) and run the classic *ankle
strategy* — an ankle torque proportional to the torso lean (and its rate) that
drives the centre of pressure to arrest the fall — while every other joint still
holds the stand by position control. Same robot, same push; the only change is
that the ankles now exert *force* instead of tracking an *angle*.

    python3 force_balance.py --push 0.6

It reports, for a sweep of shove speeds, how long a position-controlled stand
survives versus the ankle-torque stand. This establishes the *foundation*
(torque actuation is now accessible in gait_lab via
:meth:`G1Model.set_torque_mode`) and is honest about how far it gets: a *naive*
ankle strategy does **not** beat a stiff position-held ankle for a standing
shove — a 500-stiffness position ankle resists a push well, and a simple
lean-feedback torque (even with gravity-comp feedforward and a compliant hold)
does not regulate the ground-reaction force well enough to win. The force-control
payoff shows up in *dynamic* balance, and getting it needs the full machinery —
CoM/ZMP tracking with contact-force optimisation, a hip strategy, and stepping —
i.e. whole-body control, not one feedback gain. That is the real next rung; this
script is the foothold (and the honest baseline) for it.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model

ANKLES = ["left_ankle_pitch_joint", "left_ankle_roll_joint",
          "right_ankle_pitch_joint", "right_ankle_roll_joint"]


def _push(model: G1Model, speed: float, theta: float) -> None:
    model.data.qvel[0] += speed * np.cos(theta)
    model.data.qvel[1] += speed * np.sin(theta)


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
    print("ankle balance under a shove: position-held ankles vs torque ankle strategy\n")
    print(f"  {'shove':>6} | {'position surv':>14} | {'torque surv':>12}")
    for speed in args.speeds:
        pos, tor = [], []
        for k in range(args.trials):
            theta = 2.0 * np.pi * k / args.trials
            pos.append(run_position(model, speed, theta, args.horizon, args.fall_height))
            tor.append(run_torque_ankle(model, speed, theta, args.horizon, args.fall_height))
        print(f"  {speed:5.1f} m/s | {np.mean(pos):7.2f}s (min {np.min(pos):.2f}) "
              f"| {np.mean(tor):6.2f}s (min {np.min(tor):.2f})")
    print("\n  Honest read: torque actuation works (the ankles are now force, not "
          "angle), but a naive ankle strategy does NOT beat the stiff position "
          "ankle for a standing shove — the payoff needs whole-body CoM/ZMP "
          "control (contact forces, hip + stepping strategies), the next rung.")


if __name__ == "__main__":
    main()
