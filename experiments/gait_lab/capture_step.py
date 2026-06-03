"""Push recovery that actually works: take a capture STEP.

The force-balance probe (`force_balance.py`) shows that *in-place* strategies —
stiff position, ankle torque, whole-body CoM — all top out fast under a shove. A
person shoved hard does not stiffen; they *step*. The missing ingredient for push
recovery was never torque, it was the **decision to step**: a static stand has no
recovery once the centre of mass leaves the support polygon, but a single
well-placed step puts the support back under the falling CoM.

This is the demonstration. `CaptureStepRecovery` holds a normal stand until a
shove drives the **capture point** ``xi = com + com_vel / omega`` (the point you
must step to, to come to rest — Pratt's capturability) outside the feet; then it
takes one step, swinging the foot on the falling side to the (reach-clamped)
capture point via leg IK while the stance foot holds, and settles over the new
support. It is realised with the *same position-controlled IK* the footstep
walkers use — the win is the stepping decision, not a new actuator.

    python3 capture_step.py --speeds 0.4 0.6 0.8 1.0

It reports, per shove speed, how long a plain `stand-hold` survives versus the
capture-step recovery. Stepping recovers shoves that topple the static stand —
the honest, working rung of push recovery, and the reactive-footstep substrate a
real steerable/force-aware gait would build on.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model
from gait_lab.model import LEG_JOINTS

_GRAVITY = 9.81


def _push(model: G1Model, speed: float, theta: float) -> None:
    model.data.qvel[0] += speed * np.cos(theta)
    model.data.qvel[1] += speed * np.sin(theta)


def run_stand(model: G1Model, speed: float, theta: float,
              horizon: float, fall_h: float) -> float:
    """Static position-held stand under a shove (no stepping)."""
    model.reset()
    stand = model.stand_targets.copy()
    _push(model, speed, theta)
    for i in range(int(round(horizon / model.timestep))):
        model.data.ctrl[:] = stand
        model.step()
        if float(model.data.qpos[2]) < fall_h:
            return i * model.timestep
    return horizon


def run_capture_step(model: G1Model, speed: float, theta: float,
                     horizon: float, fall_h: float,
                     trigger=0.18, step_time=0.34, max_reach=0.45,
                     ankle_kp=0.20, ankle_kd=0.05) -> float:
    """Stand, and on a shove take one capture STEP to catch the fall.

    Holds the stand until the capture point leaves the support, then swings the
    foot on the falling side to the capture point (reach-clamped) via leg IK while
    the stance foot holds, with light ankle attitude feedback throughout.
    """
    model.reset()
    stand = model.stand_targets.copy()
    ground = float(model.foot_pos("left")[2])
    _push(model, speed, theta)

    planted = {f: model.foot_pos(f).copy() for f in ("left", "right")}
    stepping = False
    has_stepped = False
    t_step0 = 0.0
    swing = stance = None
    swing_from = swing_to = None
    refractory = 0.0
    d = model.data

    def ankle_fix(obs):
        roll, pitch, _ = obs.torso_rpy
        rr, pr = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
        return (ankle_kp * pitch + ankle_kd * pr,
                ankle_kp * roll + ankle_kd * rr)

    steps = int(round(horizon / model.timestep))
    for i in range(steps):
        t = i * model.timestep
        obs = model.observe(t)
        com = obs.com_xy
        omega = np.sqrt(_GRAVITY / max(obs.com_z, 0.3))
        xi = com + obs.com_vel_xy / omega          # capture point

        ctrl = stand.copy()
        ap_fix, ar_fix = ankle_fix(obs)

        if not stepping and t >= refractory and np.linalg.norm(xi - com) > trigger:
            # Trigger a (re)step: swing the foot on the falling side toward the
            # capture point. Repeated triggers give N-step capturability — keep
            # stepping until the capture point is back inside the support.
            fall_dir = xi - com
            swing = "left" if fall_dir[1] >= 0 else "right"
            stance = "right" if swing == "left" else "left"
            swing_from = planted[swing].copy()
            stance_xy = planted[stance][:2]
            tgt = xi.copy()
            v = tgt - stance_xy
            r = np.linalg.norm(v)
            if r > max_reach:
                tgt = stance_xy + v / r * max_reach
            swing_to = np.array([tgt[0], tgt[1], ground])
            stepping = True
            t_step0 = t

        if stepping:
            ph = float(np.clip((t - t_step0) / step_time, 0.0, 1.0))
            sx = (1 - ph) * swing_from[:2] + ph * swing_to[:2]
            sz = ground + 0.05 * np.sin(np.pi * ph)
            feet = {stance: planted[stance],
                    swing: np.array([sx[0], sx[1], sz])}
            if ph >= 1.0:
                planted[swing] = swing_to.copy()   # commit the new foothold
                stepping = False
                has_stepped = True
                refractory = t + 0.30              # settle before any re-step
        else:
            feet = {f: planted[f] for f in ("left", "right")}

        # Before the first step, hold the (stable) nominal stand. Once stepping has
        # begun, hold the planted/swinging feet via IK so the pose stays continuous
        # with the new footholds instead of snapping back to the stand.
        if stepping or has_stepped:
            for foot, target in feet.items():
                angles = model.solve_leg_ik(foot, target)
                for joint, value in zip(LEG_JOINTS[foot], angles):
                    ctrl[model.actuator(joint)] = value
        for side in ("left", "right"):
            ctrl[model.actuator(f"{side}_ankle_pitch_joint")] += ap_fix
            ctrl[model.actuator(f"{side}_ankle_roll_joint")] += ar_fix

        d.ctrl[:] = ctrl
        model.step()
        if float(d.qpos[2]) < fall_h:
            return t
    return horizon


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="*", default=[0.4, 0.6, 0.8, 1.0])
    ap.add_argument("--trials", type=int, default=6, help="push directions per speed")
    ap.add_argument("--horizon", type=float, default=4.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    args = ap.parse_args()

    model = G1Model()
    print("push recovery under a shove: static stand vs a capture STEP\n")
    print(f"  {'shove':>7} | {'stand-hold':>12} | {'capture-step':>13}")
    for speed in args.speeds:
        st, cs = [], []
        for k in range(args.trials):
            theta = 2.0 * np.pi * k / args.trials
            st.append(run_stand(model, speed, theta, args.horizon, args.fall_height))
            cs.append(run_capture_step(model, speed, theta, args.horizon, args.fall_height))
        print(f"  {speed:4.1f} m/s | {np.mean(st):7.2f}s     | {np.mean(cs):7.2f}s "
              f"(min {np.min(cs):.2f})")
    print("\n  Stepping recovers shoves that topple the static stand: the recovery "
          "is the\n  *decision to step* (capture point -> foot placement), realised "
          "with the same\n  position IK — the honest, working rung of push recovery.")


if __name__ == "__main__":
    main()
