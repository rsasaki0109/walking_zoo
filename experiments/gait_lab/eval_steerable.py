"""Evaluate the steerable RL gait: does it track a velocity + yaw command?

The straight ``rl-residual`` policy answers "can a learned residual break the
~3 s balance ceiling?". This one answers a different, harder question: can the
*same* residual idea be made to *follow a command* — walk at a requested forward
speed and turn at a requested yaw rate — so Nav2 can actually drive it?

This is the honest check. It rolls :class:`RLSteerableWalk` out over a grid of
commands and, for each, reports how long it stayed up, how far it walked, the
net heading change, and the mean tracking error of forward speed and yaw rate.
A genuinely steerable gait survives every command and tracks it *approximately*
(tight tracking off a coarse position-controlled CPG is hard — see the README).

    python3 eval_steerable.py
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model, GaitHarness
from gait_lab.controllers import Command, RLSteerableWalk, RLSteerableFootstepWalk


def _rollout(harness, make, cmd, perturb_seed=None):
    """Roll a steerable controller out under a fixed command, tracking the
    realised forward speed and yaw rate each sim step."""
    m = harness.model
    m.reset()
    if perturb_seed is not None:
        m.perturb(perturb_seed, 0.015)
    ctrl = make()
    ctrl.reset(m)

    steps = int(round(harness.horizon / m.timestep))
    settle = int(round(harness.settle / m.timestep))
    start_xy = start_yaw = None
    fell_at = None
    vx_err = wz_err = 0.0
    n = 0
    for i in range(steps):
        t = i * m.timestep
        obs = m.observe(t)
        m.data.ctrl[:] = ctrl.update(obs, cmd)
        m.step()
        h = float(m.data.qpos[2])
        if i == settle:
            start_xy = m.data.qpos[0:2].copy()
            start_yaw = float(m.observe(t).torso_rpy[2])
        if i > settle:
            o = m.observe(t)
            vx_err += abs(float(o.com_vel_xy[0]) - cmd.forward_speed)
            wz_err += abs(float(o.torso_ang_vel[2]) - cmd.yaw_rate)
            n += 1
        if fell_at is None and h < harness.fall_height:
            fell_at = t
            break
    if start_xy is None:
        start_xy = m.observe(0.0).base_pos_xy.copy()
        start_yaw = 0.0
    end_xy = m.data.qpos[0:2].copy()
    end_yaw = float(m.observe(0.0).torso_rpy[2])
    survival = fell_at if fell_at is not None else harness.horizon
    return {
        "survival": survival,
        "fell": fell_at is not None,
        "forward": float(end_xy[0] - start_xy[0]),
        "lateral": float(end_xy[1] - start_xy[1]),
        "dyaw": float(np.arctan2(np.sin(end_yaw - start_yaw),
                                 np.cos(end_yaw - start_yaw))),
        "vx_err": vx_err / max(n, 1),
        "wz_err": wz_err / max(n, 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--policy", default=None, help="path to the steerable policy npz")
    ap.add_argument("--footstep", action="store_true",
                    help="evaluate the footstep-substrate steerable policy "
                         "(RLSteerableFootstepWalk / rl_policy_steer_fs.npz)")
    args = ap.parse_args()

    model = G1Model()
    harness = GaitHarness(model, horizon=args.horizon)

    cls = RLSteerableFootstepWalk if args.footstep else RLSteerableWalk

    def make():
        return cls(args.policy)

    cmds = [
        ("stand",        Command(0.0, 0.0)),
        ("slow",         Command(0.25, 0.0)),
        ("cruise",       Command(0.40, 0.0)),
        ("turn left",    Command(0.25, 0.4)),
        ("turn right",   Command(0.25, -0.4)),
        ("arc left",     Command(0.35, 0.25)),
    ]

    print(f"steerable gait — horizon {args.horizon:.0f}s "
          f"(tracking is approximate; see README)\n")
    print(f"  {'command':<11} {'vx*':>5} {'yaw*':>5} | {'surv':>5} {'fwd':>6} "
          f"{'lat':>6} {'dyaw':>6} | {'vx_err':>6} {'wz_err':>6}")
    survs = []
    for label, cmd in cmds:
        r = _rollout(harness, make, cmd)
        survs.append(r["survival"])
        flag = "" if not r["fell"] else "  FELL"
        print(f"  {label:<11} {cmd.forward_speed:5.2f} {cmd.yaw_rate:5.2f} | "
              f"{r['survival']:5.2f} {r['forward']:+6.2f} {r['lateral']:+6.2f} "
              f"{np.degrees(r['dyaw']):+6.0f} | {r['vx_err']:6.3f} {r['wz_err']:6.3f}{flag}")

    full = sum(s >= args.horizon - 0.1 for s in survs)
    print(f"\n  {full}/{len(cmds)} commands walked the full horizon  "
          f"(worst survival {min(survs):.2f}s)")
    # Sign check: a +yaw command should turn one way, -yaw the other.
    rl = _rollout(harness, make, Command(0.25, 0.4))
    rr = _rollout(harness, make, Command(0.25, -0.4))
    turned = "yes" if rl["dyaw"] > 0.1 and rr["dyaw"] < -0.1 else "weak/none"
    print(f"  turning responds to command sign: {turned} "
          f"(left {np.degrees(rl['dyaw']):+.0f}deg, right {np.degrees(rr['dyaw']):+.0f}deg)")


if __name__ == "__main__":
    main()
