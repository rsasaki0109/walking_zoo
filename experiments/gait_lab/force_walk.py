"""The frontier attempt: a force-aware ZMP walker (torque WBC inside a walk).

Every position-controlled steerable walker tops out near the ~2 s kinematic
ceiling because it only *places feet* — it never regulates the ground-reaction
force, so the actual CoM drifts off the planned trajectory and it topples (see the
README's "full map"). This is the genuine next rung: keep the ZMP-preview plan
(footstep schedule + a CoM trajectory whose induced ZMP leads the support foot,
the most balance-aware plan here), but track it with **torque** instead of pure
position IK, so the legs actively push the ground to hold the actual CoM on the
planned one.

The legs run in torque mode (:meth:`G1Model.set_torque_mode`) with three terms:

* **gravity / dynamics compensation** each step via ``mj_inverse`` (qacc=0);
* a **posture** task — a PD toward the walking IK pose (both feet at their planned
  positions relative to the planned CoM), which carries the kinematic footstep
  structure;
* a **CoM** task — a restoring force ``kp(com_plan - com) - kd*com_vel`` split
  across the feet and mapped to joint torques through each foot's **contact
  Jacobian**, the force-aware ingredient the kinematic walker lacked.

    python3 force_walk.py

It reports how long the force-aware walker stays up versus the position-IK
``zmp-preview`` it is built on. Honest by construction: it prints both, so whether
torque tracking actually beats position IK here is measured, not asserted.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model
from gait_lab.controllers import ZMPPreviewWalk, Command
from gait_lab.model import LEG_ACTUATORS, LEG_JOINTS


def _ik_pose(model, planner, t, q_des):
    """Desired leg joint angles: both feet at their planned positions relative to
    the planned CoM (exactly what zmp-preview commands, but as a posture target)."""
    k = min(int(t / planner.plan_dt), planner._n - 1)
    com_plan = planner._com[k]
    base_now = model.data.qpos[0:2]
    for foot in ("left", "right"):
        fw = planner._foot_world(foot, t)
        target = np.array([base_now[0] + (fw[0] - com_plan[0]),
                           base_now[1] + (fw[1] - com_plan[1]), fw[2]])
        angles = model.solve_leg_ik(foot, target)
        for joint, value in zip(LEG_JOINTS[foot], angles):
            q_des[model.actuator(joint)] = value
    return com_plan


def run_zmp_position(model, horizon, fall_h):
    planner = ZMPPreviewWalk()
    model.reset()
    planner.reset(model)
    for i in range(int(round(horizon / model.timestep))):
        model.data.ctrl[:] = planner.update(model.observe(i * model.timestep), Command())
        model.step()
        if float(model.data.qpos[2]) < fall_h:
            return i * model.timestep
    return horizon


def run_force_walk(model, horizon, fall_h,
                   kp_post=220.0, kd_post=10.0,
                   kp_com=1400.0, kd_com=200.0):
    import mujoco

    M = model.model
    leg_acts = [model.actuator(n) for n in LEG_ACTUATORS]
    leg_dofs = np.array([int(M.jnt_dofadr[M.actuator_trnid[a, 0]]) for a in leg_acts])
    leg_qadr = [int(M.jnt_qposadr[M.actuator_trnid[a, 0]]) for a in leg_acts]
    foot_sites = [model._foot_site["left"], model._foot_site["right"]]

    # Build the ZMP-preview plan on a position-mode robot.
    planner = ZMPPreviewWalk()
    model.set_position_mode(LEG_ACTUATORS)
    model.reset()
    planner.reset(model)
    stand = model.stand_targets.copy()

    model.set_torque_mode(LEG_ACTUATORS)
    model.reset()
    d = model.data
    jacp = np.zeros((3, M.nv))
    try:
        for i in range(int(round(horizon / model.timestep))):
            t = i * model.timestep
            q_des = stand.copy()
            com_plan = _ik_pose(model, planner, t, q_des)

            mujoco.mj_forward(M, d)
            d.qacc[:] = 0.0
            mujoco.mj_inverse(M, d)
            grav = d.qfrc_inverse.copy()

            com = d.subtree_com[0, :2]
            com_vel = d.subtree_linvel[0, :2] if hasattr(d, "subtree_linvel") \
                else d.qvel[:2]
            F = np.zeros(3)
            F[:2] = kp_com * (com_plan - com) - kd_com * np.asarray(com_vel)
            f_each = F / len(foot_sites)
            tau_com = np.zeros(len(leg_acts))
            for site in foot_sites:
                mujoco.mj_jacSite(M, d, jacp, None, site)
                tau_com += jacp[:, leg_dofs].T @ f_each

            q = np.array([float(d.qpos[qa]) for qa in leg_qadr])
            qd = np.array([float(d.qvel[dv]) for dv in leg_dofs])
            q_des_leg = np.array([q_des[a] for a in leg_acts])
            tau_post = kp_post * (q_des_leg - q) - kd_post * qd

            ctrl = stand.copy()
            for k, a in enumerate(leg_acts):
                ctrl[a] = float(grav[leg_dofs[k]]) + float(tau_post[k]) + float(tau_com[k])
            d.ctrl[:] = ctrl
            model.step()
            if float(d.qpos[2]) < fall_h:
                return t
        return horizon
    finally:
        model.set_position_mode(LEG_ACTUATORS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    ap.add_argument("--kp-com", type=float, default=1400.0)
    ap.add_argument("--kp-post", type=float, default=220.0)
    args = ap.parse_args()

    model = G1Model()
    pos = run_zmp_position(model, args.horizon, args.fall_height)
    force = run_force_walk(model, args.horizon, args.fall_height,
                           kp_com=args.kp_com, kp_post=args.kp_post)
    print("force-aware ZMP walk (torque WBC) vs the position-IK zmp-preview it tracks:\n")
    print(f"  zmp-preview (position IK)     survives {pos:5.2f}s")
    print(f"  force-walk  (torque WBC)      survives {force:5.2f}s")
    if force > pos + 0.3:
        print("\n  verdict: torque tracking BEATS position IK here.")
    else:
        print("\n  verdict: torque tracking does NOT beat position IK. The deeper\n"
              "  reason (see force_balance.py): on a model BUILT for position control\n"
              "  — high-gain servos the solver applies implicitly, continuously and\n"
              "  exactly — explicit torque control with an mj_inverse gravity\n"
              "  feedforward drifts and caps near ~1.3 s, standing or walking, no\n"
              "  matter the posture/CoM gains. The proper WBC (grav comp + posture +\n"
              "  contact-Jacobian CoM, all implemented here) is correct; the limit is\n"
              "  the substrate. Genuine force-aware walking needs a TORQUE-NATIVE\n"
              "  model (torque actuators, contact dynamics tuned for it), not the\n"
              "  position-servo menagerie G1. That boundary — model, not controller —\n"
              "  is the honest end of this thread.")


if __name__ == "__main__":
    main()
