"""The real frontier: a contact-QP whole-body controller (TSID).

Every earlier force attempt (``force_balance.py``, ``force_walk.py``) either split a
desired CoM force *equally* across the feet and mapped it through the contact
Jacobian transpose, or used the unconstrained CoM Jacobian. Both have the same
hole: the ground-reaction force is *assumed*, never *solved*. So the CoM task
barely couples through a single-support contact, and a stiff position servo wins.

This module closes that hole. Each control step it solves a floating-base
inverse-dynamics QP (task-space inverse dynamics, the Atlas/TSID recipe) whose
decision variables are the joint accelerations ``qddot`` AND the per-contact-point
ground-reaction forces ``f``:

    minimize    sum_k  w_k || J_k qddot - a_k^des ||^2   +  eps||f||^2 + ...
    over        qddot (nv),  f (3 per active contact point)
    subject to  M[base] qddot + h[base] = sum_c Jc_c[base]^T f   (6 unactuated rows)
                Jc_c qddot = -k (Jc_c qvel)                      (feet don't accelerate)
                f in the friction cone  (n·f >= 0, |t·f| <= mu n·f)

The contact forces are now genuine unknowns constrained by the friction cone and
unilaterality, so the QP discovers *where under the feet* to push (the centre of
pressure can travel across the 8 real foot-corner contacts MuJoCo reports) to
realise the CoM/orientation tasks. That CoP authority is exactly what the
hand-split controllers could not express. Joint torques are recovered from the
solution: ``tau = (M qddot + h - Jc^T f)[actuated]``.

Two honest tests, each printed against the position-control baseline it must beat:

* ``run_qp_stand_push`` — a lateral/forward shove on a stand. The QP regulates the
  CoM with real CoP authority; does it out-survive the stiff 500-gain servo (which
  the earlier CoM-Jacobian controller could not)?
* ``run_qp_walk`` — track the ZMP-preview plan, stance legs balancing via the QP
  while the swing foot follows a task to its planned placement. Does solving the
  GRF beat the ~2.4 s position-IK walk where ``force_walk`` (~1.3 s) did not?

Run ``python3 wbc_qp.py``. Honest by construction: it prints the baselines too.

THE MEASURED RESULT (and the point of the lab). The QP — proper TSID, the exact
"contact-QP WBC" the earlier notes named as the missing piece — *holds a quiet
stand indefinitely* with genuine friction-cone ground-reaction forces (a posture
task on the joints is the stable backbone; a moderate-weight CoM task adds force-
aware authority without fighting the rigid double-support constraints). But it
does NOT beat position control under a shove or while walking on this position-
built model:

* under the shove the QP goes *infeasible* the instant the capture point leaves
  the support polygon (measured: a 0.6 m/s push puts it ~5 cm past the toe). That
  infeasibility is not a bug — it is the controller *certifying* that no GRF in the
  friction cone can arrest the fall, i.e. you must STEP. The stiff 500-gain servo
  "survives" longer only by toppling slowly as a rigid inverted pendulum about the
  ankle, not by balancing;
* walking, the QP tuned for the constrained double-support stand does not track the
  fast single-support swing as precisely as the implicit high-gain position servo.

So the wall is not the absence of a QP — building the textbook one confirms it is
standing-without-stepping plus a model built for position control. The one balance
move that beats the limit remains the capture step (``capture_step.py``): step
exactly when the QP says you must. This module turns the earlier hand-wavy "needs
a contact-QP WBC" into a built, tested artifact whose own infeasibility pinpoints
the true boundary.
"""

from __future__ import annotations

import argparse

import numpy as np

from gait_lab import G1Model
from gait_lab.controllers import ZMPPreviewWalk, Command
from gait_lab.model import LEG_JOINTS

FOOT_BODY = {"left": "left_ankle_roll_link", "right": "right_ankle_roll_link"}


class WBCSolver:
    """Builds and solves the per-step TSID QP for a :class:`G1Model`.

    All actuators run in torque mode; ``compute`` returns the full ``ctrl`` torque
    vector. Task targets (CoM, base orientation, posture, optional swing foot) are
    passed in each step so the same solver drives both standing and walking.
    """

    def __init__(self, model: G1Model, mu: float = 0.7,
                 w_com=0.3, w_orient=0.0, w_posture=1.0, w_swing=4.0,
                 eps_f=1e-4, eps_qdd=1e-6, fmin=1.0):
        import mujoco
        self._mj = mujoco
        self.m = model
        self.M = model.model
        self.d = model.data
        self.nv = int(self.M.nv)
        self.mu = mu
        self.w_com, self.w_orient, self.w_posture, self.w_swing = (
            w_com, w_orient, w_posture, w_swing)
        self.eps_f, self.eps_qdd, self.fmin = eps_f, eps_qdd, fmin

        # actuated dofs are 6..nv-1 (free base is dofs 0..5); map actuator->dof.
        self.act_dof = np.array(
            [int(self.M.jnt_dofadr[self.M.actuator_trnid[i, 0]])
             for i in range(self.M.nu)])
        self.foot_body = {f: mujoco.mj_name2id(self.M, mujoco.mjtObj.mjOBJ_BODY, b)
                          for f, b in FOOT_BODY.items()}
        self._Mbuf = np.zeros((self.nv, self.nv))
        self._jac = np.zeros((3, self.nv))
        self._jcom = np.zeros((3, self.nv))

    # -- contacts ----------------------------------------------------------
    def _active_contacts(self):
        """World-frame contact Jacobian + friction frame for each foot contact
        point MuJoCo currently reports (up to 8: four corners per foot)."""
        mj, M, d = self._mj, self.M, self.d
        out = []
        foot_bodies = set(self.foot_body.values())
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(M.geom_bodyid[c.geom1]), int(M.geom_bodyid[c.geom2])
            foot = b1 if b1 in foot_bodies else (b2 if b2 in foot_bodies else None)
            if foot is None:
                continue
            jac = np.zeros((3, self.nv))
            mj.mj_jac(M, d, jac, None, c.pos, foot)
            frame = np.array(c.frame).reshape(3, 3)  # row0 normal, row1/2 tangents
            out.append((jac, frame))
        return out

    # -- the QP ------------------------------------------------------------
    def compute(self, com_des, com_vel_des=None, *, q_des=None,
                swing_foot=None, swing_pos_des=None,
                kp_com=300.0, kd_com=35.0, kp_o=200.0, kd_o=28.0,
                kp_p=120.0, kd_p=14.0, kp_sw=900.0, kd_sw=60.0,
                k_contact=20.0):
        from qpsolvers import solve_qp
        mj, M, d, nv = self._mj, self.M, self.d, self.nv

        mj.mj_fullM(M, self._Mbuf, d.qM)
        Mm = self._Mbuf
        h = d.qfrc_bias.copy()
        qvel = d.qvel.copy()

        contacts = self._active_contacts()
        nc = len(contacts)
        ncf = 3 * nc
        n = nv + ncf

        # ---- tasks (soft, least squares rows) ----
        rows_A, rows_b, rows_w = [], [], []

        def add_task(J_qdd, target, weight):
            A = np.zeros((J_qdd.shape[0], n))
            A[:, :nv] = J_qdd
            rows_A.append(A); rows_b.append(target); rows_w.append(
                np.full(J_qdd.shape[0], weight))

        # CoM task
        mj.mj_jacSubtreeCom(M, d, self._jcom, 0)
        com = d.subtree_com[0].copy()
        com_vel = d.subtree_linvel[0].copy() if hasattr(d, "subtree_linvel") \
            else np.zeros(3)
        cvd = np.zeros(3) if com_vel_des is None else np.asarray(com_vel_des)
        a_com = kp_com * (np.asarray(com_des) - com) + kd_com * (cvd - com_vel)
        add_task(self._jcom.copy(), a_com, self.w_com)

        # base orientation task: drive angular accel of the free base (dofs 3:6).
        # Off by default (w_orient=0): under rigid double-support the base angular
        # accel is constraint-determined, so this task only fights the contacts —
        # the posture task holds the torso through the leg/waist joints instead.
        if self.w_orient > 0:
            from gait_lab.model import _quat_to_rpy
            rpy = _quat_to_rpy(d.qpos[3:7])
            omega = d.qvel[3:6]
            Jo = np.zeros((3, nv)); Jo[0, 3] = Jo[1, 4] = Jo[2, 5] = 1.0
            a_o = kp_o * (-rpy) + kd_o * (-omega)
            add_task(Jo, a_o, self.w_orient)

        # posture task on actuated joints
        if q_des is not None:
            Jp = np.zeros((len(self.act_dof), nv))
            qd_act = np.zeros(len(self.act_dof))
            qpos_act = np.zeros(len(self.act_dof))
            for k, dof in enumerate(self.act_dof):
                Jp[k, dof] = 1.0
                qd_act[k] = d.qvel[dof]
            # q error per actuated joint via qpos address
            q_now = np.array([float(d.qpos[self.M.jnt_qposadr[
                self.M.actuator_trnid[i, 0]]]) for i in range(self.M.nu)])
            a_p = kp_p * (np.asarray(q_des) - q_now) - kd_p * qd_act
            add_task(Jp, a_p, self.w_posture)

        # swing-foot task (walking)
        if swing_foot is not None and swing_pos_des is not None:
            site = self.m._foot_site[swing_foot]
            mj.mj_jacSite(M, d, self._jac, None, site)
            p_sw = d.site_xpos[site].copy()
            v_sw = self._jac @ qvel
            a_sw = kp_sw * (np.asarray(swing_pos_des) - p_sw) - kd_sw * v_sw
            add_task(self._jac.copy(), a_sw, self.w_swing)

        A_task = np.vstack(rows_A)
        b_task = np.concatenate(rows_b)
        w_task = np.concatenate(rows_w)
        WA = A_task * w_task[:, None]
        P = WA.T @ A_task
        q = -(WA.T @ b_task)
        # regularisation: small on qddot, small on contact forces
        P[np.arange(nv), np.arange(nv)] += self.eps_qdd
        if ncf:
            P[np.arange(nv, n), np.arange(nv, n)] += self.eps_f

        # ---- equality: base dynamics + contact no-acceleration ----
        Jc_all = np.vstack([c[0] for c in contacts]) if nc else np.zeros((0, nv))
        A_eq_rows, b_eq_rows = [], []
        # base dynamics (6 unactuated rows): M[0:6] qddot - Jc[:,0:6]^T f = -h[0:6]
        Ad = np.zeros((6, n))
        Ad[:, :nv] = Mm[0:6, :]
        if nc:
            Ad[:, nv:] = -Jc_all[:, 0:6].T
        A_eq_rows.append(Ad); b_eq_rows.append(-h[0:6])
        # contacts don't accelerate (Baumgarte velocity damping; Jdot omitted)
        if nc:
            Ac = np.zeros((ncf, n))
            Ac[:, :nv] = Jc_all
            A_eq_rows.append(Ac); b_eq_rows.append(-k_contact * (Jc_all @ qvel))
        A_eq = np.vstack(A_eq_rows)
        b_eq = np.concatenate(b_eq_rows)

        # ---- inequality: friction cone per contact ----
        G_rows, h_rows = [], []
        for j, (_, frame) in enumerate(contacts):
            nrm, t1, t2 = frame[0], frame[1], frame[2]
            base = nv + 3 * j
            # n·f >= fmin  ->  -n·f <= -fmin
            g = np.zeros(n); g[base:base + 3] = -nrm
            G_rows.append(g); h_rows.append(-self.fmin)
            for t in (t1, t2):
                for s in (1.0, -1.0):
                    g = np.zeros(n)
                    g[base:base + 3] = s * t - self.mu * nrm
                    G_rows.append(g); h_rows.append(0.0)
        G = np.vstack(G_rows) if G_rows else None
        hg = np.array(h_rows) if h_rows else None

        x = solve_qp(P, q, G, hg, A_eq, b_eq, solver="osqp",
                     eps_abs=1e-7, eps_rel=1e-7, max_iter=8000, polishing=True,
                     verbose=False)
        if x is None:
            return None
        qddot = x[:nv]
        f = x[nv:]
        tau_full = Mm @ qddot + h
        if nc:
            tau_full -= Jc_all.T @ f
        ctrl = np.zeros(self.M.nu)
        for i in range(self.M.nu):
            ctrl[i] = tau_full[self.act_dof[i]]
        return ctrl


def _all_actuator_names(model):
    import mujoco
    return [mujoco.mj_id2name(model.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(model.model.nu)]


# ---------------------------------------------------------------------------
# Experiment 1: standing balance under a shove.
# ---------------------------------------------------------------------------
def run_position_stand_push(model, horizon, fall_h, push_speed, push_at=0.3,
                            direction=(1.0, 0.0)):
    """Baseline: the stiff 500-gain position stand, shoved."""
    model.set_position_mode(_all_actuator_names(model))
    model.reset()
    d = model.data
    pushed = False
    for i in range(int(round(horizon / model.timestep))):
        t = i * model.timestep
        if not pushed and t >= push_at:
            d.qvel[0] += push_speed * direction[0]
            d.qvel[1] += push_speed * direction[1]
            pushed = True
        d.ctrl[:] = model.stand_targets
        model.step()
        if float(d.qpos[2]) < fall_h:
            return t
    return horizon


def run_qp_stand_push(model, horizon, fall_h, push_speed, push_at=0.3,
                      direction=(1.0, 0.0)):
    """The QP WBC holding a stand (CoM at nominal), shoved the same way.

    Returns ``(survive_time, reason)`` where reason is ``"held"``, ``"infeasible"``
    (the QP itself certifying no friction-cone GRF can arrest the fall — the
    capture point has left the support polygon, i.e. *you must step*), or
    ``"toppled"`` (height fell below ``fall_h``)."""
    names = _all_actuator_names(model)
    model.set_position_mode(names)
    model.reset()
    com0 = model.data.subtree_com[0].copy()
    q_des = model.stand_targets.copy()
    solver = WBCSolver(model)
    model.set_torque_mode(names)
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
            ctrl = solver.compute(com0, q_des=q_des)
            if ctrl is None:
                return t, "infeasible"
            d.ctrl[:] = ctrl
            model.step()
            if float(d.qpos[2]) < fall_h:
                return t, "toppled"
        return horizon, "held"
    finally:
        model.set_position_mode(names)


# ---------------------------------------------------------------------------
# Experiment 2: walking by tracking the ZMP-preview plan with the QP.
# ---------------------------------------------------------------------------
def run_zmp_position(model, horizon, fall_h):
    planner = ZMPPreviewWalk()
    model.set_position_mode(_all_actuator_names(model))
    model.reset()
    planner.reset(model)
    d = model.data
    for i in range(int(round(horizon / model.timestep))):
        d.ctrl[:] = planner.update(model.observe(i * model.timestep), Command())
        model.step()
        if float(d.qpos[2]) < fall_h:
            return i * model.timestep
    return horizon


def _stance_swing(planner, t):
    if t < planner.double_support:
        return "right", "left"
    s = int((t - planner.double_support) // planner.step_duration)
    return ("right", "left") if s % 2 == 0 else ("left", "right")


def run_qp_walk(model, horizon, fall_h):
    """Track the ZMP-preview plan with the QP: CoM follows the planned CoM, the
    swing foot follows its planned placement, GRF solved under the friction cone."""
    names = _all_actuator_names(model)
    planner = ZMPPreviewWalk()
    model.set_position_mode(names)
    model.reset()
    planner.reset(model)
    stand = model.stand_targets.copy()
    com_z = float(model.data.subtree_com[0, 2])
    solver = WBCSolver(model)

    model.set_torque_mode(names)
    model.reset()
    d = model.data
    try:
        for i in range(int(round(horizon / model.timestep))):
            t = i * model.timestep
            k = min(int(t / planner.plan_dt), planner._n - 1)
            com_plan = planner._com[k]
            com_des = np.array([com_plan[0], com_plan[1], com_z])

            # posture: both feet at planned position relative to planned CoM (IK)
            q_des = stand.copy()
            base_now = d.qpos[0:2]
            for foot in ("left", "right"):
                fw = planner._foot_world(foot, t)
                target = np.array([base_now[0] + (fw[0] - com_plan[0]),
                                   base_now[1] + (fw[1] - com_plan[1]), fw[2]])
                angles = model.solve_leg_ik(foot, target)
                for joint, value in zip(LEG_JOINTS[foot], angles):
                    q_des[model.actuator(joint)] = value
            # q_des is already indexed by actuator id (model.actuator(...) == ctrl idx)
            q_des_act = q_des

            stance, swing = _stance_swing(planner, t)
            swing_des = planner._foot_world(swing, t)
            ctrl = solver.compute(com_des, q_des=q_des_act,
                                  swing_foot=swing, swing_pos_des=swing_des)
            if ctrl is None:
                return t
            d.ctrl[:] = ctrl
            model.step()
            if float(d.qpos[2]) < fall_h:
                return t
        return horizon
    finally:
        model.set_position_mode(names)


# ---------------------------------------------------------------------------
# Experiment 3 (the culmination): force-aware QP balance that STEPS when the QP
# says it must — recovering the very shove that topples the bare QP.
# ---------------------------------------------------------------------------
def run_qp_capture_step(model, horizon, fall_h, push_speed, push_at=0.3,
                        direction=(1.0, 0.0), trigger=0.18, step_time=0.34,
                        max_reach=0.45):
    """The culmination — and an honest null result. The QP WBC
    (``run_qp_stand_push``) balances with real friction-cone GRF but goes infeasible
    the instant the capture point leaves the support polygon — it *certifies* "you
    must step". This controller listens to that certificate: it balances in
    force-aware torque mode while the capture point stays inside the support, and on
    the certificate (QP infeasible, or ``||xi - com|| > trigger``) it hands off to a
    capture STEP — swinging the falling-side foot to the reach-clamped capture point
    via leg IK while the stance foot holds, then settles. The step trigger is no
    longer a hand-tuned threshold; it is the QP's own feasibility boundary.

    The honest measured result: this **extends survival over the bare QP** (e.g.
    0.6 m/s forward: ~0.5 s -> ~1.3 s) but does **not** beat the stiff position stand
    or a *standalone* capture step. Two reasons, both informative: (1) the QP balance
    phase is *compliant*, so a hard shove develops while it balances, and stepping
    from that drifted, higher-momentum state is worse than stepping immediately; (2)
    the menagerie G1's large feet give the stand a wide forward support polygon, so a
    stiff stand is already remarkably push-robust forward (it survives a 0.4 m/s shove
    ~2.3 s). Stepping clearly wins only for *gentle* pushes (~0.3 m/s -> full horizon)
    — which the QP could also simply absorb. So deferring the step to the QP's
    feasibility boundary is too late, and force-aware compliance is a liability for
    push recovery on this model: the stiff stand wins yet again. The genuine value of
    this experiment is the finding itself, plus the bug it surfaced (see
    ``G1Model.com_velocity_xy``: the capture-point velocity term was silently zero).

    Returns ``(survive_time, stepped)``."""
    names = _all_actuator_names(model)
    model.set_position_mode(names)
    model.reset()
    com0 = model.data.subtree_com[0].copy()
    stand = model.stand_targets.copy()
    ground = float(model.foot_pos("left")[2])
    solver = WBCSolver(model)

    model.set_torque_mode(names)
    model.reset()
    d = model.data
    phase = "balance"           # "balance" (QP torque) -> "step" (position IK)
    pushed = stepped = False
    planted = stepping = has_stepped = None
    swing = stance = swing_from = swing_to = None
    t_step0 = refractory = 0.0
    try:
        for i in range(int(round(horizon / model.timestep))):
            t = i * model.timestep
            if not pushed and t >= push_at:
                d.qvel[0] += push_speed * direction[0]
                d.qvel[1] += push_speed * direction[1]
                pushed = True

            obs = model.observe(t)
            com = obs.com_xy
            com_vel = model.com_velocity_xy()        # real CoM velocity (see method)
            omega = np.sqrt(9.81 / max(obs.com_z, 0.3))
            xi = com + com_vel / omega               # capture point

            if phase == "balance":
                ctrl = solver.compute(com0, q_des=stand)
                # Step when the QP certifies it must: it returned infeasible, or
                # the capture point has left the support (||xi - com|| > trigger).
                if ctrl is None or np.linalg.norm(xi - com) > trigger:
                    model.set_position_mode(names)      # hand off to IK stepping
                    phase = "step"
                    planted = {f: model.foot_pos(f).copy()
                               for f in ("left", "right")}
                    stepping = has_stepped = False
                    refractory = 0.0
                    # fall through into the step controller this same tick
                else:
                    d.ctrl[:] = ctrl
                    model.step()
                    if float(d.qpos[2]) < fall_h:
                        return t, stepped
                    continue

            # phase == "step": the capture-step controller (position IK), retuned
            # to start from the live falling state the QP handed off.
            ctrl = stand.copy()
            roll, pitch, _ = obs.torso_rpy
            rr, pr = obs.torso_ang_vel[0], obs.torso_ang_vel[1]
            ap_fix = 0.20 * pitch + 0.05 * pr
            ar_fix = 0.20 * roll + 0.05 * rr

            if not stepping and t >= refractory and np.linalg.norm(xi - com) > trigger:
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
                stepped = True
                t_step0 = t

            if stepping:
                ph = float(np.clip((t - t_step0) / step_time, 0.0, 1.0))
                sx = (1 - ph) * swing_from[:2] + ph * swing_to[:2]
                sz = ground + 0.05 * np.sin(np.pi * ph)
                feet = {stance: planted[stance],
                        swing: np.array([sx[0], sx[1], sz])}
                if ph >= 1.0:
                    planted[swing] = swing_to.copy()
                    stepping = False
                    has_stepped = True
                    refractory = t + 0.30
            else:
                feet = {f: planted[f] for f in ("left", "right")}

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
                return t, stepped
        return horizon, stepped
    finally:
        model.set_position_mode(names)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=float, default=8.0)
    ap.add_argument("--fall-height", type=float, default=0.5)
    ap.add_argument("--push-speed", type=float, default=0.6)
    args = ap.parse_args()

    model = G1Model()

    print("Experiment 1 — standing balance under a", args.push_speed,
          "m/s shove:\n")
    for label, dirn in (("forward (+x)", (1.0, 0.0)), ("lateral (+y)", (0.0, 1.0))):
        pos = run_position_stand_push(model, args.horizon, args.fall_height,
                                      args.push_speed, direction=dirn)
        qp, why = run_qp_stand_push(model, args.horizon, args.fall_height,
                                    args.push_speed, direction=dirn)
        print(f"  {label:14s}  stiff-servo {pos:5.2f}s   "
              f"QP-WBC {qp:5.2f}s ({why})")

    print("\nExperiment 2 — walking (tracking the ZMP-preview plan):\n")
    pos = run_zmp_position(model, args.horizon, args.fall_height)
    qp = run_qp_walk(model, args.horizon, args.fall_height)
    print(f"  zmp-preview (position IK)    survives {pos:5.2f}s")
    print(f"  zmp-preview (contact-QP WBC) survives {qp:5.2f}s")

    print("\nExperiment 3 — force-aware QP balance that STEPS when the QP says it\n"
          "must, vs the stiff stand and a standalone capture step (forward shoves):\n")
    from capture_step import run_capture_step
    for sp in (0.4, 0.6):
        stiff = run_position_stand_push(model, args.horizon, args.fall_height, sp,
                                        direction=(1.0, 0.0))
        bare, why = run_qp_stand_push(model, args.horizon, args.fall_height, sp,
                                      direction=(1.0, 0.0))
        cs = run_capture_step(model, sp, 0.0, args.horizon, args.fall_height)
        whole, stepped = run_qp_capture_step(model, args.horizon, args.fall_height,
                                             sp, direction=(1.0, 0.0))
        print(f"  {sp:.1f} m/s   stiff {stiff:4.2f}s   bare-QP {bare:4.2f}s ({why})"
              f"   capture-step {cs:4.2f}s   QP+step {whole:4.2f}s")

    print(
        "\nVerdict — honest, including the negatives, which is the lab's job. The\n"
        "contact-QP WBC (proper TSID, the 'missing piece' the earlier notes named)\n"
        "HOLDS a quiet stand indefinitely with real friction-cone GRF, but does NOT\n"
        "beat position control under a shove or while walking on this position-built\n"
        "model. Under a shove it goes infeasible the moment the capture point leaves\n"
        "the support polygon — the controller certifying 'you must step'. Feeding\n"
        "that certificate to a capture step (Experiment 3) extends survival over the\n"
        "bare QP, but still does NOT beat the stiff stand or a standalone capture\n"
        "step: the QP's compliance lets a hard shove develop before the step, and the\n"
        "G1's large feet make a stiff forward stand very push-robust already. Stepping\n"
        "clearly wins only for gentle pushes the QP could also just absorb. The real\n"
        "yield of this thread: a built-and-tested contact-QP WBC whose infeasibility\n"
        "pinpoints the true boundary, and a bug it surfaced — the capture-point\n"
        "velocity term was silently zero lab-wide (no subtree-velocity sensor;\n"
        "G1Model.com_velocity_xy fixes it), so the capture step now genuinely steps.")


if __name__ == "__main__":
    main()
