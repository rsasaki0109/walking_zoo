"""Load the Unitree G1 MuJoCo model and expose a gait-friendly view of it.

The raw MuJoCo ``MjModel``/``MjData`` are awkward for gait code: actuators,
joints, and qpos addresses are all indexed differently. ``G1Model`` resolves
those once so a controller can say ``model.actuator("left_knee_joint")`` and get
a stable index into ``data.ctrl``, plus a ``stand_targets`` vector to perturb.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DEFAULT_MENAGERIE = os.environ.get(
    "WALKING_ZOO_MENAGERIE_PATH", "/tmp/walking_zoo_mujoco_menagerie"
)

# Leg actuators a gait controller usually drives. Names match the menagerie G1.
LEG_ACTUATORS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

# Per-foot leg-joint chains (6 DOF each), in kinematic order. Used by the IK so a
# controller can command a foot world position and get joint targets back.
LEG_JOINTS = {
    "left": [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ],
    "right": [
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
    ],
}
FOOT_SITE = {"left": "left_foot", "right": "right_foot"}


@dataclass
class Observation:
    """What a controller sees each step. Kept small and physics-agnostic."""

    t: float
    base_height: float
    base_pos_xy: np.ndarray          # world x, y of the floating base
    base_lin_vel: np.ndarray         # world linear velocity of the base (x, y, z)
    torso_rpy: np.ndarray            # roll, pitch, yaw of the torso (rad)
    torso_ang_vel: np.ndarray        # body angular velocity (rad/s)
    com_xy: np.ndarray               # whole-body centre of mass (world x, y)
    com_vel_xy: np.ndarray           # whole-body CoM velocity (world x, y)
    com_z: float                     # whole-body CoM height (world z)


def _quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """MuJoCo quaternion (w, x, y, z) -> roll, pitch, yaw."""
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


class G1Model:
    """A physics-ready Unitree G1 with a resolved actuator/joint view."""

    def __init__(self, menagerie_path: str | None = None):
        import mujoco  # imported lazily so non-physics code can import gait_lab

        self._mj = mujoco
        path = Path(menagerie_path or DEFAULT_MENAGERIE)
        scene = path / "unitree_g1" / "scene.xml"
        if not scene.exists():
            raise FileNotFoundError(
                f"G1 scene not found at {scene}. Set WALKING_ZOO_MENAGERIE_PATH to a "
                "google-deepmind/mujoco_menagerie checkout, e.g.\n"
                "  git clone https://github.com/google-deepmind/mujoco_menagerie.git "
                f"{DEFAULT_MENAGERIE}"
            )
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.timestep = float(self.model.opt.timestep)
        self.nu = int(self.model.nu)

        # actuator i drives joint trnid[i,0]; its target lives at that joint's qpos addr
        self._act_qadr = np.array(
            [
                self.model.jnt_qposadr[self.model.actuator_trnid[i, 0]]
                for i in range(self.nu)
            ]
        )
        self._name2act = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
            for i in range(self.nu)
        }
        # Per-foot kinematic chains for IK: site id, dof columns, qpos addresses.
        self._foot_site = {
            foot: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
            for foot, site in FOOT_SITE.items()
        }
        self._leg_dofs = {}
        self._leg_qadr = {}
        for foot, joints in LEG_JOINTS.items():
            jids = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                for j in joints
            ]
            self._leg_dofs[foot] = np.array([self.model.jnt_dofadr[j] for j in jids])
            self._leg_qadr[foot] = np.array([self.model.jnt_qposadr[j] for j in jids])
        self._ik_scratch = mujoco.MjData(self.model)
        self._ik_jacp = np.zeros((3, self.model.nv))

        # Standing keyframe (named "stand" in the menagerie scene).
        self.reset()
        self.stand_qpos = self.data.qpos.copy()
        self.stand_targets = self.stand_qpos[self._act_qadr].copy()

    # -- lookups -----------------------------------------------------------
    def actuator(self, name: str) -> int:
        """Index into ``data.ctrl`` for a named actuator."""
        return self._name2act[name]

    def has_actuator(self, name: str) -> bool:
        return name in self._name2act

    # -- physics -----------------------------------------------------------
    def reset(self) -> None:
        """Reset to the standing keyframe (index 0 = 'stand')."""
        if self.model.nkey > 0:
            self._mj.mj_resetDataKeyframe(self.model, self.data, 0)
        else:  # pragma: no cover - menagerie G1 always ships a keyframe
            self._mj.mj_resetData(self.model, self.data)
        self._mj.mj_forward(self.model, self.data)

    def perturb(self, seed: int, scale: float = 0.015) -> None:
        """Tilt the base and jitter joint angles for robustness testing.

        Deterministic for a given ``seed``. Call right after :meth:`reset`.
        """
        rng = np.random.default_rng(seed)
        axis_angle = rng.normal(0.0, scale, 3)
        angle = float(np.linalg.norm(axis_angle))
        if angle > 1e-9:
            axis = axis_angle / angle
            self.data.qpos[3:7] = [
                np.cos(angle / 2),
                *(axis * np.sin(angle / 2)),
            ]
        self.data.qpos[7:] += rng.normal(0.0, scale, self.data.qpos.shape[0] - 7)
        self._mj.mj_forward(self.model, self.data)

    def push(self, rng: np.random.Generator, speed: float) -> None:
        """Shove the robot: add a horizontal velocity kick (m/s) to the base in a
        random direction. Models an external disturbance for push-recovery
        training / benchmarking. ``rng`` makes the direction reproducible."""
        theta = float(rng.uniform(0.0, 2.0 * np.pi))
        self.data.qvel[0] += speed * np.cos(theta)
        self.data.qvel[1] += speed * np.sin(theta)

    def step(self) -> None:
        self._mj.mj_step(self.model, self.data)

    def observe(self, t: float) -> Observation:
        d = self.data
        return Observation(
            t=t,
            base_height=float(d.qpos[2]),
            base_pos_xy=d.qpos[0:2].copy(),
            base_lin_vel=d.qvel[0:3].copy(),
            torso_rpy=_quat_to_rpy(d.qpos[3:7]),
            torso_ang_vel=d.qvel[3:6].copy(),
            com_xy=d.subtree_com[0, 0:2].copy(),
            com_vel_xy=d.subtree_linvel[0, 0:2].copy()
            if hasattr(d, "subtree_linvel")
            else d.qvel[0:2].copy(),
            com_z=float(d.subtree_com[0, 2]),
        )

    # -- kinematics for IK-based controllers -------------------------------
    def foot_pos(self, foot: str) -> np.ndarray:
        """Current world position of a foot site ('left' or 'right')."""
        return self.data.site_xpos[self._foot_site[foot]].copy()

    def solve_leg_ik(
        self,
        foot: str,
        target_world: np.ndarray,
        *,
        iters: int = 16,
        damp: float = 1e-2,
        tol: float = 1e-4,
    ) -> np.ndarray:
        """Damped-least-squares IK: 6 leg-joint angles placing the foot at a target.

        Warm-started from the *current* physics pose, holding the floating base
        and all other joints fixed. Returns joint angles in ``LEG_JOINTS[foot]``
        order (which equals the actuator order for those joints).
        """
        mujoco = self._mj
        s = self._ik_scratch
        s.qpos[:] = self.data.qpos
        site = self._foot_site[foot]
        dofs = self._leg_dofs[foot]
        qadr = self._leg_qadr[foot]
        for _ in range(iters):
            mujoco.mj_kinematics(self.model, s)
            mujoco.mj_comPos(self.model, s)
            err = target_world - s.site_xpos[site]
            if np.linalg.norm(err) < tol:
                break
            mujoco.mj_jacSite(self.model, s, self._ik_jacp, None, site)
            jac = self._ik_jacp[:, dofs]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damp * np.eye(3), err)
            s.qpos[qadr] += dq
        return s.qpos[qadr].copy()
