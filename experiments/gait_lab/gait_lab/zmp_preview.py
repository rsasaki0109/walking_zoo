"""Kajita-style ZMP preview control for a linear-inverted-pendulum CoM.

Given a future Zero-Moment-Point reference (where the support foot will be), the
preview controller produces a smooth CoM trajectory whose induced ZMP tracks that
reference *and leads it* — the CoM starts shifting toward the next footstep before
the foot is even planted, which is what keeps a walker balanced. This is the
classic cart-table preview servo (Kajita et al., 2003), solved via the discrete
algebraic Riccati equation.

Used by :class:`gait_lab.controllers.ZMPPreviewWalk`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are


@dataclass
class PreviewGains:
    Gi: float                # integral (tracking) gain
    Gx: np.ndarray           # (3,) state-feedback gain on [pos, vel, acc]
    Gd: np.ndarray           # (N,) preview gains over the future ZMP reference
    A: np.ndarray            # (3,3) cart-table dynamics
    B: np.ndarray            # (3,1)
    C: np.ndarray            # (1,3) ZMP output
    dt: float
    horizon: int             # N preview steps


def design_preview(dt: float, com_height: float, horizon: int,
                   *, gravity: float = 9.81, q_zmp: float = 1.0,
                   r_jerk: float = 1e-6) -> PreviewGains:
    """Solve for the preview gains of the cart-table model."""
    A = np.array([[1, dt, dt ** 2 / 2], [0, 1, dt], [0, 0, 1.0]])
    B = np.array([[dt ** 3 / 6], [dt ** 2 / 2], [dt]])
    C = np.array([[1.0, 0.0, -com_height / gravity]])

    # Augment with the integrated tracking error as a leading state.
    Aa = np.block([[np.ones((1, 1)), C @ A], [np.zeros((3, 1)), A]])
    Ba = np.block([[C @ B], [B]])
    Ca = np.array([[1.0, 0.0, 0.0, 0.0]])
    Q = np.zeros((4, 4))
    Q[0, 0] = q_zmp

    P = solve_discrete_are(Aa, Ba, Q, r_jerk)
    K = float(r_jerk + (Ba.T @ P @ Ba)[0, 0])
    gain = (Ba.T @ P @ Aa) / K          # (1,4): [Gi, Gx0, Gx1, Gx2]
    Gi = float(gain[0, 0])
    Gx = np.asarray(gain[0, 1:]).ravel()

    # Preview gains over the future reference.
    Gd = np.zeros(horizon)
    Ac = Aa - Ba @ gain
    X = -Ac.T @ P @ Ca.T
    Gd[0] = -Gi
    for j in range(1, horizon):
        Gd[j] = float((Ba.T @ X)[0, 0]) / K
        X = Ac.T @ X
    return PreviewGains(Gi, Gx, Gd, A, B, C, dt, horizon)


class PreviewAxis:
    """Run the preview servo for one horizontal axis, statefully."""

    def __init__(self, gains: PreviewGains):
        self.g = gains
        self.x = np.zeros(3)      # [pos, vel, acc]
        self.err_sum = 0.0

    def step(self, zmp_future: np.ndarray) -> float:
        """Advance one control step. ``zmp_future[j]`` = ZMP ref j steps ahead.

        Returns the new CoM position on this axis.
        """
        g = self.g
        zmp = float((g.C @ self.x)[0])
        self.err_sum += zmp - zmp_future[0]
        preview = float(g.Gd[: len(zmp_future)] @ zmp_future)
        u = -g.Gi * self.err_sum - float(g.Gx @ self.x) - preview
        self.x = g.A @ self.x + (g.B.ravel() * u)
        return float(self.x[0])

    def reset(self, pos: float = 0.0) -> None:
        self.x = np.array([pos, 0.0, 0.0])
        self.err_sum = 0.0
