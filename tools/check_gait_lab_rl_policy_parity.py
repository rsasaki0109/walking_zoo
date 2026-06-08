#!/usr/bin/env python3
"""Verify embedded C++ RL inference matches gait_lab Python ``_policy``."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
GAIT_LAB = REPO / "experiments" / "gait_lab"
ATOL = 1e-5


def _locate_infer_binary() -> Path | None:
    env = os.environ.get("GAIT_LAB_RL_POLICY_INFER")
    if env:
        path = Path(env)
        return path if path.is_file() else None
    install = REPO / "install" / "locomotion_ros2_gait_lab_sil" / "lib"
    if not install.is_dir():
        return None
    for candidate in install.rglob("gait_lab_rl_policy_infer"):
        if candidate.is_file():
            return candidate
    return None


def _python_policy(policy_file: str):
    sys.path.insert(0, str(GAIT_LAB))
    from gait_lab.controllers import RLResidualWalk, RLSteerableWalk  # noqa: E402

    path = GAIT_LAB / "gait_lab" / policy_file
    if not path.exists():
        return None
    cls = RLSteerableWalk if "steer" in policy_file else RLResidualWalk
    ctrl = cls(str(path))
    ctrl._load()
    return ctrl._policy


def _cpp_policy(binary: Path, policy_file: str, observation: np.ndarray) -> np.ndarray:
    path = GAIT_LAB / "gait_lab" / policy_file
    cmd = [str(binary), str(path), *[f"{float(v):.17g}" for v in observation]]
    out = subprocess.check_output(cmd, text=True, stderr=subprocess.PIPE)
    return np.array([float(tok) for tok in out.split()], dtype=np.float64)


def _fixed_observations(dim: int) -> list[np.ndarray]:
    rng = np.random.default_rng(0)
    obs = []
    obs.append(np.zeros(dim, dtype=np.float64))
    obs.append(np.linspace(-0.4, 0.4, dim, dtype=np.float64))
    obs.append(rng.normal(size=dim))
    return obs


def _rollout_observations(policy_file: str, samples: int = 4) -> list[np.ndarray]:
    sys.path.insert(0, str(GAIT_LAB))
    from gait_lab import G1Model  # noqa: E402
    from gait_lab.controllers import Command, RLResidualWalk, RLSteerableWalk  # noqa: E402
    from gait_lab.rl_env import observe_policy  # noqa: E402

    path = GAIT_LAB / "gait_lab" / policy_file
    if not path.exists():
        return []
    steer = "steer" in policy_file
    cls = RLSteerableWalk if steer else RLResidualWalk
    model = G1Model()
    ctrl = cls(str(path))
    ctrl.reset(model)
    cmd = Command(0.25, 0.15) if steer else Command(0.3, 0.0)

    vectors: list[np.ndarray] = []
    steps = int(round(3.0 / model.timestep))
    for i in range(steps):
        t = i * model.timestep
        obs = model.observe(t)
        model.data.ctrl[:] = ctrl.update(obs, cmd)
        model.step()
        if i % max(1, steps // samples) == 0:
            vectors.append(
                observe_policy(
                    model,
                    obs,
                    ctrl._leg_qadr,
                    ctrl._leg_dofadr,
                    ctrl._stand_leg,
                    ctrl._freq,
                    t,
                    cmd=cmd if steer else None,
                ).astype(np.float64)
            )
        if len(vectors) >= samples:
            break
    return vectors


def _check_policy(binary: Path, policy_file: str) -> tuple[bool, str]:
    py_policy = _python_policy(policy_file)
    if py_policy is None:
        return True, f"skip {policy_file} (not trained)"

    observations = _fixed_observations(
        36 if "steer" in policy_file else 34)
    observations.extend(_rollout_observations(policy_file))

    worst = 0.0
    for idx, observation in enumerate(observations):
        py_action = py_policy(observation)
        cpp_action = _cpp_policy(binary, policy_file, observation)
        if py_action.shape != cpp_action.shape:
            return False, (
                f"{policy_file} case {idx}: action shape "
                f"{py_action.shape} != {cpp_action.shape}"
            )
        delta = float(np.max(np.abs(py_action - cpp_action)))
        worst = max(worst, delta)
        if not np.allclose(py_action, cpp_action, atol=ATOL, rtol=0.0):
            return False, (
                f"{policy_file} case {idx}: max |py-cpp|={delta:.2e} "
                f"(tol {ATOL:.0e})"
            )
    return True, f"{policy_file}: {len(observations)} cases, worst |py-cpp|={worst:.2e}"


def main() -> int:
    binary = _locate_infer_binary()
    if binary is None:
        print(
            "gait_lab_rl_policy_infer not found; build locomotion_ros2_gait_lab_sil "
            "or set GAIT_LAB_RL_POLICY_INFER",
            file=sys.stderr,
        )
        return 1

    os.environ.setdefault("LOCOMOTION_ROS2_GAIT_LAB_PATH", str(GAIT_LAB))

    policies = [
        "rl_policy.npz",
        "rl_policy_steer.npz",
        "rl_policy_steer_fs.npz",
    ]
    failed = False
    for policy_file in policies:
        ok, message = _check_policy(binary, policy_file)
        print(message)
        if not ok:
            failed = True

    if failed:
        print("gait_lab RL policy parity failed", file=sys.stderr)
        return 1
    print("gait_lab RL policy parity passed (Python _policy == C++ infer)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
