#!/usr/bin/env bash
# Launch a gait_lab RL training run with the correct environment.
#
# Two non-obvious requirements (both have bitten this project before):
#   * the MuJoCo/RL deps live in the isaacsim venv, and ROS's PYTHONPATH must be
#     replaced with gait_lab's so `import gait_lab` resolves;
#   * multi-process rollout collection MUST pin BLAS/OpenMP to 1 thread per
#     process or N workers thrash the cores (measured ~8x slower, load average
#     into the dozens). See memory: gait-lab-rl-training-env.
#
# Usage:  ./run_train.sh [train_rl.py args...]
set -euo pipefail
cd "$(dirname "$0")"

export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export MUJOCO_GL=egl
export PYTHONPATH="$PWD"

VENV="${GAIT_LAB_VENV:-/media/sasaki/aiueo/isaacsim-venv}"
exec "$VENV/bin/python3" train_rl.py "$@"
