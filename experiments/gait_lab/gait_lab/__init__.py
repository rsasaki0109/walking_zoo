"""gait_lab: a physics-driven testbed for comparing walking gait algorithms.

walking_zoo itself is *not* a gait research stack — it is the runtime/safety/
adapter layer. gait_lab sits alongside it as an experiment: it drives a real
MuJoCo Unitree G1 through *physics* (position actuators, ``mj_step``) so that
different gait-generation algorithms can be plugged in behind one
``GaitController`` interface and compared on the same robot with the same
metrics (forward distance, survival time, lateral drift, ...).

This mirrors the walking_zoo thesis: a gait algorithm is just another command
source behind a stable interface. Here the "interface" is ``GaitController`` and
the "runtime" is the MuJoCo physics harness.
"""

from .controllers import (
    Command,
    GaitController,
    StandHold,
    OpenLoopCPG,
    BalancedCPG,
    SteerableCPG,
    CapturePointWalk,
    OptimizedCapturePoint,
    ZMPPreviewWalk,
    SteerableZMPWalk,
    LearnedFeedbackWalk,
    RLResidualWalk,
    RLSteerableWalk,
    RLSteerableFootstepWalk,
    SteerableFootstepGait,
    ReactiveSteerableWalk,
    CONTROLLERS,
)
from .harness import GaitHarness, rollout
from .metrics import GaitMetrics
from .model import G1Model, Observation

__all__ = [
    "Command",
    "GaitController",
    "StandHold",
    "OpenLoopCPG",
    "BalancedCPG",
    "SteerableCPG",
    "CapturePointWalk",
    "OptimizedCapturePoint",
    "ZMPPreviewWalk",
    "SteerableZMPWalk",
    "LearnedFeedbackWalk",
    "RLResidualWalk",
    "RLSteerableWalk",
    "RLSteerableFootstepWalk",
    "SteerableFootstepGait",
    "ReactiveSteerableWalk",
    "CONTROLLERS",
    "GaitHarness",
    "rollout",
    "GaitMetrics",
    "G1Model",
    "Observation",
]
