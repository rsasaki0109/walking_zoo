# gait_lab — a physics testbed for walking gait algorithms

walking_zoo is the runtime/safety/adapter layer for walking robots — it is
deliberately **not** a gait research stack. `gait_lab` sits *alongside* it as an
experiment that answers a natural question: *"can I actually try different gait
algorithms here?"*

It drives a real MuJoCo **Unitree G1** through **physics** (position actuators +
`mj_step`, not kinematic playback), so a gait algorithm's quality actually shows
up: a bad gait falls over, a better one stays up and walks. Every algorithm
lives behind one small interface and is scored on the same robot with the same
metrics — an apples-to-apples comparison.

This mirrors the walking_zoo thesis: a gait generator is just another command
source behind a stable interface. Here that interface is `GaitController` and
the "runtime" is the physics harness.

## What's included

Six algorithms spanning four classes (CPG, reactive model-based, optimised,
preview model-based):

| algorithm        | idea                                                          |
|------------------|---------------------------------------------------------------|
| `stand-hold`     | hold the standing keyframe (baseline: stable, goes nowhere)   |
| `open-loop-cpg`  | fixed sinusoidal stepping, **no feedback** (the honest failure) |
| `balanced-cpg`   | stepping + lateral weight-shift + torso-attitude feedback     |
| `capture-point`  | LIPM capture-point footstep placement + leg **inverse kinematics** |
| `optimized-cp`   | the capture-point gait with parameters found by **optimisation** (CEM), not by hand |
| `zmp-preview`    | **ZMP preview control** (Kajita) plans a CoM trajectory tracked via IK |

A representative run (`run_compare.py`, 8 s horizon):

```
algorithm           forward     speed       survival  drift     minH   status
stand-hold         fwd=-0.000m  speed=-0.000m/s  survive= 8.00s  drift=0.000m  minH=0.79m  [ok]
open-loop-cpg      fwd=+0.100m  speed=+0.130m/s  survive= 1.07s  drift=0.168m  minH=0.50m  [FELL]
balanced-cpg       fwd=+0.269m  speed=+0.096m/s  survive= 3.09s  drift=0.267m  minH=0.50m  [FELL]
capture-point      fwd=+0.614m  speed=+0.814m/s  survive= 1.05s  drift=0.038m  minH=0.50m  [FELL]
optimized-cp       fwd=+1.250m  speed=+1.228m/s  survive= 1.32s  drift=0.128m  minH=0.50m  [FELL]
zmp-preview        fwd=+0.658m  speed=+0.310m/s  survive= 2.42s  drift=0.194m  minH=0.50m  [FELL]

farthest walker: optimized-cp (+1.250 m, survived 1.32s)
most stable:     stand-hold (survived 8.00s)
```

The story the numbers tell, and the reason a comparison testbed is worth having:

* **open-loop** stepping topples a humanoid in ~1 s — feedback is not optional.
* **balanced-cpg** (lateral weight-shift so the swing foot can unload + ankle
  attitude feedback) survives **~3× longer** and creeps forward: the *most
  stable* stepper.
* **capture-point** reasons about *where* to put the next foot — it models the
  robot as a linear inverted pendulum, places the swing foot at the
  instantaneous capture point (`xi = x_com + v_com/omega`) via leg IK, and walks
  **~6× farther and far straighter** (drift 0.04 m vs 0.17 m) than open-loop —
  the *farthest walker*. But it is the *least durable*: kinematic footstep
  placement commits to long strides without true dynamic (ZMP/force) balance, so
  it eventually topples.

* **optimized-cp** is the *same algorithm and interface* as `capture-point`,
  but its parameters were found by **optimisation** (`optimize.py`, a
  Cross-Entropy Method over physics rollouts) instead of by hand. It walks
  **~2× farther** than the hand-tuned version (1.25 m vs 0.61 m) — the testbed's
  concrete answer to "does an optimisation-based gait beat hand-tuning?". Note
  it optimised the *distance* objective: it does not out-*stabilise*
  `balanced-cpg`, because that is not what it was rewarded for. Optimisation
  closes the gap on the axis you optimise.
* **zmp-preview** is the most *principled* model-based walker: it plans a whole
  CoM trajectory up front with **Kajita preview control** (a cart-table LIPM
  whose induced ZMP tracks — and leads — the footstep reference), then realises
  it via IK. It is the best all-rounder among the steppers: it walks farther
  than `balanced-cpg` *and* survives longer than the reactive `capture-point`,
  because planning ahead lets the CoM sway over the next stance foot *before*
  the step instead of reacting after.

There is no free lunch here — *farthest walker* and *most stable* are different
algorithms. None is a robustly-walking controller; that gap is exactly what the
testbed measures, and a learned policy plugs in the same way (see below).

## Optimising a gait

`optimize.py` searches a controller's `TUNABLES` parameter space with the
Cross-Entropy Method, scoring each candidate by a physics rollout (distance
walked, with a small survival term). It warm-starts at the hand-tuned defaults:

```bash
python3 optimize.py --iters 10 --pop 18 --seed 0   # ~a few minutes; deterministic
```

The discovered parameters are baked into `OptimizedCapturePoint`
(`controllers.OPTIMIZED_CAPTURE_POINT_PARAMS`) so the result is reproducible and
needs no optimiser at run time. A **learned policy** is the same shape: swap the
parameter/inference source behind the identical `GaitController` interface —
load weights in `reset`, run the network in `update`.

## Running it

`gait_lab` needs `mujoco` and the menagerie G1 model — neither is part of the
ROS 2 workspace, so run it from a Python environment that has MuJoCo. The
`zmp-preview` controller additionally needs `scipy` (for the Riccati solve behind
the preview gains); every other algorithm is numpy-only and the comparison skips
`zmp-preview` cleanly if scipy is absent.

```bash
# one-time: a local mujoco_menagerie checkout (the G1 scene)
git clone https://github.com/google-deepmind/mujoco_menagerie.git \
    /tmp/walking_zoo_mujoco_menagerie

cd experiments/gait_lab
python3 run_compare.py                       # metrics table
python3 run_compare.py --json out/cmp.json   # + machine-readable metrics
MUJOCO_GL=egl python3 run_compare.py --gif out/   # + one GIF per algorithm
```

Point at a non-default model checkout with `--menagerie /path` or the
`WALKING_ZOO_MENAGERIE_PATH` environment variable. GIF encoding needs `imageio`
or `pillow` (optional); rendering needs a GL backend (`MUJOCO_GL=egl` is
headless-friendly).

## Adding your own gait algorithm

The whole point. Subclass `GaitController`, return a position-actuator target
vector, and register it:

```python
from gait_lab.controllers import GaitController, CONTROLLERS

class MyGait(GaitController):
    name = "my-gait"

    def update(self, obs, cmd):
        ctrl = self.stand.copy()          # start from the standing pose
        # obs gives torso_rpy, torso_ang_vel, com_xy, com_vel_xy, base_height ...
        self._leg(ctrl, "left_knee_joint", 0.3)
        return ctrl
```

Then add it to `CONTROLLERS()` (or call `rollout(model, MyGait())` directly) and
it shows up in the comparison with the same metrics as everything else. A
learned policy fits the same shape: load weights in `reset`, run inference in
`update`.

## Tests

```bash
# needs a venv with mujoco + pytest; clear the ROS PYTHONPATH so pytest does not
# auto-load the ROS launch_testing plugin
env -u PYTHONPATH MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    python3 -m pytest test_gait_lab.py -q
```

The suite skips cleanly if mujoco or the G1 model is unavailable, and asserts the
core comparison invariants (stand survives the full horizon, open-loop topples
early, `balanced-cpg` out-survives and out-walks it, metrics are finite and
deterministic).

## Why this is separate from the ROS packages

It depends on MuJoCo and a model checkout that the hardware-free ROS 2 build must
not require. Keeping it under `experiments/` lets the runtime stay lean while
still giving an honest, runnable answer to "can I try gait algorithms here?" A
future step is to expose the best controller as a walking_zoo adapter so a gait
algorithm validated here can drive the real runtime/safety pipeline.
