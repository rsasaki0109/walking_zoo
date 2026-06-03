# gait_lab — sharing kit

Copy-paste posts for surfacing the honest gait benchmark. The hook is always the
same: **a physics testbed where bad humanoid gaits actually fall over, and the
textbook controllers lose for reasons you can watch.** Lead with `assets/gait_zoo.gif`.

Repo: https://github.com/rsasaki0109/walking_zoo
Lab: https://github.com/rsasaki0109/walking_zoo/tree/main/experiments/gait_lab

> Honesty note before posting: every number below is in the repo's tests and
> README. Don't round up. The whole appeal is that the negatives are reported.

---

## Show HN

**Title:** Show HN: A physics testbed where bad humanoid walking gaits actually fall over

**Body:**

I kept reading "our controller walks" locomotion repos where the robot is on rails —
kinematic playback that can't fall. So I built the opposite: a MuJoCo Unitree G1
driven through real physics (position actuators + `mj_step`), where every gait
algorithm lives behind one small interface and is scored on the same robot with the
same metrics. A bad gait topples; a good one stays up and walks. Same robot, same
command, side by side, live.

Nine controllers in the gallery so far — open-loop CPG, balanced CPG, capture-point
footsteps, a CEM-optimized variant, a continuous DCM step-adjustment walker,
ZMP-preview control, a learned linear feedback policy, and a PPO residual. Only the RL
residual holds the full horizon; the rest topple at times you can watch flip red in
the GIF.

The part I didn't expect, and the reason I think it's worth sharing: I built the
textbook force-aware controller — a full contact-QP whole-body controller (task-space
inverse dynamics, friction-cone ground-reaction forces solved per step) — expecting
it to beat the dumb stiff position servo. It doesn't. It holds a quiet stand, but
under a shove it goes *infeasible* the instant the capture point leaves the support
polygon — which is the controller correctly *certifying* "no force can save this, you
must step." Then I chased why the stiff servo wins and found the servo's standing
advantage was largely a simulation idealization: MuJoCo integrates the servo's
velocity-damping term implicitly (a free, unconditionally-stable inner loop). Re-run
both controllers as honest explicit torque and the servo can't even hold a quiet
stand — the model-based QP can. But apply the same audit to *walking* and the verdict
does NOT flip: position tracking genuinely beats torque there. Standing balance was a
crutch; walking authority was real.

It's a research playground next to a ROS2 walking runtime, not a product. ~50 tests,
every controller reproducible, the GIFs regenerate from one script. Happy to take
holes in the methodology — finding them is the point.

---

## r/robotics

**Title:** I built a physics testbed that honestly benchmarks humanoid walking
controllers — the textbook whole-body controller loses, and you can watch why

**Body:**

[gait_zoo.gif]

Nine walking controllers, one MuJoCo Unitree G1, same command, scored through real
physics so a bad gait actually falls. The status chip flips red the instant a gait
topples. Only the PPO residual holds the full horizon; the kinematic footstep walkers
walk then fall.

The interesting results are the negatives:

- The **textbook contact-QP whole-body controller** (TSID, friction-cone GRF solved
  per step) does **not** beat a stiff position servo. Under a shove it goes infeasible
  exactly when the capture point exits the support polygon — it *certifies* "you must
  step" rather than pretending it can balance.
- That QP was secretly planning ankle torques ~4× the joint limit, silently clamped by
  the sim. Adding the real torque limits (the *complete* TSID) fixes the fiction at no
  cost to survival — because the wall is the support polygon, not the torque budget.
- The stiff servo's *standing* win turned out to be a sim idealization (MuJoCo
  integrates its damping implicitly). On honest explicit-torque footing the servo
  can't hold a quiet stand and the model-based QP can — but the same audit on
  **walking** does not flip: position tracking really does beat torque there.
- The one move that recovers a real push is to **step** (capture-point footstep), not
  any amount of standing-balance cleverness.

Second GIF (`--push 0.6`): the same zoo under a recurring shove — every stand-and-walk
controller goes down, which is the whole point.

Repo + reproducible tests: https://github.com/rsasaki0109/walking_zoo/tree/main/experiments/gait_lab
Tear the methodology apart — that's what it's for.

---

## X / short thread

1/ I built a physics testbed where bad humanoid walking gaits actually fall over.
9 controllers, one MuJoCo G1, same command, live fall detection. Only the RL residual
walks the full horizon 👇 [gait_zoo.gif]

2/ The fun part is the negatives. I built the textbook contact-QP whole-body
controller (TSID) expecting it to beat a dumb stiff position servo. It doesn't. Under
a shove it goes *infeasible* — correctly certifying "you must step."

3/ Then: the stiff servo's standing win was a sim idealization — MuJoCo integrates its
damping implicitly, a free stable inner loop. On honest explicit torque the servo
can't even hold a quiet stand; the model-based QP can.

4/ But run the SAME audit on walking and it does NOT flip — position tracking really
beats torque there. Standing balance was a crutch; walking authority was real. Honest
benchmarks, negatives included: https://github.com/rsasaki0109/walking_zoo

---

## Talking points (keep them accurate)

- 9 controllers in the live gallery; only `rl-residual` holds the full 5 s horizon.
- Contact-QP WBC (TSID) holds a quiet stand, loses under a shove by going infeasible
  (= "must step"). Not a bug — a certificate.
- Friction-only QP planned ankle torque ~383% of limit (56 steps); the complete TSID
  caps it at 100% at no survival cost. Wall = support polygon, not torque.
- Explicit-torque audit: servo can't hold a quiet stand (~1.3 s) once its implicit
  damping is paid for; QP holds. Standing verdict flips to the QP.
- Walking audit: position-IK walk loses ~⅓ to the idealization (~2.15→~1.45 s) but
  still beats the QP walk (~0.6 s). Walking verdict does NOT flip.
- The recovering move is the capture step, taken exactly when the QP says you must.
- Push-robustness frontier (`push_frontier.py`): binary-search the max shove (m/s)
  survived per direction. The capture step's polygon encloses the stiff stand by
  +55% area but ties it at the backward worst case (~0.2 m/s); the contact-QP's
  collapses to a point (certifies must-step under any shove).
- A survival-time curve (`push_frontier.py --curve`) un-flattens that binary: the
  capture step recovers a forward shove to 0.35 m/s; the QP-balance-then-step
  synthesis (`qp-capture-step`) never recovers but *doubles* the bare QP's time-to-fall
  (~1.2 s plateau vs ~0.55 s) — force authority delays the fall, only stepping recovers.
- The lab's recurring "~1 s collapse" is now *predicted* (`fall_time_theory.py`): the
  push frontier is geometry — `v* = d·ω` matches the measured lateral/backward radii to
  ~5% (forward is ankle-torque-limited) — and the fall clock is leg length, `1/ω =
  √(z/g) ≈ 0.27 s`, so the ~1 s ceiling is a few of those clocks, controller-independent.
  That's *why* force-vs-position never moved the wall.
- `dcm-walk` (continuous DCM step adjustment) walks 2nd-farthest of the steppers
  (0.81 m) but its closed loop buys no survival on position control — the open-loop
  zmp-preview outlives it. The DCM's robustness edge needs force authority; an honest
  null result that points at the same ceiling.
