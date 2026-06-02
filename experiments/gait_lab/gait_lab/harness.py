"""Physics harness: roll a controller out on the G1 and collect metrics.

This is the "runtime" for gait_lab. It owns the physics loop (``mj_step``),
feeds each controller's ``ctrl`` into the position actuators, watches for a
fall, and optionally records frames for a GIF — exactly the loop walking_zoo's
real adapters run, just instrumented for comparison.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controllers import Command, GaitController
from .metrics import GaitMetrics
from .model import G1Model


@dataclass
class GaitHarness:
    model: G1Model
    horizon: float = 5.0
    fall_height: float = 0.5     # torso below this counts as fallen
    settle: float = 0.3          # let physics settle before scoring distance

    def rollout(
        self,
        controller: GaitController,
        cmd: Command | None = None,
        *,
        render: bool = False,
        camera_distance: float = 3.0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> tuple[GaitMetrics, list]:
        cmd = cmd or Command()
        m = self.model
        m.reset()
        controller.reset(m)

        renderer = None
        frames: list = []
        frame_every = 0
        if render:
            import mujoco

            renderer = mujoco.Renderer(m.model, height=height, width=width)
            camera = mujoco.MjvCamera()
            camera.distance = camera_distance
            camera.elevation = -18.0
            camera.azimuth = 120.0
            frame_every = max(1, int(round((1.0 / fps) / m.timestep)))

        steps = int(round(self.horizon / m.timestep))
        settle_steps = int(round(self.settle / m.timestep))
        start_xy = None
        fell_at = None
        min_h = float("inf")
        last_xy = m.observe(0.0).base_pos_xy.copy()

        for i in range(steps):
            t = i * m.timestep
            obs = m.observe(t)
            m.data.ctrl[:] = controller.update(obs, cmd)
            m.step()

            h = float(m.data.qpos[2])
            min_h = min(min_h, h)
            if i == settle_steps:
                start_xy = m.data.qpos[0:2].copy()
            if fell_at is None and h < self.fall_height:
                fell_at = t
                last_xy = m.data.qpos[0:2].copy()
                if not render:
                    break  # no point simulating a collapsed robot for metrics
            if renderer is not None and (i % frame_every == 0):
                # Track the base so a walking robot stays in frame.
                camera.lookat[:] = [m.data.qpos[0], m.data.qpos[1], 0.6]
                renderer.update_scene(m.data, camera=camera)
                frames.append(renderer.render().copy())

        if start_xy is None:  # fell before settling
            start_xy = m.observe(0.0).base_pos_xy.copy()
        if fell_at is None:
            last_xy = m.data.qpos[0:2].copy()
        if renderer is not None:
            renderer.close()

        survival = fell_at if fell_at is not None else self.horizon
        scored_time = max(survival - self.settle, 1e-3)
        forward = float(last_xy[0] - start_xy[0])
        lateral = float(abs(last_xy[1] - start_xy[1]))
        metrics = GaitMetrics(
            name=controller.name,
            horizon=self.horizon,
            survival_time=survival,
            forward_distance=forward,
            lateral_drift=lateral,
            mean_speed=forward / scored_time,
            min_base_height=min_h,
            fell=fell_at is not None,
        )
        return metrics, frames


def rollout(model: G1Model, controller: GaitController, **kw) -> GaitMetrics:
    """Convenience: metrics only, no rendering."""
    horizon = kw.pop("horizon", 5.0)
    fall_height = kw.pop("fall_height", 0.5)
    cmd = kw.pop("cmd", None)
    harness = GaitHarness(model, horizon=horizon, fall_height=fall_height)
    metrics, _ = harness.rollout(controller, cmd=cmd, render=False)
    return metrics
