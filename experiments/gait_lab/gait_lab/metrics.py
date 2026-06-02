"""Metrics collected while rolling out a gait controller in physics."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GaitMetrics:
    """Outcome of one physics rollout, comparable across algorithms."""

    name: str
    horizon: float            # commanded rollout length (s)
    survival_time: float      # time before the torso dropped below fall_height (s)
    forward_distance: float   # net base displacement along +x (m)
    lateral_drift: float      # absolute base displacement along y (m)
    mean_speed: float         # forward_distance / survival_time (m/s)
    min_base_height: float    # lowest torso height seen (m)
    fell: bool                # did it fall before the horizon ended?

    def as_row(self) -> str:
        status = "FELL" if self.fell else "ok"
        return (
            f"{self.name:18s} fwd={self.forward_distance:+6.3f}m  "
            f"speed={self.mean_speed:+5.3f}m/s  survive={self.survival_time:5.2f}s  "
            f"drift={self.lateral_drift:5.3f}m  minH={self.min_base_height:4.2f}m  [{status}]"
        )

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "horizon": round(self.horizon, 3),
            "survival_time": round(self.survival_time, 3),
            "forward_distance": round(self.forward_distance, 4),
            "lateral_drift": round(self.lateral_drift, 4),
            "mean_speed": round(self.mean_speed, 4),
            "min_base_height": round(self.min_base_height, 4),
            "fell": self.fell,
        }


HEADER = (
    "algorithm           forward     speed       survival  drift     minH   status"
)
