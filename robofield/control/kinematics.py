"""Differential-drive kinematics for the RoboField car."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DiffDrive:
    wheel_radius_mm: float
    track_mm: float
    max_wheel_rpm: float

    @classmethod
    def from_config(cls, cfg: dict) -> "DiffDrive":
        m = cfg["robot_model"]
        return cls(m["wheel_radius_mm"], m["track_mm"], m["max_wheel_rpm"])

    @property
    def max_wheel_speed(self) -> float:
        """mm/s at the wheel rim at full command."""
        return self.max_wheel_rpm * 2 * math.pi / 60 * self.wheel_radius_mm

    @property
    def max_omega(self) -> float:
        """rad/s when spinning in place at full command."""
        return 2 * self.max_wheel_speed / self.track_mm

    def forward(self, left: float, right: float) -> tuple[float, float]:
        """Wheel commands (-1..1) -> body (v mm/s, omega rad/s)."""
        vl, vr = left * self.max_wheel_speed, right * self.max_wheel_speed
        return (vr + vl) / 2, (vr - vl) / self.track_mm

    def inverse(self, v: float, omega: float) -> tuple[float, float]:
        """Body (v, omega) -> wheel commands, scaled down together if any exceeds +-1
        (keeps the path's curvature instead of clipping one wheel)."""
        vl = v + omega * self.track_mm / 2
        vr = v - omega * self.track_mm / 2
        left, right = vl / self.max_wheel_speed, vr / self.max_wheel_speed
        s = max(1.0, abs(left), abs(right))
        return left / s, right / s


def integrate(x: float, y: float, th: float, v: float, w: float, dt: float) -> tuple[float, float, float]:
    """Exact unicycle integration over dt (arc motion)."""
    if abs(w) < 1e-9:
        return x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th
    th2 = th + w * dt
    r = v / w
    return x + r * (math.sin(th2) - math.sin(th)), y - r * (math.cos(th2) - math.cos(th)), th2
