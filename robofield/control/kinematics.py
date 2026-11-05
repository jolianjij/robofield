"""Differential-drive kinematics for the RoboField car."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DiffDrive:
    wheel_radius_mm: float
    track_mm: float
    max_wheel_rpm: float
    trim_left: float = 1.0     # multiplicative per-wheel speed trim (measure real drift, see docs/tomorrow.md)
    trim_right: float = 1.0

    @classmethod
    def from_config(cls, cfg: dict) -> "DiffDrive":
        m = cfg["robot_model"]
        return cls(m["wheel_radius_mm"], m["track_mm"], m["max_wheel_rpm"],
                   m.get("trim_left", 1.0), m.get("trim_right", 1.0))

    @property
    def max_wheel_speed(self) -> float:
        """mm/s at the wheel rim at full command."""
        return self.max_wheel_rpm * 2 * math.pi / 60 * self.wheel_radius_mm

    @property
    def max_omega(self) -> float:
        """rad/s when spinning in place at full command."""
        return 2 * self.max_wheel_speed / self.track_mm

    def forward(self, left: float, right: float) -> tuple[float, float]:
        """Wheel commands (-1..1) -> body (v mm/s, omega rad/s). Trim is a firmware-level
        correction applied to the commanded duty, so it is undone here to recover true body
        motion from a trimmed command."""
        vl = (left / self.trim_left if self.trim_left else left) * self.max_wheel_speed
        vr = (right / self.trim_right if self.trim_right else right) * self.max_wheel_speed
        return (vr + vl) / 2, (vr - vl) / self.track_mm

    def inverse(self, v: float, omega: float) -> tuple[float, float]:
        """Body (v, omega) -> wheel commands, scaled down together if any exceeds +-1
        (keeps the path's curvature instead of clipping one wheel), then trimmed per wheel to
        correct a straight-line drift from unmatched motors.

        Must match `forward()`'s sign convention exactly (forward(*inverse(v, w)) == (v, w)):
        this was previously inverted (vl/vr swapped), so commanding a turn rate made the robot
        spin the opposite way. A closed-loop heading controller fighting its own sign error
        doesn't just turn the wrong way once - every correction reinforces whatever rotation
        already exists, which is indistinguishable from a sustained, undampable oscillation.
        Confirmed in sim: commanding w=+2.0 rad/s through the old inverse() produced true
        omega=-1.66 rad/s. Verify this sign on the real car (`tools/cars.py test 0`) before
        trusting any heading control there."""
        vl = v - omega * self.track_mm / 2
        vr = v + omega * self.track_mm / 2
        left, right = vl / self.max_wheel_speed, vr / self.max_wheel_speed
        s = max(1.0, abs(left), abs(right))
        left, right = left / s, right / s
        left, right = left * self.trim_left, right * self.trim_right
        return max(-1.0, min(1.0, left)), max(-1.0, min(1.0, right))


def deadzones_from_config(cfg: dict) -> tuple[float, float]:
    """Per-wheel motor dead zone (left, right): falls back to the single `deadzone` value
    when `deadzone_left` / `deadzone_right` are not set, so unmeasured robots keep working."""
    m = cfg["robot_model"]
    base = m.get("deadzone", 0.0)
    return m.get("deadzone_left", base), m.get("deadzone_right", base)


def safety_margin_mm(cfg: dict) -> float:
    """Clearance from the walls for path planning / the geofence, grounded in the robot's own
    footprint (half its diagonal, so a corner never clips a wall even mid-turn) plus a
    configurable extra clearance for tracking/estimator error."""
    m = cfg["robot_model"]
    half_diag = math.hypot(m["length_mm"], m["width_mm"]) / 2
    clearance = cfg.get("control", {}).get("clearance_mm", 20.0)
    return half_diag + clearance


def integrate(x: float, y: float, th: float, v: float, w: float, dt: float) -> tuple[float, float, float]:
    """Exact unicycle integration over dt (arc motion)."""
    if abs(w) < 1e-9:
        return x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th
    th2 = th + w * dt
    r = v / w
    return x + r * (math.sin(th2) - math.sin(th)), y - r * (math.cos(th2) - math.cos(th)), th2
