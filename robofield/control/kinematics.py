"""Differential-drive kinematics for the RoboField car."""
from __future__ import annotations

import math
from bisect import bisect_left
from dataclasses import dataclass


@dataclass(frozen=True)
class DiffDrive:
    wheel_radius_mm: float
    track_mm: float
    max_wheel_rpm: float
    trim_left: float = 1.0     # per-wheel speed correction for a straight-line drift (tools/calibrate_motors.py)
    trim_right: float = 1.0
    # Measured wheel speed (mm/s) per command, as ((u, speed), ...) rising from (0, 0). The real car is
    # far from linear: its firmware lifts every non-zero command above each wheel's dead zone, so the
    # smallest command already drives ~100 mm/s (see config/field.yaml). Empty = ideal linear motor.
    speed_curve: tuple = ()

    @classmethod
    def from_config(cls, cfg: dict) -> "DiffDrive":
        m = cfg["robot_model"]
        curve = tuple((float(u), float(s)) for u, s in m.get("speed_curve", ()))
        return cls(m["wheel_radius_mm"], m["track_mm"], m["max_wheel_rpm"],
                   m.get("trim_left", 1.0), m.get("trim_right", 1.0), curve)

    @property
    def max_wheel_speed(self) -> float:
        """mm/s at the wheel rim at full command."""
        if self.speed_curve:
            return self.speed_curve[-1][1]
        return self.max_wheel_rpm * 2 * math.pi / 60 * self.wheel_radius_mm

    @property
    def min_wheel_speed(self) -> float:
        """Slowest speed a wheel can turn continuously (0 for an ideal motor)."""
        return self.speed_curve[1][1] if len(self.speed_curve) > 1 else 0.0

    @property
    def max_omega(self) -> float:
        """rad/s when spinning in place at full command."""
        return 2 * self.max_wheel_speed / self.track_mm

    # ---------------------------------------------------------------- one wheel
    def wheel_speed(self, u: float) -> float:
        """Command (-1..1) -> wheel rim speed (mm/s)."""
        if not self.speed_curve:
            return u * self.max_wheel_speed
        a = abs(u)
        us = [p[0] for p in self.speed_curve]
        i = min(max(1, bisect_left(us, a)), len(us) - 1)
        (u0, s0), (u1, s1) = self.speed_curve[i - 1], self.speed_curve[i]
        s = s0 + (s1 - s0) * (a - u0) / (u1 - u0) if u1 > u0 else s1
        return math.copysign(min(s, self.max_wheel_speed), u)

    def wheel_command(self, speed: float) -> float:
        """Wheel rim speed (mm/s) -> command. A speed below the slowest the wheel can hold is
        rounded to whichever is closer: stopped, or that slowest speed."""
        if not self.speed_curve:
            return max(-1.0, min(1.0, speed / self.max_wheel_speed))
        a = abs(speed)
        if a < 1.0 or a < self.min_wheel_speed / 2:
            return 0.0
        a = max(a, self.min_wheel_speed)
        ss = [p[1] for p in self.speed_curve]
        i = min(max(1, bisect_left(ss, a)), len(ss) - 1)
        (u0, s0), (u1, s1) = self.speed_curve[i - 1], self.speed_curve[i]
        u = u0 + (u1 - u0) * (a - s0) / (s1 - s0) if s1 > s0 else u1
        return math.copysign(min(1.0, u), speed)

    # ---------------------------------------------------------------- body
    def forward(self, left: float, right: float) -> tuple[float, float]:
        """Wheel commands (-1..1) -> body (v mm/s, omega rad/s). Undoes the trim that inverse()
        applied, so this recovers the motion that was asked for."""
        vl = self.wheel_speed(left) / (self.trim_left or 1.0)
        vr = self.wheel_speed(right) / (self.trim_right or 1.0)
        return (vr + vl) / 2, (vr - vl) / self.track_mm

    def inverse(self, v: float, omega: float) -> tuple[float, float]:
        """Body (v, omega) -> wheel commands. Wheel speeds are scaled down together if either is
        over the top speed (keeps the path's curvature instead of clipping one wheel), trimmed per
        wheel, then mapped through the measured speed curve.

        Must match `forward()`'s sign convention exactly (forward(*inverse(v, w)) == (v, w)):
        this was previously inverted (vl/vr swapped), so commanding a turn rate made the robot
        spin the opposite way. A closed-loop heading controller fighting its own sign error
        doesn't just turn the wrong way once - every correction reinforces whatever rotation
        already exists, which is indistinguishable from a sustained, undampable oscillation.
        Confirmed in sim (commanding w=+2.0 rad/s produced -1.66 rad/s) and on the real car."""
        vl = v - omega * self.track_mm / 2
        vr = v + omega * self.track_mm / 2
        top = self.max_wheel_speed
        s = max(1.0, abs(vl) / top, abs(vr) / top)
        vl, vr = vl / s, vr / s
        # Slower than the motors can go: speed both wheels up together (same ratio, same curvature)
        # until the faster one is at the slowest real speed. A controller asking for "slowly" still
        # moves, instead of stalling short of every goal; the slower wheel then rounds to 0 or the
        # minimum, whichever is closer, so low-speed steering is coarse but never lost.
        m = max(abs(vl), abs(vr))
        if 1.0 <= m < self.min_wheel_speed:
            vl, vr = vl * self.min_wheel_speed / m, vr * self.min_wheel_speed / m
        return self.wheel_command(vl * self.trim_left), self.wheel_command(vr * self.trim_right)


def deadzones_from_config(cfg: dict) -> tuple[float, float]:
    """Per-wheel dead zone the LAPTOP should compensate (left, right). 0 for a car whose firmware
    already does it (config/field.yaml explains); falls back to the single `deadzone` value."""
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
