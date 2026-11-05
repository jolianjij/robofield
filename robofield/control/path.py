"""Simple path planning: a straight line from start to goal, kept inside a safe area
(a margin from the walls and towers), split into evenly spaced waypoints."""
from __future__ import annotations

import math


def clamp_to_field(x: float, y: float, field: float = 1000.0, margin: float = 60.0) -> tuple[float, float]:
    return min(max(x, margin), field - margin), min(max(y, margin), field - margin)


def straight_path(start: tuple[float, float], goal: tuple[float, float], step: float = 100.0,
                  field: float = 1000.0, margin: float = 60.0) -> list[tuple[float, float]]:
    """Waypoints from start to goal every `step` mm (goal included, start excluded).
    The goal is pulled inside the safe area so the robot never aims at a wall."""
    gx, gy = clamp_to_field(*goal, field, margin)
    sx, sy = start
    d = math.hypot(gx - sx, gy - sy)
    n = max(1, math.ceil(d / step))
    return [(sx + (gx - sx) * k / n, sy + (gy - sy) * k / n) for k in range(1, n + 1)]


def path_length(points, start=None) -> float:
    pts = ([start] if start else []) + list(points)
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
