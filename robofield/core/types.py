"""Shared data types. All positions in mm, angles in radians (see docs/coordinates.md)."""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


@dataclass(frozen=True)
class Pose:
    marker_id: int
    x: float          # mm
    y: float          # mm
    theta: float      # rad, 0 = +X, CCW positive
    t: float          # capture timestamp, seconds
    quality: float = 1.0  # 0..1, estimator confidence
