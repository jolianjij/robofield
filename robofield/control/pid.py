"""Generic PID controller with derivative-on-measurement and anti-windup.

Derivative-on-measurement (-kd * measured_rate instead of differencing the error) avoids
derivative kick from noisy pose estimates: it damps the robot's actual measured turn rate / speed
(from the EKF) rather than a numerically-differentiated error signal.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PID:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    i_max: float = 0.0          # anti-windup clamp on the integral term's own contribution; 0 disables ki
    out_min: float | None = None
    out_max: float | None = None
    _integral: float = field(default=0.0, init=False, repr=False)

    def reset(self) -> None:
        self._integral = 0.0

    def step(self, error: float, measured_rate: float, dt: float) -> float:
        """error: target - measured (same units whose derivative is `measured_rate`).
        measured_rate: the measured derivative of the controlled quantity (e.g. EKF omega)."""
        if self.ki and dt > 0:
            self._integral = max(-self.i_max, min(self.i_max, self._integral + error * dt))
        out = self.kp * error + self.ki * self._integral - self.kd * measured_rate
        if self.out_min is not None or self.out_max is not None:
            lo = self.out_min if self.out_min is not None else -float("inf")
            hi = self.out_max if self.out_max is not None else float("inf")
            out = max(lo, min(hi, out))
        return out
