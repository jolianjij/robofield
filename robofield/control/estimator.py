"""Extended Kalman filter for one robot: smooths tracked poses, estimates speed and turn rate,
bridges missed detections, and predicts ahead to cancel the system latency."""
from __future__ import annotations

import math

import numpy as np

from robofield.core.types import wrap_angle


class RobotEKF:
    """State [x mm, y mm, theta rad, v mm/s, omega rad/s], unicycle motion model."""

    def __init__(self, pos_std: float = 1.0, th_std: float = 0.015,
                 accel_std: float = 1500.0, alpha_std: float = 40.0):
        self.x = None
        self.P = np.diag([100.0, 100.0, 0.1, 1e4, 10.0])
        self.R = np.diag([pos_std ** 2, pos_std ** 2, th_std ** 2])
        self.qa, self.qw = accel_std ** 2, alpha_std ** 2
        self.t = None

    def _predict(self, dt: float):
        x, y, th, v, w = self.x
        self.x = np.array([x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, wrap_angle(th + w * dt), v, w])
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * math.sin(th) * dt, math.cos(th) * dt
        F[1, 2], F[1, 3] = v * math.cos(th) * dt, math.sin(th) * dt
        F[2, 4] = dt
        Q = np.zeros((5, 5))
        Q[3, 3], Q[4, 4] = self.qa * dt ** 2, self.qw * dt ** 2
        Q[0, 0] = Q[1, 1] = 0.25 * self.qa * dt ** 4
        self.P = F @ self.P @ F.T + Q

    def update(self, t: float, x: float, y: float, th: float) -> None:
        z = np.array([x, y, th])
        if self.x is None:
            self.x = np.array([x, y, th, 0.0, 0.0]); self.t = t
            return
        self._predict(max(1e-4, t - self.t)); self.t = t
        H = np.zeros((3, 5)); H[0, 0] = H[1, 1] = H[2, 2] = 1
        r = z - H @ self.x
        r[2] = wrap_angle(r[2])
        if abs(r[0]) > 80 or abs(r[1]) > 80:          # a jump this big is a bad detection: re-lock
            self.x[:3] = z; self.x[3:] = 0; return
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ r
        self.x[2] = wrap_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def coast(self, t: float) -> None:
        """No detection this frame: advance on the model only."""
        if self.x is not None:
            self._predict(max(1e-4, t - self.t)); self.t = t

    def predicted(self, ahead: float) -> tuple[float, float, float, float, float]:
        """State `ahead` seconds in the future (latency compensation)."""
        x, y, th, v, w = self.x
        n = max(1, int(ahead / 0.01))
        dt = ahead / n
        for _ in range(n):
            x += v * math.cos(th) * dt; y += v * math.sin(th) * dt; th += w * dt
        return x, y, wrap_angle(th), v, w
