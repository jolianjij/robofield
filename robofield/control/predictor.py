"""Delay prediction: where each robot will actually be when the command computed now takes effect.

The camera, processing and radio add ~0.2 s between a command and the moment its effect is seen
(tools/measure_latency.py). A controller acting on the latest pose is therefore always correcting
where the robot WAS - and turning fast, it corrects a heading it has already passed, then corrects
back: the left-right oscillation.

Extrapolating the last measured speed does not fix that: when the controller reverses its steering,
that extrapolation keeps swinging the old way and invents an overshoot. Instead this is a Smith
predictor: start from the measured state and replay the commands that were actually sent during the
delay window - they are already on their way to the wheels but not yet visible to the camera -
through the robot's motor model (DiffDrive with the measured speed curve), with a short first-order
lag for the motors and the firmware's duty ramp.
"""
from __future__ import annotations

import math
from collections import deque

from .kinematics import DiffDrive


class LatencyPredictor:
    def __init__(self, dd: DiffDrive, latency: float, tau: float = 0.06, max_turn: float = 6.0,
                 step: float = 0.01):
        self.dd = dd
        self.latency = latency        # s: command sent -> its effect seen by the camera
        self.tau = tau                # s: wheel speed lag behind its command (motor + firmware ramp)
        self.max_turn = max_turn      # rad/s: turning in place scrubs the wheels; measured ~4-6 rad/s
        self.step = step
        self._sent: dict[int, deque] = {}

    def record(self, rid: int, t: float, left: float, right: float) -> None:
        """A command (laptop convention, -1..1 per wheel) sent to robot `rid` at time t."""
        v, w = self.dd.forward(left, right)
        q = self._sent.setdefault(rid, deque())
        q.append((t, v, max(-self.max_turn, min(self.max_turn, w))))
        while len(q) > 2 and q[1][0] < t - 2 * self.latency - 0.5:
            q.popleft()

    def _cmd_at(self, q, t):
        cmd = None
        for tc, v, w in q:
            if tc > t:
                break
            cmd = (v, w)
        return cmd

    def predict(self, rid: int, now: float, x: float, y: float, th: float, v: float, w: float):
        """Measured state at `now` -> predicted (x, y, th, v, w) `latency` seconds on."""
        q = self._sent.get(rid)
        if self.latency <= 0 or not q:
            return x, y, th, v, w
        n = max(1, round(self.latency / self.step))
        dt = self.latency / n
        a = min(1.0, dt / self.tau) if self.tau > 0 else 1.0
        for k in range(n):
            cmd = self._cmd_at(q, now - self.latency + k * dt)   # this command is what the wheels do then
            if cmd is not None:
                v += a * (cmd[0] - v)
                w += a * (cmd[1] - w)
            x += v * math.cos(th + 0.5 * w * dt) * dt
            y += v * math.sin(th + 0.5 * w * dt) * dt
            th += w * dt
        th = math.atan2(math.sin(th), math.cos(th))
        return x, y, th, v, w
