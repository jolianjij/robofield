"""Point and heading controllers for a differential-drive robot.

Both are designed around two facts measured on the real car (2026-11-06):
  - the loop delay (command -> wheels -> camera -> laptop) is ~0.2 s, so the pose the controller
    sees is old; the backend predicts it forward, but a prediction is only as good as its model;
  - the car cannot move slowly: the smallest command already drives a wheel at ~100 mm/s
    (robot_model.speed_curve), so "a little correction" is never small.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robofield.core.state import RobotCommand
from robofield.core.types import wrap_angle
from .kinematics import DiffDrive
from .pid import PID


def compensate_deadzone(cmd: RobotCommand, deadzone: float | tuple[float, float], eps: float = 1e-3) -> RobotCommand:
    """Lift small non-zero wheel commands above a motor dead zone the laptop has to handle itself.
    The current car's firmware already does this per wheel, so its configured dead zone is 0 and
    this is a no-op; kept for cars whose firmware does not."""
    dzl, dzr = deadzone if isinstance(deadzone, tuple) else (deadzone, deadzone)
    f = lambda u, dz: 0.0 if abs(u) < eps else math.copysign(dz + (1 - dz) * abs(u), u)
    return RobotCommand(f(cmd.left, dzl), f(cmd.right, dzr), cmd.gripper, cmd.leds)


@dataclass
class GoToPoint:
    """Drive to a point. Speed falls off with distance and with heading error; the heading term
    is a PID damped on the measured turn rate. Arriving at speed, the robot accepts a goal it has
    just passed (within `pass_tol`) instead of reversing back and forth over it: with a ~100 mm/s
    minimum speed and a 0.2 s delay, chasing the last few millimetres is what makes it hunt."""
    dd: DiffDrive
    k_rho: float = 2.5          # 1/s, forward gain
    k_alpha: float = 3.5        # 1/s, heading gain (PID kp)
    kd_alpha: float = 1.0       # damping on the measured turn rate (PID kd)
    w_cap_frac: float = 0.45    # heading-rate command capped to this fraction of dd.max_omega
    v_max: float = 200.0        # mm/s
    tolerance: float = 12.0     # mm
    pass_tol: float = 20.0      # mm: a goal this close and already behind the robot counts as reached
    allow_reverse: bool = True  # back up toward a goal that is well behind
    deadzone: float | tuple[float, float] = 0.0

    def __post_init__(self):
        cap = self.w_cap_frac * self.dd.max_omega
        self._pid = PID(kp=self.k_alpha, kd=self.kd_alpha, out_min=-cap, out_max=cap)

    def reset(self) -> None:
        self._pid.reset()

    def command(self, x: float, y: float, th: float, gx: float, gy: float,
                omega: float = 0.0, dt: float = 1 / 30) -> tuple[RobotCommand, bool]:
        # a bad detection or EKF re-lock can report an impossible turn rate; don't let the
        # derivative term react to it
        omega = max(-self.dd.max_omega, min(self.dd.max_omega, omega))
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - th)
        behind = abs(alpha) > math.pi / 2
        if rho < self.tolerance or (behind and rho < self.pass_tol):
            self.reset()
            return RobotCommand(), True
        direction = 1.0
        if self.allow_reverse and behind:
            direction, alpha = -1.0, wrap_angle(alpha + math.pi)
        v = min(self.v_max, self.k_rho * rho) * max(0.0, math.cos(alpha)) ** 2
        w = self._pid.step(alpha, omega, dt)
        left, right = self.dd.inverse(direction * v, w)
        return compensate_deadzone(RobotCommand(left, right), self.deadzone), False


@dataclass
class TurnTo:
    """Turn in place to a heading: turn, stop, wait, nudge.

    Far from the target (outside `fine_angle`) it turns continuously, steering on the PREDICTED
    heading, so it stops early enough to account for the delay. Inside `fine_angle` it never turns
    continuously: it stops, waits `wait_s` (longer than the loop delay, so the camera has caught up
    and the robot has really stopped), then makes one short pulse sized for the remaining error,
    and waits again. Each pulse's result teaches the next one (`pulse_s_per_rad` adapts), so the
    nudges stay right as the battery, floor and motors change.

    Why not a continuous controller all the way in: with a ~0.2 s delay a continuous correction is
    always acting on where the robot WAS, and since the slowest it can turn is several rad/s (the
    motors cannot crawl), every correction overshoots the other way - the left-right oscillation
    seen on the real car. Waiting out the delay between moves removes the stale-data problem
    entirely; the predictor only has to be good enough to get the coarse turn close.

    With `continuous=True` (used to face along a path before driving) it just turns until within
    `tolerance`, with no waiting or pulses."""
    dd: DiffDrive
    k: float = 3.5              # 1/s, coarse turn PID kp
    kd: float = 1.0             # damping on the measured turn rate
    w_cap_frac: float = 0.45    # coarse turn rate cap, fraction of dd.max_omega
    tolerance: float = math.radians(4.0)
    deadzone: float | tuple[float, float] = 0.0
    fine_angle: float = math.radians(25.0)
    wait_s: float = 0.35
    pulse_speed: float = 100.0           # wheel speed during a nudge (mm/s); the car's slowest
    pulse_s_per_rad: float = 0.25        # first guess; learned from every nudge
    pulse_min_s: float = 0.03
    pulse_max_s: float = 0.25
    still_omega: float = 0.4             # rad/s: below this the robot counts as stopped
    continuous: bool = False
    _t: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self):
        cap = self.w_cap_frac * self.dd.max_omega
        self._pid = PID(kp=self.k, kd=self.kd, out_min=-cap, out_max=cap)
        self.reset()

    def reset(self, wait: bool = False) -> None:
        """wait=True: start by letting the robot settle (e.g. right after it stopped driving)."""
        self._pid.reset()
        self.state = "WAIT" if wait else "DECIDE"
        self._t0 = self._t
        self._pulse = None          # (length s, direction, error before) of the last nudge

    def _turn(self, w: float) -> RobotCommand:
        left, right = self.dd.inverse(0.0, w)
        return RobotCommand(left, right)

    def command(self, th: float, target: float, omega: float = 0.0, dt: float = 1 / 30) -> tuple[RobotCommand, bool]:
        omega = max(-self.dd.max_omega, min(self.dd.max_omega, omega))
        self._t += dt
        e = wrap_angle(target - th)
        if self.continuous:
            if abs(e) < self.tolerance:
                self._pid.reset()
                return RobotCommand(), True
            return self._turn(self._pid.step(e, omega, dt)), False

        if self.state == "PULSE":
            length, direction, _ = self._pulse
            if self._t - self._t0 < length:
                return self._turn(direction * 2 * self.pulse_speed / self.dd.track_mm), False
            self.state, self._t0 = "WAIT", self._t
            return RobotCommand(), False

        if self.state == "WAIT":
            if self._t - self._t0 < self.wait_s or abs(omega) > self.still_omega:
                return RobotCommand(), False
            if self._pulse is not None:                       # learn from the last nudge
                length, direction, e_before = self._pulse
                moved = (e_before - e) * direction            # rad turned toward the target
                if moved > math.radians(0.5):
                    seen = length / moved
                    self.pulse_s_per_rad = min(2.0, max(0.02, 0.5 * self.pulse_s_per_rad + 0.5 * seen))
                self._pulse = None
            self.state = "DECIDE"

        if self.state == "DONE":
            if abs(e) < 1.5 * self.tolerance:
                return RobotCommand(), True
            self.state = "DECIDE"

        if self.state == "TURN" and abs(e) <= self.fine_angle:   # close: stop and look again
            self._pid.reset()
            self.state, self._t0 = "WAIT", self._t
            return RobotCommand(), False

        # DECIDE (or still turning)
        if abs(e) < self.tolerance:
            self.state = "DONE"
            return RobotCommand(), True
        if abs(e) > self.fine_angle:
            self.state = "TURN"
            return self._turn(self._pid.step(e, omega, dt)), False
        length = min(self.pulse_max_s, max(self.pulse_min_s, self.pulse_s_per_rad * abs(e)))
        direction = 1.0 if e > 0 else -1.0
        self._pulse = (length, direction, e)
        self.state, self._t0 = "PULSE", self._t
        return self._turn(direction * 2 * self.pulse_speed / self.dd.track_mm), False
