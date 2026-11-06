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
    """Turn in place to a heading: pulse, wait, measure, repeat.

    Every move is an open-loop pulse at the car's slowest turning speed, sized from a two-number
    model of this car - how long it takes to start turning from a standstill (`start_s`) and how
    fast it then turns (`turn_rate`) - followed by a wait (`wait_s`, longer than the ~0.2 s loop
    delay) until the camera shows it at rest. Only then is the next move decided, on a fresh,
    still measurement. Both model numbers are re-learned from every pulse's measured result: a
    pulse that did not move the robot lengthens `start_s`, one that over- or under-shot corrects
    `turn_rate`. So it keeps working as the battery, the floor and the motors change.

    Why not a continuous turn steering on the predicted heading: from a standstill this car needs
    extra time to overcome friction and wheel scrub before it rotates at all, so short corrections
    predicted to turn it in fact barely do - the controller sees a predicted heading that is
    already "close", stops, and the robot stalls at the wrong orientation; or, turning fast on
    stale camera data, it overshoots one way then the other. Waiting out the delay between moves
    removes both problems: no decision is ever made on data older than the move that preceded it."""
    dd: DiffDrive
    tolerance: float = math.radians(4.0)
    deadzone: float | tuple[float, float] = 0.0
    wait_s: float = 0.35
    pulse_speed: float = 150.0           # mm/s of the driving wheel while turning
    mode: str = "pivot"                  # "pivot" (one wheel), "spin" (both, in place), "continuous" (old)
    start_s: float = 0.06                # learned: time from a standstill before it really turns
    turn_rate: float = 2.5               # learned: rad/s once turning (pivot: wheel speed / track)
    pulse_max_s: float = 0.6
    still_omega: float = 0.4             # rad/s: below this the robot counts as stopped
    learn: dict | None = None            # share one learned model between several TurnTo's
    _t: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self):
        if self.learn is None:
            self.learn = {}
        self.learn.setdefault("start_s", self.start_s)
        self.learn.setdefault("turn_rate", self.turn_rate)
        self.reset()

    def reset(self, wait: bool = False) -> None:
        """wait=True: start by letting the robot settle (e.g. right after it stopped driving)."""
        self.state = "WAIT" if wait else "DECIDE"
        self._t0 = self._t
        self._pulse = None          # (length s, direction, error before) of the last pulse
        self._pulses = getattr(self, "_pulses", 0)

    def _turn(self, direction: float) -> RobotCommand:
        """One wheel drives, the other stays still: a pivot about the still wheel. Spinning both
        wheels in place needs a hard kick to break them free sideways and then coasts a long way
        (measured on the real car); one wheel breaks free gently, turns half as fast and stops
        sooner. Which wheel does the work alternates from pulse to pulse, so the small sideways
        shift of each pivot is undone by the next."""
        if self.mode == "spin":
            left, right = self.dd.inverse(0.0, direction * 2 * self.pulse_speed / self.dd.track_mm)
            return RobotCommand(left, right)
        forward = (self._pulses % 2 == 0)
        if direction > 0:                                  # counter-clockwise
            return (RobotCommand(0.0, self.dd.wheel_command(self.pulse_speed * self.dd.trim_right)) if forward
                    else RobotCommand(-self.dd.wheel_command(self.pulse_speed * self.dd.trim_left), 0.0))
        return (RobotCommand(self.dd.wheel_command(self.pulse_speed * self.dd.trim_left), 0.0) if forward
                else RobotCommand(0.0, -self.dd.wheel_command(self.pulse_speed * self.dd.trim_right)))

    def _learn(self, e: float) -> None:
        length, direction, e_before = self._pulse
        moved = (e_before - e) * direction                       # rad turned toward the target
        m = self.learn
        if moved < math.radians(1.0):                            # it hardly moved: it needs longer to start
            m["start_s"] = min(0.4, m["start_s"] + 0.03)
            return
        turning = length - m["start_s"]
        if turning > 0.03:
            m["turn_rate"] = min(12.0, max(0.5, 0.5 * m["turn_rate"] + 0.5 * moved / turning))
        else:                                                    # moved although it should only just have started
            m["start_s"] = max(0.0, m["start_s"] - 0.02)

    def command(self, th: float, target: float, omega: float = 0.0, dt: float = 1 / 30) -> tuple[RobotCommand, bool]:
        self._t += dt
        e = wrap_angle(target - th)

        if self.mode == "continuous":
            # the old controller: steer continuously on whatever heading it is given. Kept only to
            # show the difference - on the real car it oscillates left and right at the end
            if abs(e) < self.tolerance:
                return RobotCommand(), True
            w = 3.5 * e - 1.0 * omega
            speed = max(abs(w) * self.dd.track_mm / 2, self.pulse_speed)   # hard enough to actually turn
            left, right = self.dd.inverse(0.0, math.copysign(2 * speed / self.dd.track_mm, w))
            return RobotCommand(left, right), False

        if self.state == "PULSE":
            length, direction, _ = self._pulse
            if self._t - self._t0 < length:
                return self._turn(direction), False
            self.state, self._t0 = "WAIT", self._t
            return RobotCommand(), False

        if self.state == "WAIT":
            if self._t - self._t0 < self.wait_s or abs(omega) > self.still_omega:
                return RobotCommand(), False
            if self._pulse is not None:
                self._learn(e)
                self._pulse = None
            self.state = "DECIDE"

        if self.state == "DONE":
            if abs(e) < 1.5 * self.tolerance:
                return RobotCommand(), True
            self.state = "DECIDE"

        if abs(e) < self.tolerance:
            self.state = "DONE"
            return RobotCommand(), True
        length = min(self.pulse_max_s, self.learn["start_s"] + abs(e) / self.learn["turn_rate"])
        direction = 1.0 if e > 0 else -1.0
        self._pulse = (length, direction, e)
        self._pulses += 1
        self.state, self._t0 = "PULSE", self._t
        return self._turn(direction), False
