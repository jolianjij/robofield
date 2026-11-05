"""Go-to-point controller for a differential-drive robot (polar-coordinate law)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from robofield.core.state import RobotCommand
from robofield.core.types import wrap_angle
from .kinematics import DiffDrive


def compensate_deadzone(cmd: RobotCommand, deadzone: float, eps: float = 1e-3) -> RobotCommand:
    """Lift small non-zero wheel commands above the motor dead zone so slow, precise moves happen."""
    f = lambda u: 0.0 if abs(u) < eps else math.copysign(deadzone + (1 - deadzone) * abs(u), u)
    return RobotCommand(f(cmd.left), f(cmd.right), cmd.gripper, cmd.leds)


@dataclass
class GoToPoint:
    dd: DiffDrive
    k_rho: float = 2.5          # 1/s, forward gain
    k_alpha: float = 6.0        # 1/s, heading gain
    v_max: float = 250.0        # mm/s
    tolerance: float = 10.0     # mm
    slow_radius: float = 80.0   # mm, ramp the speed down inside this radius
    allow_reverse: bool = True
    deadzone: float = 0.0       # set from the robot model; small commands are lifted above it

    def command(self, x: float, y: float, th: float, gx: float, gy: float) -> tuple[RobotCommand, bool]:
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        if rho < self.tolerance:
            return RobotCommand(), True
        alpha = wrap_angle(math.atan2(dy, dx) - th)
        direction = 1.0
        if self.allow_reverse and abs(alpha) > math.pi / 2:      # target behind: drive backwards
            direction, alpha = -1.0, wrap_angle(alpha + math.pi)
        v = min(self.v_max, self.k_rho * rho) * math.cos(alpha) ** 2
        v *= min(1.0, rho / self.slow_radius) ** 0.5 if rho < self.slow_radius else 1.0
        w = self.k_alpha * alpha
        left, right = self.dd.inverse(direction * v, w)
        return compensate_deadzone(RobotCommand(left, right), self.deadzone), False


@dataclass
class TurnTo:
    dd: DiffDrive
    k: float = 5.0
    tolerance: float = 0.03
    deadzone: float = 0.0

    def command(self, th: float, target: float) -> tuple[RobotCommand, bool]:
        e = wrap_angle(target - th)
        if abs(e) < self.tolerance:
            return RobotCommand(), True
        left, right = self.dd.inverse(0.0, self.k * e)
        return compensate_deadzone(RobotCommand(left, right), self.deadzone), False
