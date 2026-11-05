"""Go-to-point controller for a differential-drive robot (polar-coordinate law)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robofield.core.state import RobotCommand
from robofield.core.types import wrap_angle
from .kinematics import DiffDrive
from .pid import PID


def compensate_deadzone(cmd: RobotCommand, deadzone: float | tuple[float, float], eps: float = 1e-3) -> RobotCommand:
    """Lift small non-zero wheel commands above the motor dead zone so slow, precise moves happen.
    `deadzone` may be a single value (both wheels) or a (left, right) pair for motors that do not
    start moving at the same duty (measure each wheel separately, see docs/tomorrow.md)."""
    dzl, dzr = deadzone if isinstance(deadzone, tuple) else (deadzone, deadzone)
    f = lambda u, dz: 0.0 if abs(u) < eps else math.copysign(dz + (1 - dz) * abs(u), u)
    return RobotCommand(f(cmd.left, dzl), f(cmd.right, dzr), cmd.gripper, cmd.leds)


@dataclass
class GoToPoint:
    """Heading control is now a PID with derivative-on-measurement (`-kd * measured_omega`
    instead of differencing the noisy angle error), so it damps using the robot's actual,
    currently-measured turn rate. This is a genuine improvement for a system this noisy and
    laggy, but it was NOT the cause of the violent oscillation reported earlier: that was
    `DiffDrive.inverse()` commanding turns in the opposite sign from `forward()` (see kinematics.py
    and TurnTo's docstring) - every heading correction was reinforcing whatever rotation already
    existed rather than opposing it, which looks exactly like an undampable oscillation no matter
    how the gains are tuned. With that fixed, plain proportional control (as this had originally)
    converges in well under a second in simulation; the PID here is extra robustness headroom for
    the real camera's latency and noise, not a replacement for fixing the sign bug."""
    dd: DiffDrive
    k_rho: float = 2.5          # 1/s, forward gain
    k_alpha: float = 6.0        # 1/s, heading gain (PID kp)
    kd_alpha: float = 0.4       # damping on the measured turn rate (PID kd)
    w_cap_frac: float = 1.0     # heading-rate command capped to this fraction of dd.max_omega (1 = no cap)
    v_max: float = 250.0        # mm/s
    tolerance: float = 10.0     # mm
    slow_radius: float = 80.0   # mm, ramp the speed down inside this radius
    allow_reverse: bool = True
    deadzone: float | tuple[float, float] = 0.0   # set from the robot model; small commands are lifted above it

    def __post_init__(self):
        cap = self.w_cap_frac * self.dd.max_omega
        self._pid = PID(kp=self.k_alpha, kd=self.kd_alpha, out_min=-cap, out_max=cap)

    def reset(self) -> None:
        self._pid.reset()

    def command(self, x: float, y: float, th: float, gx: float, gy: float,
                omega: float = 0.0, dt: float = 1 / 30) -> tuple[RobotCommand, bool]:
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        if rho < self.tolerance:
            self.reset()
            return RobotCommand(), True
        alpha = wrap_angle(math.atan2(dy, dx) - th)
        direction = 1.0
        if self.allow_reverse and abs(alpha) > math.pi / 2:      # target behind: drive backwards
            direction, alpha = -1.0, wrap_angle(alpha + math.pi)
        v = min(self.v_max, self.k_rho * rho) * math.cos(alpha) ** 2
        v *= min(1.0, rho / self.slow_radius) ** 0.5 if rho < self.slow_radius else 1.0
        w = self._pid.step(alpha, omega, dt)
        left, right = self.dd.inverse(direction * v, w)
        return compensate_deadzone(RobotCommand(left, right), self.deadzone), False


@dataclass
class TurnTo:
    """Turn in place to a target heading.

    The violent, never-settling oscillation reported against this controller traced back to
    `DiffDrive.inverse()`: it commanded turns with the opposite sign from `forward()` (confirmed
    in simulation - commanding +2 rad/s produced a true, physical -1.66 rad/s). A closed-loop
    heading controller with that bug doesn't just turn the wrong way once: every correction
    reinforces whatever rotation already exists, since the controller "sees" its own commands as
    opposing a spin that they are actually accelerating. That is indistinguishable from a
    sustained limit cycle no matter how the gains are tuned - and was reproduced here even feeding
    the controller the true, noise-free pose and turn rate, which is what showed it was a sign bug
    and not a tuning or estimation problem. See kinematics.py's `inverse()` docstring for the fix.

    With that fixed, plain proportional control converges in well under a second in simulation.
    The PID here (derivative-on-measurement: `-kd * measured_omega`, not a differenced error) is
    a genuine but secondary improvement: real camera latency and pose noise are worse than sim, so
    some damping on the actual measured turn rate gives more margin before a real robot's
    oscillation would need retuning. Re-sweep `k` / `kd` on the real car; inertia, friction and
    true latency all differ from sim (see docs/tomorrow.md for the sweep method and results).

    Inside `creep_angle` it switches to short power pulses separated by pauses ("creep-pulse"),
    because a continuous command big enough to clear the motor's dead zone overshoots a target
    that is only a few degrees away; the pause lets the robot coast to a stop between pulses
    instead of oscillating around the target. Creep only engages below `creep_omega_max` too: a
    caller can start this close to the target while still spinning fast (e.g. PoseController
    handing off from FOLLOW to ALIGN) and creep's own pulses are far too weak to brake that, so
    without the speed gate it just sails through the target repeatedly instead of stopping. The
    PID keeps braking (using its real kd authority) until the spin has actually died down."""
    dd: DiffDrive
    k: float = 5.0              # 1/s, PID kp
    kd: float = 0.4             # damping on the measured turn rate
    w_cap_frac: float = 1.0     # commanded turn rate capped to this fraction of dd.max_omega (1 = no cap)
    tolerance: float = 0.03     # rad
    deadzone: float | tuple[float, float] = 0.0
    creep_angle: float = math.radians(8.0)    # below this AND slow enough, switch to creep-pulse
    creep_omega_max: float = 1.0               # rad/s: creep's weak pulses can't brake anything faster
    creep_power: float | None = None          # None = just above the dead zone (see __post_init__)
    pulse_on_s: float = 0.09
    pulse_off_s: float = 0.12
    _t: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self):
        cap = self.w_cap_frac * self.dd.max_omega
        self._pid = PID(kp=self.k, kd=self.kd, out_min=-cap, out_max=cap)
        if self.creep_power is None:
            dzl, dzr = self.deadzone if isinstance(self.deadzone, tuple) else (self.deadzone, self.deadzone)
            self.creep_power = max(dzl, dzr) * 1.15

    def reset(self) -> None:
        self._pid.reset()
        self._t = 0.0

    def _creep(self, e: float, dt: float) -> RobotCommand:
        self._t += dt
        on = (self._t % (self.pulse_on_s + self.pulse_off_s)) < self.pulse_on_s
        if not on:
            return RobotCommand()
        left, right = self.dd.inverse(0.0, math.copysign(1.0, e))
        s = max(abs(left), abs(right), 1e-9)
        return RobotCommand(left / s * self.creep_power, right / s * self.creep_power)

    def command(self, th: float, target: float, omega: float = 0.0, dt: float = 1 / 30) -> tuple[RobotCommand, bool]:
        e = wrap_angle(target - th)
        if abs(e) < self.tolerance:
            self.reset()
            return RobotCommand(), True
        if abs(e) < self.creep_angle and abs(omega) < self.creep_omega_max:
            return self._creep(e, dt), False
        self._t = 0.0
        w = self._pid.step(e, omega, dt)
        left, right = self.dd.inverse(0.0, w)
        return compensate_deadzone(RobotCommand(left, right), self.deadzone), False
