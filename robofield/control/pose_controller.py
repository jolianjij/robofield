"""Drive to a position AND an orientation, along a planned or drawn path. Three phases:

  1. TURN    - turn in place to face the path (TurnTo: pulse, wait, measure)
  2. FOLLOW  - pure pursuit along the path to the goal point, on the delay-predicted pose
  3. ALIGN   - turn in place to the requested final heading (TurnTo, tighter tolerance)

FOLLOW is pure pursuit: aim at a "carrot" `lookahead` mm ahead on the path and drive the circular
arc that reaches it (curvature = 2 sin(alpha) / distance). It is a geometric law, not a tuned
heading loop, so straight lines do not weave and curves are followed as curves. Speed ramps up at
`accel`, is capped in tight curves so the sideways acceleration stays under `lat_accel`, and ramps
down toward the goal like a real braking distance (v = sqrt(2 * accel * distance left)).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robofield.core.state import RobotCommand
from robofield.core.types import wrap_angle
from .goto import GoToPoint, TurnTo
from .kinematics import DiffDrive
from .path import straight_path


@dataclass
class PoseController:
    dd: DiffDrive
    deadzone: float | tuple[float, float] = 0.0
    # FOLLOW (pure pursuit)
    lookahead: float = 120.0       # mm: carrot distance along the path
    v_path: float = 200.0          # mm/s cruise speed
    accel: float = 400.0           # mm/s^2: speeding up, and the braking distance into the goal
    lat_accel: float = 500.0       # mm/s^2: caps speed in curves (v^2 * curvature)
    reface_angle: float = math.radians(70.0)   # carrot this far off the nose: stop and turn to it
    pos_tol: float = 12.0          # mm
    pass_tol: float = 20.0         # mm: a goal this close and already behind counts as reached
    # TURN (face the path)
    face_tol: float = math.radians(20.0)
    # ALIGN (final heading) - see TurnTo
    ang_tol: float = math.radians(4.0)
    wait_s: float = 0.35
    pulse_speed: float = 150.0     # mm/s of the driving wheel during a turn pulse
    turn_style: float = 1.0        # 1: pulse-and-wait pivots; 0: the old continuous turn (to film the difference)
    settle_steps: int = 3          # ALIGN must report "done" this many steps in a row
    field_mm: float = 1000.0
    margin: float = 60.0
    phase: str = "IDLE"
    path: list = field(default_factory=list)
    goal: tuple = (0.0, 0.0, 0.0)
    learned: dict = field(default_factory=dict)

    def __post_init__(self):
        # self.learned: this car's turning model, learned by both turns from every pulse (pass the
        # same dict to a rebuilt controller to keep what it learned)
        mode = "pivot" if self.turn_style >= 0.5 else "continuous"
        # final turn: shorter pulses, so a big turn is split across alternating wheels and the
        # sideways shift of each pivot is mostly undone by the next
        self.turn = TurnTo(self.dd, tolerance=self.ang_tol, wait_s=self.wait_s, pulse_speed=self.pulse_speed,
                           pulse_max_s=0.35, learn=self.learned, mode=mode)
        self.face = TurnTo(self.dd, tolerance=self.face_tol, wait_s=self.wait_s, pulse_speed=self.pulse_speed,
                           learn=self.learned, mode=mode)
        self.goto = GoToPoint(self.dd, tolerance=self.pos_tol, pass_tol=self.pass_tol)   # kept for tools
        self._v = 0.0

    # ---------------------------------------------------------------- goals
    def _start(self, start_xy, align: bool):
        gx, gy = self.path[-1]
        self._align = align
        self._idx = 0
        self._settled = 0
        self._align_steps = 0
        self._v = 0.0
        self.face.reset()
        self.turn.reset()
        self.phase = "TURN" if math.dist(start_xy, (gx, gy)) > self.pos_tol else ("ALIGN" if align else "DONE")

    def set_goal(self, x: float, y: float, theta: float, start_xy: tuple[float, float]):
        self.path = straight_path(start_xy, (x, y), step=40.0, field=self.field_mm, margin=self.margin)
        gx, gy = self.path[-1]
        self.goal = (gx, gy, theta)
        self._start(start_xy, True)

    def set_path(self, points, start_xy: tuple[float, float], final_theta: float | None = None, spacing: float = 20.0):
        """Follow a drawn path (list of (x, y) mm). Points are kept inside the safe area and
        resampled every `spacing` mm. With final_theta, the robot turns to it at the end."""
        from .path import clamp_to_field
        pts = [clamp_to_field(x, y, self.field_mm, self.margin) for x, y in points]
        res = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.dist(a, b) // spacing))
            res += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        # start from the path point nearest the robot so it does not drive back to the beginning
        i0 = min(range(len(res)), key=lambda i: math.dist(res[i], start_xy))
        self.path = res[i0:] if i0 < len(res) - 1 else res[-1:]
        gx, gy = self.path[-1]
        self.goal = (gx, gy, final_theta if final_theta is not None else 0.0)
        self._start(start_xy, final_theta is not None)

    # ---------------------------------------------------------------- helpers
    def _carrot(self, x, y):
        """Point `lookahead` mm ahead along the path from the waypoint nearest the robot."""
        while self._idx < len(self.path) - 1 and math.dist((x, y), self.path[self._idx]) < self.lookahead:
            self._idx += 1
        return self.path[self._idx]

    def _remaining(self, x, y) -> float:
        """Distance left along the path to the goal."""
        p = self.path
        return math.dist((x, y), p[self._idx]) + sum(math.dist(p[i], p[i + 1]) for i in range(self._idx, len(p) - 1))

    # ---------------------------------------------------------------- control
    def command(self, x: float, y: float, th: float, omega: float = 0.0, dt: float = 1 / 30) -> RobotCommand:
        gx, gy, gth = self.goal
        if self.phase == "TURN":
            cx, cy = self._carrot(x, y)
            cmd, done = self.face.command(th, math.atan2(cy - y, cx - x), omega, dt)
            if not done:
                return cmd
            self.phase, self._v = "FOLLOW", 0.0

        if self.phase == "FOLLOW":
            d_goal = math.dist((x, y), (gx, gy))
            behind = abs(wrap_angle(math.atan2(gy - y, gx - x) - th)) > math.pi / 2
            at_end = self._idx >= len(self.path) - 1      # a looping path passes its own end point early
            if at_end and (d_goal < self.pos_tol or (behind and d_goal < self.pass_tol)):
                self.phase = "ALIGN" if self._align else "DONE"
                self.turn.reset(wait=True)
                return RobotCommand()
            cx, cy = self._carrot(x, y)
            dist = max(1.0, math.dist((x, y), (cx, cy)))
            alpha = wrap_angle(math.atan2(cy - y, cx - x) - th)
            if abs(alpha) > self.reface_angle and d_goal > self.pass_tol:
                self.phase = "TURN"
                self.face.reset()
                return RobotCommand()
            kappa = 2 * math.sin(alpha) / dist
            v_goal = math.sqrt(2 * self.accel * self._remaining(x, y))
            v_curve = math.sqrt(self.lat_accel / max(abs(kappa), 1e-6))
            target = min(self.v_path, v_goal, v_curve)
            self._v = target if target < self._v else min(target, self._v + self.accel * dt)
            v = max(self._v, self.dd.min_wheel_speed)       # the car cannot go slower than this anyway
            left, right = self.dd.inverse(v, v * kappa)
            return RobotCommand(left, right)

        if self.phase == "ALIGN":
            cmd, done = self.turn.command(th, gth, omega, dt)
            self._settled = self._settled + 1 if done else 0
            self._align_steps += 1
            # a hard ceiling, so a robot that cannot settle (stuck wheel, lost tracking) stops trying
            if self._settled >= self.settle_steps or self._align_steps > 900:
                self.phase = "DONE"
            return cmd
        return RobotCommand()

    @property
    def done(self) -> bool:
        return self.phase == "DONE"
