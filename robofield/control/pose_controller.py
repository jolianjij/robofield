"""Drive to a position AND an orientation, along a planned path. Three phases:

  1. TURN    - turn in place to face the path
  2. FOLLOW  - follow the waypoints (carrot / lookahead following), then stop precisely on the
               goal with the go-to-point law
  3. ALIGN   - turn in place to the requested final heading
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robofield.core.state import RobotCommand
from robofield.core.types import wrap_angle
from .goto import GoToPoint, TurnTo, compensate_deadzone
from .kinematics import DiffDrive
from .path import straight_path


@dataclass
class PoseController:
    dd: DiffDrive
    deadzone: float = 0.0
    lookahead: float = 90.0        # mm: carrot distance along the path
    v_path: float = 220.0          # mm/s while following the path
    k_heading: float = 5.0         # 1/s
    pos_tol: float = 10.0          # mm
    ang_tol: float = math.radians(1.5)
    settle_steps: int = 6          # heading must stay inside the tolerance this many steps (0.2 s)
    face_tol: float = math.radians(20.0)   # start driving once roughly facing the path
    field_mm: float = 1000.0
    margin: float = 60.0
    phase: str = "IDLE"
    path: list = field(default_factory=list)
    goal: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        self.goto = GoToPoint(self.dd, deadzone=self.deadzone, tolerance=self.pos_tol, v_max=180.0, allow_reverse=False)
        self.turn = TurnTo(self.dd, k=4.0, deadzone=self.deadzone, tolerance=self.ang_tol)
        self.face = TurnTo(self.dd, deadzone=self.deadzone, tolerance=self.face_tol)

    def set_goal(self, x: float, y: float, theta: float, start_xy: tuple[float, float]):
        self.path = straight_path(start_xy, (x, y), field=self.field_mm, margin=self.margin)
        gx, gy = self.path[-1]
        self.goal = (gx, gy, theta)
        self.phase = "TURN" if math.dist(start_xy, (gx, gy)) > self.pos_tol else "ALIGN"
        self._align = True
        self._idx = 0
        self._settled = 0
        self._align_steps = 0

    def set_path(self, points, start_xy: tuple[float, float], final_theta: float | None = None, spacing: float = 30.0):
        """Follow a drawn path (list of (x, y) mm). Points are kept inside the safe area and
        resampled every `spacing` mm. With final_theta, the robot turns to it at the end."""
        from .path import clamp_to_field
        pts = [clamp_to_field(x, y, self.field_mm, self.margin) for x, y in points]
        res = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            d = math.dist(a, b)
            n = max(1, int(d // spacing))
            res += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        # start from the path point nearest the robot so it does not drive back to the beginning
        i0 = min(range(len(res)), key=lambda i: math.dist(res[i], start_xy))
        self.path = res[i0:] if i0 < len(res) - 1 else res[-1:]
        gx, gy = self.path[-1]
        self.goal = (gx, gy, final_theta if final_theta is not None else 0.0)
        self._align = final_theta is not None
        self.phase = "TURN"
        self._idx = 0
        self._settled = 0
        self._align_steps = 0

    def _carrot(self, x, y):
        """Point `lookahead` mm ahead along the path from the waypoint nearest the robot."""
        while self._idx < len(self.path) - 1 and math.dist((x, y), self.path[self._idx]) < self.lookahead:
            self._idx += 1
        return self.path[self._idx]

    def command(self, x: float, y: float, th: float) -> RobotCommand:
        gx, gy, gth = self.goal
        if self.phase == "TURN":
            cx, cy = self._carrot(x, y)
            cmd, done = self.face.command(th, math.atan2(cy - y, cx - x))
            if done:
                self.phase = "FOLLOW"
            else:
                return cmd
        if self.phase == "FOLLOW":
            near_end = self._idx >= len(self.path) - 1
            if near_end and math.dist((x, y), (gx, gy)) < self.lookahead:   # last stretch: precise stop
                cmd, done = self.goto.command(x, y, th, gx, gy)
                if not done:
                    return cmd
                self.phase = "ALIGN" if getattr(self, "_align", True) else "DONE"
            else:
                cx, cy = self._carrot(x, y)
                alpha = wrap_angle(math.atan2(cy - y, cx - x) - th)
                v = self.v_path * max(0.0, math.cos(alpha))
                left, right = self.dd.inverse(v, self.k_heading * alpha)
                return compensate_deadzone(RobotCommand(left, right), self.deadzone)
        if self.phase == "ALIGN":
            cmd, done = self.turn.command(th, gth)
            # the system has ~70 ms of delay, so only finish once the heading has STAYED in tolerance
            self._settled = self._settled + 1 if done else 0
            self._align_steps += 1
            err = abs(wrap_angle(gth - th))
            # finish when settled, or after 3 s if within 3 deg (a stalled motor dead zone can stop
            # it just short; measure the real dead zone in system identification to avoid this)
            if self._settled >= self.settle_steps or (self._align_steps > 90 and err < math.radians(3)):
                self.phase = "DONE"
            return cmd if not done else RobotCommand()
        return RobotCommand()

    @property
    def done(self) -> bool:
        return self.phase == "DONE"
