"""Safety layer between every controller and the robots. Runs on every command, every step.

Rules (each can be tuned in SafetySettings / the GUI):
  1. EMERGENCY STOP: latched, all motors off until released
  2. Tracking lost: a robot not seen by the camera for > lost_timeout_s is stopped
  3. Tracker silent: no camera data for > tracker_timeout_s -> everything stops
  4. Geofence: a robot predicted to leave the safe area (wall margin) loses its forward speed
     (it may still turn, so it can face away and leave the edge)
  5. Collision avoidance: a robot heading toward another one closer than min_separation stops moving
     toward it
  6. Speed limit: global cap on forward speed and turn rate
  7. Low battery: robot disabled (the car firmware also cuts its motors)
The car firmware adds its own last line of defence: 200 ms command watchdog, battery cutoff.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from robofield.control.goto import compensate_deadzone
from robofield.control.kinematics import DiffDrive
from robofield.core.state import RobotCommand, WorldState


@dataclass
class SafetySettings:
    speed_limit: float = 0.7              # fraction of top speed
    geofence_margin_mm: float = 50.0
    lookahead_s: float = 0.35             # how far ahead the geofence / collision check looks
    min_separation_mm: float = 130.0
    avoid_collisions: bool = True
    lost_timeout_s: float = 0.4
    tracker_timeout_s: float = 0.6
    low_battery_mv: int = 6600


@dataclass
class Safety:
    dd: DiffDrive
    field_mm: float = 1000.0
    deadzone: float = 0.0
    settings: SafetySettings = field(default_factory=SafetySettings)
    estop: bool = False
    _last_seen: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)       # robot id -> why it was limited this step

    def filter(self, ws: WorldState, cmds: dict[int, RobotCommand], tracker_ok: bool, now: float,
               battery: dict[int, int] | None = None) -> dict[int, RobotCommand]:
        s, out, self.events = self.settings, {}, {}
        for rid, r in ws.robots.items():
            if r.visible:
                self._last_seen[rid] = now
        for rid, c in cmds.items():
            stop = RobotCommand(0.0, 0.0, c.gripper, c.leds)
            r = ws.robots.get(rid)
            if self.estop:
                out[rid] = stop; self.events[rid] = "E-STOP"; continue
            if not tracker_ok:
                out[rid] = stop; self.events[rid] = "camera data lost"; continue
            if r is None or now - self._last_seen.get(rid, -1e9) > s.lost_timeout_s:
                out[rid] = stop; self.events[rid] = "not seen by camera"; continue
            if battery and 1000 < battery.get(rid, 99999) < s.low_battery_mv:
                out[rid] = stop; self.events[rid] = "battery low"; continue
            v, w = self.dd.forward(c.left, c.right)
            v0, w0 = v, w
            vmax, wmax = s.speed_limit * self.dd.max_wheel_speed, s.speed_limit * self.dd.max_omega
            if abs(v) > vmax:
                v = math.copysign(vmax, v); self.events.setdefault(rid, "speed limit")
            w = max(-wmax, min(wmax, w))
            hx, hy = math.cos(r.theta), math.sin(r.theta)
            if v != 0:                                     # geofence: predicted position
                px, py = r.x + v * hx * s.lookahead_s, r.y + v * hy * s.lookahead_s
                m = s.geofence_margin_mm
                outside_now = not (m <= r.x <= self.field_mm - m and m <= r.y <= self.field_mm - m)
                outside_next = not (m <= px <= self.field_mm - m and m <= py <= self.field_mm - m)
                towards_inside = (px - 500) ** 2 + (py - 500) ** 2 < (r.x - 500) ** 2 + (r.y - 500) ** 2
                if outside_next and not (outside_now and towards_inside):
                    v = 0.0; self.events[rid] = "geofence"
            if s.avoid_collisions and v != 0:              # collision: moving toward a close robot
                for oid, o in ws.robots.items():
                    if oid == rid or not o.visible:
                        continue
                    dx, dy = o.x - r.x, o.y - r.y
                    d = math.hypot(dx, dy)
                    approaching = (v * (hx * dx + hy * dy)) > 0
                    if d < s.min_separation_mm + abs(v) * s.lookahead_s and approaching:
                        v = 0.0; self.events[rid] = f"too close to robot {oid}"; break
            if v == v0 and w == w0:
                out[rid] = c                                # untouched: keep the controller's exact command
            else:
                left, right = self.dd.inverse(v, w)
                out[rid] = compensate_deadzone(RobotCommand(left, right, c.gripper, c.leds), self.deadzone)
        return out
