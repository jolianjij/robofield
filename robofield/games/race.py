"""Race: 3 laps around the outer ring of field tiles, as fast as possible, without leaving the
ring (into the forbidden inner 2x2 block) or scraping the outer wall.

The field is a 4x4 grid of `tile_mm` tiles (see config/field.yaml `field.tile_size_mm`); the outer
ring is the 12 border tiles, a `tile_mm`-wide square loop. The track's centerline runs down the
middle of that ring, offset `tile_mm / 2` in from both the outer wall and the inner block, so the
same `margin_mm` used elsewhere for safety clearance (robofield.control.kinematics.safety_margin_mm)
bounds how far off centerline the robot may drift before it is "off track" on either side.
"""
from __future__ import annotations

import math

import numpy as np

from robofield.core.types import wrap_angle
from .base import Game, register


@register("race")
class Race(Game):
    n_robots = 1
    max_steps = 2700               # 90 s at 30 Hz: generous for 3 laps at a cautious pace
    obs_dim = 9
    laps_to_win = 3
    lookahead_mm = 150.0            # how far ahead along the centerline the observation looks,
                                     # so the policy can see a corner coming and brake for it

    def __init__(self, field_mm: float = 1000.0, tile_mm: float = 250.0, margin_mm: float = 63.0):
        super().__init__(field_mm)
        self.tile_mm = tile_mm
        self.half_width = tile_mm / 2 - margin_mm      # allowed drift off centerline, each side
        c = tile_mm / 2
        f = field_mm - c
        # centerline: a closed square loop through the middle of the outer ring, one 90-degree
        # corner per field corner, walked in a fixed direction (so `s` only ever counts forward)
        self._corners = [(c, c), (f, c), (f, f), (c, f)]
        self._seg_len = f - c
        self.perimeter = 4 * self._seg_len

    # ---------------------------------------------------------------- centerline geometry
    def _pose_at(self, s: float) -> tuple[float, float, float]:
        s = s % self.perimeter
        seg = int(s // self._seg_len)
        t = (s % self._seg_len) / self._seg_len
        (x0, y0), (x1, y1) = self._corners[seg], self._corners[(seg + 1) % 4]
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, math.atan2(y1 - y0, x1 - x0)

    def _project(self, x: float, y: float) -> tuple[float, float, float]:
        """Nearest centerline point: (arc-length s, signed lateral offset, track heading there)."""
        best = None
        for i in range(4):
            x0, y0 = self._corners[i]
            x1, y1 = self._corners[(i + 1) % 4]
            dx, dy = x1 - x0, y1 - y0
            ux, uy = dx / self._seg_len, dy / self._seg_len
            t = max(0.0, min(self._seg_len, (x - x0) * ux + (y - y0) * uy))
            px, py = x0 + ux * t, y0 + uy * t
            dist2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or dist2 < best[0]:
                lateral = (x - px) * uy - (y - py) * ux
                best = (dist2, i * self._seg_len + t, lateral, math.atan2(dy, dx))
        _, s, lateral, heading = best
        return s, lateral, heading

    # ---------------------------------------------------------------- Game interface
    random_heading = True            # start facing anywhere, not along the track: the policy has to
    lateral_frac = 0.6               # orient itself first; and anywhere across the lane (this fraction
                                     # of the half width either side), like a car put down by hand

    def layout(self):
        start_s = self.rng.uniform(0, self.perimeter)
        cx, cy, heading = self._pose_at(start_s)
        lat = self.rng.uniform(-1, 1) * self.lateral_frac * self.half_width
        x, y = cx + lat * math.sin(heading), cy - lat * math.cos(heading)
        if self.random_heading:
            heading = self.rng.uniform(-math.pi, math.pi)
        self._raw_s_prev, _, _ = self._project(x, y)
        self._progress = 0.0
        self._laps = 0
        self._off_track = False
        return {"robots": {0: (x, y, heading)}, "objects": {}}

    def observe(self, ws, i=0):
        r = ws.robots[0]
        s, lateral, heading = self._project(r.x, r.y)
        dth = wrap_angle(heading - r.theta)
        ax, ay, _ = self._pose_at(s + self.lookahead_mm)
        lx, ly = self.to_robot_frame(r, ax, ay)
        lap_frac = (self._laps % self.laps_to_win) / self.laps_to_win
        return np.array([lateral / self.half_width, math.cos(dth), math.sin(dth),
                         lx / self.lookahead_mm, ly / self.lookahead_mm,
                         r.v / 335, r.omega / 16, lap_frac, 0.0], dtype=np.float32)

    def reward(self, prev, ws, cmds):
        r = ws.robots[0]
        s, lateral, _ = self._project(r.x, r.y)
        ds = s - self._raw_s_prev
        if ds > self.perimeter / 2:           # unwrap across the start/finish line
            ds -= self.perimeter
        elif ds < -self.perimeter / 2:
            ds += self.perimeter
        self._raw_s_prev = s
        self._progress += ds
        new_laps = int(self._progress // self.perimeter)
        lap_bonus = 0.0
        if new_laps > self._laps:
            lap_bonus = 5.0 * (new_laps - self._laps)
            self._laps = new_laps
        rew = ds / 50.0 - 0.01 + lap_bonus                 # progress - time penalty + lap bonus
        c = cmds.get(0)
        if c is not None:
            rew -= 0.001 * (c.left ** 2 + c.right ** 2)     # energy
        if abs(lateral) > self.half_width and not self._off_track:
            self._off_track = True
            rew -= 10.0
        return {0: rew}

    def done(self, ws):
        return self._off_track or self._laps >= self.laps_to_win

    def info(self, ws):
        return {"laps": self._laps, "off_track": self._off_track, "progress_m": self._progress / 1000,
                "success": self._laps >= self.laps_to_win}

    def overlays(self):
        pts = self._corners + [self._corners[0]]
        return [{"type": "path", "points": [list(p) for p in pts], "color": "#2e7d32", "label": "centerline"}]
