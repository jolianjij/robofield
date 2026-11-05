"""Sumo: push the other robot out of the ring. Two-player, zero-sum."""
from __future__ import annotations

import math

import numpy as np

from .base import Game, register


@register("sumo")
class Sumo(Game):
    n_robots = 2
    max_steps = 450               # 15 s
    ring_r = 330.0
    obs_dim = 12

    def layout(self):
        a = self.rng.uniform(-math.pi, math.pi)
        c = 500.0
        d = 150.0
        p0 = (c + d * math.cos(a), c + d * math.sin(a), a + math.pi + self.rng.normal(0, 0.3))
        p1 = (c - d * math.cos(a), c - d * math.sin(a), a + self.rng.normal(0, 0.3))
        self.winner = None
        return {"robots": {0: p0, 1: p1}, "objects": {}}

    def _out(self, r):
        return math.hypot(r.x - 500, r.y - 500) > self.ring_r

    def observe(self, ws, i):
        me, op = ws.robots[i], ws.robots[1 - i]
        ox, oy = self.to_robot_frame(me, op.x, op.y)
        cx, cy = self.to_robot_frame(me, 500, 500)
        dth = math.atan2(math.sin(op.theta - me.theta), math.cos(op.theta - me.theta))
        dist_edge = self.ring_r - math.hypot(me.x - 500, me.y - 500)
        return np.array([ox / 500, oy / 500, math.hypot(ox, oy) / 500, cx / 500, cy / 500, dist_edge / self.ring_r,
                         math.cos(dth), math.sin(dth), me.v / 335, me.omega / 16, op.v / 335, op.omega / 16],
                        dtype=np.float32)

    def reward(self, prev, ws, cmds):
        rew = {0: 0.0, 1: 0.0}
        for i in (0, 1):
            me, op = ws.robots[i], ws.robots[1 - i]
            pm, po = prev.robots[i], prev.robots[1 - i]
            # shaping: push the opponent away from the centre, stay central yourself
            rew[i] += (math.hypot(op.x - 500, op.y - 500) - math.hypot(po.x - 500, po.y - 500)) / 200
            rew[i] -= (math.hypot(me.x - 500, me.y - 500) - math.hypot(pm.x - 500, pm.y - 500)) / 200
        out = [self._out(ws.robots[i]) for i in (0, 1)]
        if out[0] != out[1]:
            self.winner = 1 if out[0] else 0
            rew[self.winner] += 10.0
            rew[1 - self.winner] -= 10.0
        return rew

    def done(self, ws):
        return self.winner is not None or any(self._out(ws.robots[i]) for i in (0, 1))

    def info(self, ws):
        return {"winner": self.winner}

    def overlays(self):
        return [{"type": "ring", "x": 500, "y": 500, "r": self.ring_r, "color": "#c62828", "label": "ring"}]
