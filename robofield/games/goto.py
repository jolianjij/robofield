"""Go to a goal: the first RL task. Reach a random target point as fast as possible."""
from __future__ import annotations

import math

import numpy as np

from .base import Game, register


@register("goto")
class GoToGoal(Game):
    n_robots = 1
    max_steps = 300
    reach_mm = 30.0
    obs_dim = 10

    def layout(self):
        while True:
            start = (*self.rng.uniform(120, 880, 2), self.rng.uniform(-math.pi, math.pi))
            goal = self.rng.uniform(120, 880, 2)
            if math.hypot(goal[0] - start[0], goal[1] - start[1]) > 150:
                break
        self.goal = tuple(goal)
        self.reached = False
        self._last_action = np.zeros(2)
        return {"robots": {0: start}, "objects": {}}

    def begin(self, ws):
        """A random goal at least 150 mm from wherever the car is."""
        r = ws.robots[0]
        while True:
            goal = self.rng.uniform(120, 880, 2)
            if math.hypot(goal[0] - r.x, goal[1] - r.y) > 150:
                break
        self.goal = tuple(goal)
        self.reached = False

    def observe(self, ws, i=0):
        r = ws.robots[0]
        gx, gy = self.to_robot_frame(r, *self.goal)
        d = math.hypot(gx, gy)
        return np.array([gx / 1000, gy / 1000, d / 1000, gx / (d + 1e-6), gy / (d + 1e-6),
                         r.v / 335, r.omega / 16, r.x / 1000, r.y / 1000, 0.0], dtype=np.float32)

    def reward(self, prev, ws, cmds):
        p, r = prev.robots[0], ws.robots[0]
        d0 = math.hypot(self.goal[0] - p.x, self.goal[1] - p.y)
        d1 = math.hypot(self.goal[0] - r.x, self.goal[1] - r.y)
        rew = (d0 - d1) / 100.0 - 0.01                    # progress, small time penalty
        c = cmds.get(0)
        if c is not None:
            rew -= 0.002 * (c.left ** 2 + c.right ** 2)     # energy
        if d1 < self.reach_mm and not self.reached:
            self.reached = True
            rew += 5.0
        return {0: rew}

    def done(self, ws):
        return self.reached

    def info(self, ws):
        r = ws.robots[0]
        return {"distance": math.hypot(self.goal[0] - r.x, self.goal[1] - r.y), "success": self.reached}

    def overlays(self):
        return [{"type": "circle", "x": self.goal[0], "y": self.goal[1], "r": self.reach_mm, "color": "#2e7d32", "label": "goal"}]
