"""Park in the centre: drive to the middle of the field and stop there, facing +x."""
import math

import numpy as np

from robofield.lab import Task


class ParkCenter(Task):
    n_robots = 1
    max_steps = 360              # 12 s at 30 steps per second

    centre = (500.0, 500.0)
    within_mm = 30.0
    within_deg = 15.0

    def layout(self):
        """Start anywhere at least 200 mm from the centre, facing anywhere."""
        while True:
            x, y = self.rng.uniform(150, 850, 2)
            if math.dist((x, y), self.centre) > 200:
                break
        self.parked = False
        return {"robots": {0: (x, y, self.rng.uniform(-math.pi, math.pi))}, "objects": {}}

    def begin(self, ws):
        self.parked = False

    def observe(self, ws, i=0):
        car = ws.robots[0]
        cx, cy = self.to_robot_frame(car, *self.centre)      # the centre: ahead (x) and to the left (y)
        return np.array([cx / 500, cy / 500, math.cos(car.theta), math.sin(car.theta),
                         car.v / 335, car.omega / 16], dtype=np.float32)

    def reward(self, prev, ws, cmds):
        p, car = prev.robots[0], ws.robots[0]
        before = math.dist((p.x, p.y), self.centre)
        now = math.dist((car.x, car.y), self.centre)
        r = (before - now) / 100 - 0.01                       # closer is good; time costs a little
        if now < 80:                                          # near the middle: turning to +x counts too
            r += 0.05 * (math.cos(car.theta) - math.cos(p.theta))
        heading_ok = abs(math.degrees(math.atan2(math.sin(car.theta), math.cos(car.theta)))) < self.within_deg
        if now < self.within_mm and heading_ok and abs(car.v) < 40 and not self.parked:
            self.parked = True
            r += 10
        return {0: r}

    def done(self, ws):
        return self.parked

    def info(self, ws):
        car = ws.robots[0]
        return {"distance": math.dist((car.x, car.y), self.centre), "success": self.parked}

    def overlays(self):
        return [{"type": "circle", "x": self.centre[0], "y": self.centre[1], "r": self.within_mm, "color": "#2e7d32"}]
