"""Patrol a square: visit four corners of the safe area, twice round, then stop."""
from robofield.lab import Policy


class PatrolSquare(Policy):

    def start(self, car, world):
        m = 250                                               # how far in from the walls
        s = world.size
        self.points = [(m, m), (s - m, m), (s - m, s - m), (m, s - m)]
        # start at the corner nearest the car, so it does not cross the whole field first
        self.next = min(range(4), key=lambda i: self.distance(car, *self.points[i]))
        self.visited = 0

    def step(self, car, world):
        x, y = self.points[self.next]
        if self.arrived(car, x, y, within=30):
            self.next = (self.next + 1) % 4
            self.visited += 1
            if self.visited == 9:                             # the start corner, then two full rounds
                return self.finish()
        return self.go_to(car, x, y, speed=220)
