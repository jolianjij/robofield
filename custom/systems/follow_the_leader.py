"""Follow the leader: the first car circles the middle, the second follows 160 mm behind it."""
import math

from robofield.lab import System


class FollowTheLeader(System):
    cars = 2                     # the first two cars chosen to run it
    seconds = 40                 # then it stops

    def start(self, world):
        self.leader_id, self.follower_id = world.cars[0].id, world.cars[1].id

    def step(self, world):
        if world.t > self.seconds:
            return self.finish()
        leader = world.car(self.leader_id)
        follower = world.car(self.follower_id)
        # the leader chases a point that moves round a 250 mm circle
        a = world.t * 0.5
        lx, ly = 500 + 250 * math.cos(a), 500 + 250 * math.sin(a)
        # the follower aims at a spot 160 mm behind the leader, and waits if it gets too close
        bx = leader.x - 160 * math.cos(leader.theta)
        by = leader.y - 160 * math.sin(leader.theta)
        follow = self.stop() if self.distance(follower, leader.x, leader.y) < 140 else self.go_to(follower, bx, by, speed=230)
        return {leader.id: self.go_to(leader, lx, ly, speed=170), follower.id: follow}
