"""World state and commands shared by every backend (simulator or real field).

Units: mm, rad, s, in the field frame of docs/coordinates.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RobotState:
    id: int
    x: float = 0.0            # mm, midpoint between the wheels
    y: float = 0.0
    theta: float = 0.0        # rad, driving direction
    v: float = 0.0            # mm/s forward speed (estimated)
    omega: float = 0.0        # rad/s
    visible: bool = True      # seen by the tracker in the latest frame
    battery_mv: int = 0
    t: float = 0.0            # time of the pose, s


@dataclass
class ObjectState:
    id: int
    kind: str = "object"      # ball, box, ...
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    visible: bool = True
    t: float = 0.0


@dataclass
class WorldState:
    t: float = 0.0
    robots: dict[int, RobotState] = field(default_factory=dict)
    objects: dict[int, ObjectState] = field(default_factory=dict)


@dataclass
class RobotCommand:
    left: float = 0.0          # -1..1 fraction of max wheel speed
    right: float = 0.0
    gripper: float = 0.0       # 0 = open, 1 = closed
    leds: tuple = ((0, 0, 0), (0, 0, 0), (0, 0, 0))

    def clipped(self) -> "RobotCommand":
        c = lambda v: max(-1.0, min(1.0, float(v)))
        return RobotCommand(c(self.left), c(self.right), max(0.0, min(1.0, float(self.gripper))), self.leds)
