"""Game plugin interface. A game only reads WorldState and writes rewards/termination, so the
same game runs in the simulator and on the real field."""
from __future__ import annotations

import math
from abc import ABC, abstractmethod

import numpy as np

from robofield.core.state import RobotCommand, WorldState

REGISTRY: dict[str, type["Game"]] = {}


def register(name: str):
    def deco(cls):
        REGISTRY[name] = cls
        cls.name = name
        return cls
    return deco


def make_game(name: str, **kw) -> "Game":
    return REGISTRY[name](**kw)


class Game(ABC):
    name = "game"
    n_robots = 1
    objects: tuple = ()           # (("ball", 30), ...) spawned by the simulator
    max_steps = 300               # 10 s at 30 Hz

    def __init__(self, field_mm: float = 1000.0):
        self.field = field_mm
        self.rng = np.random.default_rng()

    def seed(self, seed: int | None):
        self.rng = np.random.default_rng(seed)

    @abstractmethod
    def layout(self) -> dict:
        """Start positions: {"robots": {i: (x, y, th)}, "objects": {id: (x, y)}}; also picks targets."""

    @abstractmethod
    def observe(self, ws: WorldState, i: int) -> np.ndarray:
        """Observation vector for robot i (what a policy sees)."""

    @abstractmethod
    def reward(self, prev: WorldState, ws: WorldState, cmds: dict[int, RobotCommand]) -> dict[int, float]:
        ...

    @abstractmethod
    def done(self, ws: WorldState) -> bool:
        ...

    def info(self, ws: WorldState) -> dict:
        return {}

    def overlays(self) -> list[dict]:
        """Virtual objects for visualisation: [{"type": "circle", "x":..,"y":..,"r":..,"color":..}]."""
        return []

    # helpers
    @staticmethod
    def to_robot_frame(r, x: float, y: float) -> tuple[float, float]:
        dx, dy = x - r.x, y - r.y
        c, s = math.cos(r.theta), math.sin(r.theta)
        return c * dx + s * dy, -s * dx + c * dy
