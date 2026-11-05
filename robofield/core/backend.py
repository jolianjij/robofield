"""The one interface that user code, games and RL environments talk to.

SimBackend (MuJoCo) and RealBackend (camera + radio) both implement it, so the same code runs
in simulation and on the real field: change `backend:` in the config and nothing else.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from .state import RobotCommand, WorldState


class Backend(ABC):
    control_hz: float = 30.0

    @abstractmethod
    def reset(self, layout: dict | None = None) -> WorldState:
        """Start a new episode. layout = {"robots": {id: (x, y, theta)}, "objects": {...}}.
        The real backend cannot teleport robots: it returns the current state."""

    @abstractmethod
    def apply(self, commands: dict[int, RobotCommand]) -> None:
        """Set the commands each robot follows until the next step."""

    @abstractmethod
    def step(self) -> WorldState:
        """Advance one control period (1 / control_hz) and return the newest world state."""

    @abstractmethod
    def state(self) -> WorldState:
        """Latest world state without advancing time."""

    def stop(self) -> None:
        self.apply({rid: RobotCommand() for rid in self.state().robots})

    def close(self) -> None:
        pass
