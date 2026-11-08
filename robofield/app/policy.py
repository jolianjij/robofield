"""Drive a car with a trained RL policy, inside the Control Center (so through the safety layer,
on the map, in the 3D view), on the simulator or on the real field.

The policy sees exactly what it saw in training: the game's observation of the car, computed from
the delay-predicted state the backend reports, plus its own previous action. Its two outputs are
the wheel commands. The game keeps score (laps, goal reached...) so the run ends like an episode.
"""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from robofield.core.state import RobotCommand, WorldState
from robofield.games import make_game


def game_for(model_path: Path, fallback: str | None = None) -> str:
    """The game a saved policy belongs to: its .json (written by tools/train.py), else its name."""
    from robofield.games import REGISTRY
    meta = model_path.with_suffix(".json")
    if meta.exists():
        return json.loads(meta.read_text(encoding="utf-8"))["game"]
    if fallback:
        return fallback
    for name in sorted(REGISTRY, key=len, reverse=True):
        if model_path.stem.startswith(name):
            return name
    raise ValueError(f"cannot tell which game {model_path.name} was trained for")


class PolicyDriver:
    """One car driven by one policy for one episode of its game."""

    def __init__(self, model_path: str | Path, game: str | None = None, speed: float = 0.6,
                 no_reverse: bool = False, dd=None):
        from stable_baselines3 import PPO          # imported here: the Control Center runs without it
        self.path = Path(model_path)
        self.game_name = game or game_for(self.path)
        self.game = make_game(self.game_name)
        if self.game.n_robots != 1:
            raise ValueError(f"'{self.game_name}' is a {self.game.n_robots}-car game; only one-car policies can drive yet")
        self.model = PPO.load(str(self.path), device="cpu")
        self.speed = max(0.05, min(1.0, float(speed)))
        self.no_reverse = no_reverse
        self.dd = dd
        self.steps = 0
        self.info: dict = {}
        self.result: str = ""
        self._last = np.zeros(2, dtype=np.float32)
        self._prev = None

    @staticmethod
    def view(ws: WorldState, rid: int) -> WorldState:
        """The world as a one-car game sees it: the driven car as robot 0."""
        return WorldState(ws.t, {0: replace(ws.robots[rid], id=0)}, ws.objects)

    def start_pose(self) -> tuple[float, float, float]:
        """A start pose from the game's own layout (simulator only: the car is placed there)."""
        return self.game.layout()["robots"][0]

    def ready(self, ws: WorldState, rid: int) -> tuple[bool, str]:
        return self.game.ready(self.view(ws, rid))

    def begin(self, ws: WorldState, rid: int, placed: bool) -> None:
        """placed: the car was just put on the game's own start pose (layout() already reset the
        score). Otherwise the episode starts from wherever the car is (begin())."""
        v = self.view(ws, rid)
        if not placed:
            self.game.begin(v)
        self._prev, self._last, self.steps, self.result = v, np.zeros(2, dtype=np.float32), 0, ""
        self.info = self.game.info(v)

    def step(self, ws: WorldState, rid: int) -> tuple[RobotCommand, bool]:
        v = self.view(ws, rid)
        obs = np.concatenate([self.game.observe(v, 0), self._last]).astype(np.float32)
        action, _ = self.model.predict(obs, deterministic=True)
        action = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        self._last = action
        cmd = RobotCommand(float(action[0]), float(action[1]))
        if self.no_reverse and self.dd is not None:
            # keep the steering, drop any net backward motion (a forward-only race)
            fv, fw = self.dd.forward(cmd.left, cmd.right)
            cmd = RobotCommand(*self.dd.inverse(max(0.0, fv), fw))
        self.game.reward(self._prev, v, {0: cmd})       # the game's bookkeeping: laps, goal reached...
        self._prev = v
        self.steps += 1
        self.info = self.game.info(v)
        done = self.game.done(v)
        if done or self.steps >= self.game.max_steps:
            ok = self.info.get("success")
            self.result = ("finished" if ok else "ended") if done else "time limit"
            return RobotCommand(), True
        return cmd, False

    def summary(self) -> dict:
        clean = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.info.items()}
        return {"model": self.path.name, "game": self.game_name, "speed": self.speed, "steps": self.steps,
                "seconds": round(self.steps / 30, 1), "info": clean, "result": self.result,
                "overlays": self.game.overlays()}
