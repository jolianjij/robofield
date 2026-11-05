"""Gymnasium single-agent environment over any Backend (simulator by default)."""
from __future__ import annotations

import gymnasium as gym
import numpy as np

from robofield.core import load_config
from robofield.core.state import RobotCommand
from robofield.games import make_game


class RoboFieldEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(self, game: str = "goto", config: str | dict = "config/field.yaml", backend=None,
                 randomize: bool = True, seed: int = 0, render_mode: str | None = None):
        self.cfg = load_config(config) if isinstance(config, str) else config
        self.game = make_game(game)
        if backend is None:
            from robofield.sim.backend import SimBackend
            backend = SimBackend(self.cfg, self.game.n_robots, self.game.objects, seed=seed, randomize=randomize,
                                 markers=render_mode is not None)
        self.backend = backend
        self.render_mode = render_mode
        n = self.game.obs_dim
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n + 2,), np.float32)   # + last action
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        self._steps = 0

    def _obs(self, ws):
        return np.concatenate([self.game.observe(ws, 0), self._last]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.game.seed(seed)
        elif self._steps == 0 and not hasattr(self, "_seeded"):
            self.game.seed(int(self.np_random.integers(1 << 31))); self._seeded = True
        ws = self.backend.reset(self.game.layout())
        self._prev, self._steps, self._last = ws, 0, np.zeros(2, np.float32)
        return self._obs(ws), self.game.info(ws)

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float32), -1, 1)
        cmd = {0: RobotCommand(float(a[0]), float(a[1]))}
        self.backend.apply(cmd)
        ws = self.backend.step()
        r = self.game.reward(self._prev, ws, cmd)[0]
        self._prev, self._last = ws, a
        self._steps += 1
        terminated = self.game.done(ws)
        truncated = self._steps >= self.game.max_steps
        return self._obs(ws), float(r), terminated, truncated, self.game.info(ws)

    def render(self):
        return self.backend.render(640, 360)

    def close(self):
        self.backend.close()
