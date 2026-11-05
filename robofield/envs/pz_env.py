"""PettingZoo parallel (multi-agent) environment, e.g. sumo."""
from __future__ import annotations

import numpy as np
from gymnasium import spaces
from pettingzoo import ParallelEnv

from robofield.core import load_config
from robofield.core.state import RobotCommand
from robofield.games import make_game


class RoboFieldParallelEnv(ParallelEnv):
    metadata = {"name": "robofield_v0", "render_modes": ["rgb_array"]}

    def __init__(self, game: str = "sumo", config: str | dict = "config/field.yaml", backend=None,
                 randomize: bool = True, seed: int = 0):
        self.cfg = load_config(config) if isinstance(config, str) else config
        self.game = make_game(game)
        if backend is None:
            from robofield.sim.backend import SimBackend
            backend = SimBackend(self.cfg, self.game.n_robots, self.game.objects, seed=seed, randomize=randomize, markers=False)
        self.backend = backend
        self.possible_agents = [f"robot_{i}" for i in range(self.game.n_robots)]
        self.agents = list(self.possible_agents)
        self._seed = seed

    def observation_space(self, agent):
        return spaces.Box(-np.inf, np.inf, (self.game.obs_dim,), np.float32)

    def action_space(self, agent):
        return spaces.Box(-1.0, 1.0, (2,), np.float32)

    def reset(self, seed=None, options=None):
        self.game.seed(seed if seed is not None else self._seed)
        self._seed += 1
        ws = self.backend.reset(self.game.layout())
        self.agents, self._prev, self._steps = list(self.possible_agents), ws, 0
        obs = {a: self.game.observe(ws, i) for i, a in enumerate(self.agents)}
        return obs, {a: {} for a in self.agents}

    def step(self, actions):
        cmds = {i: RobotCommand(*np.clip(actions[a], -1, 1).astype(float)) for i, a in enumerate(self.possible_agents) if a in actions}
        self.backend.apply(cmds)
        ws = self.backend.step()
        rew = self.game.reward(self._prev, ws, cmds)
        self._prev = ws
        self._steps += 1
        done = self.game.done(ws)
        trunc = self._steps >= self.game.max_steps
        obs = {a: self.game.observe(ws, i) for i, a in enumerate(self.possible_agents)}
        rewards = {a: float(rew[i]) for i, a in enumerate(self.possible_agents)}
        terms = {a: done for a in self.possible_agents}
        truncs = {a: trunc for a in self.possible_agents}
        infos = {a: self.game.info(ws) for a in self.possible_agents}
        if done or trunc:
            self.agents = []
        return obs, rewards, terms, truncs, infos
