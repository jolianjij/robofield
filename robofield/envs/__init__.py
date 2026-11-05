"""Gymnasium / PettingZoo environments."""
import gymnasium as gym

from .gym_env import RoboFieldEnv
from .pz_env import RoboFieldParallelEnv

gym.register("RoboField-GoTo-v0", entry_point="robofield.envs.gym_env:RoboFieldEnv", kwargs={"game": "goto"})

__all__ = ["RoboFieldEnv", "RoboFieldParallelEnv"]
