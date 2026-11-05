"""Game plugins. Add a game: subclass Game, decorate with @register("name")."""
from .base import Game, REGISTRY, make_game, register
from . import goto, sumo  # noqa: F401  (register built-in games)

__all__ = ["Game", "REGISTRY", "make_game", "register"]
