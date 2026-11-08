"""RoboField Control Center: the hardware interaction layer (live map, car control, LEDs,
drawn paths, safety). Everything else (games, RL, multi-agent) builds on top of ControlCenter."""
from .control_center import ControlCenter
from .safety import Safety, SafetySettings

__all__ = ["ControlCenter", "Safety", "SafetySettings", "AppRuntime"]


def __getattr__(name):            # AppRuntime on demand: the Control Center runs without the app's extras
    if name == "AppRuntime":
        from .runtime import AppRuntime
        return AppRuntime
    raise AttributeError(name)
