"""Load the YAML configuration."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path = "config/field.yaml") -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    # Corner markers: [x, y] or [x, y, z] in mm; missing z means on the field surface.
    corners = {}
    for k, v in cfg["markers"]["corners"].items():
        v = [float(c) for c in v]
        corners[int(k)] = tuple(v + [0.0] * (3 - len(v)))
    cfg["markers"]["corners"] = corners
    cfg["_path"] = str(path)   # so a tool that mutates the live config can write it back to the right file
    return cfg
