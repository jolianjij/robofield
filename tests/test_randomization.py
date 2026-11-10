"""Domain-randomization ranges come from config/field.yaml (sim.randomization)."""
import copy
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
from robofield.core import load_config  # noqa: E402
from robofield.sim.backend import RANDOMIZATION, SimBackend  # noqa: E402

CFG = load_config(Path(__file__).resolve().parents[1] / "config" / "field.yaml")


def test_config_ranges_match_the_defaults():
    assert {k: tuple(v) for k, v in CFG["sim"]["randomization"].items()} == RANDOMIZATION


def test_config_overrides_a_range():
    cfg = copy.deepcopy(CFG)
    cfg["sim"]["randomization"] = {"latency_s": [0.30, 0.30], "motor_gain": [1.0, 1.0]}
    b = SimBackend(cfg, 1, seed=0, randomize=True, markers=False)
    for _ in range(3):
        b.reset()
        assert b.p["latency"] == pytest.approx(0.30) and (b.p["gain"] == 1.0).all()
    assert b.ranges["friction_scale"] == RANDOMIZATION["friction_scale"]      # the others keep their defaults
