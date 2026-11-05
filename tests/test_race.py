import math

import pytest

pytest.importorskip("mujoco")
from robofield.control import DiffDrive, PoseController  # noqa: E402
from robofield.core import load_config  # noqa: E402
from robofield.games import make_game  # noqa: E402
from robofield.sim.backend import SimBackend  # noqa: E402

CFG = load_config("config/field.yaml")


def test_race_track_geometry_is_consistent():
    game = make_game("race")
    game.seed(0)
    # every point on the centerline projects back onto itself with zero lateral offset
    for s in range(0, int(game.perimeter), 137):
        x, y, _ = game._pose_at(s)
        s2, lateral, _ = game._project(x, y)
        assert abs(lateral) < 1e-6
        assert min(abs(s2 - s), abs(s2 - s + game.perimeter), abs(s2 - s - game.perimeter)) < 1e-6


def test_race_scripted_lap_completes_without_leaving_track():
    game = make_game("race")
    game.seed(0)
    layout = game.layout()
    b = SimBackend(CFG, 1, seed=0, randomize=False, markers=False)
    s = b.reset(layout)
    dd = DiffDrive.from_config(CFG)
    pc = PoseController(dd, deadzone=CFG["robot_model"]["deadzone"])
    prev = s
    for i in range(2700):
        r = s.robots[0]
        sval, _, _ = game._project(r.x, r.y)
        if i % 15 == 0:
            tx, ty, th = game._pose_at(sval + 220)
            pc.set_goal(tx, ty, th, (r.x, r.y))
            pc.phase, pc._idx, pc._align = "FOLLOW", len(pc.path) - 1, False
        cmd = pc.command(r.x, r.y, r.theta, r.omega, 1 / 30)
        b.apply({0: cmd})
        s = b.step()
        rew = game.reward(prev, s, {0: cmd})
        prev = s
        if game.done(s):
            break
    info = game.info(s)
    assert info["success"]
    assert not info["off_track"]
    assert info["laps"] >= game.laps_to_win
