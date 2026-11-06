import math

import numpy as np
import pytest

pytest.importorskip("mujoco")
from robofield.control import DiffDrive, PoseController, straight_path  # noqa: E402
from robofield.core import RobotCommand, load_config  # noqa: E402
from robofield.core.types import wrap_angle  # noqa: E402
from robofield.sim.backend import SimBackend  # noqa: E402

CFG = load_config("config/field.yaml")


def test_straight_path_stays_inside_margin():
    pts = straight_path((500, 500), (990, -40), step=100, margin=60)
    assert pts[-1] == (940, 60)
    assert all(60 <= x <= 940 and 60 <= y <= 940 for x, y in pts)
    assert all(math.dist(a, b) <= 100.01 for a, b in zip([(500, 500)] + pts, pts))


def test_pose_controller_reaches_position_and_heading():
    # The sim reproduces the real car's measured motor curve (no crawling below ~100 mm/s) and its
    # ~0.2 s loop delay; the controller steers on the same delay-predicted state as on the field.
    b = SimBackend(CFG, 1, seed=5, randomize=True, markers=False)
    b.predict = True
    dd = DiffDrive.from_config(CFG)
    rng = np.random.default_rng(3)
    for _ in range(6):
        start = (*rng.uniform(150, 850, 2), rng.uniform(-math.pi, math.pi))
        goal = (*rng.uniform(150, 850, 2), rng.uniform(-math.pi, math.pi))
        s = b.reset({"robots": {0: start}})
        pc = PoseController(dd)
        pc.set_goal(*goal, (s.robots[0].x, s.robots[0].y))
        for _ in range(900):
            r = s.robots[0]
            cmd = pc.command(r.x, r.y, r.theta, r.omega, 1 / CFG["control"]["rate_hz"])
            if pc.done:
                break
            b.apply({0: cmd}); s = b.step()
        for _ in range(15):                                  # let it come to rest
            b.apply({0: RobotCommand()}); b.step()
        tx, ty, tth = b.true_pose(0)
        gx, gy, gth = pc.goal
        assert pc.done
        # the final turn pivots about one wheel (TurnTo: it cannot fine-turn in place without
        # overshooting), which shifts the centre a few cm on a large turn - position for heading
        assert math.hypot(tx - gx, ty - gy) < 40
        assert abs(math.degrees(wrap_angle(tth - gth))) < 6
