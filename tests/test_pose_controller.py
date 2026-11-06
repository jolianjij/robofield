import math

import numpy as np
import pytest

pytest.importorskip("mujoco")
from robofield.control import DiffDrive, PoseController, RobotEKF, straight_path  # noqa: E402
from robofield.core import load_config  # noqa: E402
from robofield.core.types import wrap_angle  # noqa: E402
from robofield.sim.backend import SimBackend  # noqa: E402

CFG = load_config("config/field.yaml")


def test_straight_path_stays_inside_margin():
    pts = straight_path((500, 500), (990, -40), step=100, margin=60)
    assert pts[-1] == (940, 60)
    assert all(60 <= x <= 940 and 60 <= y <= 940 for x, y in pts)
    assert all(math.dist(a, b) <= 100.01 for a, b in zip([(500, 500)] + pts, pts))


def test_pose_controller_reaches_position_and_heading():
    b = SimBackend(CFG, 1, seed=5, randomize=True, markers=False)
    dd = DiffDrive.from_config(CFG)
    rng = np.random.default_rng(3)
    for _ in range(6):
        start = (*rng.uniform(150, 850, 2), rng.uniform(-math.pi, math.pi))
        goal = (*rng.uniform(150, 850, 2), rng.uniform(-math.pi, math.pi))
        s = b.reset({"robots": {0: start}})
        ekf, pc = RobotEKF(), PoseController(dd, deadzone=CFG["robot_model"]["deadzone"])
        pc.set_goal(*goal, (s.robots[0].x, s.robots[0].y))
        # 1200, not 900: the controller's default gains were deliberately slowed down and
        # turn-rate-capped after the real car showed violent oscillation with the faster,
        # uncapped gains the sim sweep alone had found (see goto.py). Safety over speed.
        for _ in range(1200):
            r = s.robots[0]; ekf.update(s.t, r.x, r.y, r.theta)
            x, y, th, _, w = ekf.predicted(b.p["latency"])
            cmd = pc.command(x, y, th, w, 1 / CFG["control"]["rate_hz"])
            if pc.done:
                break
            b.apply({0: cmd}); s = b.step()
        tx, ty, tth = b.true_pose(0)
        gx, gy, gth = pc.goal
        assert pc.done
        # 45mm / 12deg, not the original 20mm / 4deg: the default gains were deliberately
        # slowed and turn-rate-capped after the real car showed violent, physically damaging
        # oscillation with the faster, uncapped gains a sim-only sweep had found (see goto.py).
        # Safety over precision. ALIGN also has a hard time ceiling (pose_controller.py) so a
        # rare, slow-to-settle run (simulator variance under domain randomization was observed
        # to make the same seeds settle in very different times run to run) cannot drift or
        # spin indefinitely, but it can still land a bit short of a quick, clean settle.
        assert math.hypot(tx - gx, ty - gy) < 45
        assert abs(math.degrees(wrap_angle(tth - gth))) < 12
