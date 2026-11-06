"""The real-field software stack (CarLink, PoseReceiver, RealBackend, GoToPoint) against a simulated
world that speaks the exact car firmware UDP protocol and the Pi tracker pose format."""
import math
import time

import pytest

mujoco = pytest.importorskip("mujoco")

from fake_world import FakeWorld  # noqa: E402
from robofield.comm.udp_link import CarLink  # noqa: E402
from robofield.control import DiffDrive, GoToPoint  # noqa: E402
from robofield.core import RobotCommand, load_config  # noqa: E402
from robofield.real import RealBackend  # noqa: E402
from robofield.sim.backend import SimBackend  # noqa: E402

CFG = load_config("config/field.yaml")
# This test validates the generic RealBackend/CarLink protocol plumbing against an
# idealised simulated car, not any one physical unit's wiring. `robots[].wheels_swapped`
# in config/field.yaml records a real quirk of one specific car (confirmed on hardware,
# see kinematics.py); the simulated stand-in here does not have that quirk, so this test
# uses a config copy without it.
CFG_REAL_TEST = {**CFG, "robots": [{"id": 0, "name": "car-0", "max_speed_mm_s": 330}]}


def test_carlink_discovers_and_sets_id():
    sim = SimBackend(CFG, 1, seed=0, markers=False)
    world = FakeWorld(sim, cmd_port=24210, status_port=24211, pose_port=25005).start()
    link = CarLink(status_port=24211, cmd_port=24210)
    try:
        time.sleep(0.4)
        cars = link.online()
        assert 0 in cars and cars[0].ip == "127.0.0.1"
        link.set_id("127.0.0.1", 3)
        time.sleep(0.4)
        assert 3 in link.online(0.3)
    finally:
        link.close(); world.stop()


def test_real_backend_drives_to_points():
    sim = SimBackend(CFG, 1, seed=1, markers=False)
    sim.reset({"robots": {0: (200, 200, 0.0)}})
    world = FakeWorld(sim, cmd_port=34210, status_port=34211, pose_port=35005).start()
    backend = RealBackend(CFG_REAL_TEST, robot_ids=[0], pose_port=35005)
    backend.cars.close()
    backend.cars = CarLink(status_port=34211, cmd_port=34210)
    ctl = GoToPoint(DiffDrive.from_config(CFG), deadzone=CFG["robot_model"]["deadzone"], v_max=200)
    reached = []
    try:
        backend.reset()
        for gx, gy in ((700, 300), (600, 750)):
            t0 = time.monotonic(); ok = False
            while time.monotonic() - t0 < 12:
                ws = backend.step(); r = ws.robots.get(0)
                if r is None:
                    continue
                cmd, done = ctl.command(r.x, r.y, r.theta, gx, gy, r.omega, 1 / backend.control_hz)
                backend.apply({0: cmd})
                if done:
                    ok = True; break
            # "done" is decided on the delay-predicted pose: commands already in flight still have
            # to land, so measure once the robot is at rest
            for _ in range(15):
                backend.apply({0: RobotCommand()}); backend.step()
            tx, ty, _ = sim.true_pose(0)
            reached.append((ok, math.hypot(tx - gx, ty - gy), time.monotonic() - t0))
    finally:
        backend.close(); world.stop()
    print(reached)
    assert all(ok and err < 30 for ok, err, _ in reached), reached
