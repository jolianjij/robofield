import math

from robofield.app.safety import Safety
from robofield.control import DiffDrive
from robofield.core.state import RobotCommand, RobotState, WorldState

DD = DiffDrive(16, 42, 130)


def ws(*robots):
    return WorldState(t=0, robots={r.id: r for r in robots})


def test_estop_and_tracker_loss_stop_everything():
    s = Safety(DD)
    w = ws(RobotState(0, 500, 500, 0))
    assert s.filter(w, {0: RobotCommand(1, 1)}, True, 0.0)[0].left > 0
    s.estop = True
    assert s.filter(w, {0: RobotCommand(1, 1)}, True, 0.1)[0].left == 0
    s.estop = False
    assert s.filter(w, {0: RobotCommand(1, 1)}, False, 0.2)[0].left == 0          # camera data lost


def test_geofence_blocks_driving_out_but_allows_turning():
    s = Safety(DD)
    w = ws(RobotState(0, 500, 960, math.pi / 2))                                  # near the back wall, facing it
    c = s.filter(w, {0: RobotCommand(0.8, 0.8)}, True, 0.0)[0]
    v, _ = DD.forward(c.left, c.right)
    assert v == 0 and s.events[0] == "geofence"
    c = s.filter(w, {0: RobotCommand(-0.5, 0.5)}, True, 0.05)[0]                  # turning is still allowed
    assert c.left < 0 < c.right


def test_collision_avoidance():
    s = Safety(DD)
    w = ws(RobotState(0, 400, 500, 0), RobotState(1, 500, 500, math.pi))
    c = s.filter(w, {0: RobotCommand(0.8, 0.8)}, True, 0.0)[0]
    assert DD.forward(c.left, c.right)[0] == 0 and "too close" in s.events[0]
    c = s.filter(w, {0: RobotCommand(-0.8, -0.8)}, True, 0.05)[0]                 # backing away is fine
    assert DD.forward(c.left, c.right)[0] < 0
