"""The Lab (custom tasks, policies, systems) and automatic tuning, in the simulator."""
import importlib.util
import math
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
from robofield import lab  # noqa: E402
from robofield.core import load_config  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "field.yaml")
EXAMPLES = [("task", "park_center"), ("policy", "patrol_square"), ("system", "follow_the_leader")]


@pytest.mark.parametrize("kind,name", EXAMPLES)
def test_examples_pass_the_check(kind, name):
    res = lab.check(kind, name, lab.path_of(kind, name).read_text(encoding="utf-8"))
    assert res["ok"], res


@pytest.mark.parametrize("kind", ["task", "policy", "system"])
def test_templates_pass_the_check(kind):
    res = lab.check(kind, "my_thing", lab.template(kind, "my_thing"))
    assert res["ok"], res


def test_check_points_at_the_broken_line():
    src = "from robofield.lab import Policy\n\n\nclass P(Policy):\n    def step(self, car, world):\n        return self.go_to(car, nowhere, 1)\n"
    res = lab.check("policy", "broken", src)
    assert not res["ok"] and res["errors"][0]["line"] == 6 and "nowhere" in res["errors"][0]["message"]
    res = lab.check("policy", "nothing", "x = 1\n")
    assert not res["ok"] and "no class" in res["errors"][0]["message"]


def test_custom_tasks_become_games():
    errors = lab.register_tasks()
    assert "park_center" not in errors
    from robofield.games import REGISTRY, make_game
    g = make_game("park_center")
    assert REGISTRY["park_center"].obs_dim == 6 and g.n_robots == 1


def sim_cc(n=2):
    from robofield.app import ControlCenter
    from robofield.sim.backend import SimBackend
    cc = ControlCenter(CFG, SimBackend(CFG, n_robots=n, seed=3, randomize=False, markers=False), "sim",
                       cars_file=ROOT / "tests" / "_cars_lab.json")
    cc.state = cc.backend.reset({"robots": {0: (300, 300, 0.0), 1: (700, 700, math.pi)}})
    return cc


def test_policy_and_system_drive_through_the_control_center():
    cc = sim_cc()
    try:
        start = [(cc.state.robots[i].x, cc.state.robots[i].y) for i in (0, 1)]
        ok, msg = cc.run_program("policy", "patrol_square", [0])
        assert ok, msg
        for _ in range(60):
            cc.step_once()
        assert cc.modes[0] == "program"
        assert math.dist(start[0], (cc.state.robots[0].x, cc.state.robots[0].y)) > 20
        ok, msg = cc.run_program("system", "follow_the_leader", [0, 1])
        assert ok, msg
        for _ in range(60):
            cc.step_once()
        snap = {r["id"]: r for r in cc.snapshot()["robots"]}
        assert snap[0]["program"]["name"] == "follow_the_leader" and snap[1]["program"]["running"]
        assert not cc.run_program("system", "follow_the_leader", [0])[0]     # needs two cars
    finally:
        cc.close()
        (ROOT / "tests" / "_cars_lab.json").unlink(missing_ok=True)


def test_a_crashing_program_stops_with_its_line(tmp_path, monkeypatch):
    monkeypatch.setattr(lab, "CUSTOM", tmp_path)
    lab.save("policy", "crashy", "from robofield.lab import Policy\n\nclass C(Policy):\n    def step(self, car, world):\n        return 1 / 0\n")
    cc = sim_cc()
    try:
        ok, _ = cc.run_program("policy", "crashy", [0])
        assert ok
        for _ in range(3):
            cc.step_once()
        p = cc.snapshot()["robots"][0]["program"]
        assert p["result"] == "error" and "line 5" in p["error"] and cc.modes[0] == "idle"
    finally:
        cc.close()
        (ROOT / "tests" / "_cars_lab.json").unlink(missing_ok=True)


def test_training_a_custom_task(tmp_path):
    pytest.importorskip("stable_baselines3")
    out = subprocess.run([sys.executable, "tools/train.py", "--game", "park_center", "--name", "tiny_park", "--timesteps", "512",
                          "--envs", "1", "--n-steps", "256", "--batch-size", "64", "--dir", str(tmp_path)],
                         cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stdout + out.stderr
    assert (tmp_path / "tiny_park.zip").exists()


def test_autotune_course_in_the_simulator():
    spec = importlib.util.spec_from_file_location("autotune", ROOT / "tools" / "autotune.py")
    at = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(at)
    from robofield.control import PoseController
    from robofield.control.kinematics import DiffDrive
    pc = PoseController(DiffDrive.from_config(CFG))
    params = {k: (math.degrees(getattr(pc, k[:-4])) if k.endswith("_deg") else getattr(pc, k)) for k in at.KEYS}
    m = at.run_sim((params, 1, str(ROOT / "config" / "field.yaml")))
    assert m["failed"] == 0 and 3 < m["seconds"] < 40
    worse = dict(m, seconds=m["seconds"] + 5)
    assert at.score(worse, 0.5) > at.score(m, 0.5)
