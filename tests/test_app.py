"""The RoboField app: trainer, policy driving, parking, background jobs and the HTTP API, all in the
simulator (no hardware, no window)."""
import json
import math
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
from robofield.core import load_config  # noqa: E402
from robofield.core.state import RobotState, WorldState  # noqa: E402
from robofield.games import make_game  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "field.yaml")


@pytest.fixture(scope="module")
def tiny_policy(tmp_path_factory):
    """A goto policy trained for a few thousand steps (it drives badly; it only has to exist)."""
    pytest.importorskip("stable_baselines3")
    d = tmp_path_factory.mktemp("models")
    out = subprocess.run([sys.executable, "tools/train.py", "--game", "goto", "--name", "tiny", "--timesteps", "1024",
                          "--envs", "1", "--n-steps", "256", "--batch-size", "64", "--dir", str(d)],
                         cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stdout + out.stderr
    return d / "tiny.zip", out.stdout


def test_trainer_writes_policy_curve_and_metadata(tiny_policy):
    path, stdout = tiny_policy
    assert path.exists()
    meta = json.loads(path.with_suffix(".json").read_text())
    assert meta["game"] == "goto" and meta["status"] == "done" and meta["timesteps_done"] >= 1024
    curve = [json.loads(line) for line in path.with_suffix(".progress.jsonl").read_text().splitlines()]
    assert curve and curve[-1]["steps"] >= 1024
    assert "PROGRESS " in stdout and "RESULT " in stdout


def sim_cc(n=2):
    from robofield.app import ControlCenter
    from robofield.sim.backend import SimBackend
    cc = ControlCenter(CFG, SimBackend(CFG, n_robots=n, seed=1, randomize=False, markers=False), "sim",
                       cars_file=ROOT / "tests" / "_cars_test.json")
    cc.state = cc.backend.reset()
    return cc


def test_policy_drives_a_simulated_car_and_parks_the_other(tiny_policy):
    path, _ = tiny_policy
    cc = sim_cc(2)
    try:
        ok, msg = cc.run_policy(0, str(path), speed=1.0)
        assert ok, msg
        drv = cc.policies[0]
        goal = drv.game.goal
        other = cc.state.robots[1]
        start = cc.state.robots[0]
        # the parked car keeps clear of the start, the goal and the line between them
        assert math.dist((other.x, other.y), goal) > 150
        assert math.dist((other.x, other.y), (start.x, start.y)) > 150
        for _ in range(40):
            cc.step_once()
        snap = cc.snapshot()["robots"][0]
        assert snap["policy"]["model"] == "tiny.zip" and snap["policy"]["steps"] > 0
        assert snap["mode"] in ("policy", "idle")
    finally:
        cc.close()
        (ROOT / "tests" / "_cars_test.json").unlink(missing_ok=True)


def test_parking_spots_stay_off_the_race_track():
    cc = sim_cc(1)
    try:
        race = make_game("race")
        spots = cc._parking_spots(2, (125, 125), race.overlays())
        for x, y in spots:                          # the race lane is the outer ring: park inside it
            _, lateral, _ = race._project(x, y)
            assert abs(lateral) > race.half_width + 50
    finally:
        cc.close()
        (ROOT / "tests" / "_cars_test.json").unlink(missing_ok=True)


def test_race_ready_and_begin():
    race = make_game("race")
    on_ring = WorldState(0, {0: RobotState(0, 125, 500, math.pi / 2)})
    middle = WorldState(0, {0: RobotState(0, 500, 500, 0)})
    assert race.ready(on_ring)[0] and not race.ready(middle)[0]
    race.begin(on_ring)
    assert race._laps == 0 and not race._off_track and race._progress == 0


def test_job_manager_reads_progress_and_result():
    from robofield.app.jobs import JobManager
    jm = JobManager()
    code = "print('hello'); print('PROGRESS {\"steps\": 5}'); print('RESULT {\"ok\": 1}')"
    job = jm.start("tool", "echo", ["-c", code])
    for _ in range(100):
        if job.status != "running":
            break
        time.sleep(0.05)
    assert job.status == "done"
    assert job.progress == [{"steps": 5}] and job.result == {"ok": 1}
    assert jm.get(job.id, 0)["lines"] == ["hello"]


def test_app_api_in_the_simulator():
    from robofield.app.runtime import AppRuntime
    from robofield.app.server import serve
    rt = AppRuntime(CFG, "sim", None, 2, port=0)
    httpd = serve(rt, host="127.0.0.1", port=0)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"

    def get(p):
        return urllib.request.urlopen(base + p, timeout=5)

    def post(p, body):
        req = urllib.request.Request(base + p, json.dumps(body).encode(), {"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=5).read())
    try:
        assert json.loads(get("/api/app").read())["mode"] == "sim"
        scene = json.loads(get("/api/scene").read())
        assert scene["field"]["width"] == CFG["field"]["width_mm"] and len(scene["towers"]) == 4
        games = {g["name"]: g for g in json.loads(get("/api/games").read())["games"]}
        assert games["race"]["trainable"] and not games["sumo"]["trainable"]
        for page in ("/app", "/view3d", "/training", "/tools", "/"):
            assert get(page).status == 200
        assert post("/api/sim/place", {"id": 1, "x": 300, "y": 700, "theta": 0})["ok"]
        time.sleep(0.3)
        r1 = next(r for r in json.loads(get("/api/state").read())["robots"] if r["id"] == 1)
        assert math.dist((r1["x"], r1["y"]), (300, 700)) < 15
        assert not post("/api/jobs/train", {"game": "sumo", "timesteps": 1000, "envs": 1})["ok"]
    finally:
        httpd.shutdown()
        rt.close()
