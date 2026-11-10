"""The experiment pipeline (robofield/experiments.py, tools/experiment.py): the protocol, the metrics,
the logs and the report, run in the simulator."""
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
from robofield import experiments as ex  # noqa: E402
from robofield.core import load_config  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "field.yaml")
SPEC = {"name": "t", "episodes": 4, "seed": 3, "start": [500, 500, 0.0], "area": [150, 850], "min_hop_mm": 200,
        "time_limit_s": 10, "success_mm": 30, "speed": 1.0, "settle_s": 0.5, "robot": 0,
        "conditions": [{"name": "baseline", "controller": "baseline"}]}


def test_targets_are_seeded_and_spread():
    a, b = ex.targets(SPEC), ex.targets(SPEC)
    assert a == b and len(a) == 4
    prev = (500, 500)
    for p in a:
        assert 150 <= p[0] <= 850 and 150 <= p[1] <= 850 and math.dist(p, prev) >= 200
        prev = p


def test_episode_metrics():
    ep = ex.Episode("t", "c", "sim", 0, 100, 0, 0, 0, 0.0)
    ep.samples = [ex.Sample(0.1 * k, 10.0 * k, 5.0 if 0 < k < 10 else 0.0, 0, 0, 0, 0, 0) for k in range(1, 11)]
    ep.finish((100, 0), 30, timed_out=False)
    assert ep.success and ep.final_error_mm == 0 and ep.straight_mm == 100
    assert ep.time_to_target_s == pytest.approx(0.8)              # (70, 5) is 30.4 mm away; (80, 5) is inside
    assert ep.max_deviation_mm == 5 and 0.9 < ep.path_efficiency < 1.0
    late = ex.Episode("t", "c", "sim", 1, 100, 0, 0, 0, 0.0)
    late.samples = ep.samples
    assert not late.finish((100, 0), 30, timed_out=True).success  # in reach, but out of time


def test_baseline_in_the_simulator_logs_and_reports(tmp_path):
    runner = ex.SimRunner(CFG, 0, seed=3)
    try:
        eps = ex.run_condition(SPEC, SPEC["conditions"][0], runner, log=lambda *a: None)
    finally:
        runner.close()
    assert len(eps) == 4 and all(e.success for e in eps)
    run = ex.save(tmp_path / "run", eps, ex.provenance(SPEC, runner.name))
    rows = ex.load_runs([run])
    assert len(rows) == 4 and (run / "trajectories" / "baseline_sim_000.csv").exists()
    assert json.loads((run / "meta.json").read_text())["spec"]["seed"] == 3
    s = ex.summarize(rows)[0]
    assert s["success_rate"] == 1.0 and s["error_mean_mm"] < 30
    pytest.importorskip("matplotlib")
    out = subprocess.run([sys.executable, "tools/experiment.py", "report", str(run), "--out", str(tmp_path / "rep")],
                         cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    for f in ("table.tex", "table.md", "summary.csv", "success_rate.pdf", "trajectories.png"):
        assert (tmp_path / "rep" / f).exists()


def test_randomized_worlds_are_the_same_for_every_condition():
    seen = []
    for _ in range(2):
        runner = ex.SimRunner(CFG, 0, seed=5, randomize=True)
        try:
            draws = []
            for i in range(3):
                runner.begin_episode(i)
                draws.append(round(runner.backend.p["latency"], 6))
            seen.append(draws)
        finally:
            runner.close()
    assert seen[0] == seen[1] and len(set(seen[0])) > 1


def test_a_policy_drives_to_a_given_goal(tmp_path):
    pytest.importorskip("stable_baselines3")
    out = subprocess.run([sys.executable, "tools/train.py", "--game", "goto", "--name", "tiny_goto", "--timesteps", "512",
                          "--envs", "1", "--n-steps", "256", "--batch-size", "64", "--dir", str(tmp_path)],
                         cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stderr
    runner = ex.SimRunner(CFG, 0, seed=3)
    try:
        runner.place((500, 500, 0.0))
        runner.start({"controller": "policy", "model": str(tmp_path / "tiny_goto.zip")}, (700, 300), 1.0)
        assert runner.cc.policies[0].game.goal == (700.0, 300.0)
        s, busy = runner.sample(0.0)
        assert busy and s.t > 0
    finally:
        runner.close()
