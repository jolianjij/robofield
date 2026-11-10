"""Experiments that run identically in the simulator and on the real field, with the same logs.

An experiment is a YAML spec (experiments/*.yaml): a task, a list of conditions (controllers) and a
protocol. The protocol for the go-to-target task: the car starts at a fixed pose, then drives to a
seeded list of targets in order; each target is one episode, starting where the last one ended (the
real field cannot place cars). Every condition gets the same targets.

Two runners carry it out through the same Control Center code (controllers, policies, safety):

    SimRunner  an in-process Control Center on the MuJoCo simulator, stepped as fast as it can go
    AppRunner  the running RoboField app over its HTTP API: the real field (or its simulator), with
               the app's STOP button, map and 3D view live while it runs

Each episode is logged the same way: a row of metrics (episodes.csv / episodes.jsonl) and its
trajectory (trajectories/*.csv). tools/experiment.py runs specs and turns results into figures.
"""
from __future__ import annotations

import csv
import json
import math
import platform
import subprocess
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


# ------------------------------------------------------------------ spec
def load_spec(path) -> dict:
    spec = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    spec.setdefault("name", Path(path).stem)
    spec.setdefault("robot", 0)
    spec.setdefault("episodes", 20)
    spec.setdefault("seed", 0)
    spec.setdefault("start", [500, 500, 0.0])
    spec.setdefault("time_limit_s", 10.0)
    spec.setdefault("success_mm", 30.0)
    spec.setdefault("speed", 1.0)
    spec.setdefault("area", [150, 850])
    spec.setdefault("min_hop_mm", 200.0)
    spec.setdefault("settle_s", 0.8)
    if not spec.get("conditions"):
        raise ValueError("the spec needs at least one condition")
    return spec


def targets(spec: dict) -> list[tuple[float, float]]:
    """The seeded target list: inside `area`, each at least min_hop_mm from the one before (the first
    from the start pose). The same for every condition and both runners."""
    rng = np.random.default_rng(int(spec["seed"]))
    lo, hi = spec["area"]
    prev = tuple(spec["start"][:2])
    out = []
    while len(out) < int(spec["episodes"]):
        p = tuple(float(v) for v in np.round(rng.uniform(lo, hi, 2), 1))
        if math.dist(p, prev) >= spec["min_hop_mm"]:
            out.append(p)
            prev = p
    return out


# ------------------------------------------------------------------ one episode
@dataclass
class Sample:
    t: float
    x: float
    y: float
    theta: float
    v: float
    omega: float
    left: float
    right: float
    safety: str = ""


@dataclass
class Episode:
    experiment: str
    condition: str
    backend: str                  # "sim" or "real" (or "app-sim")
    index: int
    target_x: float
    target_y: float
    start_x: float
    start_y: float
    start_theta: float
    success: bool = False
    timed_out: bool = False
    time_to_target_s: float | None = None
    duration_s: float = 0.0
    path_length_mm: float = 0.0
    straight_mm: float = 0.0
    path_efficiency: float | None = None
    max_deviation_mm: float = 0.0
    final_error_mm: float = 0.0
    safety_steps: int = 0
    samples: list = field(default_factory=list, repr=False)

    def finish(self, final_xy, success_mm: float, timed_out: bool):
        tx, ty = self.target_x, self.target_y
        s = self.samples
        self.timed_out = timed_out
        self.final_error_mm = round(math.dist(final_xy, (tx, ty)), 2)
        self.success = (not timed_out) and self.final_error_mm <= success_mm
        hit = next((p.t for p in s if math.dist((p.x, p.y), (tx, ty)) <= success_mm), None)
        self.time_to_target_s = None if hit is None else round(hit, 3)
        self.duration_s = round(s[-1].t, 3) if s else 0.0
        pts = [(self.start_x, self.start_y)] + [(p.x, p.y) for p in s]
        self.path_length_mm = round(sum(math.dist(a, b) for a, b in zip(pts, pts[1:])), 1)
        self.straight_mm = round(math.dist((self.start_x, self.start_y), (tx, ty)), 1)
        self.path_efficiency = round(self.straight_mm / self.path_length_mm, 4) if self.path_length_mm > 1 else None
        self.max_deviation_mm = round(max((_seg_dist((p.x, p.y), (self.start_x, self.start_y), (tx, ty)) for p in s),
                                          default=0.0), 1)
        self.safety_steps = sum(1 for p in s if p.safety)
        return self

    def row(self) -> dict:
        d = asdict(self)
        d.pop("samples")
        return d


def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


# ------------------------------------------------------------------ runners
class SimRunner:
    """The Control Center in-process on the MuJoCo simulator, unpaced: the same controllers, policy
    driver and safety layer as the app, as fast as the computer allows."""
    name = "sim"

    def __init__(self, cfg: dict, robot: int = 0, seed: int = 0, randomize: bool = False):
        from robofield.app import ControlCenter
        from robofield.sim.backend import SimBackend
        self.rid, self.hz, self.seed = robot, float(cfg["control"]["rate_hz"]), seed
        self._cars = Path(tempfile.mkdtemp()) / "cars.json"      # never touch config/cars.json
        self.backend = SimBackend(cfg, n_robots=robot + 1, seed=seed, randomize=randomize, markers=False)
        self.cc = ControlCenter(cfg, self.backend, "sim", cars_file=self._cars)
        self.cc.state = self.backend.reset()
        self.t = 0.0
        if randomize:
            self.name = "sim-randomized"

    def place(self, pose):
        ok, msg = self.cc.place_car(self.rid, *pose)
        if not ok:
            raise RuntimeError(msg)

    def begin_episode(self, index: int):
        """Every condition meets the same worlds: before episode i, re-seed the simulator from
        (seed, i) and reset it where the car stands, so a randomized simulator draws world i (the
        plain simulator only re-seeds its tracking noise)."""
        import numpy as np
        self.backend.rng = np.random.default_rng(10_000 * int(self.seed) + int(index))
        x, y, th = self.backend.true_pose(self.rid)
        self.place((x, y, th))

    def start(self, condition: dict, goal, speed: float):
        x, y = self._pose()[:2]
        if condition["controller"] == "baseline":
            self.cc.update_car(self.rid, max_speed=speed)        # the same speed scale as a policy gets
            ok, msg = self.cc.set_path(self.rid, [(x, y), tuple(goal)], None)
        else:
            ok, msg = self.cc.run_policy(self.rid, condition["model"], speed=speed, place=False, goal=tuple(goal))
        if not ok:
            raise RuntimeError(msg)

    def _pose(self):
        r = self.cc.state.robots[self.rid]
        return r.x, r.y, r.theta, r.v, r.omega

    def sample(self, t0: float) -> tuple[Sample, bool]:
        self.cc.step_once()
        self.t += 1 / self.hz
        x, y, th, v, w = self._pose()
        c = self.cc.last_cmds.get(self.rid)
        busy = self.cc.modes.get(self.rid, "idle") != "idle"
        return Sample(round(self.t - t0, 4), x, y, th, v, w, c.left if c else 0.0, c.right if c else 0.0,
                      self.cc.safety.events.get(self.rid, "")), busy

    def stop(self):
        self.cc.stop(self.rid)

    def settle(self, seconds: float):
        """Let the car come to rest; its (x, y, heading)."""
        for _ in range(int(seconds * self.hz)):
            self.cc.step_once()
            self.t += 1 / self.hz
        return self._pose()[:3]

    def now(self):
        return self.t

    def close(self):
        self.cc.close()


class AppRunner:
    """The running RoboField app, through its HTTP API: on the real field (or the app's simulator)."""

    def __init__(self, url: str = "http://127.0.0.1:8000", robot: int = 0):
        import urllib.request  # noqa: F401
        self.url, self.rid = url.rstrip("/"), robot
        info = self._api("/api/app")
        self.name = "real" if info["mode"] == "real" else "app-sim"
        self._last = None

    def _api(self, path, body=None):
        import urllib.request
        req = urllib.request.Request(self.url + path, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def _robot(self):
        return next((r for r in self._api("/api/state")["robots"] if r["id"] == self.rid), None)

    def place(self, pose):
        """The real field cannot place a car: drive it there (go-to-pose; not part of any episode)."""
        if self.name == "app-sim":
            ok = self._api("/api/sim/place", {"id": self.rid, "x": pose[0], "y": pose[1], "theta": pose[2]})
            if ok.get("ok"):
                return
        res = self._api("/api/goal", {"id": self.rid, "x": pose[0], "y": pose[1], "theta": pose[2]})
        if not res.get("ok"):
            raise RuntimeError(res.get("msg"))
        t0 = time.monotonic()
        time.sleep(0.5)
        while time.monotonic() - t0 < 40:
            r = self._robot()
            if r and r["mode"] == "idle":
                return
            time.sleep(0.1)
        raise RuntimeError("could not reach the start pose")

    def start(self, condition: dict, goal, speed: float):
        r = self._robot()
        if r is None or not r["visible"]:
            raise RuntimeError("the car is not seen by the camera")
        if condition["controller"] == "baseline":
            self._api("/api/car", {"id": self.rid, "max_speed": speed})
            res = self._api("/api/path", {"id": self.rid, "points": [[r["x"], r["y"]], list(goal)], "theta": None})
        else:
            res = self._api("/api/policy", {"id": self.rid, "model": condition["model"], "speed": speed,
                                            "place": False, "goal": list(goal)})
        if not res.get("ok"):
            raise RuntimeError(res.get("msg"))
        time.sleep(0.15)                                     # let the loop take it before judging "idle"

    def sample(self, t0: float) -> tuple[Sample, bool]:
        time.sleep(0.05)                                     # ~20 samples per second over HTTP
        r = self._robot()
        if r is None or not r["visible"]:
            raise RuntimeError("the car is not seen by the camera")
        cmd = r.get("cmd") or [0, 0]
        return Sample(round(time.monotonic() - t0, 4), r["x"], r["y"], r["theta"], r["v"], r.get("omega", 0.0),
                      cmd[0], cmd[1], r.get("safety", "")), r["mode"] != "idle"

    def stop(self):
        self._api("/api/stop", {"id": self.rid})

    def begin_episode(self, index: int):
        """The real field is the real field: nothing to draw."""

    def settle(self, seconds: float):
        """Let the car come to rest; its (x, y, heading)."""
        time.sleep(seconds)
        r = self._robot()
        return r["x"], r["y"], r["theta"]

    def now(self):
        return time.monotonic()

    def close(self):
        try:
            self._api("/api/car", {"id": self.rid, "max_speed": 1.0})
        except OSError:
            pass


# ------------------------------------------------------------------ the protocol
def run_condition(spec: dict, condition: dict, runner, log=print) -> list[Episode]:
    """Start pose, then every target in order; one Episode per target."""
    goals = targets(spec)
    runner.place(spec["start"])
    runner.settle(0.5)
    episodes = []
    for i, goal in enumerate(goals):
        runner.begin_episode(i)
        x, y, th = runner.settle(0.0)
        ep = Episode(spec["name"], condition["name"], runner.name, i, goal[0], goal[1], round(x, 1), round(y, 1),
                     round(float(th), 4))
        runner.start(condition, goal, float(spec["speed"]))
        t0, timed_out = runner.now(), False
        while True:
            s, busy = runner.sample(t0)
            ep.samples.append(s)
            if not busy:
                break
            if s.t > spec["time_limit_s"]:
                timed_out = True
                runner.stop()
                break
        final = runner.settle(spec["settle_s"])
        ep.finish(final[:2], spec["success_mm"], timed_out)
        episodes.append(ep)
        log(f"  {condition['name']:>10s} {runner.name:>5s} #{i:02d}  {'ok  ' if ep.success else 'MISS'} "
            f"error {ep.final_error_mm:6.1f} mm  time {ep.time_to_target_s if ep.time_to_target_s is not None else '-':>6} s  "
            f"path {ep.path_length_mm:6.0f} mm")
    return episodes


def provenance(spec: dict, runner_name: str) -> dict:
    def git(*a):
        try:
            return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True, timeout=5).stdout.strip()
        except OSError:
            return ""
    models = {}
    for c in spec["conditions"]:
        if c.get("model"):
            meta = ROOT / "models" / f"{c['model']}.json"
            models[c["model"]] = json.loads(meta.read_text(encoding="utf-8")) if meta.exists() else {}
    return {"spec": spec, "runner": runner_name, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "git_commit": git("rev-parse", "HEAD"), "git_dirty": bool(git("status", "--porcelain", "--", "robofield", "config")),
            "python": platform.python_version(), "machine": platform.platform(), "models": models,
            "config": (ROOT / "config" / "field.yaml").read_text(encoding="utf-8"),
            "tuning": (ROOT / "config" / "tuning.yaml").read_text(encoding="utf-8") if (ROOT / "config" / "tuning.yaml").exists() else None}


def save(out_dir: Path, episodes: list[Episode], meta: dict) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=1, default=str), encoding="utf-8")
    rows = [e.row() for e in episodes]
    with (out_dir / "episodes.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with (out_dir / "episodes.jsonl").open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    tdir = out_dir / "trajectories"
    tdir.mkdir(exist_ok=True)
    for e in episodes:
        with (tdir / f"{e.condition}_{e.backend}_{e.index:03d}.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "theta", "v", "omega", "left", "right", "safety"])
            for s in e.samples:
                w.writerow([s.t, round(s.x, 2), round(s.y, 2), round(s.theta, 4), round(s.v, 1), round(s.omega, 3),
                            round(s.left, 4), round(s.right, 4), s.safety])
    return out_dir


def load_runs(dirs) -> list[dict]:
    """Every episode row from one or more run folders."""
    rows = []
    for d in dirs:
        for line in (Path(d) / "episodes.jsonl").read_text(encoding="utf-8").splitlines():
            if line.strip():
                rows.append(json.loads(line))
    return rows


def summarize(rows: list[dict]) -> list[dict]:
    """Per (condition, backend): success rate and mean / std of the metrics (successful episodes for
    time and path efficiency, all episodes for the final error)."""
    groups: dict = {}
    for r in rows:
        groups.setdefault((r["condition"], r["backend"]), []).append(r)
    out = []
    for (cond, backend), rs in groups.items():
        ok = [r for r in rs if r["success"]]

        def stat(vals):
            vals = [v for v in vals if v is not None]
            return (round(float(np.mean(vals)), 3), round(float(np.std(vals)), 3)) if vals else (None, None)
        t = stat([r["time_to_target_s"] for r in ok])
        e = stat([r["final_error_mm"] for r in rs])
        eff = stat([r["path_efficiency"] for r in ok])
        out.append({"condition": cond, "backend": backend, "episodes": len(rs), "success_rate": round(len(ok) / len(rs), 3),
                    "time_mean_s": t[0], "time_std_s": t[1], "error_mean_mm": e[0], "error_std_mm": e[1],
                    "efficiency_mean": eff[0], "efficiency_std": eff[1],
                    "safety_steps_mean": round(float(np.mean([r["safety_steps"] for r in rs])), 2)})
    return out
