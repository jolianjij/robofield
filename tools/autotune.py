"""Find good motion-control settings automatically: drive a test course again and again with different
settings and keep what scores best.

    python tools/autotune.py --sim                          # randomized simulators, fast (a few minutes)
    python tools/autotune.py --sim --trials 200 --precision 0.7
    python tools/autotune.py --real --url http://127.0.0.1:8000 --trials 16    # the real car, through the app

The course starts at (500, 330) facing +y: a 340 mm straight, a 120 deg turn on the spot, a half
circle (r = 170 mm) back to the start, and a final 90 deg turn. It ends where it began, so the real
car can repeat it. Each try is scored, lower is better:

    score = time weight x seconds  +  stop error / 5 mm  +  worst distance from the line or path / 10 mm
            + final heading error / 1.5 deg  +  2 per wobble  +  30 per maneuver that did not finish

--precision (0..1) moves the balance: 0 cares mostly about time, 1 mostly about accuracy.

The search is the cross-entropy method: try a population of settings around the current best guess,
move the guess toward the best quarter of them, narrow down, repeat. In the simulator every try runs
in several randomized simulators (delay, friction, motor mismatch...), so the result does not depend
on one lucky world; at the end the best settings and the starting ones are compared on fresh worlds.
On the real car every try is one drive of the course; keep the area clear and stay near the STOP button.

Prints "PROGRESS {json}" after every try and "RESULT {json}" at the end (the app's Tuning tab shows
them and applies the result). Create the file given by --stop-file to end early with the best so far.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# the settings it tunes (ranges from the tuning page); delay, trims and on/off switches are measured, not tuned
KEYS = ["v_path", "accel", "lat_accel", "lookahead", "pos_tol", "pass_tol", "reface_angle_deg",
        "face_tol_deg", "ang_tol_deg", "wait_s", "pulse_speed"]
GROUPS = {"all": KEYS,
          "lines": ["v_path", "accel", "lat_accel", "lookahead", "pos_tol", "pass_tol", "reface_angle_deg"],
          "turns": ["face_tol_deg", "ang_tol_deg", "wait_s", "pulse_speed"]}
START = (500.0, 330.0, math.pi / 2)
TOP = (500.0, 670.0)
CENTRE, RADIUS = (500.0, 500.0), 170.0


def schema():
    from robofield.app.control_center import TUNING_SCHEMA
    return {row[0]: (row[3], row[4], row[5]) for row in TUNING_SCHEMA}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def poly_dist(p, pts):
    return min(seg_dist(p, a, b) for a, b in zip(pts, pts[1:]))


def wobbles(headings):
    steps = [s for s in (wrap(b - a) for a, b in zip(headings, headings[1:])) if abs(s) > math.radians(1.5)]
    return sum(1 for a, b in zip(steps, steps[1:]) if (a > 0) != (b > 0))


def half_circle():
    """From the top of the course round the left side back down to the start, counter-clockwise."""
    return [(CENTRE[0] + RADIUS * math.cos(a), CENTRE[1] + RADIUS * math.sin(a))
            for a in np.linspace(math.pi / 2, 3 * math.pi / 2, 25)]


def score(m: dict, precision: float) -> float:
    w_time = 2.0 * (1 - precision) + 0.25
    w_acc = 0.5 + 1.5 * precision
    return round(w_time * m["seconds"] + w_acc * (m["stop_mm"] / 5 + m["dev_mm"] / 10 + m["heading_deg"] / 1.5)
                 + 2 * m["wobbles"] + 30 * m["failed"], 3)


def summarize(maneuvers: list[dict]) -> dict:
    return {"seconds": round(sum(x["seconds"] for x in maneuvers), 2),
            "stop_mm": round(sum(x.get("stop_mm", 0) for x in maneuvers), 1),
            "dev_mm": round(sum(x.get("dev_mm", 0) for x in maneuvers), 1),
            "heading_deg": round(sum(x.get("heading_deg", 0) for x in maneuvers), 2),
            "wobbles": sum(x.get("wobbles", 0) for x in maneuvers),
            "failed": sum(x.get("failed", 0) for x in maneuvers)}


# ------------------------------------------------------------------ the course in a simulator
def run_sim(args) -> dict:
    """One drive of the course in one randomized simulator, with the app's safety layer."""
    params, seed, cfg_path = args
    from robofield.app.safety import Safety, SafetySettings
    from robofield.control import DiffDrive, PoseController, deadzones_from_config, safety_margin_mm
    from robofield.core import RobotCommand, load_config
    from robofield.sim.backend import SimBackend

    cfg = load_config(cfg_path)
    dd = DiffDrive.from_config(cfg)
    margin = safety_margin_mm(cfg)
    b = SimBackend(cfg, 1, seed=seed, randomize=True, markers=False)
    b.predict = True
    safety = Safety(dd, cfg["field"]["width_mm"], deadzones_from_config(cfg), SafetySettings(geofence_margin_mm=margin))
    kw = {k: v for k, v in params.items() if not k.endswith("_deg")}
    kw.update({k[:-4]: math.radians(v) for k, v in params.items() if k.endswith("_deg")})   # degrees -> radians
    learned: dict = {}
    hz = cfg["control"]["rate_hz"]
    ws = b.reset({"robots": {0: START}})

    def drive(setup, limit_s):
        nonlocal ws
        pc = PoseController(dd, deadzone=deadzones_from_config(cfg), field_mm=cfg["field"]["width_mm"], margin=margin,
                            learned=learned, **kw)
        r = ws.robots[0]
        setup(pc, (r.x, r.y))
        log, n = [], 0
        while not pc.done and n < limit_s * hz:
            r = ws.robots[0]
            cmd = pc.command(r.x, r.y, r.theta, r.omega, 1 / hz)
            safe = safety.filter(ws, {0: cmd}, True, n / hz)
            b.apply(safe)
            ws = b.step()
            x, y, th = b.true_pose(0)
            log.append((x, y, th, pc.phase))
            n += 1
        for _ in range(12):                                  # come to rest
            b.apply({0: RobotCommand()})
            ws = b.step()
        return log, b.true_pose(0), n / hz, not pc.done

    out = []
    # 1. straight
    log, end, sec, fail = drive(lambda pc, s: pc.set_goal(TOP[0], TOP[1], math.pi / 2, s), 12)
    out.append({"name": "straight", "seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], TOP),
                "dev_mm": max((seg_dist(e[:2], START[:2], TOP) for e in log if e[3] == "FOLLOW"), default=0),
                "heading_deg": abs(math.degrees(wrap(end[2] - math.pi / 2))), "wobbles": wobbles([e[2] for e in log if e[3] == "ALIGN"])})
    # 2. turn 120 deg on the spot
    here, target = end, wrap(math.pi / 2 + math.radians(120))
    log, end, sec, fail = drive(lambda pc, s: pc.set_goal(here[0], here[1], target, s), 12)
    out.append({"name": "turn", "seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], here[:2]),
                "heading_deg": abs(math.degrees(wrap(end[2] - target))), "wobbles": wobbles([e[2] for e in log])})
    # 3. half circle back to the start, then the final turn to face +y again
    pts = half_circle()
    log, end, sec, fail = drive(lambda pc, s: pc.set_path(pts, s, math.pi / 2, nearest=False), 16)
    out.append({"name": "curve", "seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], pts[-1]),
                "dev_mm": max((poly_dist(e[:2], pts) for e in log if e[3] == "FOLLOW"), default=0),
                "heading_deg": abs(math.degrees(wrap(end[2] - math.pi / 2))), "wobbles": wobbles([e[2] for e in log if e[3] == "ALIGN"])})
    b.close()
    return summarize(out)


# ------------------------------------------------------------------ the course on the real car (through the app)
class RealCourse:
    def __init__(self, url: str, robot: int):
        self.url, self.rid = url.rstrip("/"), robot

    def api(self, path, body=None):
        req = urllib.request.Request(self.url + path, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def robot(self):
        return next((r for r in self.api("/api/state")["robots"] if r["id"] == self.rid), None)

    def go(self, body, path=False, limit=20.0):
        res = self.api("/api/path" if path else "/api/goal", {"id": self.rid, **body})
        if not res.get("ok"):
            raise RuntimeError(res.get("msg"))
        t0, log, started = time.monotonic(), [], False
        while time.monotonic() - t0 < limit:
            r = self.robot()
            if r is None or not r["visible"]:
                raise RuntimeError("the car is not seen by the camera")
            log.append((r["x"], r["y"], r["theta"], r["phase"]))
            started = started or r["mode"] != "idle"
            if started and r["mode"] == "idle":
                break
            time.sleep(0.05)
        failed = not (started and r["mode"] == "idle")
        if failed:
            self.api("/api/stop", {"id": self.rid})
        time.sleep(0.8)
        r = self.robot()
        return log, (r["x"], r["y"], r["theta"]), time.monotonic() - t0 - 0.8, failed

    def park(self):
        self.go({"x": START[0], "y": START[1], "theta": START[2]}, limit=30)

    def run(self, params: dict) -> dict:
        res = self.api("/api/tuning", params)
        if not res.get("ok"):
            raise RuntimeError(res.get("msg"))
        r = self.robot()
        if r is None or not r["visible"]:
            raise RuntimeError("the car is not seen by the camera")
        if math.dist((r["x"], r["y"]), START[:2]) > 40 or abs(wrap(r["theta"] - START[2])) > 0.3:
            self.park()                                       # not scored
        out = []
        log, end, sec, fail = self.go({"x": TOP[0], "y": TOP[1], "theta": math.pi / 2})
        out.append({"seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], TOP),
                    "dev_mm": max((seg_dist(e[:2], START[:2], TOP) for e in log if e[3] == "FOLLOW"), default=0),
                    "heading_deg": abs(math.degrees(wrap(end[2] - math.pi / 2))), "wobbles": wobbles([e[2] for e in log if e[3] == "ALIGN"])})
        here, target = end, wrap(math.pi / 2 + math.radians(120))
        log, end, sec, fail = self.go({"x": here[0], "y": here[1], "theta": target})
        out.append({"seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], here[:2]),
                    "heading_deg": abs(math.degrees(wrap(end[2] - target))), "wobbles": wobbles([e[2] for e in log])})
        pts = half_circle()
        log, end, sec, fail = self.go({"points": [list(p) for p in pts], "theta": math.pi / 2}, path=True, limit=25)
        out.append({"seconds": sec, "failed": int(fail), "stop_mm": math.dist(end[:2], pts[-1]),
                    "dev_mm": max((poly_dist(e[:2], pts) for e in log if e[3] == "FOLLOW"), default=0),
                    "heading_deg": abs(math.degrees(wrap(end[2] - math.pi / 2))), "wobbles": wobbles([e[2] for e in log if e[3] == "ALIGN"])})
        return summarize(out)


# ------------------------------------------------------------------ the search
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--sim", action="store_true", help="tune in randomized simulators")
    where.add_argument("--real", action="store_true", help="tune on the real car through the running app")
    ap.add_argument("--url", default="http://127.0.0.1:8000", help="the running app (for --real, and for the start values)")
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--trials", type=int, default=0, help="settings to try (default: 160 in the simulator, 12 on the car)")
    ap.add_argument("--worlds", type=int, default=3, help="simulator only: randomized worlds per try")
    ap.add_argument("--precision", type=float, default=0.5, help="0 = fastest, 1 = most accurate")
    ap.add_argument("--group", choices=sorted(GROUPS), default="all", help="which settings to tune")
    ap.add_argument("--start", default=None, help='start values as JSON (default: the app\'s current values)')
    ap.add_argument("--workers", type=int, default=max(1, min(12, (os.cpu_count() or 4) - 2)))
    ap.add_argument("--config", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--stop-file", default=None)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    ranges = schema()
    keys = GROUPS[a.group]
    if a.start:
        start = json.loads(a.start)
    else:
        try:
            start = RealCourse(a.url, a.robot).api("/api/tuning")["values"]
        except OSError:
            from robofield.app.control_center import TUNING_SCHEMA  # noqa: F401  (defaults, without the app)
            from robofield.control import PoseController
            from robofield.control.kinematics import DiffDrive
            from robofield.core import load_config
            pc = PoseController(DiffDrive.from_config(load_config(a.config)))
            start = {k: (math.degrees(getattr(pc, k[:-4])) if k.endswith("_deg") else getattr(pc, k)) for k in KEYS}
    start = {k: float(start[k]) for k in KEYS}
    lo = np.array([ranges[k][0] for k in keys], float)
    hi = np.array([ranges[k][1] for k in keys], float)
    step = np.array([ranges[k][2] for k in keys], float)

    def to_params(z) -> dict:
        v = lo + np.clip(z, 0, 1) * (hi - lo)
        v = np.round(v / step) * step
        p = dict(start)
        p.update({k: round(float(x), 4) for k, x in zip(keys, v)})
        return p

    z0 = (np.array([start[k] for k in keys]) - lo) / (hi - lo)
    rng = np.random.default_rng(a.seed)
    real = a.real
    trials = a.trials or (12 if real else 160)
    stop_file = Path(a.stop_file) if a.stop_file else None
    t0 = time.monotonic()
    history = []

    if real:
        course = RealCourse(a.url, a.robot)

        def evaluate(batch, seeds):
            out = []
            for z in batch:
                try:
                    m = course.run(to_params(z))
                except RuntimeError as e:
                    course.api("/api/stop", {"id": a.robot})
                    raise SystemExit(f"stopped: {e}")
                out.append((score(m, a.precision), m))
            return out
        pool = None
    else:
        from multiprocessing import Pool
        pool = Pool(a.workers)

        def evaluate(batch, seeds):
            jobs = [(to_params(z), int(s), a.config) for z in batch for s in seeds]
            res = pool.map(run_sim, jobs)
            out = []
            for i in range(len(batch)):
                ms = res[i * len(seeds):(i + 1) * len(seeds)]
                avg = {k: float(np.mean([m[k] for m in ms])) for k in ms[0]}
                out.append((score(avg, a.precision), avg))
            return out

    print(f"autotune on {'the real car' if real else f'{a.worlds} randomized simulators per try'}: "
          f"{trials} tries, settings {', '.join(keys)}, precision {a.precision}", flush=True)
    seeds = list(range(a.seed * 1000, a.seed * 1000 + a.worlds))
    try:
        (base_score, base_m), = evaluate([z0], seeds)
    except SystemExit:
        if real:
            course.api("/api/tuning", start)
        raise
    best = (base_score, z0, base_m)
    print(f"start values: score {base_score} ({base_m})", flush=True)

    def report(n, s, m, z):
        nonlocal best
        if s < best[0]:
            best = (s, z, m)
        history.append(s)
        rec = {"trial": n, "trials": trials, "score": s, "best": best[0], "start": base_score,
               "t": round(time.monotonic() - t0, 1), "metrics": m}
        print("PROGRESS " + json.dumps(rec), flush=True)

    report(0, base_score, base_m, z0)
    n = 1
    if real:                                                  # (1+1) evolution strategy: few, careful tries
        sigma = 0.12
        try:
            while n < trials and not (stop_file and stop_file.exists()):
                z = np.clip(best[1] + rng.normal(0, sigma, len(keys)), 0, 1)
                (s, m), = evaluate([z], seeds)
                improved = s < best[0]
                report(n, s, m, z)
                sigma = min(0.25, sigma * 1.5) if improved else max(0.03, sigma * 0.85)
                n += 1
        finally:
            course.api("/api/tuning", start)                  # leave the app on its own values: Apply is your choice
    else:                                                     # cross-entropy method
        pop = max(8, min(24, trials // 4))
        mean, sigma = z0.copy(), np.full(len(keys), 0.18)
        it = 0
        while n < trials and not (stop_file and stop_file.exists()):
            k = min(pop, trials - n)
            batch = [np.clip(mean + rng.normal(0, 1, len(keys)) * sigma, 0, 1) for _ in range(k)]
            it_seeds = [s + 17 * (it + 1) for s in seeds]          # new worlds every round: no lucky overfit
            results = evaluate(batch, it_seeds)
            for z, (s, m) in zip(batch, results):
                report(n, s, m, z)
                n += 1
            order = np.argsort([s for s, _ in results])
            elite = np.array([batch[i] for i in order[:max(2, k // 4)]])
            mean = 0.7 * elite.mean(axis=0) + 0.3 * mean
            sigma = np.maximum(0.03, 0.6 * sigma + 0.4 * elite.std(axis=0))
            it += 1

    best_params = to_params(best[1])
    final = {}
    if not real:                                              # fair comparison on worlds neither has seen
        check = [9000 + s for s in range(max(4, a.worlds * 2))]
        (s_new, m_new), (s_old, m_old) = evaluate([best[1], z0], check)
        final = {"check_best": s_new, "check_start": s_old, "check_metrics_best": m_new, "check_metrics_start": m_old}
        if s_new >= s_old:                                    # not better on fresh worlds: keep what you have
            best_params, best = dict(start), (base_score, z0, base_m)
            final["note"] = "no setting beat the start values on fresh simulators; keeping them"
        pool.close()
    changed = {k: [start[k], best_params[k]] for k in keys if abs(best_params[k] - start[k]) > 1e-9}
    res = {"where": "real" if real else "sim", "trials": n, "seconds": round(time.monotonic() - t0, 1),
           "precision": a.precision, "start_score": base_score, "best_score": best[0], "best": best_params,
           "changed": changed, "metrics_best": best[2], "metrics_start": base_m, **final}
    better = 100 * (1 - (final.get("check_best", best[0]) / max(1e-9, final.get("check_start", base_score))))
    print(f"best score {best[0]} (start {base_score}); {len(changed)} settings changed; "
          f"{better:.0f}% better{' on fresh simulators' if final else ''}", flush=True)
    print("RESULT " + json.dumps(res), flush=True)


if __name__ == "__main__":
    main()
