"""Film a saved move: park the car at its start, record the field camera AND the Control Center page,
play the move, stop both recordings. Needs a running Control Center (python tools/control_center.py --real --pi <pi>).

    python tools/film_move.py hook                         # the opening shot (a move you drew and saved)
    python tools/film_move.py compare --style before       # old controller, no delay prediction
    python tools/film_move.py compare --style after        # pulse turns + delay prediction (the fix)
    python tools/film_move.py compare --style both         # before, then after, from the same start

"compare" is created on first use: an S-curve across the field ending in a 120 deg final turn - it
shows both the path tracking and the final orientation. Each take saves, in recordings/:
    <move>_<style>_<time>_camera.mp4   the Pi camera with the tracking overlay
    <move>_<style>_<time>_screen.mp4   the Control Center page (map, path, camera panel)
    <move>_<style>_<time>.json         how it went: time, stop error, final heading error, wobbles
Parking at the start is not recorded. The Control Center's STOP (or Ctrl+C here) stops the car.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STYLES = {   # tuning for each style; "after" is also what the car is left with
    "before": {"turn_style": 0, "predict": 0},
    "after": {"turn_style": 1, "predict": 1},
}
CAPTIONS = {"before": ("Before: no prediction", "bad"), "after": ("After: predicting the delay", "ok")}


def api(base, path, body=None, timeout=3):
    req = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def robot(base, rid):
    return next((r for r in api(base, "/api/state")["robots"] if r["id"] == rid), None)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compare_move() -> dict:
    """S-curve from the lower left to the upper right, then a 120 deg turn on the spot."""
    pts = []
    for i in range(41):
        u = i / 40
        x = 230 + 490 * u
        y = 300 + 350 * (3 * u * u - 2 * u * u * u)          # smoothstep: leaves and arrives heading +x
        pts.append([round(x, 1), round(y, 1)])
    return {"name": "compare", "points": pts, "final_theta": math.radians(120)}


def check_seen(base, rid):
    """The tracker must be running and see the car - never film (or drive) on a stale position."""
    for _ in range(50):
        s = api(base, "/api/state")
        r = next((r for r in s["robots"] if r["id"] == rid), None)
        if s["tracker"]["ok"] and r and r["visible"]:
            return r
        time.sleep(0.2)
    raise SystemExit("the tracker is down or does not see the car - nothing recorded "
                     f"(tracker: {s['tracker']}; is the Pi on the network?)")


def wait_idle(base, rid, timeout, log=None):
    """Wait for the car to start something and finish it; optionally record (t, x, y, theta, phase)."""
    t0, started = time.monotonic(), False
    while time.monotonic() - t0 < timeout:
        r = robot(base, rid)
        if r and r["x"] is not None and log is not None:
            log.append((time.monotonic() - t0, r["x"], r["y"], r["theta"], r["phase"], r["mode"]))
        if r:
            started = started or r["mode"] != "idle"
            if started and r["mode"] == "idle":
                return True
        time.sleep(0.05)
    return False


def wobbles(log, phase):
    th = [e[3] for e in log if e[4] == phase]
    steps = [s for s in (wrap(b - a) for a, b in zip(th, th[1:])) if abs(s) > math.radians(1.5)]
    return sum(1 for a, b in zip(steps, steps[1:]) if (a > 0) != (b > 0))


def start_recorders(a, stem: Path, style: str, stop: Path):
    procs = []
    py = sys.executable
    if not a.no_camera:
        procs.append(subprocess.Popen([py, str(ROOT / "tools/record_camera.py"), "--pi", a.pi, "--out", f"{stem}_camera.mp4",
                                       "--stop-file", str(stop)], stdout=subprocess.PIPE, text=True, cwd=ROOT))
    if not a.no_screen:
        url = a.url + "/"
        if style in CAPTIONS and not a.no_caption:
            cap, tone = CAPTIONS[style]
            url += "?" + urllib.parse.urlencode({"caption": cap, "tone": tone})
        procs.append(subprocess.Popen([py, str(ROOT / "tools/record_screen.py"), "--url", url, "--out", f"{stem}_screen.mp4",
                                       "--stop-file", str(stop)], stdout=subprocess.PIPE, text=True, cwd=ROOT))
    for p in procs:                                   # wait until each one is actually writing frames
        line = p.stdout.readline()
        if not line.startswith("recording"):
            stop.touch()                              # let the others close their files, then give up
            for q in procs:
                try:
                    q.wait(15)
                except subprocess.TimeoutExpired:
                    q.kill()
            stop.unlink(missing_ok=True)
            raise SystemExit(f"a recorder did not start ({p.args[1]}): {line.strip() or 'no output'}"
                             " - is the Pi tracker running?")
        print("  " + line.strip())
    return procs


def stop_recorders(procs, stop: Path):
    stop.touch()
    for p in procs:
        out, _ = p.communicate(timeout=30)
        for line in out.splitlines():
            print("  " + line.strip())
    stop.unlink(missing_ok=True)


def take(a, base, move: dict, style: str):
    rid = a.robot
    stamp = time.strftime("%Y%m%d_%H%M%S")
    stem = ROOT / "recordings" / f"{move['name']}_{style}_{stamp}"
    stem.parent.mkdir(parents=True, exist_ok=True)
    stop = stem.with_suffix(".stop")
    (x0, y0), far = move["points"][0], move["points"][min(len(move["points"]) - 1, 3)]
    th0 = math.atan2(far[1] - y0, far[0] - x0)

    print(f"\n== {move['name']} / {style}")
    r = check_seen(base, rid)
    api(base, "/api/tuning", STYLES["after"])        # park with the good controller (not recorded)
    if math.dist((r["x"], r["y"]), (x0, y0)) > 30 or abs(wrap(r["theta"] - th0)) > math.radians(6):
        print("  parking at the start ...")
        res = api(base, "/api/moves/start", {"id": rid, "name": move["name"]})
        if not res.get("ok"):
            raise SystemExit(res.get("msg"))
        t0 = time.monotonic()
        time.sleep(0.5)
        while time.monotonic() - t0 < 60:                 # two legs: behind the start, then straight in
            r = robot(base, rid)
            if r and not r.get("parking") and r["mode"] == "idle":
                break
            time.sleep(0.1)
        time.sleep(0.8)
        r = check_seen(base, rid)
        off_mm, off_deg = math.dist((r["x"], r["y"]), (x0, y0)), abs(math.degrees(wrap(r["theta"] - th0)))
        print(f"  parked {off_mm:.0f} mm / {off_deg:.1f} deg from the start")
        if off_mm > 45 or off_deg > 12:
            raise SystemExit("could not park at the start - nothing recorded; try again")

    procs = start_recorders(a, stem, style, stop)     # recording before the controller is switched,
    log = []                                          # so a failed start never leaves it on "before"
    try:
        if style in STYLES:
            api(base, "/api/tuning", STYLES[style])
        check_seen(base, rid)
        time.sleep(a.lead)
        print("  playing ...")
        res = api(base, "/api/moves/play", {"id": rid, "name": move["name"]})
        if not res.get("ok"):
            raise RuntimeError(res.get("msg"))
        t_play = time.monotonic()
        # the old controller may never settle: cut it off (still on camera) after --before-seconds
        finished = wait_idle(base, rid, a.before_seconds if style == "before" else a.timeout, log)
        dur = time.monotonic() - t_play
        if not finished:
            api(base, "/api/stop", {"id": rid})
        time.sleep(a.tail)
    except KeyboardInterrupt:
        api(base, "/api/stop", {"id": rid})
        finished, dur = False, 0.0
    finally:
        stop_recorders(procs, stop)
        api(base, "/api/tuning", STYLES["after"])    # always leave the car on the good controller

    r = robot(base, rid)
    gx, gy = move["points"][-1]
    result = {"move": move["name"], "style": style, "finished": finished, "seconds": round(dur, 1),
              "stop_error_mm": round(math.dist((r["x"], r["y"]), (gx, gy)), 1),
              "path_wobbles": wobbles(log, "FOLLOW"), "turn_wobbles": wobbles(log, "ALIGN"),
              "camera": f"{stem.name}_camera.mp4", "screen": f"{stem.name}_screen.mp4"}
    if move.get("final_theta") is not None:
        result["heading_error_deg"] = round(abs(math.degrees(wrap(r["theta"] - move["final_theta"]))), 1)
    stem.with_suffix(".json").write_text(json.dumps({**result, "log": log}, indent=1))
    print("  " + ", ".join(f"{k} {v}" for k, v in result.items() if k not in ("camera", "screen")))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("move", help="name of a saved move (config/moves/<name>.json); 'compare' is created if missing")
    ap.add_argument("--style", choices=["as-is", "before", "after", "both"], default="as-is",
                    help="controller to film with (as-is: whatever the Control Center has now)")
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--pi", default="10.127.58.149")
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--lead", type=float, default=2.0, help="seconds of the car standing still before it moves")
    ap.add_argument("--tail", type=float, default=2.0, help="seconds kept after it stops")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--before-seconds", type=float, default=20.0, help="stop the 'before' take after this long")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-screen", action="store_true")
    ap.add_argument("--no-caption", action="store_true", help="no Before/After label on the page")
    a = ap.parse_args()
    base = a.url.rstrip("/")
    a.url = base

    names = [m["name"] for m in api(base, "/api/moves")["moves"]]
    if a.move not in names:
        if a.move != "compare":
            raise SystemExit(f"no saved move '{a.move}' (saved: {', '.join(names) or 'none'})")
        m = compare_move()
        api(base, "/api/moves/save", {"name": m["name"], "points": m["points"], "theta": m["final_theta"]})
    move = next(m for m in api(base, "/api/moves")["moves"] if m["name"] == a.move)
    for style in (["before", "after"] if a.style == "both" else [a.style]):
        take(a, base, move, style)


if __name__ == "__main__":
    main()
