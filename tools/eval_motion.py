"""Score the motion controllers on the real field (or the simulator), through a running Control Center.

    python tools/control_center.py --real            # in another terminal
    python tools/eval_motion.py --robot 0             # straight, turn, curve
    python tools/eval_motion.py --robot 0 --tests turn --repeat 3

For each maneuver it reports the time taken, how far the robot strayed from the line or drawn path,
how far from the goal it stopped, its final heading error, and "wobbles": how many times the heading
reversed direction while settling (the left-right oscillation, counted).
"""
from __future__ import annotations

import argparse
import json
import math
import time
import urllib.request


def api(base, path, body=None):
    req = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read())


def robot(base, rid):
    return next((r for r in api(base, "/api/state")["robots"] if r["id"] == rid), None)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def poly_dist(p, pts):
    return min(seg_dist(p, a, b) for a, b in zip(pts, pts[1:]))


def run(base, rid, send, timeout=25.0):
    """Start a maneuver and record (t, x, y, theta, phase) until the robot is idle again."""
    res = send()
    if not res.get("ok"):
        raise RuntimeError(res.get("msg"))
    t0, log, started = time.monotonic(), [], False
    while time.monotonic() - t0 < timeout:
        r = robot(base, rid)
        if r and r["x"] is not None:
            log.append((time.monotonic() - t0, r["x"], r["y"], r["theta"], r["phase"]))
            started = started or r["mode"] != "idle"
            if started and r["mode"] == "idle":
                break
        time.sleep(0.05)
    time.sleep(0.8)                         # let it come fully to rest before the final reading
    r = robot(base, rid)
    return log, (r["x"], r["y"], r["theta"]), time.monotonic() - t0 - 0.8


def wobbles(log, phase=None):
    """Heading direction reversals of more than 1.5 deg, within one phase (or all)."""
    th = [e[3] for e in log if phase is None or e[4] == phase]
    steps = [wrap(b - a) for a, b in zip(th, th[1:])]
    steps = [s for s in steps if abs(s) > math.radians(1.5)]
    return sum(1 for a, b in zip(steps, steps[1:]) if (a > 0) != (b > 0))


def inside(x, y, m=150, size=1000):
    return m < x < size - m and m < y < size - m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--tests", nargs="*", default=["straight", "turn", "curve"])
    ap.add_argument("--repeat", type=int, default=1)
    a = ap.parse_args()
    base, rid = a.url, a.robot
    for _ in range(a.repeat):
        for test in a.tests:
            r = robot(base, rid)
            if r is None or r["x"] is None:
                print("robot not visible"); return
            x, y, th = r["x"], r["y"], r["theta"]
            if test == "straight":
                d = 350
                hx, hy = math.cos(th), math.sin(th)
                if not inside(x + d * hx, y + d * hy):                     # head for the middle instead
                    th = math.atan2(500 - y, 500 - x); hx, hy = math.cos(th), math.sin(th)
                    d = min(350, math.hypot(500 - x, 500 - y) + 150)
                goal = (x + d * hx, y + d * hy, th)
                log, end, dur = run(base, rid, lambda: api(base, "/api/goal", {"id": rid, "x": goal[0], "y": goal[1], "theta": goal[2]}))
                follow = [e for e in log if e[4] == "FOLLOW"]
                dev = max((seg_dist(e[1:3], (x, y), goal[:2]) for e in follow), default=0)
                print(f"straight {d:.0f} mm: {dur:4.1f} s   off the line max {dev:4.0f} mm   stop error "
                      f"{math.dist(end[:2], goal[:2]):4.0f} mm   heading error {abs(math.degrees(wrap(end[2] - goal[2]))):4.1f} deg   "
                      f"wobbles {wobbles(log, 'ALIGN')}")
            elif test == "turn":
                target = wrap(th + math.radians(120))
                log, end, dur = run(base, rid, lambda: api(base, "/api/goal", {"id": rid, "x": x, "y": y, "theta": target}))
                print(f"turn 120 deg:  {dur:4.1f} s   heading error {abs(math.degrees(wrap(end[2] - target))):4.1f} deg   "
                      f"drifted {math.dist(end[:2], (x, y)):4.0f} mm   wobbles {wobbles(log)}")
            elif test == "curve":
                # a half circle, bending toward the middle of the field
                cx, cy = 500, 500
                rad = 170
                a0 = math.atan2(y - cy, x - cx)
                sx, sy = cx + 220 * math.cos(a0), cy + 220 * math.sin(a0)      # start on a ring around the middle
                ccx, ccy = cx + 50 * math.cos(a0), cy + 50 * math.sin(a0)
                pts = [(sx, sy)] + [(ccx + rad * math.cos(a0 + k * math.pi / 24), ccy + rad * math.sin(a0 + k * math.pi / 24))
                                    for k in range(1, 25)]
                log, end, dur = run(base, rid, lambda: api(base, "/api/path", {"id": rid, "points": pts, "theta": None}))
                follow = [e for e in log if e[4] == "FOLLOW"]
                dev = max((poly_dist(e[1:3], pts) for e in follow[5:]), default=0)
                print(f"curve r={rad}: {dur:4.1f} s   off the path max {dev:4.0f} mm   stop error "
                      f"{math.dist(end[:2], pts[-1]):4.0f} mm")
            time.sleep(0.5)


if __name__ == "__main__":
    main()
