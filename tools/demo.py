"""Demo drives for filming, through a running Control Center (python tools/control_center.py --real).

    python tools/demo.py wander     # random poses around the field, never near the walls
    python tools/demo.py figure8    # a smooth figure-eight, over and over
    python tools/demo.py square     # the four corners of the safe area, turning to face each one

Ctrl+C stops the car and exits. The Control Center's STOP button (or Esc) also stops it at any time.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import time
import urllib.request


def api(base, path, body=None):
    req = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as r:
        return json.loads(r.read())


def wait_idle(base, rid, timeout=30):
    t0, started = time.monotonic(), False
    while time.monotonic() - t0 < timeout:
        r = next((r for r in api(base, "/api/state")["robots"] if r["id"] == rid), None)
        if r is None or not r["visible"]:
            time.sleep(0.2); continue
        started = started or r["mode"] != "idle"
        if started and r["mode"] == "idle":
            return True
        time.sleep(0.1)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["wander", "figure8", "square"])
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--pause", type=float, default=1.0, help="seconds to rest between moves")
    a = ap.parse_args()
    base, rid = a.url, a.robot
    m = 200                                        # stay this far from the walls
    corners = [(m, m, 0.0), (1000 - m, m, math.pi / 2), (1000 - m, 1000 - m, math.pi), (m, 1000 - m, -math.pi / 2)]
    k = 0
    try:
        while True:
            if a.mode == "wander":
                x, y, th = random.uniform(m, 1000 - m), random.uniform(m, 1000 - m), random.uniform(-math.pi, math.pi)
                api(base, "/api/goal", {"id": rid, "x": x, "y": y, "theta": th})
            elif a.mode == "square":
                x, y, th = corners[k % 4]
                api(base, "/api/goal", {"id": rid, "x": x, "y": y, "theta": th})
            else:
                loop = [(500 + 290 * math.sin(t), 500 + 210 * math.sin(2 * t))
                        for t in (2 * math.pi * i / 96 for i in range(96))]
                r = next(r for r in api(base, "/api/state")["robots"] if r["id"] == rid)
                i0 = min(range(len(loop)), key=lambda i: math.dist(loop[i], (r["x"], r["y"])))
                pts = loop[i0:] + loop[:i0 + 1]          # one whole lap, starting where the car is
                api(base, "/api/path", {"id": rid, "points": pts, "theta": None})
            k += 1
            wait_idle(base, rid)
            time.sleep(a.pause)
    except KeyboardInterrupt:
        pass
    finally:
        api(base, "/api/stop", {"id": rid})
        print("stopped")


if __name__ == "__main__":
    main()
