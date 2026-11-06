"""Measure how this car's motors actually respond, from the camera, and write the result into the
robot model: real wheel speed per command (-> max_wheel_rpm) and the left/right trim that makes
equal commands drive straight.

Commands are "effective" commands: the firmware already lifts every non-zero command above each
wheel's own dead zone (LEFT_DEADZONE / RIGHT_DEADZONE in the car's config.h), so the laptop sends
0..1 and should not add a dead zone of its own.

    python tools/calibrate_motors.py --robot 0            # measure and print
    python tools/calibrate_motors.py --robot 0 --write    # also save into config/field.yaml
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.comm.pose_link import POSE_PORT  # noqa: E402
from robofield.comm.protocol import Command  # noqa: E402
from robofield.comm.udp_link import CarLink  # noqa: E402
from robofield.core import load_config  # noqa: E402
from robofield.core.types import wrap_angle  # noqa: E402


class Poses:
    def __init__(self, marker_id: int):
        self.mid = marker_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", POSE_PORT))
        self.sock.setblocking(False)

    def drain(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                return out
            t = time.monotonic()
            for mid, x, y, th, _q in json.loads(data)["markers"]:
                if mid == self.mid:
                    out.append((t, x, y, th))


class Car:
    def __init__(self, cfg, rid):
        self.rid = rid
        self.swap = any(r.get("id") == rid and r.get("wheels_swapped") for r in cfg.get("robots", []))
        self.link = CarLink()
        t0 = time.monotonic()
        while rid not in self.link.poll() and time.monotonic() - t0 < 3:
            time.sleep(0.05)
        if rid not in self.link.cars:
            sys.exit(f"car {rid} is not online")

    def drive(self, left, right, seconds, poses, log):
        if self.swap:                                   # same mapping RealBackend uses
            left, right = right, left
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.link.send(Command(self.rid, left=int(left * 1000), right=int(right * 1000), servo=0))
            log.extend(poses.drain())
            time.sleep(0.01)

    def stop(self):
        for _ in range(5):
            self.link.send(Command(self.rid, left=0, right=0, servo=0))
            time.sleep(0.02)


def slope(ts, vs):
    mt, mv = statistics.fmean(ts), statistics.fmean(vs)
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den if den else 0.0


def unwrap(ths):
    out = [ths[0]]
    for th in ths[1:]:
        out.append(out[-1] + wrap_angle(th - out[-1]))
    return out


def steady(log, t_from, t_to):
    seg = [p for p in log if t_from <= p[0] <= t_to]
    if len(seg) < 4:
        return None
    ts = [p[0] for p in seg]
    th = unwrap([p[3] for p in seg])
    w = slope(ts, th)
    h = statistics.fmean(th)
    s = [p[1] * math.cos(h) + p[2] * math.sin(h) for p in seg]
    return slope(ts, s), w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--config", default="config/field.yaml")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    m = cfg["robot_model"]
    track = m["track_mm"]
    poses = Poses(cfg["markers"]["robot_id_offset"] + a.robot)
    car = Car(cfg, a.robot)
    try:
        log = []
        car.drive(0, 0, 0.6, poses, log)
        if not log:
            sys.exit("robot not visible to the camera")
        _, x0, y0, th0 = log[-1]

        # 1) pivots: wheel speed per command, both directions (no translation, safe anywhere)
        print("pivot tests (wheel speed per command):")
        gains = []
        for u in (0.15, 0.3, 0.5):
            for sign in (1, -1):
                log = []
                t0 = time.monotonic()
                car.drive(-sign * u, sign * u, 0.7, poses, log)
                car.drive(0, 0, 0.5, poses, log)
                r = steady(log, t0 + 0.35, t0 + 0.7)
                if r is None:
                    continue
                wheel = abs(r[1]) * track / 2
                gains.append((u, wheel))
                print(f"  u={u:.2f} {'ccw' if sign > 0 else 'cw '}: turn {r[1]:+6.2f} rad/s -> wheel {wheel:6.1f} mm/s"
                      f"  ({wheel / u:6.1f} mm/s per unit command)")

        # 2) straight, forward then back: drift -> trim
        log = []
        car.drive(0, 0, 0.4, poses, log)
        _, x0, y0, th0 = log[-1]
        ahead = [x0 + 260 * math.cos(th0), y0 + 260 * math.sin(th0)]
        behind = [x0 - 60 * math.cos(th0), y0 - 60 * math.sin(th0)]
        lo, hi = 70, cfg["field"]["width_mm"] - 70
        if not all(lo < c < hi for c in ahead + behind):
            sys.exit(f"not enough room in front of the robot at ({x0:.0f},{y0:.0f}) facing "
                     f"{math.degrees(th0):.0f} deg for the straight test - point it toward the middle and rerun")
        print("straight tests (drift -> trim):")
        drift = []
        u = 0.35
        for sign in (1, -1):
            log = []
            t0 = time.monotonic()
            car.drive(sign * u, sign * u, 0.8, poses, log)
            car.drive(0, 0, 0.5, poses, log)
            r = steady(log, t0 + 0.35, t0 + 0.8)
            if r is None:
                continue
            v, w = r
            vl, vr = v - w * track / 2, v + w * track / 2
            drift.append((vl / sign, vr / sign))
            print(f"  {'forward ' if sign > 0 else 'backward'}: v {v:+6.1f} mm/s, turn {w:+5.2f} rad/s, "
                  f"left {vl:+6.1f} right {vr:+6.1f} mm/s, curve radius {abs(v / w) if abs(w) > 1e-3 else 1e9:6.0f} mm")
    finally:
        car.stop()
        car.link.close()

    if not gains or not drift:
        sys.exit("not enough data")
    per_unit = statistics.median(w / u for u, w in gains if u >= 0.3)
    low = statistics.median(w / u for u, w in gains if u < 0.3)
    vl = statistics.fmean(d[0] for d in drift)
    vr = statistics.fmean(d[1] for d in drift)
    trim_l, trim_r = (min(vl, vr) / vl, 1.0) if vl > vr else (1.0, min(vl, vr) / vr)
    rpm = per_unit / (2 * math.pi * m["wheel_radius_mm"]) * 60
    print(f"\nwheel speed at full command: {per_unit:.0f} mm/s (= {rpm:.0f} rpm on a {m['wheel_radius_mm']} mm wheel)")
    print(f"small-command response: {low:.0f} mm/s per unit at u<0.3 ({100 * low / per_unit:.0f}% of the linear value)")
    print(f"trim: left {trim_l:.3f}  right {trim_r:.3f}   (straight drift: left {vl:.1f} vs right {vr:.1f} mm/s)")
    if a.write:
        path = Path(cfg["_path"])
        text = path.read_text(encoding="utf-8")
        for key, val in (("max_wheel_rpm", round(rpm, 1)), ("trim_left", round(trim_l, 3)),
                         ("trim_right", round(trim_r, 3))):
            text = set_yaml_scalar(text, key, val)
        path.write_text(text, encoding="utf-8")
        print(f"written to {path}")


def set_yaml_scalar(text: str, key: str, value) -> str:
    """Replace one `key: value` in place, keeping the line's comment and the rest of the file."""
    import re
    pat = re.compile(rf"^([ \t]*{re.escape(key)}:[ \t]*)([^#\r\n]*?)([ \t]*#[^\r\n]*)?$", re.M)
    new, n = pat.subn(lambda mo: f"{mo.group(1)}{value}{mo.group(3) or ''}", text, count=1)
    if n == 0:
        raise KeyError(f"{key} not found in the config")
    return new


if __name__ == "__main__":
    main()
