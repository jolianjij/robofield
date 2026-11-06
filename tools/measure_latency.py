"""Measure the real control-loop delay: command sent -> wheels move -> camera sees it -> laptop
receives the pose. This is the delay the latency predictor has to cancel.

From standstill, the car pivots in place for a short pulse; the time from sending the command to
the first visible heading change is the "start" delay, and from sending stop to the heading
settling is the "stop" delay. Trials alternate direction so the car ends where it started.

    python tools/measure_latency.py --robot 0
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
    """Every pose packet with its own receive time (PoseReceiver keeps only the newest)."""

    def __init__(self, marker_id: int):
        self.mid = marker_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", POSE_PORT))
        self.sock.setblocking(False)

    def drain(self) -> list[tuple[float, float, float, float]]:
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


def send_for(cars, rid, left, right, seconds, poses, log):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        cars.send(Command(rid, left=int(left * 1000), right=int(right * 1000), servo=0))
        log.extend(poses.drain())
        time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--duty", type=float, default=0.45, help="pivot wheel duty (fraction)")
    ap.add_argument("--pulse", type=float, default=0.5, help="seconds of turning per trial")
    ap.add_argument("--trials", type=int, default=6)
    ap.add_argument("--config", default="config/field.yaml")
    a = ap.parse_args()
    cfg = load_config(a.config)
    poses = Poses(cfg["markers"]["robot_id_offset"] + a.robot)
    cars = CarLink()
    try:
        t0 = time.monotonic()
        while a.robot not in cars.poll() and time.monotonic() - t0 < 3:
            time.sleep(0.05)
        if a.robot not in cars.cars:
            sys.exit(f"car {a.robot} is not online")
        starts, stops, rates = [], [], []
        for k in range(a.trials):
            sign = 1 if k % 2 == 0 else -1
            log: list = []
            send_for(cars, a.robot, 0, 0, 0.8, poses, log)               # settle, baseline
            base = [th for t, x, y, th in log[-15:]]
            if len(base) < 5:
                sys.exit("robot not visible to the camera")
            th0 = math.atan2(sum(math.sin(v) for v in base), sum(math.cos(v) for v in base))
            noise = statistics.pstdev([wrap_angle(v - th0) for v in base])
            thresh = max(math.radians(1.5), 5 * noise)
            log = []
            t_go = time.monotonic()
            send_for(cars, a.robot, -sign * a.duty, sign * a.duty, a.pulse, poses, log)
            t_stop = time.monotonic()
            send_for(cars, a.robot, 0, 0, 1.0, poses, log)
            rel = [(t, wrap_angle(th - th0)) for t, x, y, th in log]
            onset = next((t for t, d in rel if abs(d) > thresh), None)
            final = statistics.median(d for t, d in rel[-10:])
            moving = [t for t, d in rel if t > t_stop and abs(d - final) > thresh]
            if onset is None:
                print(f"trial {k}: no motion seen (duty too low?)")
                continue
            starts.append(onset - t_go)
            stops.append((moving[-1] if moving else t_stop) - t_stop)
            during = [(t, d) for t, d in rel if t_stop - 0.15 < t < t_stop]
            if len(during) >= 2:
                rates.append(abs(during[-1][1] - during[0][1]) / max(1e-3, during[-1][0] - during[0][0]))
            print(f"trial {k}: start delay {starts[-1]*1000:5.0f} ms, stop delay {stops[-1]*1000:5.0f} ms, "
                  f"turned {math.degrees(final):6.1f} deg, noise {math.degrees(noise):.2f} deg")
        if starts:
            print(f"\nmedian start delay {statistics.median(starts)*1000:.0f} ms   "
                  f"median stop delay {statistics.median(stops)*1000:.0f} ms   "
                  f"turn rate at duty {a.duty}: {statistics.median(rates):.2f} rad/s" if rates else "")
    finally:
        for _ in range(5):
            cars.send(Command(a.robot, left=0, right=0, servo=0))
            time.sleep(0.02)
        cars.close()


if __name__ == "__main__":
    main()
