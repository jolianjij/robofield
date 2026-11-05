"""Pose messages from the camera tracker (Raspberry Pi) to the laptop, as compact JSON over UDP.

{"seq": 812, "t": 1730550000.123, "fps": 41.8, "lat_ms": 23.5, "locked": true,
 "cam": [500.4, 499.1, 1299.6], "reproj": 0.41,
 "markers": [[10, 412.3, 288.9, 1.571, 0.98], ...]}       # id, x mm, y mm, theta rad, quality
"""
from __future__ import annotations

import json
import socket
import time

POSE_PORT = 5005
OVERLAY_PORT = 5006       # laptop -> Pi: goal and path to draw on the live preview


class PoseSender:
    def __init__(self, host: str = "255.255.255.255", port: int = POSE_PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.seq = 0

    def send(self, poses, t_capture: float, fps: float, locked: bool, cam=None, reproj=None, lat_ms=None):
        msg = {"seq": self.seq, "t": t_capture, "fps": round(fps, 1), "locked": locked,
               "lat_ms": None if lat_ms is None else round(lat_ms, 1),
               "cam": None if cam is None else [round(float(v), 1) for v in cam],
               "reproj": None if reproj is None else round(float(reproj), 2),
               "markers": [[p.marker_id, round(p.x, 2), round(p.y, 2), round(p.theta, 4), round(p.quality, 2)] for p in poses]}
        self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), self.addr)
        self.seq += 1


class PoseReceiver:
    def __init__(self, port: int = POSE_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.setblocking(False)
        self.latest: dict | None = None
        self.received_at = 0.0

    def poll(self) -> dict | None:
        """Newest message (older ones waiting in the buffer are skipped)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                break
            try:
                self.latest = json.loads(data)
                self.received_at = time.monotonic()
            except ValueError:
                pass
        return self.latest

    def close(self):
        self.sock.close()


class OverlaySender:
    """Laptop side: tell the Pi preview what to draw (goal pose, planned path)."""

    def __init__(self, host: str, port: int = OVERLAY_PORT):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, goal=None, path=None):
        msg = {"goal": None if goal is None else [round(float(v), 3) for v in goal],
               "path": None if path is None else [[round(x, 1), round(y, 1)] for x, y in path]}
        self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), self.addr)


class OverlayReceiver(PoseReceiver):
    """Pi side: newest overlay message (same mechanics as PoseReceiver)."""

    def __init__(self, port: int = OVERLAY_PORT):
        super().__init__(port)
