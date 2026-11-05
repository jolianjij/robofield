#!/usr/bin/env python3
"""RoboField tracker: camera -> ArUco detection -> field poses -> UDP to the laptop.

    python3 pi_tracker.py --config ../config/field.yaml --calib camera.npz
    python3 pi_tracker.py ... --laptop 192.168.1.20      # unicast instead of broadcast
    python3 pi_tracker.py ... --preview 8080             # live annotated view at http://<pi>:8080
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from frame_source import open_source  # noqa: E402
from robofield.comm.pose_link import OVERLAY_PORT, POSE_PORT, OverlayReceiver, PoseSender  # noqa: E402
from robofield.core import load_config  # noqa: E402
from robofield.vision.plane import PlaneEstimator  # noqa: E402


class Preview:
    """Minimal MJPEG server: open http://<pi-address>:<port> in a browser to see what the tracker sees."""

    def __init__(self, port: int):
        self.jpeg = None
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        if outer.jpeg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + outer.jpeg + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        threading.Thread(target=ThreadingHTTPServer(("", port), H).serve_forever, daemon=True).start()

    def update(self, gray, est, poses, fps, overlay=None, robot_z=45.0):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        corners, ids, _ = est.detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
        if est.field_locked:
            draw_field_frame(img, est, poses, overlay, robot_z)
        txt = f"{'FIELD LOCKED' if est.field_locked else 'NEED 2+ TOWERS'}  {fps:.0f} fps"
        if est.field_locked:
            c = est.camera_position
            txt += f"  cam ({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) reproj {est.reproj_error_px:.2f}px"
        cv2.putText(img, txt, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
        for k, p in enumerate(poses):
            cv2.putText(img, f"id {p.marker_id}: {p.x:7.1f} {p.y:7.1f} mm {np.degrees(p.theta):6.1f} deg",
                        (20, 100 + 45 * k), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        small = cv2.resize(img, None, fx=0.5, fy=0.5)
        self.jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()


def draw_field_frame(img, est, poses, overlay, robot_z):
    """Project the field coordinate system onto the camera image using the camera pose the tracker
    solved from the towers: 100 mm grid (tile seams every 250 mm thicker), X/Y axes at the origin,
    robot heading arrows, and the goal / path sent by the laptop. If the grid sits on the real tile
    seams, calibration and tower positions are right."""
    rvec, _ = cv2.Rodrigues(est.R)
    tvec = est.t
    D = est.D if est.D is not None else np.zeros(5)

    def proj(pts):
        p, _ = cv2.projectPoints(np.asarray(pts, np.float64).reshape(-1, 3), rvec, tvec, est.K, D)
        return p.reshape(-1, 2)

    def polyline(pts, color, w):
        p = proj(pts).astype(np.int32)
        cv2.polylines(img, [p], False, color, w, cv2.LINE_AA)

    for v in range(0, 1001, 100):
        seam = v % 250 == 0
        col, w = ((255, 255, 255), 3) if seam else ((170, 170, 170), 1)
        polyline([(v, y, 0) for y in range(0, 1001, 50)], col, w)
        polyline([(x, v, 0) for x in range(0, 1001, 50)], col, w)
    o, ax, ay = proj([(0, 0, 0), (200, 0, 0), (0, 200, 0)]).astype(int)
    cv2.arrowedLine(img, tuple(o), tuple(ax), (0, 0, 255), 6, tipLength=0.15)
    cv2.arrowedLine(img, tuple(o), tuple(ay), (0, 200, 0), 6, tipLength=0.15)
    cv2.putText(img, "X", tuple(ax + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
    cv2.putText(img, "Y", tuple(ay + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 200, 0), 4)
    for p in poses:
        a, b = proj([(p.x, p.y, robot_z), (p.x + 70 * np.cos(p.theta), p.y + 70 * np.sin(p.theta), robot_z)]).astype(int)
        cv2.arrowedLine(img, tuple(a), tuple(b), (255, 0, 255), 5, tipLength=0.3)
    if overlay:
        if overlay.get("path"):
            polyline([(x, y, 0) for x, y in overlay["path"]], (0, 165, 255), 4)
        if overlay.get("goal"):
            gx, gy, gth = overlay["goal"]
            ring = [(gx + 25 * np.cos(t), gy + 25 * np.sin(t), 0) for t in np.linspace(0, 2 * np.pi, 25)]
            polyline(ring, (0, 165, 255), 4)
            a, b = proj([(gx, gy, 0), (gx + 80 * np.cos(gth), gy + 80 * np.sin(gth), 0)]).astype(int)
            cv2.arrowedLine(img, tuple(a), tuple(b), (0, 165, 255), 5, tipLength=0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(HERE.parent / "config" / "field.yaml"))
    ap.add_argument("--calib", default=str(HERE / "camera.npz"), help="from pi_calibrate.py")
    ap.add_argument("--source", default="picamera", help="picamera | opencv:0 | file:/path/*.png")
    ap.add_argument("--laptop", default="255.255.255.255", help="laptop IP (default: broadcast)")
    ap.add_argument("--port", type=int, default=POSE_PORT)
    ap.add_argument("--preview", type=int, default=0, help="port for the live MJPEG view (0 = off)")
    ap.add_argument("--exposure-us", type=int, default=4000)
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--focus", type=float, default=0.75, help="LensPosition in dioptres (1/m)")
    a = ap.parse_args()

    cfg = load_config(a.config)
    calib = np.load(a.calib)
    K, D = calib["K"], calib["D"]
    est = PlaneEstimator(cfg, K, D)
    kw = dict(exposure_us=a.exposure_us, gain=a.gain, lens_position=a.focus) if a.source == "picamera" else {}
    src = open_source(a.source, **kw)
    sender = PoseSender(a.laptop, a.port)
    preview = Preview(a.preview) if a.preview else None
    overlay_rx = OverlayReceiver(OVERLAY_PORT) if preview else None
    robot_z = cfg["vision"].get("robot_marker_height_mm", 45)
    print(f"tracking -> {a.laptop}:{a.port}" + (f", preview on :{a.preview}" if preview else ""))

    fps, last, n = 0.0, time.monotonic(), 0
    try:
        while True:
            gray, t_cap = src.read()
            t0 = time.monotonic()
            poses = est.estimate(gray, t_cap)
            proc_ms = (time.monotonic() - t0) * 1000
            now = time.monotonic(); fps = 0.9 * fps + 0.1 / max(now - last, 1e-6); last = now
            locked = est.field_locked
            sender.send(poses, t_cap, fps, locked, est.camera_position if locked else None,
                        est.reproj_error_px if locked else None, proc_ms)
            n += 1
            if preview and n % 3 == 0:
                preview.update(gray, est, poses, fps, overlay_rx.poll(), robot_z)
            if n % 100 == 0:
                print(f"{fps:5.1f} fps  detect+pose {proc_ms:5.1f} ms  locked={locked}  robots={[p.marker_id for p in poses]}")
    except KeyboardInterrupt:
        pass
    finally:
        src.close()


if __name__ == "__main__":
    main()
