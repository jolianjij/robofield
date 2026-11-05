"""Live tracking demo: shows detected markers and field-frame poses.

    python tools/track_demo.py --config config/field.yaml [--estimator pnp]

Keys: q = quit.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera import open_camera  # noqa: E402
from robofield.core import load_config  # noqa: E402
from robofield.vision import build_estimator  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/field.yaml")
    ap.add_argument("--estimator", choices=["homography", "pnp"])
    args = ap.parse_args()
    cfg = load_config(args.config)
    if args.estimator:
        cfg["vision"]["estimator"] = args.estimator

    K = D = None
    calib = Path(cfg["camera"].get("calibration_file", ""))
    if calib.is_file():
        data = np.load(calib)
        K, D = data["K"], data["D"]
        print(f"loaded calibration {calib}")

    est = build_estimator(cfg, K, D)
    cap = open_camera(cfg["camera"])
    last, fps = time.monotonic(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        poses = est.estimate(frame)
        now = time.monotonic()
        fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)
        last = now

        corners, ids, _ = est.detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        status = "FIELD LOCKED" if est.field_locked else "SEARCHING FOR 4 CORNERS"
        cv2.putText(frame, f"{status}  {fps:.0f} fps", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        for k, p in enumerate(poses):
            txt = f"id {p.marker_id}: x={p.x:6.1f} y={p.y:6.1f} mm  th={math.degrees(p.theta):6.1f} deg  q={p.quality:.2f}"
            cv2.putText(frame, txt, (20, 80 + 32 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("RoboField tracking", cv2.resize(frame, None, fx=0.6, fy=0.6))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
