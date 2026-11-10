"""Calibrate the camera lens with a printed chessboard.

    python tools/calibrate_camera.py                     # the reference board: 8 x 5 inner corners, 28 mm
    python tools/calibrate_camera.py --cols 9 --rows 6 --square 25    # another board

Show the board at many angles and positions; press SPACE to capture (aim for 20+),
then q to compute. Saves calibration/camera.npz with K and D.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera import open_camera  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=8, help="inner corners per row (the reference board: 9 squares = 8)")
    ap.add_argument("--rows", type=int, default=5, help="inner corners per column (6 squares = 5)")
    ap.add_argument("--square", type=float, default=28.0, help="square size in mm")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--out", default="calibration/camera.npz")
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj_pts, img_pts, size = [], [], None
    cap = open_camera({"index": a.camera})
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, corners, found)
        cv2.putText(view, f"captured: {len(obj_pts)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("calibration", cv2.resize(view, None, fx=0.6, fy=0.6))
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" ") and found:
            obj_pts.append(objp)
            img_pts.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit))
        elif key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
    if len(obj_pts) < 10:
        sys.exit("need at least 10 captures")
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez(a.out, K=K, D=D, size=size)
    print(f"RMS reprojection error: {rms:.3f} px (under 0.5 is good). Saved {a.out}")


if __name__ == "__main__":
    main()
