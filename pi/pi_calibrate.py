#!/usr/bin/env python3
"""One-time lens calibration for the Camera Module 3 Wide (strong distortion at the edges).

Print a chessboard, tape it flat to a board, and count its INNER corners (where four squares meet):
the reference board has 9 x 6 squares = 8 x 5 inner corners, 28 mm squares (the defaults). Then and move it slowly under the camera: all over the image, near the corners and edges,
tilted in different directions. The script grabs a frame every 1.5 s when it sees the board and
calibrates after 30 frames. Result: camera.npz (K, D) next to this file.

    python3 pi_calibrate.py                                  # the reference board: 8 x 5, 28 mm
    python3 pi_calibrate.py --cols 9 --rows 6 --square 25    # another board
    python3 pi_calibrate.py --source url:http://192.168.1.23:8080/video    # a phone as the camera
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from frame_source import open_source  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=8, help="inner corners along a row (9 squares = 8)")
    ap.add_argument("--rows", type=int, default=5, help="inner corners along a column (6 squares = 5)")
    ap.add_argument("--square", type=float, default=28.0, help="square size, mm")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--source", default="picamera")
    ap.add_argument("--out", default=str(HERE / "camera.npz"))
    a = ap.parse_args()
    src = open_source(a.source) if a.source != "picamera" else open_source("picamera", exposure_us=12000, gain=6.0)
    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    obj, img, last = [], [], 0.0
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    size = None
    while len(obj) < a.frames:
        gray, _ = src.read()
        size = gray.shape[::-1]
        small = cv2.resize(gray, None, fx=0.5, fy=0.5)
        found, corners = cv2.findChessboardCorners(small, pattern, cv2.CALIB_CB_FAST_CHECK)
        if found and time.monotonic() - last > 1.5:
            corners = cv2.cornerSubPix(gray, corners * 2, (11, 11), (-1, -1), crit)
            obj.append(objp); img.append(corners); last = time.monotonic()
            print(f"captured {len(obj)}/{a.frames}")
    src.close()
    flags = cv2.CALIB_RATIONAL_MODEL                     # wide lens: richer distortion model
    rms, K, D, _, _ = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    np.savez(a.out, K=K, D=D, size=size, rms=rms)
    print(f"RMS reprojection error {rms:.3f} px (good: < 0.5). Saved {a.out}")
    print("K =\\n", K.round(1), "\\nD =", D.ravel().round(4))


if __name__ == "__main__":
    main()
