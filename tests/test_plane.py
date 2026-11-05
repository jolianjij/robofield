"""Plane estimator on a simulated Pi Camera 3 Wide at 1.3 m (projected corners + pixel noise)."""
import math

import numpy as np
import cv2

from robofield.vision import PlaneEstimator
from robofield.vision.plane import tower_corners

W, H = 2304, 1296
F = (W / 2) / math.tan(math.radians(102 / 2))           # ~102 deg horizontal FOV
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
TOWERS = {0: (-81, -81, 220), 1: (1081, -81, 260), 2: (1081, 1081, 180), 3: (-81, 1081, 140)}
CFG = {
    "markers": {"dictionary": "DICT_4X4_50", "corners": TOWERS, "corner_size_mm": 80, "robot_size_mm": 35},
    "camera": {"height_mm": 1300},
    "vision": {"robot_marker_height_mm": 45, "corner_refinement": "subpix"},
}


def camera(cx=500.0, cy=500.0, cz=1300.0, tilt_deg=2.0):
    """Camera looking down, slightly tilted (real mounts are never perfect)."""
    R0 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    Rt, _ = cv2.Rodrigues(np.array([math.radians(tilt_deg), math.radians(-tilt_deg / 2), 0.0]))
    R = Rt @ R0
    t = -R @ np.array([[cx], [cy], [cz]])
    return R, t


def project(R, t, pts3d, rng, noise=0.3):
    rvec, _ = cv2.Rodrigues(R)
    px, _ = cv2.projectPoints(np.asarray(pts3d, float), rvec, t, K, None)
    return px.reshape(-1, 2) + rng.normal(0, noise, (len(pts3d), 2))


def robot_corners(x, y, th, size=35, z=45):
    s = size / 2
    f = np.array([math.cos(th), math.sin(th)])
    r = np.array([f[1], -f[0]])
    loc = [(-s, s), (s, s), (s, -s), (-s, -s)]      # (right, forward) for TL, TR, BR, BL
    return [[x + a * r[0] + b * f[0], y + a * r[1] + b * f[1], z] for a, b in loc]


def test_plane_estimator_accuracy():
    rng = np.random.default_rng(0)
    R, t = camera()
    pos_err, ang_err = [], []
    for k in range(40):
        x, y, th = rng.uniform(40, 960), rng.uniform(40, 960), rng.uniform(-math.pi, math.pi)
        det = {i: project(R, t, tower_corners(*TOWERS[i], 80), rng) for i in TOWERS}
        det[10] = project(R, t, robot_corners(x, y, th), rng)
        est = PlaneEstimator(CFG, K)
        p = est.estimate_from_detections(det, 0.0)[0]
        pos_err.append(math.hypot(p.x - x, p.y - y))
        ang_err.append(abs(math.degrees(math.atan2(math.sin(p.theta - th), math.cos(p.theta - th)))))
    assert np.mean(pos_err) < 1.5 and max(pos_err) < 4, (np.mean(pos_err), max(pos_err))
    assert np.mean(ang_err) < 2.0, np.mean(ang_err)
    assert np.linalg.norm(est.camera_position - [500, 500, 1300]) < 15


def test_works_with_two_towers_occluded():
    rng = np.random.default_rng(1)
    R, t = camera()
    det = {i: project(R, t, tower_corners(*TOWERS[i], 80), rng) for i in (0, 2)}
    det[11] = project(R, t, robot_corners(300, 700, 0.5), rng)
    p = PlaneEstimator(CFG, K).estimate_from_detections(det, 0.0)[0]
    assert math.hypot(p.x - 300, p.y - 700) < 5
