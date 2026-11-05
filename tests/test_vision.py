import math

import numpy as np

from robofield.vision import HomographyEstimator
from robofield.vision.homography import parallax_correct
from synthetic import render

CFG = {
    "markers": {"dictionary": "DICT_4X4_50", "corners": {0: (0, 0), 1: (1000, 0), 2: (1000, 1000), 3: (0, 1000)},
                "corner_size_mm": 60, "robot_size_mm": 35},
    "camera": {"height_mm": 1400},
    "vision": {"corner_plane_height_mm": 0, "robot_marker_height_mm": 0, "corner_refinement": "subpix"},
}


def angle_err(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_homography_recovers_pose():
    truth = [(10, 400, 300, 35, math.radians(30)), (11, 750, 820, 35, math.radians(-120))]
    img = render(truth)
    poses = {p.marker_id: p for p in HomographyEstimator(CFG).estimate(img, t=0.0)}
    for mid, x, y, _, th in truth:
        p = poses[mid]
        assert math.hypot(p.x - x, p.y - y) < 1.5, (mid, p)
        assert angle_err(p.theta, th) < math.radians(1.5), (mid, p)


def test_parallax_correction_pulls_toward_camera():
    below = np.array([500.0, 500.0])
    seen = np.array([[900.0, 500.0]])            # where the floor homography puts it
    real = parallax_correct(seen, below, cam_h=1400, plane_h=0, marker_h=45)
    # A 45 mm tall marker 400 mm off-centre appears ~13 mm farther out than it is.
    assert abs(real[0, 0] - (500 + 400 * (1355 / 1400))) < 1e-6


def test_no_poses_without_field():
    img = np.full((600, 600), 200, np.uint8)
    assert HomographyEstimator(CFG).estimate(img, t=0.0) == []
