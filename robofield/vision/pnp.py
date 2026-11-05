"""Strategy B: full 3D pose per marker with solvePnP, expressed in the field frame.

Needs a camera calibration. Useful for comparison against the homography estimator,
and for anything that leaves the plane (tilted objects, ramps).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from robofield.core.types import Pose
from .base import PoseEstimator


def _square(size: float, z: float = 0.0) -> np.ndarray:
    """ArUco corner order TL, TR, BR, BL in the marker frame (x right, y up)."""
    s = size / 2
    return np.array([[-s, s, z], [s, s, z], [s, -s, z], [-s, -s, z]], dtype=np.float64)


class PnPEstimator(PoseEstimator):
    def __init__(self, cfg, camera_matrix, dist_coeffs=None) -> None:
        super().__init__(cfg, camera_matrix, dist_coeffs)
        m, v = cfg["markers"], cfg["vision"]
        self.corner_size = float(m["corner_size_mm"])
        self.robot_size = float(m["robot_size_mm"])
        self.T_cam_field: np.ndarray | None = None   # 4x4, field -> camera
        self.D0 = np.zeros(5)  # points are already undistorted in detect()

    def update_field(self, detections) -> bool:
        obj, img = [], []
        for i, (x, y, z) in self.corner_world.items():
            if i in detections:
                obj.append(_square(self.corner_size, z) + [x, y, 0])
                img.append(detections[i])
        if len(obj) < 2:
            return self.T_cam_field is not None
        ok, rvec, tvec = cv2.solvePnP(np.vstack(obj), np.vstack(img), self.K, self.D0,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return self.T_cam_field is not None
        T = np.eye(4)
        T[:3, :3], _ = cv2.Rodrigues(rvec)
        T[:3, 3] = tvec.ravel()
        self.T_cam_field = T
        self.field_locked = True
        return True

    def estimate_from_detections(self, detections, t):
        if not self.update_field(detections):
            return []
        T_field_cam = np.linalg.inv(self.T_cam_field)
        poses = []
        for mid, img_pts in detections.items():
            if mid in self.corner_world:
                continue
            ok, rvec, tvec = cv2.solvePnP(_square(self.robot_size), img_pts, self.K, self.D0,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            T = np.eye(4)
            T[:3, :3], _ = cv2.Rodrigues(rvec)
            T[:3, 3] = tvec.ravel()
            T_fm = T_field_cam @ T                 # marker -> field
            x, y = T_fm[0, 3], T_fm[1, 3]
            fwd = T_fm[:3, 1]                      # marker +y = forward
            theta = math.atan2(fwd[1], fwd[0])
            tilt = abs(T_fm[2, 2])                 # 1 = marker flat, facing up
            poses.append(Pose(mid, float(x), float(y), theta, t, float(tilt)))
        return poses
