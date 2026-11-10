"""Strategy C (recommended): camera pose from the towers, robots by ray-plane intersection.

1. All visible tower-marker corners (known 3D positions at different heights) -> solvePnP
   gives the camera pose in the field frame. Non-coplanar points make this very stable.
2. Each robot marker corner pixel is turned into a 3D ray and intersected with the plane
   z = robot_marker_height_mm (objects: z = object_marker_height_mm, for ids >= object_id_min). No homography, no parallax error, and no per-marker PnP,
   so no IPPE orientation flips on small markers.

Needs a camera calibration (essential anyway with a wide-angle lens).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from robofield.core.types import Pose
from .base import PoseEstimator


def tower_corners(x: float, y: float, z: float, size: float) -> np.ndarray:
    """Marker corners TL, TR, BR, BL for a marker lying flat, top edge toward +Y."""
    s = size / 2
    return np.array([[x - s, y + s, z], [x + s, y + s, z], [x + s, y - s, z], [x - s, y - s, z]])


class PlaneEstimator(PoseEstimator):
    def __init__(self, cfg, camera_matrix, dist_coeffs=None) -> None:
        super().__init__(cfg, camera_matrix, dist_coeffs)
        self.corner_size = float(cfg["markers"]["corner_size_mm"])
        self.plane_z = float(cfg["vision"].get("robot_marker_height_mm", 0))
        self.object_z = float(cfg["vision"].get("object_marker_height_mm", self.plane_z))
        self.object_min = int(cfg["markers"].get("object_id_min", 30))
        self.min_towers = int(cfg["vision"].get("min_towers", 2))
        self.R: np.ndarray | None = None   # field -> camera rotation
        self.t: np.ndarray | None = None
        self.reproj_error_px = float("nan")
        self._Kinv = np.linalg.inv(self.K)
        self._D0 = np.zeros(5)             # detect() already undistorts

    @property
    def camera_position(self) -> np.ndarray | None:
        return None if self.R is None else (-self.R.T @ self.t).ravel()

    def update_field(self, detections) -> bool:
        obj, img = [], []
        for i, (x, y, z) in self.corner_world.items():
            if i in detections:
                obj.append(tower_corners(x, y, z, self.corner_size))
                img.append(detections[i])
        if len(obj) < self.min_towers:
            return self.R is not None
        obj, img = np.vstack(obj), np.vstack(img)
        rvec0 = tvec0 = None
        if self.R is not None:   # warm start from the last pose
            rvec0, _ = cv2.Rodrigues(self.R)
            tvec0 = self.t.copy()
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self._D0, rvec0, tvec0,
                                      useExtrinsicGuess=rvec0 is not None,
                                      flags=cv2.SOLVEPNP_ITERATIVE if rvec0 is not None else cv2.SOLVEPNP_SQPNP)
        if not ok:
            return self.R is not None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self._D0, rvec, tvec)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self._D0)
        self.reproj_error_px = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        self.R, _ = cv2.Rodrigues(rvec)
        self.t = tvec.reshape(3, 1)
        self.field_locked = True
        return True

    def pixels_to_plane(self, pts: np.ndarray, z: float) -> np.ndarray:
        """Intersect camera rays through undistorted pixels with the plane at height z."""
        uv1 = np.hstack([pts, np.ones((len(pts), 1))])
        rays = (self.R.T @ (self._Kinv @ uv1.T)).T          # ray directions in field frame
        c = self.camera_position
        s = (z - c[2]) / rays[:, 2]
        return c[None, :2] + rays[:, :2] * s[:, None]

    def estimate_from_detections(self, detections, t):
        if not self.update_field(detections):
            return []
        poses = []
        for mid, img_pts in detections.items():
            if mid in self.corner_world:
                continue
            z = self.object_z if mid >= self.object_min else self.plane_z      # objects sit lower than cars
            q = self.pixels_to_plane(img_pts, z)             # TL, TR, BR, BL in field mm
            centre = q.mean(axis=0)
            fwd = (q[0] + q[1]) / 2 - (q[3] + q[2]) / 2
            sides = np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)
            quality = float(sides.min() / sides.max()) if sides.max() > 0 else 0.0
            poses.append(Pose(mid, float(centre[0]), float(centre[1]), math.atan2(fwd[1], fwd[0]), t, quality))
        return poses
