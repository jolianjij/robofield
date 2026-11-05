"""Strategy A: planar homography from the 4 corner markers, with parallax correction.

Works without camera calibration (calibration still improves accuracy at the edges).
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from robofield.core.types import Pose
from .base import PoseEstimator


def parallax_correct(q: np.ndarray, below_cam: np.ndarray, cam_h: float,
                     plane_h: float, marker_h: float) -> np.ndarray:
    """Move a point found on the reference plane (height plane_h) to where a marker at
    height marker_h really is. All heights in mm above the field surface.

    The camera ray through the marker hits the reference plane farther from the point
    under the camera when the marker is taller, so we scale back toward that point.
    """
    scale = (cam_h - marker_h) / (cam_h - plane_h)
    return below_cam + (q - below_cam) * scale


class HomographyEstimator(PoseEstimator):
    def __init__(self, cfg, camera_matrix=None, dist_coeffs=None) -> None:
        super().__init__(cfg, camera_matrix, dist_coeffs)
        v = cfg["vision"]
        self.cam_h = float(cfg["camera"]["height_mm"])
        zs = {c[2] for c in self.corner_world.values()}
        if len(zs) > 1:
            raise ValueError("homography needs all corner markers at one height; use the 'plane' estimator")
        self.plane_h = zs.pop()
        self.marker_h = float(v.get("robot_marker_height_mm", 0))
        self.H: np.ndarray | None = None          # image -> reference plane (mm)
        self.below_cam: np.ndarray | None = None  # field point under the lens
        self.image_size: tuple[int, int] | None = None

    def update_field(self, detections: dict[int, np.ndarray]) -> bool:
        """Recompute the homography when all 4 corners are visible; else keep the last one."""
        ids = [i for i in self.corner_world if i in detections]
        if len(ids) < 4:
            return self.H is not None
        img = np.array([detections[i].mean(axis=0) for i in ids])
        world = np.array([self.corner_world[i][:2] for i in ids], dtype=np.float64)
        H, _ = cv2.findHomography(img, world)
        if H is None:
            return self.H is not None
        self.H = H
        if self.image_size is not None:
            if self.K is not None:
                c = np.array([self.K[0, 2], self.K[1, 2]])
            else:
                c = np.array([self.image_size[0] / 2, self.image_size[1] / 2])
            self.below_cam = self._to_plane(c.reshape(1, 2))[0]
        self.field_locked = True
        return True

    def _to_plane(self, pts: np.ndarray) -> np.ndarray:
        return cv2.perspectiveTransform(pts.reshape(-1, 1, 2), self.H).reshape(-1, 2)

    def estimate(self, frame, t=None):
        self.image_size = (frame.shape[1], frame.shape[0])
        return super().estimate(frame, t)

    def estimate_from_detections(self, detections, t):
        if not self.update_field(detections):
            return []
        poses = []
        for mid, img_pts in detections.items():
            if mid in self.corner_world:
                continue
            q = self._to_plane(img_pts)            # TL, TR, BR, BL on the reference plane
            if self.below_cam is not None and self.marker_h != self.plane_h:
                q = parallax_correct(q, self.below_cam, self.cam_h, self.plane_h, self.marker_h)
            centre = q.mean(axis=0)
            fwd = (q[0] + q[1]) / 2 - (q[3] + q[2]) / 2   # bottom edge -> top edge
            theta = math.atan2(fwd[1], fwd[0])
            # Quality: how square the projected marker is (1 = perfect square).
            sides = np.linalg.norm(q - np.roll(q, -1, axis=0), axis=1)
            quality = float(sides.min() / sides.max()) if sides.max() > 0 else 0.0
            poses.append(Pose(mid, float(centre[0]), float(centre[1]), theta, t, quality))
        return poses
