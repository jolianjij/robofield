"""Common interface for pose estimators."""
from __future__ import annotations

import time
from abc import ABC, abstractmethod

import cv2
import numpy as np

from robofield.core.types import Pose


def make_detector(dictionary: str = "DICT_4X4_50", refinement: str = "subpix") -> cv2.aruco.ArucoDetector:
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = {
        "none": cv2.aruco.CORNER_REFINE_NONE,
        "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
        "contour": cv2.aruco.CORNER_REFINE_CONTOUR,
        "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
    }[refinement]
    return cv2.aruco.ArucoDetector(aruco_dict, params)


class PoseEstimator(ABC):
    """Turns a camera frame into field-frame poses for every non-corner marker."""

    def __init__(self, cfg: dict, camera_matrix=None, dist_coeffs=None) -> None:
        self.cfg = cfg
        self.K = camera_matrix
        self.D = dist_coeffs
        m = cfg["markers"]
        self.corner_world = {int(k): tuple(list(map(float, v)) + [0.0] * (3 - len(v)))
                             for k, v in m["corners"].items()}   # {id: (x, y, z) mm}
        self.detector = make_detector(m["dictionary"], cfg["vision"].get("corner_refinement", "subpix"))
        self.field_locked = False                  # True once the field frame is known

    def detect(self, frame: np.ndarray) -> dict[int, np.ndarray]:
        """Returns {marker_id: 4x2 image corners (TL, TR, BR, BL)}, undistorted if calibrated."""
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        out: dict[int, np.ndarray] = {}
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2).astype(np.float64)
            if self.K is not None and self.D is not None:
                pts = cv2.undistortPoints(pts.reshape(-1, 1, 2), self.K, self.D, P=self.K).reshape(4, 2)
            out[int(i)] = pts
        return out

    def estimate(self, frame: np.ndarray, t: float | None = None) -> list[Pose]:
        t = time.monotonic() if t is None else t
        return self.estimate_from_detections(self.detect(frame), t)

    @abstractmethod
    def estimate_from_detections(self, detections: dict[int, np.ndarray], t: float) -> list[Pose]:
        ...
