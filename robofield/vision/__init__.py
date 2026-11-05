"""Marker detection and pose estimation. Estimators are swappable (see docs/vision.md)."""
from .base import PoseEstimator, make_detector
from .homography import HomographyEstimator
from .pnp import PnPEstimator
from .plane import PlaneEstimator

__all__ = ["PoseEstimator", "make_detector", "HomographyEstimator", "PnPEstimator", "PlaneEstimator", "build_estimator"]


def build_estimator(cfg: dict, camera_matrix=None, dist_coeffs=None) -> PoseEstimator:
    name = cfg["vision"]["estimator"]
    if name == "homography":
        return HomographyEstimator(cfg, camera_matrix, dist_coeffs)
    if name == "plane":
        if camera_matrix is None:
            raise ValueError("the plane estimator needs a camera calibration")
        return PlaneEstimator(cfg, camera_matrix, dist_coeffs)
    if name == "pnp":
        if camera_matrix is None:
            raise ValueError("the pnp estimator needs a camera calibration")
        return PnPEstimator(cfg, camera_matrix, dist_coeffs)
    raise ValueError(f"unknown estimator: {name}")
