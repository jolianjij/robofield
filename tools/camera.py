"""Open the field camera with fixed settings (MJPG, locked focus and exposure)."""
import cv2


def open_camera(cam_cfg: dict) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(cam_cfg.get("index", 0))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*cam_cfg.get("fourcc", "MJPG")))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.get("width", 1920))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.get("height", 1080))
    cap.set(cv2.CAP_PROP_FPS, cam_cfg.get("fps", 60))
    if not cam_cfg.get("autofocus", False):
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, cam_cfg.get("focus", 0))
    if not cap.isOpened():
        raise RuntimeError("could not open camera")
    w, h = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"camera: {int(w)}x{int(h)} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps requested")
    return cap
