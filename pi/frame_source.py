"""Frame sources for the tracker: Raspberry Pi camera (picamera2), any OpenCV camera, or image files."""
from __future__ import annotations

import glob
import time

import cv2
import numpy as np


class PiCameraSource:
    """Camera Module 3 Wide on a Pi 5. Full field of view (2x2 binned) at 2304 x 1296, grayscale,
    with focus, exposure, gain and white balance locked so the image geometry and brightness stay fixed."""

    def __init__(self, width=2304, height=1296, fps=56, exposure_us=4000, gain=4.0, lens_position=0.75):
        from picamera2 import Picamera2
        from libcamera import controls
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(main={"size": (width, height), "format": "YUV420"},
                                                  controls={"FrameRate": fps})
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.5)
        self.cam.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position,      # dioptres: 1 / distance in m (0.75 = focus at ~1.3 m)
            "AeEnable": False, "ExposureTime": exposure_us, "AnalogueGain": gain,
            "AwbEnable": False,
        })
        self.w, self.h = width, height

    def read(self):
        yuv = self.cam.capture_array("main")
        return yuv[: self.h, : self.w], time.time()              # Y plane = grayscale

    def close(self):
        self.cam.stop()


class OpenCVSource:
    def __init__(self, index=0, width=1920, height=1080, fps=30):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), time.time()

    def close(self):
        self.cap.release()


class FileSource:
    """Loops over image files: for testing the tracker without a camera."""

    def __init__(self, pattern, fps=10):
        self.files = sorted(glob.glob(pattern)) or [pattern]
        self.i, self.dt = 0, 1.0 / fps

    def read(self):
        time.sleep(self.dt)
        img = cv2.imread(self.files[self.i % len(self.files)], cv2.IMREAD_GRAYSCALE)
        self.i += 1
        return img, time.time()

    def close(self):
        pass


def open_source(spec: str, **kw):
    """'picamera' | 'opencv:0' | 'file:/path/*.png'"""
    if spec == "picamera":
        return PiCameraSource(**kw)
    if spec.startswith("opencv"):
        return OpenCVSource(int(spec.split(":")[1]) if ":" in spec else 0)
    if spec.startswith("file:"):
        return FileSource(spec[5:])
    raise ValueError(spec)
