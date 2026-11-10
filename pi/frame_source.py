"""Frame sources for the tracker: the Raspberry Pi camera (picamera2), a USB camera, a network camera
(a phone running an IP-camera app, or any MJPEG/RTSP stream), or image files.

    picamera                       the Pi Camera Module (default on the Pi)
    opencv:0                       USB camera number 0
    url:http://192.168.1.23:8080/video   a phone IP-camera app, or any stream OpenCV can open
    file:/path/*.png               saved images, for testing without a camera
"""
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
    """A USB camera (an index) or a network stream (a URL). A stream sets its own size and rate in
    its app; a USB camera is asked for MJPG at the given size with autofocus off."""

    def __init__(self, index=0, width=1920, height=1080, fps=30):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open the camera {index!r}")
        if isinstance(index, int):
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        else:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # newest frame, not a backlog

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


def parse_source(spec: str):
    """('picamera' | 'opencv' | 'file', argument) for a source spec (see the module docstring)."""
    if spec == "picamera":
        return "picamera", None
    if spec.startswith("url:"):
        return "opencv", spec[4:]
    if spec.startswith(("http://", "https://", "rtsp://")):
        return "opencv", spec
    if spec.startswith("opencv"):
        arg = spec.split(":", 1)[1] if ":" in spec else "0"
        return "opencv", int(arg) if arg.isdigit() else arg
    if spec.startswith("file:"):
        return "file", spec[5:]
    raise ValueError(f"unknown camera source {spec!r}: picamera | opencv:0 | url:http://... | file:/path/*.png")


def open_source(spec: str, **kw):
    """'picamera' | 'opencv:0' | 'url:http://phone:8080/video' | 'file:/path/*.png'"""
    kind, arg = parse_source(spec)
    if kind == "picamera":
        return PiCameraSource(**kw)
    if kind == "opencv":
        return OpenCVSource(arg)
    return FileSource(arg)
