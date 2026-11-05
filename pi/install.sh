#!/bin/bash
# RoboField tracker setup on a Raspberry Pi 5 (Raspberry Pi OS 64-bit, Bookworm or newer).
set -e
sudo apt update
sudo apt install -y python3-picamera2 python3-venv python3-yaml git
cd "$(dirname "$0")/.."
python3 -m venv --system-site-packages .venv          # keeps the system picamera2 + numpy
.venv/bin/pip install --upgrade pip
# OpenCV with the ArUco module; --no-deps keeps the system numpy that picamera2 is built against
.venv/bin/pip install --no-deps "opencv-contrib-python-headless==4.10.0.84"
.venv/bin/python -c "import cv2, picamera2; print('OpenCV', cv2.__version__, '- picamera2 OK')"
echo "Next: test the camera with   rpicam-hello -t 5000"
echo "      calibrate with          .venv/bin/python pi/pi_calibrate.py"
echo "      run the tracker with    .venv/bin/python pi/pi_tracker.py --preview 8080"
