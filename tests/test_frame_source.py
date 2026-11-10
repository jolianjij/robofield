"""Camera source specs for the tracker (pi/frame_source.py): the Pi camera, USB, a phone URL, files."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pi"))
from frame_source import parse_source  # noqa: E402


@pytest.mark.parametrize("spec,expected", [
    ("picamera", ("picamera", None)),
    ("opencv", ("opencv", 0)),
    ("opencv:2", ("opencv", 2)),
    ("url:http://192.168.1.23:8080/video", ("opencv", "http://192.168.1.23:8080/video")),
    ("http://10.0.0.5:8080/video", ("opencv", "http://10.0.0.5:8080/video")),
    ("rtsp://phone/stream", ("opencv", "rtsp://phone/stream")),
    ("file:frames/*.png", ("file", "frames/*.png")),
])
def test_parse_source(spec, expected):
    assert parse_source(spec) == expected


def test_unknown_source():
    with pytest.raises(ValueError):
        parse_source("webcam")
