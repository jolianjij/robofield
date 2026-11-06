"""Record the Pi tracker's live preview (camera + tracking overlay) to an MP4 on the laptop.

    python tools/record_camera.py --pi 10.127.58.149                 # until Ctrl+C
    python tools/record_camera.py --pi 10.127.58.149 --seconds 90    # fixed length
    python tools/record_camera.py --pi 10.127.58.149 --stop-file x   # until the file x appears

Saved to recordings/ (git-ignored). The preview runs at ~10 fps; frames are written at their real
timing, so the video plays back in real time.
"""
from __future__ import annotations

import argparse
import os
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", required=True, help="Pi address")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--stop-file", default=None, help="stop as soon as this file exists (for scripts)")
    a = ap.parse_args()
    out = Path(a.out or f"recordings/camera_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    stream = None
    for _ in range(4):                                       # ride out a short WiFi hiccup
        try:
            stream = urllib.request.urlopen(f"http://{a.pi}:{a.port}/", timeout=5)
            break
        except OSError as e:
            err = e
            time.sleep(1)
    if stream is None:
        raise SystemExit(f"cannot reach the Pi camera stream at {a.pi}:{a.port}: {err}")
    buf, writer, frame, n = b"", None, None, 0
    t0 = time.monotonic()
    try:
        while not a.seconds or time.monotonic() - t0 < a.seconds:
            if a.stop_file and os.path.exists(a.stop_file):
                break
            try:
                buf += stream.read(16384)
            except (TimeoutError, OSError):                  # the Pi's stream stalled: reconnect, keep the file going
                try:
                    stream = urllib.request.urlopen(f"http://{a.pi}:{a.port}/", timeout=5)
                except OSError:
                    time.sleep(0.5)
                buf = b""
                continue
            start = buf.find(b"\xff\xd8")
            end = buf.find(b"\xff\xd9", start + 2)
            if start < 0 or end < 0:
                continue
            img = cv2.imdecode(np.frombuffer(buf[start:end + 2], np.uint8), cv2.IMREAD_COLOR)
            buf = buf[end + 2:]
            if img is None:
                continue
            if writer is None:
                h, w = img.shape[:2]
                writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (w, h))
                print(f"recording {w}x{h} to {out}", flush=True)
            # keep real-time playback: write as many frames as the elapsed time calls for
            due = int((time.monotonic() - t0) * a.fps) + 1
            while n < due:
                writer.write(img)
                n += 1
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        print(f"saved {out} ({n} frames, {n / a.fps:.1f} s)", flush=True)


if __name__ == "__main__":
    main()
