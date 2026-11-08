"""The app's camera feed: on the real field, the Pi's live view (with its tracking overlay), relayed
through the laptop; in the simulator, the overhead camera rendered by MuJoCo. One source, any number
of viewers (GET /camera.mjpg), snapshots and recording to MP4. Frames are only produced while
someone is watching or recording."""
from __future__ import annotations

import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RECORDINGS = ROOT / "recordings"


class CameraHub:
    def __init__(self, runtime, sim_size=(960, 540), sim_fps=12.0):
        self.rt = runtime
        self.sim_size, self.sim_fps = sim_size, sim_fps
        self.jpeg: bytes | None = None
        self.seq = 0
        self.fps = 0.0
        self.source = ""
        self.error = ""
        self.cond = threading.Condition()
        self.last_view = 0.0
        self._rec = None                 # (writer thread, stop event, path, started)
        self._closed = False
        threading.Thread(target=self._run, daemon=True, name="camera").start()

    # ------------------------------------------------------------------ producing frames
    def wanted(self) -> bool:
        return time.monotonic() - self.last_view < 3.0 or self._rec is not None

    def _publish(self, jpeg: bytes):
        now = time.monotonic()
        with self.cond:
            if self.jpeg is not None and getattr(self, "_t_last", None):
                self.fps = 0.85 * self.fps + 0.15 / max(1e-3, now - self._t_last)
            self._t_last = now
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def _run(self):
        renderer, model = None, None
        stream, host, buf = None, None, b""
        while not self._closed:
            if not self.wanted():
                if stream is not None:
                    stream.close(); stream = None
                time.sleep(0.2)
                continue
            cc = self.rt.cc
            try:
                if cc is not None and cc.mode != "real" and hasattr(cc.backend, "model"):
                    if stream is not None:
                        stream.close(); stream = None
                    if model is not cc.backend.model:              # new simulator (mode switch): new renderer
                        import mujoco
                        if renderer is not None:
                            renderer.close()
                        model = cc.backend.model
                        renderer = mujoco.Renderer(model, self.sim_size[1], self.sim_size[0])
                    t0 = time.monotonic()
                    with cc._blk:                                   # never read the physics mid-step
                        renderer.update_scene(cc.backend.data, camera="overhead")
                    img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                    cv2.putText(img, "SIMULATED CAMERA", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 40, 40), 2, cv2.LINE_AA)
                    self._publish(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
                    self.source, self.error = "simulator", ""
                    time.sleep(max(0.0, 1 / self.sim_fps - (time.monotonic() - t0)))
                else:
                    want = self.rt.pi
                    if not want:
                        self.source, self.error = "real", "No Pi tracker found: the camera feed comes from the Pi"
                        time.sleep(1.0)
                        continue
                    if stream is None or host != want:
                        if stream is not None:
                            stream.close()
                        stream = urllib.request.urlopen(f"http://{want}:8080/", timeout=5)
                        host, buf = want, b""
                    buf += stream.read(32768)
                    while True:                                     # every complete JPEG in the buffer
                        start = buf.find(b"\xff\xd8")
                        end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                        if start < 0 or end < 0:
                            buf = buf[start:] if start >= 0 else b""
                            break
                        self._publish(buf[start:end + 2])
                        buf = buf[end + 2:]
                    self.source, self.error = "real", ""
            except Exception as e:                                  # Pi unreachable, stream stalled...
                self.error = f"camera: {e}"
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass
                stream = None
                time.sleep(1.0)
        if renderer is not None:
            renderer.close()

    # ------------------------------------------------------------------ using frames
    def wait_frame(self, last_seq: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        self.last_view = time.monotonic()
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.jpeg

    def snapshot(self) -> tuple[bool, str]:
        _, jpeg = self.wait_frame(self.seq, 3.0)          # a fresh frame (the last one may be old)
        if jpeg is None:
            return False, self.error or "no camera frame yet"
        RECORDINGS.mkdir(exist_ok=True)
        path = RECORDINGS / f"snapshot_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        path.write_bytes(jpeg)
        return True, f"recordings/{path.name}"

    def start_recording(self, fps: float = 10.0) -> tuple[bool, str]:
        if self._rec is not None:
            return False, "already recording"
        RECORDINGS.mkdir(exist_ok=True)
        path = RECORDINGS / f"camera_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        stop = threading.Event()

        def write():
            writer, last, t_next = None, None, time.monotonic()
            while not stop.is_set():
                self.last_view = time.monotonic()              # keep the frames coming while recording
                jpeg = self.jpeg
                if jpeg is not None and jpeg is not last:
                    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    last = jpeg
                    if writer is None and frame is not None:
                        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                                 (frame.shape[1], frame.shape[0]))
                    self._frame = frame
                if writer is not None and getattr(self, "_frame", None) is not None:
                    while time.monotonic() >= t_next:             # real-time video: repeat a frame if none came
                        writer.write(self._frame)
                        t_next += 1 / fps
                time.sleep(0.01)
            if writer is not None:
                writer.release()

        th = threading.Thread(target=write, daemon=True, name="camera-recorder")
        self._rec = (th, stop, path, time.monotonic())
        self.last_view = time.monotonic()
        th.start()
        return True, f"recordings/{path.name}"

    def stop_recording(self) -> tuple[bool, str]:
        if self._rec is None:
            return False, "not recording"
        th, stop, path, _ = self._rec
        stop.set()
        th.join(5)
        self._rec = None
        return True, f"recordings/{path.name}"

    def status(self) -> dict:
        rec = None
        if self._rec is not None:
            rec = {"file": f"recordings/{self._rec[2].name}", "seconds": round(time.monotonic() - self._rec[3], 1)}
        return {"source": self.source, "fps": round(self.fps, 1), "error": self.error, "recording": rec,
                "live": time.monotonic() - getattr(self, "_t_last", 0) < 2.0}

    def close(self):
        if self._rec is not None:
            self.stop_recording()
        self._closed = True
