"""Record the Control Center web page to an MP4, the way it looks in the browser.

    python tools/record_screen.py --seconds 60
    python tools/record_screen.py --url "http://127.0.0.1:8000/?caption=Before" --stop-file x

It opens the page in its own hidden (headless) Chrome and records what that page draws, so the video
is clean - no mouse pointer, no other windows - and it does not matter what is on your screen while
it records. Saved to recordings/ (git-ignored). Stops after --seconds, on Ctrl+C, or as soon as the
--stop-file exists (for scripts that record while the car drives).
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import websockets

CHROME = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",
          r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
          r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
          "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
          "google-chrome", "chromium", "chromium-browser"]


def find_chrome() -> str:
    for c in CHROME:
        if os.path.exists(c) or shutil.which(c):
            return c
    raise SystemExit("Chrome / Edge not found")


async def record(a, out: Path) -> int:
    profile = tempfile.mkdtemp(prefix="robofield_rec_")
    chrome = subprocess.Popen(
        [find_chrome(), "--headless=new", f"--remote-debugging-port={a.cdp_port}", f"--user-data-dir={profile}",
         f"--window-size={a.width},{a.height}", "--hide-scrollbars", "--mute-audio", "--no-first-run",
         "--no-default-browser-check", "--force-device-scale-factor=1", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    n = 0
    writer = None
    try:
        for _ in range(100):                                         # wait for Chrome's debugging port
            try:
                pages = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{a.cdp_port}/json/list", timeout=1).read())
                page = next(p for p in pages if p["type"] == "page")
                break
            except (OSError, StopIteration, ValueError):
                await asyncio.sleep(0.1)
        else:
            raise SystemExit("could not talk to headless Chrome")

        async with websockets.connect(page["webSocketDebuggerUrl"], max_size=None) as ws:
            ids = iter(range(1, 1 << 30))

            async def send(method, **params):
                await ws.send(json.dumps({"id": next(ids), "method": method, "params": params}))

            await send("Emulation.setDeviceMetricsOverride", width=a.width, height=a.height, deviceScaleFactor=1, mobile=False)
            await send("Emulation.setEmulatedMedia", features=[{"name": "prefers-color-scheme", "value": a.theme}])
            await send("Page.enable")
            await send("Page.navigate", url=a.url)
            t_load = time.monotonic()
            while time.monotonic() - t_load < 2.0:                    # let the page load and draw
                try:
                    await asyncio.wait_for(ws.recv(), 0.2)
                except asyncio.TimeoutError:
                    pass
            await send("Page.startScreencast", format="jpeg", quality=88, maxWidth=a.width, maxHeight=a.height)

            writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (a.width, a.height))
            print(f"recording {a.url} at {a.width}x{a.height} to {out}", flush=True)
            img, t0 = None, time.monotonic()
            while not a.seconds or time.monotonic() - t0 < a.seconds:
                if a.stop_file and os.path.exists(a.stop_file):
                    break
                try:
                    msg = json.loads(await asyncio.wait_for(ws.recv(), 1 / a.fps))
                except asyncio.TimeoutError:
                    msg = {}
                if msg.get("method") == "Page.screencastFrame":
                    p = msg["params"]
                    await send("Page.screencastFrameAck", sessionId=p["sessionId"])
                    f = cv2.imdecode(np.frombuffer(base64.b64decode(p["data"]), np.uint8), cv2.IMREAD_COLOR)
                    if f is not None:
                        img = f if f.shape[1::-1] == (a.width, a.height) else cv2.resize(f, (a.width, a.height))
                if img is None:
                    continue
                due = int((time.monotonic() - t0) * a.fps) + 1          # real-time playback: repeat frames as needed
                while n < due:
                    writer.write(img)
                    n += 1
    finally:
        if writer is not None:
            writer.release()
        chrome.terminate()
        try:
            chrome.wait(5)
        except subprocess.TimeoutExpired:
            chrome.kill()
        shutil.rmtree(profile, ignore_errors=True)
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8000/")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C / --stop-file)")
    ap.add_argument("--stop-file", default=None, help="stop as soon as this file exists (for scripts)")
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--theme", choices=["light", "dark"], default="light")
    ap.add_argument("--cdp-port", type=int, default=9333)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = Path(a.out or f"recordings/screen_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    n = 0
    try:
        n = asyncio.run(record(a, out))
    except KeyboardInterrupt:
        pass
    print(f"saved {out} ({n} frames, {n / a.fps:.1f} s)", flush=True)


if __name__ == "__main__":
    main()
