"""The RoboField app: one window for the field, the 3D view, RL training, the tools and tuning.

    python tools/robofield_app.py                 # opens in the simulator
    python tools/robofield_app.py --real          # straight to the real field (finds the Pi itself)
    python tools/robofield_app.py --browser       # in your web browser instead of its own window
    python tools/robofield_app.py --headless      # serve only: open http://<this-computer>:8000/app yourself
    python tools/robofield_app.py --shortcut      # put a RoboField icon on the Desktop (Windows)

Switch between the simulator and the real field from the window's top bar at any time. The app
is also reachable from a phone or tablet on the same WiFi: http://<this-computer>:8000/app.
Its own window needs pywebview (pip install -e ".[app]"); without it the app opens in the browser.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
ICON = ROOT / "robofield" / "app" / "static" / "robofield.ico"


def make_shortcut() -> None:
    """A Desktop shortcut that starts the app without a console window."""
    if os.name != "nt":
        raise SystemExit("--shortcut is for Windows; elsewhere run: python tools/robofield_app.py")
    exe = Path(sys.executable)
    pyw = exe.with_name("pythonw.exe")
    target = pyw if pyw.exists() else exe
    script = (
        "$d = [Environment]::GetFolderPath('Desktop');"
        "$s = (New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path $d 'RoboField.lnk'));"
        f"$s.TargetPath = '{target}';"
        f"$s.Arguments = '\"{ROOT / 'tools' / 'robofield_app.py'}\"';"
        f"$s.WorkingDirectory = '{ROOT}';"
        f"$s.IconLocation = '{ICON}';"
        "$s.Description = 'RoboField: field, 3D view, training, tools';"
        "$s.Save(); Write-Output (Join-Path $d 'RoboField.lnk')"
    )
    out = subprocess.run(["powershell", "-NoProfile", "-Command", script], capture_output=True, text=True)
    if out.returncode != 0:
        raise SystemExit(out.stderr.strip() or "could not create the shortcut")
    print(f"shortcut created: {out.stdout.strip()}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real", action="store_true", help="start on the real field instead of the simulator")
    ap.add_argument("--pi", default=None, help="the Pi tracker's address (default: found on the network)")
    ap.add_argument("--sim-robots", type=int, default=2)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--config", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--browser", action="store_true", help="open in the web browser, not in its own window")
    ap.add_argument("--headless", action="store_true", help="only serve the app (open it yourself, e.g. from a phone)")
    ap.add_argument("--shortcut", action="store_true", help="create a Desktop shortcut and exit")
    a = ap.parse_args()
    if a.shortcut:
        return make_shortcut()

    if sys.stdout is None or sys.stderr is None:          # started without a console (pythonw): keep a log
        (ROOT / "logs").mkdir(exist_ok=True)
        log = open(ROOT / "logs" / "app.log", "a", encoding="utf-8", buffering=1)
        sys.stdout = sys.stderr = log
        print(f"\n--- RoboField app {time.strftime('%Y-%m-%d %H:%M:%S')}")

    from robofield.app.runtime import AppRuntime
    from robofield.app.server import serve
    from robofield.core import load_config

    cfg = load_config(a.config)
    import socket
    probe = socket.socket()
    try:
        probe.bind(("0.0.0.0", a.port))                  # is the port free? (another app or Control Center)
        free = True
    except OSError:
        free = False
    finally:
        probe.close()
    if not free:
        msg = (f"Port {a.port} is already in use: the RoboField app or tools/control_center.py is probably "
               f"running already. Close it, or start this one with --port {a.port + 1}.")
        print(msg)
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None, msg, "RoboField", 0x30)
        except Exception:
            pass
        raise SystemExit(1)

    rt = AppRuntime(cfg, "real" if a.real else "sim", a.pi, a.sim_robots, a.port)
    httpd = serve(rt, port=a.port)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{a.port}/app"
    print(f"RoboField app running: {url}  (also http://<this-computer>:{a.port}/app on the same network)")

    window = None
    if not a.browser and not a.headless:
        try:
            import webview
            window = webview.create_window("RoboField", url, width=1500, height=950, min_size=(980, 640),
                                           background_color="#eef0f3")
        except ImportError:
            print("pywebview is not installed (pip install -e \".[app]\"): opening the browser instead")
    try:
        if window is not None:
            webview.start()                                # returns when the window is closed
        else:
            if not a.headless:
                webbrowser.open(url)
            print("Ctrl+C to quit")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        print("closing: stopping the cars and any running jobs")
        rt.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
