"""Check that this computer is ready for RoboField, and say what to fix if it is not.

    python tools/doctor.py              # Python, every library, the simulator, the config, the ports
    python tools/doctor.py --network    # also listen 3 s for the Pi tracker and the cars on the WiFi

Nothing here moves a car. The install scripts (install.bat, install.sh) run it at the end.
"""
from __future__ import annotations

import argparse
import importlib
import importlib.metadata as md
import json
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OK, BAD, WARN = "  ok ", " FIX ", " note"
problems = 0


def say(mark: str, text: str, fix: str = "") -> None:
    global problems
    if mark == BAD:
        problems += 1
    print(f"[{mark}] {text}" + (f"\n         -> {fix}" if fix else ""))


# (import name, pip name, option that installs it, what needs it, required for the basics)
LIBRARIES = [
    ("numpy", "numpy", "", "everything", True),
    ("cv2", "opencv-contrib-python", "", "markers, vision, the camera tools", True),
    ("yaml", "PyYAML", "", "reading config/field.yaml", True),
    ("serial", "pyserial", "", "the USB gateway (optional hardware)", True),
    ("mujoco", "mujoco", "sim", "the simulator, the 3D physics, the app in Simulation mode", False),
    ("gymnasium", "gymnasium", "rl", "the RL environments", False),
    ("pettingzoo", "pettingzoo", "rl", "the multi-car RL environment", False),
    ("stable_baselines3", "stable-baselines3", "train", "training and running RL policies", False),
    ("torch", "torch", "train", "training and running RL policies", False),
    ("tensorboard", "tensorboard", "train", "learning curves in TensorBoard (optional)", False),
    ("webview", "pywebview", "app", "the app's own window (else it opens in the browser)", False),
    ("websockets", "websockets", "tools", "tools/record_screen.py", False),
    ("matplotlib", "matplotlib", "tools", "tools/goal_gui.py (older tool)", False),
    ("pytest", "pytest", "dev", "running the tests", False),
]


def check_python():
    v = sys.version_info
    if v < (3, 10):
        say(BAD, f"Python {v.major}.{v.minor} is too old", "install Python 3.10 or newer from https://www.python.org/downloads/")
    else:
        say(OK, f"Python {v.major}.{v.minor}.{v.micro} ({sys.executable})")
    if sys.prefix == sys.base_prefix:
        say(WARN, "not running inside a virtual environment",
            "fine if you meant it; install.bat / install.sh make one in .venv/")


def check_libraries():
    for mod, pip_name, extra, need, core in LIBRARIES:
        try:
            importlib.import_module(mod)
            try:
                ver = md.version(pip_name)
            except md.PackageNotFoundError:
                ver = "?"
            say(OK, f"{pip_name} {ver}")
        except Exception as e:
            how = f'pip install -e ".[{extra}]"' if extra else 'pip install -e .'
            say(BAD if core else WARN, f"{pip_name} missing ({type(e).__name__}): needed for {need}", how)
    try:
        import cv2
        cv2.aruco.ArucoDetector                                      # noqa: B018
        say(OK, "OpenCV has the ArUco module")
    except Exception:
        say(BAD, "OpenCV has no ArUco module", "pip uninstall opencv-python, then pip install opencv-contrib-python")


def check_config():
    try:
        from robofield.core import load_config
        cfg = load_config(ROOT / "config" / "field.yaml")
        say(OK, f"config/field.yaml: field {cfg['field']['width_mm']} mm, {len(cfg['markers']['corners'])} towers, "
                f"loop delay {cfg['control']['latency_s']} s")
        return cfg
    except Exception as e:
        say(BAD, f"config/field.yaml does not load: {e}", "restore it with: git checkout config/field.yaml")
        return None


def check_simulator(cfg):
    if cfg is None:
        return
    try:
        from robofield.sim.backend import SimBackend
        t0 = time.perf_counter()
        b = SimBackend(cfg, 1, seed=0, randomize=False, markers=False)
        b.reset()
        for _ in range(30):
            b.step()
        say(OK, f"simulator runs (start-up and 30 steps in {1000 * (time.perf_counter() - t0):.0f} ms)")
    except ImportError:
        return                                                       # already reported as a missing library
    except Exception as e:
        say(BAD, f"simulator does not run: {type(e).__name__}: {e}", 'pip install -e ".[sim]" again')
        return
    try:
        import mujoco
        r = mujoco.Renderer(b.model, 120, 160)
        r.update_scene(b.data, camera="overhead")
        r.render()
        r.close()
        say(OK, "simulator can draw pictures (Camera tab in Simulation mode)")
    except Exception as e:
        say(WARN, f"simulator cannot draw pictures here ({type(e).__name__}): the simulated Camera tab stays empty",
            "update the graphics driver; everything else works")


def check_custom():
    try:
        from robofield import lab
        errors = lab.register_tasks()
        n = {k: len(v) for k, v in lab.list_all().items()}
        say(OK, f"Lab files: {n['task']} tasks, {n['policy']} policies, {n['system']} systems")
        for name, err in errors.items():
            say(WARN, f"custom task {name} does not load: {err}", "open it in the app's Lab tab and press Check")
    except ImportError:
        pass


def check_ports():
    s = socket.socket()
    try:
        s.bind(("0.0.0.0", 8000))
        say(OK, "port 8000 is free for the app")
    except OSError:
        say(WARN, "port 8000 is in use: the app or tools/control_center.py is probably running",
            "close it before starting another, or use --port 8001")
    finally:
        s.close()


def check_network(seconds=3.0):
    from robofield.comm.pose_link import POSE_PORT
    from robofield.comm.protocol import UDP_STATUS_PORT
    found = {}
    socks = []
    for port in (POSE_PORT, UDP_STATUS_PORT):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", port))
            s.setblocking(False)
            socks.append((port, s))
        except OSError as e:
            say(WARN, f"cannot listen on UDP {port}: {e}", "close the app or the Control Center and try again")
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        for port, s in socks:
            try:
                data, (ip, _) = s.recvfrom(65536)
                found.setdefault(port, set()).add(ip)
                if port == POSE_PORT and "pose" not in found:
                    found["pose"] = json.loads(data)
            except (BlockingIOError, ValueError):
                pass
        time.sleep(0.01)
    for _, s in socks:
        s.close()
    if POSE_PORT in found:
        msg = found.get("pose", {})
        say(OK, f"Pi tracker at {', '.join(found[POSE_PORT])}: {'field locked' if msg.get('locked') else 'field NOT locked'}, "
                f"{msg.get('fps', '?')} fps")
    else:
        say(WARN, "no Pi tracker heard (UDP 5005)", "is the Pi on, on this WiFi, and running pi/pi_tracker.py?")
    if UDP_STATUS_PORT in found:
        say(OK, f"cars heard from: {', '.join(sorted(found[UDP_STATUS_PORT]))}")
    else:
        say(WARN, "no car heard (UDP 4211)", "is a car switched on and on this WiFi? python tools/cars.py list")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--network", action="store_true", help="also look for the Pi tracker and the cars")
    a = ap.parse_args()
    print("RoboField doctor\n")
    check_python()
    check_libraries()
    cfg = check_config()
    check_simulator(cfg)
    check_custom()
    check_ports()
    if a.network:
        check_network()
    print()
    if problems:
        print(f"{problems} thing{'s' if problems > 1 else ''} to fix (marked FIX above).")
        sys.exit(1)
    print("Ready. Start the app with: python tools/robofield_app.py")


if __name__ == "__main__":
    main()
