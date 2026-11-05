"""RoboField Control Center: live map, car control, LED colours, drawn paths, safety.

    python tools/control_center.py                      # simulator with 2 cars, no hardware
    python tools/control_center.py --sim-robots 4
    python tools/control_center.py --real --pi robofield-pi.local

Then open http://localhost:8000 in a browser (or http://<this-computer>:8000 from a phone).
"""
import argparse
import sys
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.app import ControlCenter  # noqa: E402
from robofield.app.server import serve  # noqa: E402
from robofield.core import load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true", help="real field (Pi tracker + cars over WiFi)")
    ap.add_argument("--sim-robots", type=int, default=2)
    ap.add_argument("--pi", default=None, help="Pi hostname / IP: camera view + path overlay")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--config", default="config/field.yaml")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.real:
        from robofield.real import RealBackend
        backend, mode = RealBackend(cfg), "real"
    else:
        from robofield.sim.backend import SimBackend
        backend, mode = SimBackend(cfg, n_robots=a.sim_robots, seed=0, randomize=True, markers=False), "sim"
    cc = ControlCenter(cfg, backend, mode, pi_host=a.pi).start()
    httpd = serve(cc, port=a.port)
    url = f"http://localhost:{a.port}"
    print(f"Control Center ({mode}) running: open {url}   (Ctrl+C to quit)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cc.close()


if __name__ == "__main__":
    main()
