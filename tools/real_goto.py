"""First closed-loop test on the real field: drive a car to points using the camera tracking.

    python tools/real_goto.py --robot 0                      # visits 4 points around the field
    python tools/real_goto.py --robot 0 --points 300,300 700,700
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.control import DiffDrive, GoToPoint, deadzones_from_config  # noqa: E402
from robofield.core import RobotCommand, load_config  # noqa: E402
from robofield.real import RealBackend  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--config", default="config/field.yaml")
    ap.add_argument("--points", nargs="*", default=["250,250", "750,250", "750,750", "250,750"])
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()
    cfg = load_config(a.config)
    backend = RealBackend(cfg, robot_ids=[a.robot])
    ctl = GoToPoint(DiffDrive.from_config(cfg), deadzone=deadzones_from_config(cfg), v_max=200)
    try:
        backend.reset()
        for pt in a.points:
            gx, gy = map(float, pt.split(","))
            print(f"-> ({gx:.0f}, {gy:.0f})")
            t0 = time.monotonic()
            while time.monotonic() - t0 < a.timeout:
                ws = backend.step()
                r = ws.robots.get(a.robot)
                if r is None or not r.visible:              # not seen by the camera: stay still
                    backend.apply({a.robot: RobotCommand()})
                    continue
                cmd, done = ctl.command(r.x, r.y, r.theta, gx, gy, r.omega, 1.0 / backend.control_hz)
                backend.apply({a.robot: cmd})
                if done:
                    print(f"   reached in {time.monotonic() - t0:.1f} s at ({r.x:.0f}, {r.y:.0f})")
                    break
            else:
                print("   timed out")
            backend.apply({a.robot: RobotCommand()})
            backend.step()
            time.sleep(0.5)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
