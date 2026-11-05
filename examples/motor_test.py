"""First driving test: forward, back, spin, gripper, LEDs — at 40% speed.

    python examples/motor_test.py --port /dev/ttyUSB0 --robot 0
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.comm.gateway import Gateway  # noqa: E402
from robofield.comm.protocol import Command  # noqa: E402

STEPS = [  # (label, left, right, servo, led colour, seconds)
    ("forward", 400, 400, 90, (0, 80, 0), 1.0),
    ("back", -400, -400, 90, (80, 0, 0), 1.0),
    ("spin left", -400, 400, 90, (0, 0, 80), 1.0),
    ("spin right", 400, -400, 90, (80, 80, 0), 1.0),
    ("gripper open", 0, 0, 30, (80, 0, 80), 1.0),
    ("gripper close", 0, 0, 150, (0, 80, 80), 1.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--robot", type=int, default=0)
    a = ap.parse_args()
    gw = Gateway(a.port)
    try:
        for label, l, r, servo, rgb, secs in STEPS:
            print(label)
            end = time.monotonic() + secs
            while time.monotonic() < end:          # resend at 50 Hz so the watchdog stays happy
                gw.send(Command(a.robot, left=l, right=r, servo=servo, leds=[rgb] * 3))
                s = gw.poll().get(a.robot)
                if s:
                    print(f"  battery {s.battery_mv} mV  rssi {s.rssi} dBm  watchdog {s.watchdog_tripped}", end="\r")
                time.sleep(0.02)
            print()
    finally:
        gw.close()


if __name__ == "__main__":
    main()
