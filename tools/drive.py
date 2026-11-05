"""Drive a car with the keyboard: arrow keys for direction (only while held), 1/2 to change speed.

    python tools/keyboard_drive.py --robot 0

Keys:
  UP / DOWN     forward / backward (moves only while the key is held down)
  LEFT / RIGHT  spin left / spin right (while held)
  1             speed up
  2             speed down
  SPACE         stop
  Q             quit
"""
import argparse
import curses
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.comm.protocol import Command  # noqa: E402
from robofield.comm.udp_link import CarLink  # noqa: E402

SPEED_STEP = 100
SPEED_MIN = 100
SPEED_MAX = 1000
SPEED_START = 400
RELEASE_TIMEOUT = 0.18   # no repeat of the key within this long -> treat it as released


def wait_for(link, rid, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if rid in link.online():
            return True
        time.sleep(0.1)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", type=int, default=0)
    a = ap.parse_args()

    link = CarLink()
    if not wait_for(link, a.robot):
        link.close()
        sys.exit(f"robot {a.robot} not found - is it powered on and connected?")

    def ui(scr):
        scr.nodelay(True)
        scr.keypad(True)
        speed = SPEED_START
        l = r = 0
        last_press = 0.0
        last_key = "-"
        drive_keys = {
            curses.KEY_UP: ("forward", lambda s: (s, s)),
            curses.KEY_DOWN: ("backward", lambda s: (-s, -s)),
            curses.KEY_LEFT: ("spin left", lambda s: (s, -s)),
            curses.KEY_RIGHT: ("spin right", lambda s: (-s, s)),
        }
        while True:
            k = scr.getch()
            if k == ord("q"):
                break
            elif k in drive_keys:
                last_key, fn = drive_keys[k]
                l, r = fn(speed)
                last_press = time.monotonic()
            elif k == ord(" "):
                l, r = 0, 0; last_key = "stop"
            elif k in (ord("1"), ord("2")):
                speed = min(SPEED_MAX, speed + SPEED_STEP) if k == ord("1") else max(SPEED_MIN, speed - SPEED_STEP)
                if l or r:  # still moving: rescale to the new speed, same direction
                    l = speed if l > 0 else (-speed if l < 0 else 0)
                    r = speed if r > 0 else (-speed if r < 0 else 0)
                last_key = f"speed -> {speed / 10:.0f}%"

            if (l or r) and time.monotonic() - last_press > RELEASE_TIMEOUT:
                l, r = 0, 0; last_key = "released"

            link.poll()
            link.send(Command(a.robot, left=l, right=r))
            car = link.cars.get(a.robot)
            batt = f"{car.status.battery_mv / 1000:.2f} V" if car else "?"
            scr.erase()
            scr.addstr(0, 0, f"robot {a.robot}   hold arrows to drive   1/2=speed   space=stop   q=quit")
            scr.addstr(1, 0, f"speed: {speed / 10:3.0f}%   last: {last_key}")
            scr.addstr(2, 0, f"left {l:5d}   right {r:5d}   battery {batt}")
            scr.refresh()
            time.sleep(1 / 30)
        link.send(Command(a.robot, enable=False))

    try:
        curses.wrapper(ui)
    finally:
        link.close()


if __name__ == "__main__":
    main()