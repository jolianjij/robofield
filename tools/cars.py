"""Find and manage the cars on the WiFi network.

    python tools/cars.py list                     # every car broadcasting status
    python tools/cars.py set-id 192.168.1.51 2    # give a car its robot id (stored in its flash)
    python tools/cars.py test 0                   # forward / back / spin / gripper / LEDs, 40 % speed
    python tools/cars.py drive 0                  # keyboard: w a s d, space = stop, q = quit
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.comm.protocol import Command  # noqa: E402
from robofield.comm.udp_link import CarLink  # noqa: E402


def wait_for(link, rid, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if rid in link.online():
            return True
        time.sleep(0.1)
    return False


def cmd_list(link, seconds=2.0):
    print(f"listening for {seconds:.0f} s ...")
    time.sleep(seconds)
    cars = link.online(seconds)
    if not cars:
        print("no cars found: check they are on the same WiFi network as this computer")
    for rid, c in sorted(cars.items()):
        s = c.status
        print(f"robot {rid:2d}  ip {c.ip:15s}  battery {s.battery_mv / 1000:.2f} V  rssi {s.rssi} dBm  "
              f"fw {s.fw_version}  {'LOW BATTERY ' if s.low_battery else ''}{'idle' if s.watchdog_tripped else 'driving'}")
    return cars


def run_steps(link, rid, steps):
    for label, l, r, servo, rgb, secs in steps:
        print(label)
        t_end = time.monotonic() + secs
        while time.monotonic() < t_end:          # resend at 30 Hz so the watchdog stays happy
            link.poll()
            link.send(Command(rid, left=l, right=r, servo=servo, leds=[rgb] * 3))
            time.sleep(1 / 30)
    link.send(Command(rid, enable=False))


def cmd_test(link, rid):
    if not wait_for(link, rid):
        sys.exit(f"robot {rid} not found")
    run_steps(link, rid, [("forward", 400, 400, 90, (0, 80, 0), 1.0), ("back", -400, -400, 90, (80, 0, 0), 1.0),
                          ("spin left", -400, 400, 90, (0, 0, 80), 1.0), ("spin right", 400, -400, 90, (80, 80, 0), 1.0),
                          ("gripper open", 0, 0, 30, (80, 0, 80), 1.0), ("gripper close", 0, 0, 150, (0, 80, 80), 1.0)])


def cmd_drive(link, rid):
    import curses
    if not wait_for(link, rid):
        sys.exit(f"robot {rid} not found")

    def ui(scr):
        scr.nodelay(True)
        l = r = 0
        keys = {ord("w"): (500, 500), ord("s"): (-500, -500), ord("a"): (-350, 350), ord("d"): (350, -350), ord(" "): (0, 0)}
        while True:
            k = scr.getch()
            if k == ord("q"):
                break
            if k in keys:
                l, r = keys[k]
            link.poll()
            link.send(Command(rid, left=l, right=r))
            st = link.cars[rid].status
            scr.addstr(0, 0, f"robot {rid}  w/a/s/d drive, space stop, q quit   left {l:5d} right {r:5d}   battery {st.battery_mv} mV   ")
            time.sleep(1 / 30)
        link.send(Command(rid, enable=False))
    curses.wrapper(ui)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("set-id"); p.add_argument("ip"); p.add_argument("new_id", type=int)
    p = sub.add_parser("test"); p.add_argument("robot", type=int)
    p = sub.add_parser("drive"); p.add_argument("robot", type=int)
    a = ap.parse_args()
    link = CarLink()
    try:
        if a.cmd == "list":
            cmd_list(link)
        elif a.cmd == "set-id":
            link.set_id(a.ip, a.new_id)
            print(f"sent: {a.ip} -> robot id {a.new_id}. Power-cycle the car so its OTA name updates.")
        elif a.cmd == "test":
            cmd_test(link, a.robot)
        elif a.cmd == "drive":
            cmd_drive(link, a.robot)
    finally:
        link.close()


if __name__ == "__main__":
    main()
