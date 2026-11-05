"""ControlCenter: the one place that talks to the hardware (or the simulator).

A 30 Hz loop reads the world state from the Backend, asks each robot's active controller for a
command (manual, go-to-pose, follow a drawn path), passes every command through the Safety layer,
adds each car's LED colours and gripper position, and sends the result. The web GUI, and later
games, RL policies and multi-agent coordinators, only call ControlCenter methods.
"""
from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from robofield.control import DiffDrive, PoseController, deadzones_from_config, safety_margin_mm
from robofield.core.state import RobotCommand
from .safety import Safety, SafetySettings

ROOT = Path(__file__).resolve().parents[2]
PALETTE = ["#1e88e5", "#e53935", "#43a047", "#fb8c00", "#8e24aa", "#00acc1", "#fdd835", "#6d4c41"]


@dataclass
class CarProfile:
    id: int
    name: str = ""
    color: str = "#1e88e5"                          # colour on the map
    leds: list = field(default_factory=lambda: [[0, 0, 80]] * 3)
    max_speed: float = 1.0                          # fraction, on top of the global speed limit
    gripper: float = 0.0                            # 0 open .. 1 closed
    enabled: bool = True


class ControlCenter:
    def __init__(self, cfg: dict, backend, mode: str = "sim", cars_file: str | Path | None = None,
                 pi_host: str | None = None):
        self.cfg, self.backend, self.mode, self.pi_host = cfg, backend, mode, pi_host
        self.dd = DiffDrive.from_config(cfg)
        self.deadzone = deadzones_from_config(cfg)
        self.margin = safety_margin_mm(cfg)
        self.safety = Safety(self.dd, cfg["field"]["width_mm"], self.deadzone,
                             SafetySettings(geofence_margin_mm=self.margin))
        self.cars_file = Path(cars_file) if cars_file else ROOT / "config" / "cars.json"
        self.profiles: dict[int, CarProfile] = self._load_profiles()
        self.ctl: dict[int, PoseController] = {}
        self.modes: dict[int, str] = {}                 # idle | manual | goal | path
        self.manual: dict[int, tuple] = {}              # rid -> (left, right, time)
        self.identify_until: dict[int, float] = {}
        self.trails: dict[int, list] = {}
        self.log: list[str] = []
        self.flash_jobs: dict[int, dict] = {}
        self.lock = threading.RLock()
        self.state = backend.state() if hasattr(backend, "state") else None
        self.last_cmds: dict[int, RobotCommand] = {}
        self._overlay = None
        if pi_host:
            from robofield.comm.pose_link import OverlaySender
            self._overlay = OverlaySender(pi_host)
        self.running = False

    # ------------------------------------------------------------------ car registry
    def _load_profiles(self) -> dict[int, CarProfile]:
        if self.cars_file.exists():
            data = json.loads(self.cars_file.read_text())
            return {int(k): CarProfile(**v) for k, v in data.items()}
        return {}

    def _save_profiles(self):
        self.cars_file.parent.mkdir(parents=True, exist_ok=True)
        self.cars_file.write_text(json.dumps({k: asdict(v) for k, v in self.profiles.items()}, indent=2))

    def profile(self, rid: int) -> CarProfile:
        if rid not in self.profiles:
            self.profiles[rid] = CarProfile(rid, f"car {rid}", PALETTE[rid % len(PALETTE)])
            self._save_profiles()
            self._log(f"new car {rid} registered")
        return self.profiles[rid]

    def update_car(self, rid: int, **changes):
        with self.lock:
            p = self.profile(rid)
            for k, v in changes.items():
                if hasattr(p, k) and k != "id":
                    setattr(p, k, v)
            self._save_profiles()

    def _log(self, msg: str):
        self.log.append(time.strftime("%H:%M:%S ") + msg)
        self.log = self.log[-60:]

    # ------------------------------------------------------------------ commands (called by the GUI / apps)
    def robot_ids(self) -> list[int]:
        ids = set(self.state.robots) if self.state else set()
        if self.mode == "real":
            ids |= set(self.backend.cars.cars)
        return sorted(ids)

    def _controller(self, rid) -> PoseController:
        if rid not in self.ctl:
            self.ctl[rid] = PoseController(self.dd, deadzone=self.deadzone,
                                           field_mm=self.cfg["field"]["width_mm"], margin=self.margin)
        return self.ctl[rid]

    def set_goal(self, rid: int, x: float, y: float, theta: float):
        with self.lock:
            r = self.state.robots.get(rid) if self.state else None
            if r is None:
                return False, "robot not visible"
            c = self._controller(rid)
            c.set_goal(x, y, theta, (r.x, r.y))
            self.modes[rid] = "goal"
            self._send_overlay(rid)
            self._log(f"robot {rid}: go to ({x:.0f}, {y:.0f}) {math.degrees(theta):.0f} deg")
            return True, "ok"

    def set_path(self, rid: int, points: list, final_theta: float | None = None):
        with self.lock:
            r = self.state.robots.get(rid) if self.state else None
            if r is None or len(points) < 2:
                return False, "robot not visible or path too short"
            self._controller(rid).set_path(points, (r.x, r.y), final_theta)
            self.modes[rid] = "path"
            self._send_overlay(rid)
            self._log(f"robot {rid}: follow drawn path ({len(points)} points)")
            return True, "ok"

    def drive(self, rid: int, left: float, right: float):
        """Manual driving; must be repeated (expires after 0.3 s, like a dead-man switch)."""
        with self.lock:
            self.manual[rid] = (float(left), float(right), time.monotonic())
            self.modes[rid] = "manual"

    def stop(self, rid: int | None = None):
        with self.lock:
            for i in ([rid] if rid is not None else list(set(self.modes) | set(self.robot_ids()))):
                self.modes[i] = "idle"
                self.manual.pop(i, None)
            self._log("stop all" if rid is None else f"robot {rid}: stop")

    def set_estop(self, on: bool):
        with self.lock:
            self.safety.estop = bool(on)
            if on:
                for i in list(self.modes):
                    self.modes[i] = "idle"
            self._log("EMERGENCY STOP" if on else "emergency stop released")

    def identify(self, rid: int):
        self.identify_until[rid] = time.monotonic() + 2.0

    def update_settings(self, **kw):
        with self.lock:
            for k, v in kw.items():
                if hasattr(self.safety.settings, k):
                    setattr(self.safety.settings, k, type(getattr(self.safety.settings, k))(v))

    def flash(self, rid: int, method: str = "ota"):
        """Rebuild the car firmware and upload it (over WiFi by default) with PlatformIO."""
        if self.mode != "real" or rid not in self.backend.cars.cars:
            return False, "car not online"
        ip = self.backend.cars.cars[rid].ip
        cmd = ["pio", "run", "-e", method, "-t", "upload"] + (["--upload-port", ip] if method == "ota" else [])
        logf = ROOT / "logs" / f"flash_car{rid}.log"
        logf.parent.mkdir(exist_ok=True)
        proc = subprocess.Popen(cmd, cwd=ROOT / "firmware" / "car", stdout=logf.open("w"), stderr=subprocess.STDOUT)
        self.flash_jobs[rid] = {"proc": proc, "log": str(logf), "started": time.time()}
        self._log(f"robot {rid}: flashing firmware ({method} -> {ip})")
        return True, str(logf)

    # ------------------------------------------------------------------ loop
    def _send_overlay(self, rid):
        if self._overlay and rid in self.ctl:
            c = self.ctl[rid]
            r = self.state.robots.get(rid)
            self._overlay.send(goal=c.goal, path=([(r.x, r.y)] if r else []) + list(c.path))

    def tracker_status(self) -> dict:
        if self.mode != "real":
            return {"ok": True, "fps": self.backend.control_hz, "locked": True, "cam": None, "age_s": 0}
        msg = self.backend.poses.latest
        age = time.monotonic() - self.backend.poses.received_at if msg else 1e9
        return {"ok": msg is not None and age < self.safety.settings.tracker_timeout_s and msg.get("locked", False),
                "fps": msg.get("fps") if msg else 0, "locked": bool(msg and msg.get("locked")),
                "cam": msg.get("cam") if msg else None, "age_s": round(age, 2) if msg else None}

    def step_once(self):
        with self.lock:
            now = time.monotonic()
            ws = self.state
            cmds = {}
            for rid in self.robot_ids():
                p = self.profile(rid)
                mode = self.modes.get(rid, "idle")
                cmd = RobotCommand()
                r = ws.robots.get(rid) if ws else None
                if not p.enabled:
                    mode = "idle"
                elif mode == "manual":
                    l, rr, t = self.manual.get(rid, (0, 0, 0))
                    if now - t > 0.3:
                        self.modes[rid] = mode = "idle"
                    else:
                        cmd = RobotCommand(l, rr)
                elif mode in ("goal", "path") and r is not None and r.visible:
                    c = self.ctl[rid]
                    cmd = c.command(r.x, r.y, r.theta, r.omega, 1.0 / self.backend.control_hz)
                    if c.done:
                        self.modes[rid] = "idle"
                        self._log(f"robot {rid}: arrived")
                s = max(0.0, min(1.0, p.max_speed))
                cmd = RobotCommand(cmd.left * s, cmd.right * s)
                leds = [list(map(int, c)) for c in p.leds]
                if now < self.identify_until.get(rid, 0):
                    leds = [[255, 255, 255]] * 3 if int(now * 6) % 2 else [[0, 0, 0]] * 3
                if self.safety.estop:
                    leds = [[255, 0, 0]] * 3
                cmds[rid] = RobotCommand(cmd.left, cmd.right, p.gripper, tuple(tuple(c) for c in leds))
                if r is not None:
                    tr = self.trails.setdefault(rid, [])
                    if not tr or math.dist(tr[-1], (r.x, r.y)) > 5:
                        tr.append((round(r.x, 1), round(r.y, 1)))
                        del tr[:-300]
            battery = {}
            if self.mode == "real":
                battery = {i: c.status.battery_mv for i, c in self.backend.cars.cars.items()}
            safe = self.safety.filter(ws, cmds, self.tracker_status()["ok"], now, battery) if ws else cmds
            self.last_cmds = safe
            self.backend.apply(safe)
        self.state = self.backend.step()           # paces the loop at control_hz (outside the lock)

    def _loop(self):
        period = 1.0 / self.backend.control_hz
        next_t = time.monotonic()
        while self.running:
            try:
                self.step_once()
                if self.mode != "real":          # the simulator runs faster than real time: pace it
                    next_t += period
                    time.sleep(max(0.0, next_t - time.monotonic()))
                    if time.monotonic() - next_t > 0.5:
                        next_t = time.monotonic()
            except Exception as e:                  # never let one bad step kill the loop
                self._log(f"loop error: {e!r}")
                time.sleep(0.05)

    def start(self):
        self.state = self.backend.reset()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(1)
        self.backend.stop()
        self.backend.close()

    # ------------------------------------------------------------------ snapshot for the GUI
    def snapshot(self) -> dict:
        with self.lock:
            ws = self.state
            robots = []
            for rid in self.robot_ids():
                p = self.profile(rid)
                r = ws.robots.get(rid) if ws else None
                car = self.backend.cars.cars.get(rid) if self.mode == "real" else None
                c = self.ctl.get(rid)
                mode = self.modes.get(rid, "idle")
                robots.append({
                    "id": rid, "name": p.name, "color": p.color, "leds": p.leds, "max_speed": p.max_speed,
                    "gripper": p.gripper, "enabled": p.enabled,
                    "x": r.x if r else None, "y": r.y if r else None, "theta": r.theta if r else None,
                    "v": r.v if r else 0, "visible": bool(r and r.visible),
                    "online": (car is not None and time.monotonic() - car.last_seen < 1.0) if self.mode == "real" else True,
                    "ip": car.ip if car else None,
                    "battery_mv": car.status.battery_mv if car else (r.battery_mv if r else 0),
                    "rssi": car.status.rssi if car else None, "fw": car.status.fw_version if car else None,
                    "mode": mode, "phase": c.phase if c and mode in ("goal", "path") else "",
                    "goal": list(c.goal) if c and mode in ("goal", "path") else None,
                    "path": [list(pt) for pt in c.path] if c and mode in ("goal", "path") else [],
                    "trail": self.trails.get(rid, [])[-150:],
                    "safety": self.safety.events.get(rid, ""),
                    "flashing": bool(rid in self.flash_jobs and self.flash_jobs[rid]["proc"].poll() is None),
                })
            m = self.cfg["markers"]
            return {
                "mode": self.mode, "estop": self.safety.estop, "tracker": self.tracker_status(),
                "settings": asdict(self.safety.settings), "robots": robots, "log": self.log[-12:],
                "field": {"size": self.cfg["field"]["width_mm"], "wall": self.cfg["field"]["wall_thickness_mm"],
                          "towers": [[tid, x, y] for tid, (x, y, z) in m["corners"].items()]},
                "pi_preview": f"http://{self.pi_host}:8080" if self.pi_host else None,
            }
