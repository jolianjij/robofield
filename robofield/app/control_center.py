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
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

from robofield.control import DiffDrive, PoseController, deadzones_from_config, safety_margin_mm
from robofield.core.state import RobotCommand
from .safety import Safety, SafetySettings

ROOT = Path(__file__).resolve().parents[2]
PALETTE = ["#1e88e5", "#e53935", "#43a047", "#fb8c00", "#8e24aa", "#00acc1", "#fdd835", "#6d4c41"]

# (key, group, label, min, max, step, unit) - drives both ControlCenter's tuning API and the
# tuning web page's form, so the two never drift apart. *_deg keys are degrees here and in the API,
# converted to radians only where PoseController itself needs them.
TUNING_SCHEMA = [
    ("v_path",           "Lines & curves",    "Cruise speed",                 100, 340,  5,     "mm/s"),
    ("accel",            "Lines & curves",    "Acceleration / braking",       100, 1500, 25,    "mm/s2"),
    ("lat_accel",        "Lines & curves",    "Curve speed limit (side accel)", 100, 2000, 25,  "mm/s2"),
    ("lookahead",        "Lines & curves",    "Lookahead",                    50,  250,  5,     "mm"),
    ("pos_tol",          "Lines & curves",    "Arrive within",                5,   40,   1,     "mm"),
    ("pass_tol",         "Lines & curves",    "Accept a just-passed goal within", 10, 80, 1,    "mm"),
    ("reface_angle_deg", "Lines & curves",    "Stop and re-face if off by",   30,  120,  5,     "deg"),
    ("face_tol_deg",     "Final orientation", "Face the path within",         5,   45,   1,     "deg"),
    ("ang_tol_deg",      "Final orientation", "Final heading within",         1,   15,   0.5,   "deg"),
    ("wait_s",           "Final orientation", "Wait between nudges",          0.1, 1.0,  0.05,  "s"),
    ("pulse_speed",      "Final orientation", "Nudge wheel speed",            100, 250,  5,     "mm/s"),
    ("turn_style",       "Final orientation", "New pulse turns (1) / old continuous (0)", 0, 1, 1, ""),
    ("predict",          "Delay prediction",  "Prediction on (1) / off (0)",  0,   1,    1,     ""),
    ("latency_s",        "Delay prediction",  "Loop delay to predict over",   0.0, 0.5,  0.01,  "s"),
    ("trim_left",        "Motors",            "Left trim",                    0.7, 1.2,  0.005, "x"),
    ("trim_right",       "Motors",            "Right trim",                   0.7, 1.2,  0.005, "x"),
]
TUNING_KEYS = {row[0] for row in TUNING_SCHEMA}
_DEG_KEYS = {"reface_angle_deg": "reface_angle", "face_tol_deg": "face_tol", "ang_tol_deg": "ang_tol"}
_PC_KEYS = TUNING_KEYS - set(_DEG_KEYS) - {"predict", "latency_s", "trim_left", "trim_right"}


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
        self._learned: dict[int, dict] = {}             # each car's learned turning model, kept across rebuilds
        self.last_path: dict[int, tuple] = {}           # rid -> (points, final_theta) of the last path drawn
        self._gen = 0                                   # bumped by every new order (lets a playing move notice a STOP)
        self.parking: dict[int, bool] = {}              # rid -> driving to a move's start (two legs)
        self._dd0 = self.dd
        self.tuning: dict = self._default_tuning()
        self._load_saved_tuning()
        self._apply_tuning()
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
                                           field_mm=self.cfg["field"]["width_mm"], margin=self.margin,
                                           learned=self._learned.setdefault(rid, {}),
                                           **self._pose_controller_kwargs())
        return self.ctl[rid]

    # ------------------------------------------------------------------ live tuning
    def _default_tuning(self) -> dict:
        pc = PoseController(self._dd0)                 # a throwaway instance, just for its field defaults
        d = {k: float(getattr(pc, k)) for k in _PC_KEYS}
        d.update({k: math.degrees(getattr(pc, attr)) for k, attr in _DEG_KEYS.items()})
        d["predict"] = 1.0
        d["latency_s"] = float(self.cfg["control"]["latency_s"])
        d["trim_left"], d["trim_right"] = self._dd0.trim_left, self._dd0.trim_right   # measured, from the config
        return d

    def _tuning_path(self) -> Path:
        return Path(self.cfg.get("_path", "config/field.yaml")).with_name("tuning.yaml")

    def _load_saved_tuning(self):
        import yaml
        path = self._tuning_path()
        if path.exists():
            saved = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            self.tuning.update({k: float(v) for k, v in saved.items() if k in TUNING_KEYS})

    def _apply_tuning(self):
        """Push the values that live outside PoseController to where they are used."""
        if hasattr(self.backend, "predict"):
            self.backend.predict = self.tuning["predict"] >= 0.5
        if hasattr(self.backend, "latency"):
            self.backend.latency = self.tuning["latency_s"]
        self.dd = replace(self.dd, trim_left=self.tuning["trim_left"], trim_right=self.tuning["trim_right"])
        self.safety.dd = self.dd
        self.ctl.clear()    # next goal/path rebuilds every controller with the new values
        for rid, mode in list(getattr(self, "modes", {}).items()):   # a move in progress just lost its controller: end it
            if mode in ("goal", "path"):
                self.modes[rid] = "idle"

    def _pose_controller_kwargs(self) -> dict:
        kw = {k: self.tuning[k] for k in _PC_KEYS}
        kw.update({attr: math.radians(self.tuning[k]) for k, attr in _DEG_KEYS.items()})
        return kw

    def get_tuning(self) -> dict:
        with self.lock:
            return {"values": dict(self.tuning), "schema": [list(row) for row in TUNING_SCHEMA]}

    def set_tuning(self, **changes):
        with self.lock:
            bad = set(changes) - TUNING_KEYS
            if bad:
                return False, f"unknown parameter(s): {', '.join(sorted(bad))}"
            self.tuning.update({k: float(v) for k, v in changes.items()})
            self._apply_tuning()
            self._log("tuning: " + ", ".join(f"{k}={float(v):g}" for k, v in changes.items()))
            return True, "ok"

    def reset_tuning(self):
        with self.lock:
            self.tuning = self._default_tuning()
            self._apply_tuning()
            self._log("tuning reset to defaults")
            return True, "ok"

    def save_tuning(self):
        """Persist the current values to config/tuning.yaml (loaded on top of the defaults at startup).
        A separate file, so field.yaml - measurements with their explanations - is never rewritten."""
        import yaml
        with self.lock:
            path = self._tuning_path()
            body = yaml.safe_dump({k: round(float(v), 4) for k, v in self.tuning.items()}, sort_keys=False)
            header = ("# Saved from the live tuning page (/tuning). Overrides the defaults at startup;\n"
                      "# delete this file to go back to them.\n")
            path.write_text(header + body, encoding="utf-8")
            self._log(f"tuning saved to {path}")
            return True, str(path)

    def set_goal(self, rid: int, x: float, y: float, theta: float):
        with self.lock:
            r = self.state.robots.get(rid) if self.state else None
            if r is None:
                return False, "robot not visible"
            c = self._controller(rid)
            c.set_goal(x, y, theta, (r.x, r.y))
            self.modes[rid] = "goal"
            self._gen += 1
            self._send_overlay(rid)
            self._log(f"robot {rid}: go to ({x:.0f}, {y:.0f}) {math.degrees(theta):.0f} deg")
            return True, "ok"

    def set_path(self, rid: int, points: list, final_theta: float | None = None, from_start: bool = False):
        """from_start: follow the whole path from its first point (a replayed move), instead of
        joining it at the point nearest the car (a freshly drawn one)."""
        with self.lock:
            r = self.state.robots.get(rid) if self.state else None
            if r is None or len(points) < 2:
                return False, "robot not visible or path too short"
            self._controller(rid).set_path(points, (r.x, r.y), final_theta, nearest=not from_start)
            self.modes[rid] = "path"
            self._gen += 1
            self.last_path[rid] = ([list(map(float, p)) for p in points], final_theta)
            self._send_overlay(rid)
            self._log(f"robot {rid}: follow drawn path ({len(points)} points)")
            return True, "ok"

    # ------------------------------------------------------------------ saved moves (a drawn path, kept to replay)
    @property
    def moves_dir(self) -> Path:
        return Path(self.cfg.get("_path", "config/field.yaml")).parent / "moves"

    @staticmethod
    def _move_name(name: str) -> str:
        clean = "".join(c for c in str(name).strip().replace(" ", "_") if c.isalnum() or c in "_-")
        if not clean:
            raise ValueError("give the move a name (letters, digits, - or _)")
        return clean

    def save_move(self, rid: int, name: str, points: list | None = None, theta: float | None = None):
        """Save `points` (or, without them, the path last drawn for car `rid`) as a named move."""
        with self.lock:
            if points is None:
                if rid not in self.last_path:
                    return False, "draw a path for this car first"
                points, theta = self.last_path[rid]
            if len(points) < 2:
                return False, "path too short"
            name = self._move_name(name)
            self.moves_dir.mkdir(parents=True, exist_ok=True)
            (self.moves_dir / f"{name}.json").write_text(json.dumps(
                {"name": name, "points": [[round(x, 1), round(y, 1)] for x, y in points], "final_theta": theta}))
            self._log(f"saved move '{name}' ({len(points)} points)")
            return True, name

    def list_moves(self) -> list[dict]:
        out = []
        for f in sorted(self.moves_dir.glob("*.json")) if self.moves_dir.exists() else []:
            m = json.loads(f.read_text())
            out.append({"name": m["name"], "points": m["points"], "final_theta": m["final_theta"]})
        return out

    def _load_move(self, name: str) -> dict:
        f = self.moves_dir / f"{self._move_name(name)}.json"
        if not f.exists():
            raise ValueError(f"no saved move '{name}'")
        return json.loads(f.read_text())

    @staticmethod
    def _move_start(m: dict) -> tuple[float, float, float]:
        (x0, y0), far = m["points"][0], m["points"][min(len(m["points"]) - 1, 3)]
        return x0, y0, math.atan2(far[1] - y0, far[0] - x0)

    def _wait_goal(self, rid: int, timeout: float = 40.0) -> bool:
        """Wait for the go-to just ordered to finish; False if it was stopped or replaced meanwhile."""
        gen, t0 = self._gen, time.monotonic()
        while self.modes.get(rid) == "goal" and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return self._gen == gen and self.modes.get(rid) == "idle"

    def _park(self, rid: int, x0: float, y0: float, th0: float, back: float = 120.0) -> bool:
        """Drive to (x0, y0) facing th0, arriving straight along th0: first to a point `back` mm behind
        it, then straight in. A big turn on the spot pivots about a wheel and shifts the car by
        centimetres; this way the last turn is tiny and the car really ends up on the start."""
        from robofield.control.path import clamp_to_field
        r = self.state.robots.get(rid) if self.state else None
        if r is None:
            return False
        c, s = math.cos(th0), math.sin(th0)
        along = (r.x - x0) * c + (r.y - y0) * s          # < 0: behind the start
        across = -(r.x - x0) * s + (r.y - y0) * c
        lined_up = -2.5 * back < along < -40 and abs(across) < 25 and abs(math.remainder(r.theta - th0, 2 * math.pi)) < 0.3
        if not lined_up:
            bx, by = clamp_to_field(x0 - back * c, y0 - back * s, self.cfg["field"]["width_mm"], self.margin)
            ok, _ = self.set_goal(rid, bx, by, th0)
            if not ok or not self._wait_goal(rid):
                return False
            time.sleep(0.3)
        ok, _ = self.set_goal(rid, x0, y0, th0)
        return ok and self._wait_goal(rid)

    def move_to_start(self, rid: int, name: str):
        """Park the car where the move begins, facing along it - ready to film."""
        m = self._load_move(name)
        if self.state is None or self.state.robots.get(rid) is None:
            return False, "robot not visible"
        self.parking[rid] = True

        def run():
            try:
                self._park(rid, *self._move_start(m))
            finally:
                self.parking[rid] = False
        threading.Thread(target=run, daemon=True).start()
        return True, "parking at the start"

    def play_move(self, rid: int, name: str):
        """Drive a saved move. If the car is not already at its start (facing along it), it goes
        there first, then follows the path."""
        m = self._load_move(name)
        x0, y0, th0 = self._move_start(m)
        r = self.state.robots.get(rid) if self.state else None
        if r is None:
            return False, "robot not visible"
        at_start = math.dist((r.x, r.y), (x0, y0)) < 40 and abs(math.remainder(r.theta - th0, 2 * math.pi)) < 0.45

        def run():
            if not at_start:
                self.parking[rid] = True
                try:
                    if not self._park(rid, x0, y0, th0):      # stopped or redirected meanwhile
                        return
                finally:
                    self.parking[rid] = False
                gen = self._gen
                time.sleep(0.5)                           # a still, deliberate beat before it sets off
                if self._gen != gen:
                    return
            self.set_path(rid, m["points"], m["final_theta"], from_start=True)
            self._log(f"robot {rid}: playing move '{m['name']}'")

        threading.Thread(target=run, daemon=True).start()
        return True, "going to the start first" if not at_start else "playing"

    def drive(self, rid: int, left: float, right: float):
        """Manual driving; must be repeated (expires after 0.3 s, like a dead-man switch)."""
        with self.lock:
            self.manual[rid] = (float(left), float(right), time.monotonic())
            self.modes[rid] = "manual"
            self._gen += 1

    def stop(self, rid: int | None = None):
        with self.lock:
            for i in ([rid] if rid is not None else list(set(self.modes) | set(self.robot_ids()))):
                self.modes[i] = "idle"
                self.manual.pop(i, None)
            self._gen += 1
            self._log("stop all" if rid is None else f"robot {rid}: stop")

    def set_estop(self, on: bool):
        with self.lock:
            self.safety.estop = bool(on)
            self._gen += 1
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
                elif mode in ("goal", "path") and rid not in self.ctl:
                    self.modes[rid] = "idle"                # (cannot happen any more; never crash the loop on it)
                elif mode in ("goal", "path") and r is not None and r.visible:
                    c = self.ctl[rid]
                    cmd = c.command(r.x, r.y, r.theta, r.omega, 1.0 / self.backend.control_hz)
                    if c.done:
                        self.modes[rid] = "idle"
                        self._log(f"robot {rid}: arrived")
                s = max(0.0, min(1.0, p.max_speed))
                if s < 1.0 and (cmd.left or cmd.right):
                    # scale the body motion, not each wheel's command: on this car's non-linear
                    # motors a per-wheel multiply sends both wheels to the same minimum speed and
                    # erases steering; inverse() keeps the turn ratio, as slow as the car can go
                    v, w = self.dd.forward(cmd.left, cmd.right)
                    cmd = RobotCommand(*self.dd.inverse(v * s, w * s))
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
                    "parking": bool(self.parking.get(rid)),
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
