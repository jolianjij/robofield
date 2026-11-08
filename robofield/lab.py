"""The Lab: write your own tasks, policies and systems for the robots, in Python, in custom/.

    Task    something a car should LEARN with reinforcement learning: where it starts, what it sees,
            what earns reward, when an episode ends. custom/tasks/<name>.py; trains on the Training tab.
    Policy  a hand-written program that drives ONE car: every step (30 per second) it gets the car and
            the world and says how to drive. custom/policies/<name>.py.
    System  a program that drives SEVERAL cars together: formations, follow the leader, patrols.
            custom/systems/<name>.py.

The RoboField app's Lab tab edits, checks and runs them. Programs run inside the Control Center,
so every command still passes through the safety layer, on the simulator and on the real field.

Units in programs are the friendly ones: millimetres, degrees, seconds (speed in mm/s, turning in
degrees per second). Tasks use the same units as the rest of RoboField (mm, radians), like the
built-in games in robofield/games/.

    python -m robofield.lab check policy my_policy custom/policies/my_policy.py    # what the app runs
"""
from __future__ import annotations

import json
import math
import shutil
import sys
import time
import traceback
import types
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from robofield.core.state import RobotCommand, RobotState, WorldState
from robofield.games.base import REGISTRY, Game

ROOT = Path(__file__).resolve().parents[1]
CUSTOM = ROOT / "custom"
FOLDERS = {"task": "tasks", "policy": "policies", "system": "systems"}
Task = Game                       # a task IS a game: the same class the RL environments use


# ------------------------------------------------------------------ what a program sees
@dataclass
class Car:
    id: int
    x: float                       # mm
    y: float                       # mm
    heading: float                 # degrees, 0 = +x, counter-clockwise positive
    speed: float                   # mm/s forward
    turn_rate: float               # degrees per second
    visible: bool = True
    name: str = ""

    @property
    def theta(self) -> float:
        """The heading in radians."""
        return math.radians(self.heading)


@dataclass
class Thing:
    id: int
    kind: str                      # "ball", "box", ...
    x: float
    y: float


@dataclass
class World:
    t: float                       # seconds since the program started
    cars: list = field(default_factory=list)       # every car, in id order
    things: list = field(default_factory=list)     # balls, boxes...
    size: float = 1000.0           # the field is size x size mm

    def car(self, car_id: int) -> Car | None:
        return next((c for c in self.cars if c.id == car_id), None)

    def others(self, me: Car) -> list:
        return [c for c in self.cars if c.id != me.id]

    @property
    def ball(self) -> Thing | None:
        return next((t for t in self.things if t.kind == "ball"), None)


DONE = object()                    # returned by finish(): the program has ended


class _Driving:
    """Helpers shared by policies and systems. They return a command for one car."""
    dd = None                      # the car's kinematics (set when the program starts)

    def drive(self, speed: float = 0.0, turn: float = 0.0) -> RobotCommand:
        """Forward speed in mm/s (negative = backwards) and turning in degrees per second
        (positive = counter-clockwise). The car cannot drive slower than about 100 mm/s: slower
        requests are sped up along the same curve."""
        left, right = self.dd.inverse(float(speed), math.radians(float(turn)))
        return RobotCommand(left, right)

    def wheels(self, left: float, right: float) -> RobotCommand:
        """Raw wheel commands from -1 (full backwards) to 1 (full forwards)."""
        return RobotCommand(max(-1.0, min(1.0, float(left))), max(-1.0, min(1.0, float(right))))

    def stop(self) -> RobotCommand:
        return RobotCommand()

    def finish(self):
        """Return this from step() to end the program."""
        return DONE

    @staticmethod
    def distance(car: Car, x: float, y: float) -> float:
        """Millimetres from the car to (x, y)."""
        return math.hypot(x - car.x, y - car.y)

    @staticmethod
    def angle_to(car: Car, x: float, y: float) -> float:
        """Degrees the car must turn to face (x, y): positive = to its left, -180..180."""
        a = math.degrees(math.atan2(y - car.y, x - car.x)) - car.heading
        return (a + 180) % 360 - 180

    def face(self, car: Car, x: float, y: float, tolerance: float = 8.0) -> RobotCommand:
        """Turn on the spot toward (x, y); stops once within `tolerance` degrees."""
        e = self.angle_to(car, x, y)
        if abs(e) < tolerance:
            return self.stop()
        return self.drive(0, math.copysign(min(240.0, 60 + 2.5 * abs(e)), e))

    def go_to(self, car: Car, x: float, y: float, speed: float = 200.0, arrive: float = 20.0) -> RobotCommand:
        """Drive toward (x, y): turn first if it is far off to the side, slow down near it,
        stop within `arrive` mm. Simple and good enough for programs; for precise stops and a
        final heading the Control Center's go-to-pose is better."""
        d = self.distance(car, x, y)
        if d < arrive:
            return self.stop()
        e = self.angle_to(car, x, y)
        if abs(e) > 60:
            return self.face(car, x, y)
        v = min(speed, 2.5 * d) * max(0.0, math.cos(math.radians(e))) ** 2
        return self.drive(v, max(-200.0, min(200.0, 3.0 * e)))

    def arrived(self, car: Car, x: float, y: float, within: float = 25.0) -> bool:
        return self.distance(car, x, y) < within


class Policy(_Driving):
    """A hand-written program that drives one car. Override start() and step()."""

    def start(self, car: Car, world: World) -> None:
        """Called once, before the first step."""

    def step(self, car: Car, world: World):
        """Called 30 times a second. Return a command (self.drive(...), self.go_to(...), ...),
        or self.finish() when done."""
        return self.stop()


class System(_Driving):
    """A program that drives several cars together. Set `cars` to how many it needs."""
    cars = 2

    def start(self, world: World) -> None:
        """Called once, before the first step."""

    def step(self, world: World):
        """Called 30 times a second. Return {car id: command} for the cars it drives, or
        self.finish() when done. A car left out of the dict stops."""
        return {}


BASES = {"task": Game, "policy": Policy, "system": System}


def world_from(ws: WorldState, t0: float, size: float, names: dict | None = None) -> World:
    cars = [Car(r.id, r.x, r.y, math.degrees(r.theta), r.v, math.degrees(r.omega), r.visible,
                (names or {}).get(r.id, f"car {r.id}")) for r in sorted(ws.robots.values(), key=lambda r: r.id)]
    things = [Thing(o.id, o.kind, o.x, o.y) for o in ws.objects.values()]
    return World(ws.t - t0, cars, things, size)


# ------------------------------------------------------------------ files
def _clean(name: str) -> str:
    n = "".join(c for c in str(name).strip().replace(" ", "_").replace("-", "_") if c.isalnum() or c == "_").lower()
    if not n or n[0].isdigit():
        raise ValueError("a name is letters, digits and _, starting with a letter")
    return n


def path_of(kind: str, name: str) -> Path:
    if kind not in FOLDERS:
        raise ValueError(f"kind is one of {', '.join(FOLDERS)}")
    return CUSTOM / FOLDERS[kind] / f"{_clean(name)}.py"


def module_from(kind: str, name: str, source: str) -> types.ModuleType:
    """Run a program's source as its own module; errors point at its lines in custom/<kind>/<name>.py."""
    filename = f"custom/{FOLDERS[kind]}/{_clean(name)}.py"
    mod = types.ModuleType(f"custom_{kind}_{_clean(name)}")
    mod.__file__ = filename
    exec(compile(source, filename, "exec"), mod.__dict__)
    return mod


def class_in(kind: str, mod) -> type:
    base = BASES[kind]
    found = [v for v in vars(mod).values()
             if isinstance(v, type) and issubclass(v, base) and v is not base and v.__module__ == mod.__name__]
    if not found:
        raise TypeError(f"no class based on {base.__name__} in this file (class MyThing({'Task' if kind == 'task' else base.__name__}): ...)")
    return found[0]


def load_class(kind: str, name: str, source: str | None = None) -> type:
    src = source if source is not None else path_of(kind, name).read_text(encoding="utf-8")
    return class_in(kind, module_from(kind, name, src))


def _obs_dim(cls) -> int:
    g = cls()
    g.seed(0)
    lay = g.layout()
    ws = WorldState(0.0, {i: RobotState(i, *p) for i, p in lay.get("robots", {}).items()}, {})
    return int(np.asarray(g.observe(ws, 0)).size)


def register_tasks() -> dict:
    """Make every custom task a game (for the RL environments and the Training tab). Returns
    {name: error} for the files that could not be loaded."""
    errors = {}
    folder = CUSTOM / FOLDERS["task"]
    for f in sorted(folder.glob("*.py")) if folder.exists() else []:
        if f.name.startswith("_"):
            continue
        try:
            cls = load_class("task", f.stem)
            cls.name = f.stem
            if not getattr(cls, "obs_dim", None):
                cls.obs_dim = _obs_dim(cls)
            cls.custom = True
            REGISTRY[f.stem] = cls
        except Exception as e:                                # a broken file must not break the app
            errors[f.stem] = f"{type(e).__name__}: {e}"
    return errors


def list_all() -> dict:
    out = {}
    for kind, folder in FOLDERS.items():
        items = []
        for f in sorted((CUSTOM / folder).glob("*.py")) if (CUSTOM / folder).exists() else []:
            if f.name.startswith("_"):
                continue
            about = ""
            try:
                src = f.read_text(encoding="utf-8")
                cls = load_class(kind, f.stem, src)
                about = (cls.__doc__ or "").strip().split("\n")[0]
                extra = {"cars": getattr(cls, "cars", None) if kind == "system" else None,
                         "trainable": getattr(cls, "n_robots", 1) == 1 if kind == "task" else None}
                err = ""
            except Exception as e:
                extra, err = {}, f"{type(e).__name__}: {e}"
            items.append({"name": f.stem, "about": about, "error": err,
                          "modified": time.strftime("%Y-%m-%d %H:%M", time.localtime(f.stat().st_mtime)), **extra})
        out[kind] = items
    return out


def save(kind: str, name: str, source: str) -> tuple[bool, str]:
    p = path_of(kind, name)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(source.replace("\r\n", "\n"), encoding="utf-8")
    if kind == "task":
        register_tasks()
    return True, f"custom/{FOLDERS[kind]}/{p.name}"


def delete(kind: str, name: str) -> tuple[bool, str]:
    p = path_of(kind, name)
    if not p.exists():
        return False, "no such file"
    bin_ = CUSTOM / ".deleted"
    bin_.mkdir(parents=True, exist_ok=True)
    dest = bin_ / f"{p.stem}_{time.strftime('%Y%m%d_%H%M%S')}.py"
    shutil.move(str(p), dest)
    if kind == "task":
        REGISTRY.pop(p.stem, None)
    return True, f"moved to custom/.deleted/{dest.name}"


# ------------------------------------------------------------------ templates
TEMPLATES = {
"task": '''"""{title}: one line saying what the car should learn."""
import math

import numpy as np

from robofield.lab import Task


class {cls}(Task):
    n_robots = 1                 # one car (two-car tasks need self-play, coming later)
    max_steps = 450              # 15 s at 30 steps per second, then the episode is cut off

    def layout(self):
        """Where an episode starts. Pick targets here too. Positions in mm, angles in radians."""
        start = (*self.rng.uniform(150, 850, 2), self.rng.uniform(-math.pi, math.pi))
        self.target = tuple(self.rng.uniform(150, 850, 2))
        self.reached = False
        return {{"robots": {{0: start}}, "objects": {{}}}}

    def begin(self, ws):
        """The real field cannot place the car: start from where it is, with a new target."""
        self.target = tuple(self.rng.uniform(150, 850, 2))
        self.reached = False

    def observe(self, ws, i=0):
        """What the car sees: a few numbers, relative to the car, scaled to about -1..1."""
        car = ws.robots[0]
        tx, ty = self.to_robot_frame(car, *self.target)        # target ahead (x) and to the left (y)
        return np.array([tx / 1000, ty / 1000, car.v / 335, car.omega / 16], dtype=np.float32)

    def reward(self, prev, ws, cmds):
        """How good the last step was. Reward progress, not just the end, so it can learn."""
        before = math.dist((prev.robots[0].x, prev.robots[0].y), self.target)
        now = math.dist((ws.robots[0].x, ws.robots[0].y), self.target)
        r = (before - now) / 100 - 0.01                          # closer is good, time costs a little
        if now < 30 and not self.reached:
            self.reached = True
            r += 5
        return {{0: r}}

    def done(self, ws):
        return self.reached

    def info(self, ws):
        return {{"success": self.reached}}

    def overlays(self):
        """What the 3D view draws while a policy drives this task."""
        return [{{"type": "circle", "x": self.target[0], "y": self.target[1], "r": 30, "color": "#2e7d32"}}]
''',
"policy": '''"""{title}: one line saying what this program makes the car do."""
from robofield.lab import Policy


class {cls}(Policy):

    def start(self, car, world):
        """Called once. car.x, car.y in mm, car.heading in degrees; world.t in seconds."""
        self.points = [(300, 300), (700, 300), (700, 700), (300, 700)]
        self.next = 0
        self.laps = 0

    def step(self, car, world):
        """Called 30 times a second: return how to drive."""
        x, y = self.points[self.next]
        if self.arrived(car, x, y):
            self.next = (self.next + 1) % len(self.points)
            if self.next == 0:
                self.laps += 1
                if self.laps == 2:
                    return self.finish()
        return self.go_to(car, x, y, speed=200)
''',
"system": '''"""{title}: one line saying what the cars do together."""
import math

from robofield.lab import System


class {cls}(System):
    cars = 2                     # how many cars it drives (the first ones, by id)

    def start(self, world):
        self.leader, self.follower = world.cars[0], world.cars[1]

    def step(self, world):
        """Return {{car id: command}} for every car it drives."""
        leader = world.car(self.leader.id)
        follower = world.car(self.follower.id)
        a = world.t * 0.5                                     # the leader circles the middle
        lx, ly = 500 + 250 * math.cos(a), 500 + 250 * math.sin(a)
        behind_x = leader.x - 160 * math.cos(leader.theta)    # the follower aims 160 mm behind it
        behind_y = leader.y - 160 * math.sin(leader.theta)
        if world.t > 40:
            return self.finish()
        return {{leader.id: self.go_to(leader, lx, ly, speed=180),
                follower.id: self.go_to(follower, behind_x, behind_y, speed=220)}}
''',
}


def template(kind: str, name: str) -> str:
    n = _clean(name)
    return TEMPLATES[kind].format(title=n.replace("_", " ").capitalize(),
                                  cls="".join(w.capitalize() for w in n.split("_")) or "MyThing")


# ------------------------------------------------------------------ checking a program in the simulator
def _where(exc: BaseException, kind: str, name: str) -> int | None:
    filename = f"custom/{FOLDERS[kind]}/{_clean(name)}.py"
    if isinstance(exc, SyntaxError) and exc.filename == filename:
        return exc.lineno
    lines = [f.lineno for f in traceback.extract_tb(exc.__traceback__) if f.filename == filename]
    return lines[-1] if lines else None


def check(kind: str, name: str, source: str) -> dict:
    """Load the program and run it briefly in a simulator. {"ok", "errors": [{"line", "message"}], "notes"}."""
    from robofield.control import DiffDrive
    from robofield.core import load_config
    from robofield.sim.backend import SimBackend

    notes, t0 = [], time.monotonic()
    cfg = load_config(ROOT / "config" / "field.yaml")
    try:
        cls = class_in(kind, module_from(kind, name, source))
        notes.append(f"found {cls.__name__}")
        if kind == "task":
            g = cls()
            g.seed(1)
            lay = g.layout()
            n = getattr(cls, "n_robots", 1)
            b = SimBackend(cfg, n, getattr(cls, "objects", ()), seed=1, randomize=False, markers=False)
            prev = ws = b.reset(lay)
            obs = np.asarray(g.observe(ws, 0), dtype=np.float64)
            if obs.ndim != 1 or not np.all(np.isfinite(obs)):
                raise ValueError(f"observe() must return a flat list of finite numbers, got shape {obs.shape}")
            dim = getattr(cls, "obs_dim", None)
            if dim and dim != obs.size:
                raise ValueError(f"obs_dim is {dim} but observe() returned {obs.size} numbers (or leave obs_dim out)")
            rng, rewards, steps, ended = np.random.default_rng(0), [], 0, False
            for steps in range(1, 151):
                cmds = {i: RobotCommand(*rng.uniform(-1, 1, 2)) for i in range(n)}
                b.apply(cmds)
                ws = b.step()
                r = g.reward(prev, ws, cmds)
                if not isinstance(r, dict) or 0 not in r or not np.isfinite(float(r[0])):
                    raise ValueError("reward() must return {0: a number}")
                rewards.append(float(r[0]))
                o = np.asarray(g.observe(ws, 0), dtype=np.float64)
                if o.shape != obs.shape or not np.all(np.isfinite(o)):
                    raise ValueError("observe() changed length or returned a non-number")
                prev = ws
                if g.done(ws):
                    ended = True
                    break
            g.info(ws)
            g.overlays()
            notes.append(f"observation: {obs.size} numbers, e.g. {np.round(obs, 2).tolist()[:6]}")
            notes.append(f"reward per step with random driving: {min(rewards):.3f} to {max(rewards):.3f}")
            notes.append(f"episode {'ended' if ended else 'still going'} after {steps} steps (max {cls.max_steps})")
            notes.append("ready to train on the Training tab" if n == 1 else f"{n}-car task: training needs self-play (coming)")
        else:
            n = 1 if kind == "policy" else int(getattr(cls, "cars", 2))
            b = SimBackend(cfg, n, seed=1, randomize=False, markers=False)
            ws = b.reset({"robots": {i: (300 + 400 * (i % 2), 300 + 400 * (i // 2), 0.0) for i in range(n)}})
            prog = cls()
            prog.dd = DiffDrive.from_config(cfg)
            w = world_from(ws, ws.t, cfg["field"]["width_mm"])
            prog.start(w.cars[0], w) if kind == "policy" else prog.start(w)
            start = [(c.x, c.y) for c in w.cars]
            steps, ended = 0, False
            for steps in range(1, 301):
                w = world_from(ws, 0.0, cfg["field"]["width_mm"])
                out = prog.step(w.cars[0], w) if kind == "policy" else prog.step(w)
                if out is DONE:
                    ended = True
                    break
                if kind == "policy":
                    if not isinstance(out, RobotCommand):
                        raise TypeError(f"step() must return a command (self.drive(...), self.go_to(...)...), got {type(out).__name__}")
                    cmds = {0: out}
                else:
                    if not isinstance(out, dict) or not all(isinstance(c, RobotCommand) for c in out.values()):
                        raise TypeError("step() must return {car id: command}")
                    cmds = out
                b.apply(cmds)
                ws = b.step()
            moved = [math.dist(s, (ws.robots[i].x, ws.robots[i].y)) for i, s in enumerate(start)]
            notes.append(f"ran {steps} steps ({steps / 30:.1f} s){', then finished' if ended else ''}; "
                         + ", ".join(f"car {i} moved {m:.0f} mm" for i, m in enumerate(moved)))
        return {"ok": True, "errors": [], "notes": notes, "seconds": round(time.monotonic() - t0, 2)}
    except Exception as e:
        tb = "".join(traceback.format_exception_only(type(e), e)).strip()
        return {"ok": False, "errors": [{"line": _where(e, kind, name), "message": tb}], "notes": notes,
                "seconds": round(time.monotonic() - t0, 2)}


def check_isolated(kind: str, name: str, source: str, timeout: float = 60.0) -> dict:
    """check() in a separate Python process, so an endless loop or a crash cannot hang the app."""
    import subprocess
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False, encoding="utf-8") as f:
        f.write(source)
        tmp = f.name
    try:
        out = subprocess.run([sys.executable, "-m", "robofield.lab", "check", kind, name, tmp], cwd=ROOT,
                             capture_output=True, text=True, timeout=timeout,
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        line = next((ln for ln in reversed(out.stdout.splitlines()) if ln.startswith("{")), None)
        if line is None:
            return {"ok": False, "errors": [{"line": None, "message": (out.stderr or out.stdout).strip()[-800:]}], "notes": []}
        return json.loads(line)
    except subprocess.TimeoutExpired:
        return {"ok": False, "errors": [{"line": None, "message": f"it did not finish within {timeout:.0f} s: "
                                         "is there an endless loop in step()?"}], "notes": []}
    finally:
        Path(tmp).unlink(missing_ok=True)


# ------------------------------------------------------------------ running a policy or system in the Control Center
class ProgramRunner:
    """A policy (one car) or a system (several cars) running inside the Control Center."""

    def __init__(self, kind: str, name: str, car_ids: list[int], dd, size: float, speed: float = 1.0):
        self.kind, self.name, self.ids = kind, _clean(name), list(car_ids)
        cls = load_class(kind, name)
        if kind == "system" and len(self.ids) < int(getattr(cls, "cars", 2)):
            raise ValueError(f"{name} needs {cls.cars} cars, {len(self.ids)} given")
        self.prog = cls()
        self.prog.dd = dd
        self.size, self.speed = size, max(0.05, min(1.0, float(speed)))
        self.t0, self.steps, self.result, self.error, self.slow = 0.0, 0, "", "", 0

    def begin(self, ws: WorldState, names: dict):
        self.t0 = ws.t
        w = world_from(ws, self.t0, self.size, names)
        if self.kind == "policy":
            self.prog.start(w.car(self.ids[0]), w)
        else:
            w.cars = [c for c in w.cars if c.id in self.ids]
            self.prog.start(w)

    def step(self, ws: WorldState, names: dict) -> dict:
        """{car id: command}; sets self.result when the program ends or fails."""
        t = time.monotonic()
        w = world_from(ws, self.t0, self.size, names)
        try:
            if self.kind == "policy":
                car = w.car(self.ids[0])
                out = self.prog.step(car, w)
                cmds = None if out is DONE else {self.ids[0]: out}
            else:
                w.cars = [c for c in w.cars if c.id in self.ids] + [c for c in w.cars if c.id not in self.ids]
                out = self.prog.step(w)
                cmds = None if out is DONE else {int(k): v for k, v in out.items() if int(k) in self.ids}
            if cmds is None:
                self.result = "finished"
                return {}
            for c in cmds.values():
                if not isinstance(c, RobotCommand):
                    raise TypeError("step() must return commands (self.drive(...), self.go_to(...), ...)")
        except Exception as e:
            line = _where(e, self.kind, self.name)
            self.error = f"{type(e).__name__}: {e}" + (f" (line {line})" if line else "")
            self.result = "error"
            return {}
        self.steps += 1
        if time.monotonic() - t > 0.02:
            self.slow += 1
        return cmds

    def summary(self, running: bool) -> dict:
        return {"kind": self.kind, "name": self.name, "cars": self.ids, "running": running, "speed": self.speed,
                "seconds": round(self.steps / 30, 1), "result": self.result, "error": self.error,
                "slow_steps": self.slow}


def main(argv=None):
    a = sys.argv[1:] if argv is None else argv
    if len(a) == 4 and a[0] == "check":
        _, kind, name, path = a
        print(json.dumps(check(kind, name, Path(path).read_text(encoding="utf-8"))))
        return
    print(__doc__)


if __name__ == "__main__":
    import robofield.lab as _lab      # run inside the real module, so the program and the checker share its classes
    _lab.main()
