"""The RoboField app's runtime: one Control Center that can be switched between the simulator and
the real field without restarting, plus background jobs (training, evaluation, tools) and the list
of trained policies. tools/robofield_app.py starts it; robofield/app/server.py serves it."""
from __future__ import annotations

import json
import os
import re
import socket
import threading
import time
from pathlib import Path

from .control_center import ControlCenter
from .jobs import JobManager

ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "models"
_CHECKPOINT = re.compile(r"_\d+_steps$")

# Tools the app can run. exclusive: talks to the cars / the Pi itself, so the app must not be on
# the real field at the same time. moves: drives a car (the page asks first). {robot}, {pi},
# {url} are filled in from the page and the app.
TOOLS = [
    {"id": "cars_list", "title": "Find cars", "args": ["tools/cars.py", "list"], "exclusive": True,
     "moves": False, "needs": "cars",
     "about": "Every car broadcasting on the WiFi: id, IP address, signal, battery, firmware version."},
    {"id": "cars_test", "title": "Car self-test", "args": ["tools/cars.py", "test", "{robot}"], "exclusive": True,
     "moves": True, "needs": "cars",
     "about": "Forward, back, spin, gripper and LEDs at 40% speed. Lift the car off the table the first time."},
    {"id": "latency", "title": "Measure loop delay", "args": ["tools/measure_latency.py", "--robot", "{robot}"],
     "exclusive": True, "moves": True, "needs": "field",
     "about": "Short turns in place, timed from command to camera. Put the result in control.latency_s."},
    {"id": "motors", "title": "Calibrate motors", "args": ["tools/calibrate_motors.py", "--robot", "{robot}"],
     "exclusive": True, "moves": True, "needs": "field", "write_flag": "--write",
     "about": "Spins and drives straight to measure top speed and left/right trim. Car mid-field, facing the centre."},
    {"id": "eval_motion", "title": "Score motion", "args": ["tools/eval_motion.py", "--robot", "{robot}", "--url", "{url}"],
     "exclusive": False, "moves": True, "needs": None,
     "about": "Straight line, 120° turn and half circle through this app: time, path error, stop error, wobbles."},
    {"id": "record_camera", "title": "Record the camera", "args": ["tools/record_camera.py", "--pi", "{pi}", "--seconds", "{seconds}"],
     "exclusive": False, "moves": False, "needs": "pi",
     "about": "The Pi camera with its tracking overlay, to recordings/ as MP4."},
    {"id": "tests", "title": "Run the test suite", "args": ["-m", "pytest", "-q"], "exclusive": False,
     "moves": False, "needs": None, "about": "All automated tests, no hardware needed."},
    {"id": "markers", "title": "Generate markers", "args": ["tools/generate_markers.py", "--out", "markers/", "--dpi", "300"],
     "exclusive": False, "moves": False, "needs": None,
     "about": "Printable ArUco markers at exact size, in markers/. Print at 100% scale."},
]


def discover_pi(timeout: float = 3.0) -> str | None:
    """The Pi tracker broadcasts poses on UDP 5005: its address is the sender's."""
    from robofield.comm.pose_link import POSE_PORT
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", POSE_PORT))
        s.settimeout(timeout)
        _, (ip, _) = s.recvfrom(65536)
        return ip
    except (socket.timeout, OSError):
        return None
    finally:
        s.close()


class AppRuntime:
    def __init__(self, cfg: dict, mode: str = "sim", pi: str | None = None, sim_robots: int = 2, port: int = 8000):
        self.cfg, self.port = cfg, port
        self.jobs = JobManager()
        self.lock = threading.Lock()
        self.mode, self.pi, self.sim_robots = mode, pi, sim_robots
        self.status = "starting"
        self.error = ""
        self.cc: ControlCenter | None = None
        self.cc = self._make(mode, pi, sim_robots)
        self.status = "ready"

    # ------------------------------------------------------------------ simulator / real field
    def _make(self, mode: str, pi: str | None, sim_robots: int) -> ControlCenter:
        if mode == "real":
            from robofield.real import RealBackend
            backend = RealBackend(self.cfg)
        else:
            from robofield.sim.backend import SimBackend
            backend = SimBackend(self.cfg, n_robots=sim_robots, seed=0, randomize=True, markers=False)
        return ControlCenter(self.cfg, backend, mode, pi_host=pi).start()

    def switch(self, mode: str, pi: str | None = None, sim_robots: int | None = None) -> tuple[bool, str]:
        if mode not in ("sim", "real"):
            return False, "mode is sim or real"
        if mode == "real":
            busy = [j.title for j in self.jobs.running("tool") if j.meta.get("exclusive")]
            if busy:
                return False, f"wait for '{busy[0]}' to finish: it is using the cars"
        with self.lock:
            self.status = "switching"
            old = self.cc
            tuning = dict(old.tuning) if old else None
            if old:
                old.close()                               # stops the cars, frees the network ports
            if mode == "real" and not pi:
                pi = discover_pi(3.0)
            n = int(sim_robots or self.sim_robots)
            try:
                self.cc = self._make(mode, pi, n)
                self.mode, self.pi, self.sim_robots, self.error = mode, pi, n, ""
                msg = "simulator" if mode == "sim" else (f"real field, Pi at {pi}" if pi else
                                                         "real field (no Pi tracker heard on the network)")
            except Exception as e:                        # e.g. a port in use: fall back to the simulator
                self.error = f"could not start the {mode} mode: {e}"
                self.cc = self._make("sim", None, n)
                self.mode, self.pi = "sim", None
                msg = self.error
            if tuning:
                self.cc.set_tuning(**tuning)
            self.cc._log(f"app: {msg}")
            self.status = "ready"
            return not self.error, msg

    def info(self) -> dict:
        return {"mode": self.mode, "pi": self.pi, "sim_robots": self.sim_robots, "status": self.status,
                "error": self.error, "port": self.port, "cpus": os.cpu_count() or 4}

    # ------------------------------------------------------------------ policies and games
    @staticmethod
    def games() -> list[dict]:
        from robofield.games import REGISTRY
        return [{"name": n, "cars": c.n_robots, "trainable": c.n_robots == 1,
                 "about": (c.__doc__ or "").strip().split("\n")[0]} for n, c in sorted(REGISTRY.items())]

    @staticmethod
    def models() -> list[dict]:
        from .policy import game_for
        out = []
        for z in sorted(MODELS.glob("*.zip"), key=lambda p: -p.stat().st_mtime) if MODELS.exists() else []:
            if _CHECKPOINT.search(z.stem):
                continue
            meta_path = z.with_suffix(".json")
            meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
            try:
                game = meta.get("game") or game_for(z)
            except ValueError:
                game = None
            out.append({"name": z.stem, "file": z.name, "game": game, "steps": meta.get("timesteps_done"),
                        "status": meta.get("status", "saved"), "ep_rew_mean": meta.get("ep_rew_mean"),
                        "success": meta.get("success"), "seconds": meta.get("seconds"),
                        "modified": time.strftime("%Y-%m-%d %H:%M", time.localtime(z.stat().st_mtime)),
                        "size_kb": round(z.stat().st_size / 1024)})
        return out

    @staticmethod
    def curve(name: str) -> list[dict]:
        p = MODELS / f"{name}.progress.jsonl"
        if not p.exists():
            return []
        return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]

    def start_training(self, game: str, timesteps: int, envs: int, name: str | None = None,
                       randomize: bool = True, resume: str | None = None) -> tuple[bool, str]:
        if self.jobs.running("train"):
            return False, "a training run is already going: stop it first"
        games = {g["name"]: g for g in self.games()}
        if game not in games or not games[game]["trainable"]:
            return False, f"'{game}' cannot be trained here yet (one-car games only)"
        name = name or f"{game}_{time.strftime('%Y%m%d_%H%M%S')}"
        name = "".join(c for c in name.strip().replace(" ", "_") if c.isalnum() or c in "_-")
        if not name:
            return False, "give the policy a name"
        if (MODELS / f"{name}.zip").exists() and not resume:
            return False, f"models/{name}.zip already exists: choose another name"
        args = ["tools/train.py", "--game", game, "--name", name, "--timesteps", str(int(timesteps)),
                "--envs", str(max(1, int(envs)))]
        if not randomize:
            args.append("--no-randomize")
        if resume:
            args += ["--resume", str(MODELS / f"{resume}.zip")]
        job = self.jobs.start("train", f"Train {name} ({game})", args, stop_file=MODELS / f"{name}.stop",
                              meta={"name": name, "game": game, "timesteps": int(timesteps)})
        return True, str(job.id)

    def start_eval(self, model: str, episodes: int = 10, randomize: bool = False) -> tuple[bool, str]:
        path = MODELS / f"{model}.zip"
        if not path.exists():
            return False, f"no policy {model}"
        args = ["tools/train.py", "--eval", str(path), "--episodes", str(int(episodes))]
        if randomize:
            args.append("--randomize-eval")
        job = self.jobs.start("eval", f"Evaluate {model}", args, meta={"name": model})
        return True, str(job.id)

    def start_tool(self, tool_id: str, robot: int = 0, seconds: int = 60, write: bool = False) -> tuple[bool, str]:
        tool = next((t for t in TOOLS if t["id"] == tool_id), None)
        if tool is None:
            return False, "no such tool"
        if tool["exclusive"] and self.mode == "real":
            return False, "switch the app to the simulator first: this tool talks to the cars itself"
        if tool["needs"] == "pi" and not self.pi:
            return False, "no Pi address: switch to the real field (it finds the Pi), or enter it"
        if any(j.meta.get("exclusive") for j in self.jobs.running("tool")) and tool["exclusive"]:
            return False, "another tool is using the cars"
        fill = {"robot": str(int(robot)), "pi": self.pi or "", "seconds": str(int(seconds)),
                "url": f"http://127.0.0.1:{self.port}"}
        args = [a.format(**fill) for a in tool["args"]]
        if write and tool.get("write_flag"):
            args.append(tool["write_flag"])
        job = self.jobs.start("tool", tool["title"], args, meta={"tool": tool_id, "exclusive": tool["exclusive"]})
        return True, str(job.id)

    def close(self):
        self.jobs.close_all()
        if self.cc:
            self.cc.close()
