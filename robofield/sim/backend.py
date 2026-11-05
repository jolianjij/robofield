"""MuJoCo implementation of the Backend interface.

Models what the real system adds on top of physics, so policies learn to cope with it:
  - command latency (camera + processing + radio), as a delay line
  - DC motor torque-speed curve (in the MJCF), dead zone, per-wheel gain mismatch, firmware duty ramp
  - tracking noise on the reported poses
Domain randomization (cfg sim.randomize) re-draws these, plus friction and mass, every reset.
"""
from __future__ import annotations

import math
from collections import deque

import mujoco
import numpy as np

from robofield.core.backend import Backend
from robofield.core.state import ObjectState, RobotCommand, RobotState, WorldState
from robofield.core.types import wrap_angle
from robofield.control.kinematics import DiffDrive
from .scene import MM, build_scene


class SimBackend(Backend):
    def __init__(self, cfg: dict, n_robots: int = 1, objects: tuple = (), seed: int = 0,
                 randomize: bool | None = None, markers: bool = True):
        self.cfg = cfg
        self.n = n_robots
        self.objects_spec = tuple(objects)
        self.rng = np.random.default_rng(seed)
        sim = cfg.get("sim", {})
        self.randomize = sim.get("randomize", False) if randomize is None else randomize
        self.control_hz = float(cfg["control"]["rate_hz"])
        self.model = mujoco.MjModel.from_xml_string(build_scene(cfg, n_robots, objects, markers))
        self.data = mujoco.MjData(self.model)
        self.substeps = max(1, round(1 / (self.control_hz * self.model.opt.timestep)))
        self.dd = DiffDrive.from_config(cfg)
        m = cfg["robot_model"]
        self.omega_max = m["max_wheel_rpm"] * 2 * math.pi / 60          # rad/s at the wheel
        # firmware duty ramp: duty may change by this much per second (accel limit / top speed)
        self.duty_rate = m.get("wheel_accel_limit_mm_s2", 1000) / self.dd.max_wheel_speed
        self.nominal = dict(latency=cfg["control"]["latency_s"], deadzone=m["deadzone"],
                            noise_xy=sim.get("pose_noise_mm", 0.8),
                            noise_th=sim.get("heading_noise_rad", 0.01))
        self._free = [self.model.joint(f"r{i}_free").qposadr[0] for i in range(n_robots)]
        self._dof = [self.model.joint(f"r{i}_free").dofadr[0] for i in range(n_robots)]
        self._act = [(self.model.actuator(f"r{i}_left_motor").id, self.model.actuator(f"r{i}_right_motor").id)
                     for i in range(n_robots)]
        self._obj = {oid: (kind, self.model.body(f"obj{oid}").jntadr[0]) for kind, oid in self.objects_spec}
        self._wheel_geoms = [g for g in range(self.model.ngeom) if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER]
        self._base_friction = self.model.geom_friction.copy()
        self._base_mass = self.model.body_mass.copy()
        self._renderer = None
        self.reset()

    # ---------------------------------------------------------------- episode
    def _draw_params(self):
        p = dict(self.nominal)
        p["gain"] = np.ones((self.n, 2))
        self.model.geom_friction[:] = self._base_friction
        self.model.body_mass[:] = self._base_mass
        if self.randomize:
            r = self.rng
            p["latency"] = r.uniform(0.04, 0.12)
            p["deadzone"] = r.uniform(0.04, 0.12)
            p["gain"] = r.uniform(0.88, 1.12, size=(self.n, 2))          # unmatched motors: the car drifts
            p["noise_xy"] = r.uniform(0.3, 2.0)
            for g in self._wheel_geoms:
                self.model.geom_friction[g, 0] = self._base_friction[g, 0] * r.uniform(0.6, 1.3)
            for i in range(self.n):
                b = self.model.body(f"robot{i}").id
                self.model.body_mass[b] = self._base_mass[b] * r.uniform(0.9, 1.25)
        self.p = p
        self._delay = max(0, round(p["latency"] * self.control_hz))

    def _random_layout(self):
        poses, tries = {}, 0
        while len(poses) < self.n and tries < 1000:
            tries += 1
            x, y = self.rng.uniform(100, 900, 2)
            if all(math.hypot(x - px, y - py) > 120 for px, py, _ in poses.values()):
                poses[len(poses)] = (x, y, self.rng.uniform(-math.pi, math.pi))
        return {"robots": poses, "objects": {}}

    def reset(self, layout: dict | None = None) -> WorldState:
        self._draw_params()
        mujoco.mj_resetData(self.model, self.data)
        layout = layout or self._random_layout()
        r = self.cfg["robot_model"]["wheel_radius_mm"] * MM
        for i in range(self.n):
            x, y, th = layout.get("robots", {}).get(i, (200 + 100 * i, 500, 0.0))
            a = self._free[i]
            self.data.qpos[a:a + 3] = (x * MM, y * MM, r)
            self.data.qpos[a + 3:a + 7] = (math.cos(th / 2), 0, 0, math.sin(th / 2))
        for oid, (kind, adr) in self._obj.items():
            x, y = layout.get("objects", {}).get(oid, (500, 500))
            self.data.qpos[adr:adr + 3] = (x * MM, y * MM, 0.0205 if kind == "box" else 0.02)
            self.data.qpos[adr + 3:adr + 7] = (1, 0, 0, 0)
        mujoco.mj_forward(self.model, self.data)
        self._cmd = {i: RobotCommand() for i in range(self.n)}
        self._queue = deque([dict(self._cmd)] * (self._delay + 1), maxlen=self._delay + 1)
        self._ramp = np.zeros((self.n, 2))
        for _ in range(int(0.05 / self.model.opt.timestep)):     # settle
            mujoco.mj_step(self.model, self.data)
        self._t = 0.0
        self._state = self._observe()
        return self._state

    # ---------------------------------------------------------------- stepping
    def apply(self, commands: dict[int, RobotCommand]) -> None:
        for rid, c in commands.items():
            if 0 <= rid < self.n:
                self._cmd[rid] = c.clipped()

    def _duty(self, u: float, gain: float) -> float:
        """Command -> motor duty: dead zone (the motor does not move below it) and gain mismatch."""
        dz = self.p["deadzone"]
        if abs(u) < dz:
            return 0.0
        return max(-1.0, min(1.0, u * gain))

    def step(self) -> WorldState:
        self._queue.append(dict(self._cmd))
        active = self._queue[0]                                  # the command from `latency` ago
        dt = self.model.opt.timestep
        targets = np.array([[self._duty(active[i].left, self.p["gain"][i, 0]),
                             self._duty(active[i].right, self.p["gain"][i, 1])] for i in range(self.n)])
        max_step = self.duty_rate * dt
        for _ in range(self.substeps):
            self._ramp += np.clip(targets - self._ramp, -max_step, max_step)      # firmware ramp
            for i, (al, ar) in enumerate(self._act):
                self.data.ctrl[al] = self._ramp[i, 0]
                self.data.ctrl[ar] = self._ramp[i, 1]
            mujoco.mj_step(self.model, self.data)
        self._t += self.substeps * dt
        self._state = self._observe()
        return self._state

    def state(self) -> WorldState:
        return self._state

    # ---------------------------------------------------------------- observation
    def true_pose(self, i: int) -> tuple[float, float, float]:
        a = self._free[i]
        x, y = self.data.qpos[a] / MM, self.data.qpos[a + 1] / MM
        w, qx, qy, qz = self.data.qpos[a + 3:a + 7]
        th = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return x, y, th

    def _observe(self) -> WorldState:
        ws = WorldState(t=self._t)
        nx, nt = self.p["noise_xy"], self.p["noise_th"]
        for i in range(self.n):
            x, y, th = self.true_pose(i)
            dof = self._dof[i]
            vx, vy = self.data.qvel[dof] / MM, self.data.qvel[dof + 1] / MM
            v = vx * math.cos(th) + vy * math.sin(th)
            ws.robots[i] = RobotState(i, x + self.rng.normal(0, nx), y + self.rng.normal(0, nx),
                                      wrap_angle(th + self.rng.normal(0, nt)), v, self.data.qvel[dof + 5], True, 11100, self._t)
        for oid, (kind, adr) in self._obj.items():
            ws.objects[oid] = ObjectState(oid, kind, self.data.qpos[adr] / MM, self.data.qpos[adr + 1] / MM, 0.0, True, self._t)
        return ws

    def render(self, width: int | None = None, height: int | None = None, camera: str = "overhead") -> np.ndarray:
        w = width or self.cfg["camera"]["width"]
        h = height or self.cfg["camera"]["height"]
        if self._renderer is None or self._renderer.width != w or self._renderer.height != h:
            self._renderer = mujoco.Renderer(self.model, h, w)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
