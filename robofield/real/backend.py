"""RealBackend: the same Backend interface as the simulator, on the real field.

Poses come from pi/pi_tracker.py (UDP, robofield.comm.pose_link), are filtered by an EKF per robot,
and commands go to the cars over WiFi (robofield.comm.udp_link). step() paces the loop at control_hz.
"""
from __future__ import annotations

import time

from robofield.comm.pose_link import PoseReceiver
from robofield.comm.protocol import Command
from robofield.comm.udp_link import CarLink
from robofield.control.estimator import RobotEKF
from robofield.core.backend import Backend
from robofield.core.state import ObjectState, RobotCommand, RobotState, WorldState


class RealBackend(Backend):
    def __init__(self, cfg: dict, robot_ids: list[int] | None = None, predict_latency: bool = True,
                 pose_port: int | None = None):
        self.cfg = cfg
        self.control_hz = float(cfg["control"]["rate_hz"])
        self.latency = cfg["control"]["latency_s"] if predict_latency else 0.0
        self.offset = cfg["markers"]["robot_id_offset"]
        self.robot_ids = robot_ids
        # per-robot: this specific car's physical left/right wheels may be wired swapped
        # relative to what DiffDrive/the simulator call "left" and "right" (see config/field.yaml
        # robots[].wheels_swapped) - swapped at the radio boundary only, in _send() below, so the
        # kinematics model stays correct and sim-matching for every other robot and for sim-to-real RL.
        self._wheels_swapped = {r["id"]: bool(r.get("wheels_swapped", False)) for r in cfg.get("robots", [])}
        self.poses = PoseReceiver(pose_port) if pose_port else PoseReceiver()
        self.cars = CarLink()
        self.ekf: dict[int, RobotEKF] = {}
        self._cmd: dict[int, RobotCommand] = {}
        self._next = time.monotonic()
        self._state = WorldState()

    def reset(self, layout=None) -> WorldState:
        """Robots cannot be teleported: stop them and return the current state."""
        self.stop()
        time.sleep(0.2)
        return self.step()

    def stop(self) -> None:
        self._cmd = {rid: RobotCommand() for rid in set(self._cmd) | set(self.cars.cars)}

    def apply(self, commands: dict[int, RobotCommand]) -> None:
        self._cmd.update({rid: c.clipped() for rid, c in commands.items()})

    def _battery_gain(self, rid: int) -> float:
        """Scale commands by nominal / measured pack voltage so speed stays constant as it drains."""
        b = self.cfg.get("battery", {})
        car = self.cars.cars.get(rid)
        if not b.get("compensate") or car is None or car.status.battery_mv < 1000:
            return 1.0
        return min(1.3, max(0.8, b.get("nominal_mv", 7400) / car.status.battery_mv))

    def _send(self):
        for rid, c in self._cmd.items():
            k = self._battery_gain(rid)
            left, right = max(-1, min(1, c.left * k)), max(-1, min(1, c.right * k))
            if self._wheels_swapped.get(rid, False):
                left, right = right, left
            self.cars.send(Command(rid, left=int(left * 1000), right=int(right * 1000),
                                   servo=int(c.gripper * 180), leds=list(c.leds)))

    def step(self) -> WorldState:
        self._next += 1.0 / self.control_hz
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()                # fell behind: do not try to catch up
        self.cars.poll()
        self._send()
        msg = self.poses.poll()
        now = time.monotonic()
        ws = WorldState(t=now)
        seen = set()
        fresh = msg is not None and now - self.poses.received_at < 0.5
        if fresh:
            for mid, x, y, th, q in msg["markers"]:
                if mid >= 30:
                    ws.objects[mid] = ObjectState(mid, "object", x, y, th, True, now)
                    continue
                rid = mid - self.offset
                if rid < 0 or (self.robot_ids is not None and rid not in self.robot_ids):
                    continue
                self.ekf.setdefault(rid, RobotEKF()).update(now, x, y, th)
                seen.add(rid)
        for rid, f in self.ekf.items():
            if rid not in seen:
                f.coast(now)
            x, y, th, v, w = f.predicted(self.latency) if self.latency else tuple(f.x)
            car = self.cars.cars.get(rid)
            ws.robots[rid] = RobotState(rid, x, y, th, v, w, rid in seen, car.status.battery_mv if car else 0, now)
        self._state = ws
        return ws

    def state(self) -> WorldState:
        return self._state

    def close(self):
        self.cars.close()
        self.poses.close()
