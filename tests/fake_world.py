"""Software-in-the-loop stand-ins for the real hardware, used by tests and demos:
FakeCar   - speaks the car firmware's UDP protocol (commands in, status out), drives a SimBackend robot
FakeTracker - sends poses from the simulator in the Pi tracker's JSON format."""
import socket
import struct
import threading
import time

from robofield.comm.pose_link import PoseSender
from robofield.comm.protocol import (COMMAND_SIZE, MAGIC, STATUS_SIZE, TYPE_COMMAND, TYPE_CONFIG, Status, crc8)
from robofield.core.state import RobotCommand
from robofield.core.types import Pose


class FakeWorld:
    """One SimBackend plays the physical field; fake cars and a fake tracker wrap it."""

    def __init__(self, sim, cmd_port=4210, status_port=4211, pose_port=5005, host="127.0.0.1"):
        self.sim, self.host = sim, host
        self.cmd_port, self.status_port, self.pose_port = cmd_port, status_port, pose_port
        self.ids = {i: i for i in range(sim.n)}           # car index -> robot id it answers to
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind((host, cmd_port)); self.rx.settimeout(0.001)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.poses = PoseSender(host, pose_port)
        self.last_cmd = {}
        self.seq_ack = {i: 0 for i in range(sim.n)}
        self.running = False

    def _handle(self, data):
        if len(data) == COMMAND_SIZE and data[0] == MAGIC and data[2] == TYPE_COMMAND and crc8(data[:-1]) == data[-1]:
            rid = data[3]; seq, l, r = struct.unpack("<Hhh", data[4:10]); servo = data[10]; flags = data[20]
            for idx, my in self.ids.items():
                if rid in (my, 255):
                    self.seq_ack[idx] = seq
                    en = bool(flags & 1)
                    self.last_cmd[idx] = (time.monotonic(), RobotCommand(l / 1000 if en else 0, r / 1000 if en else 0, servo / 180))
        elif len(data) == 6 and data[0] == MAGIC and data[2] == TYPE_CONFIG and crc8(data[:-1]) == data[-1]:
            self.ids[0] = data[3]                         # config goes to "this IP": the first car

    def _loop(self):
        dt = 1 / self.sim.control_hz
        next_status = 0
        while self.running:
            t0 = time.monotonic()
            try:
                while True:
                    data, _ = self.rx.recvfrom(64); self._handle(data)
            except (socket.timeout, BlockingIOError):
                pass
            cmds = {}
            for idx in range(self.sim.n):                   # firmware watchdog: 200 ms
                t, c = self.last_cmd.get(idx, (0, RobotCommand()))
                cmds[idx] = c if time.monotonic() - t < 0.2 else RobotCommand()
            self.sim.apply(cmds)
            ws = self.sim.step()
            poses = [Pose(10 + i, *self.sim.true_pose(i), ws.t) for i in range(self.sim.n)]
            self.poses.send(poses, time.time(), self.sim.control_hz, True, (500, 500, 1300), 0.4, 20.0)
            if t0 > next_status:
                next_status = t0 + 0.1
                for idx, rid in self.ids.items():
                    s = Status(rid, self.seq_ack[idx], 3950, 2, 0, -50).encode()
                    self.tx.sendto(s, (self.host, self.status_port))
            time.sleep(max(0, dt - (time.monotonic() - t0)))

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True); self.thread.start()
        return self

    def stop(self):
        self.running = False; self.thread.join(1); self.rx.close()
