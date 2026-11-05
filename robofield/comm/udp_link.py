"""Talk to the cars over WiFi (UDP). Cars broadcast status; we learn each car's IP from it and
send commands to that IP (unicast: acknowledged by WiFi, far more reliable than broadcast)."""
from __future__ import annotations

import socket
import time
from dataclasses import dataclass

from .protocol import (STATUS_SIZE, UDP_CMD_PORT, UDP_STATUS_PORT, Command, Status, config_packet)


@dataclass
class CarInfo:
    robot_id: int
    ip: str
    status: Status
    last_seen: float


class CarLink:
    def __init__(self, status_port: int = UDP_STATUS_PORT, cmd_port: int = UDP_CMD_PORT):
        self.cmd_port = cmd_port
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind(("", status_port))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cars: dict[int, CarInfo] = {}
        self._seq: dict[int, int] = {}

    def poll(self) -> dict[int, CarInfo]:
        """Read every status packet waiting; returns the known cars."""
        while True:
            try:
                data, (ip, _) = self.rx.recvfrom(64)
            except (BlockingIOError, InterruptedError):
                break
            if len(data) != STATUS_SIZE:
                continue
            try:
                s = Status.decode(data)
            except ValueError:
                continue
            self.cars[s.robot_id] = CarInfo(s.robot_id, ip, s, time.monotonic())
        return self.cars

    def online(self, max_age: float = 1.0) -> dict[int, CarInfo]:
        now = time.monotonic()
        return {i: c for i, c in self.poll().items() if now - c.last_seen < max_age}

    def send(self, cmd: Command) -> bool:
        car = self.cars.get(cmd.robot_id)
        if car is None:
            return False
        cmd.seq = self._seq.get(cmd.robot_id, 0)
        self._seq[cmd.robot_id] = (cmd.seq + 1) & 0xFFFF
        self.tx.sendto(cmd.encode(), (car.ip, self.cmd_port))
        return True

    def set_id(self, ip: str, new_id: int) -> None:
        self.tx.sendto(config_packet(new_id), (ip, self.cmd_port))

    def stop_all(self) -> None:
        for rid in list(self.cars):
            self.send(Command(rid, enable=False))

    def close(self) -> None:
        self.stop_all()
        self.rx.close(); self.tx.close()
