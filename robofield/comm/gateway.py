"""Talk to the ESP32 gateway over USB serial."""
from __future__ import annotations

import serial

from .protocol import STATUS_SIZE, Command, FrameReader, Status, frame


class Gateway:
    def __init__(self, port: str, baud: int = 921600) -> None:
        self.ser = serial.Serial(port, baud, timeout=0)
        self.reader = FrameReader()
        self.status: dict[int, Status] = {}   # latest status per robot
        self._seq: dict[int, int] = {}

    def send(self, cmd: Command) -> None:
        cmd.seq = self._seq.get(cmd.robot_id, 0)
        self._seq[cmd.robot_id] = (cmd.seq + 1) & 0xFFFF
        self.ser.write(frame(cmd.encode()))

    def poll(self) -> dict[int, Status]:
        """Read whatever arrived; returns the latest status per robot."""
        for pkt in self.reader.feed(self.ser.read(self.ser.in_waiting or 0)):
            if len(pkt) == STATUS_SIZE:
                try:
                    s = Status.decode(pkt)
                    self.status[s.robot_id] = s
                except ValueError:
                    pass
        return self.status

    def stop_all(self) -> None:
        self.ser.write(frame(Command(robot_id=255, enable=False).encode()))

    def close(self) -> None:
        self.stop_all()
        self.ser.close()
