"""Packet encoding/decoding. Must match docs/protocol.md and firmware/common/protocol.h."""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

MAGIC = 0xA5
VERSION = 1
TYPE_COMMAND = 0x01
TYPE_STATUS = 0x02
TYPE_CONFIG = 0x03
UDP_CMD_PORT = 4210          # cars listen here
UDP_STATUS_PORT = 4211       # cars broadcast status here
BROADCAST_ID = 255

FLAG_ENABLE = 0x01
FLAG_BRAKE = 0x02
STATUS_WATCHDOG = 0x01
STATUS_LOW_BATTERY = 0x02

_CMD = struct.Struct("<BBBBHhhB9sB")   # without crc -> 21 bytes
_STATUS = struct.Struct("<BBBBHHBBb")  # without crc -> 11 bytes
COMMAND_SIZE = _CMD.size + 1
STATUS_SIZE = _STATUS.size + 1

FRAME_HEADER = b"\xAA\x55"


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07, init 0x00."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


@dataclass
class Command:
    robot_id: int
    seq: int = 0
    left: int = 0          # -1000..1000
    right: int = 0         # -1000..1000
    servo: int = 90        # 0..180
    leds: list[tuple[int, int, int]] = field(default_factory=lambda: [(0, 0, 0)] * 3)
    enable: bool = True
    brake: bool = False

    def encode(self) -> bytes:
        led_bytes = bytes(_clamp(c, 0, 255) for rgb in self.leds[:3] for c in rgb)
        led_bytes = led_bytes.ljust(9, b"\x00")
        flags = (FLAG_ENABLE if self.enable else 0) | (FLAG_BRAKE if self.brake else 0)
        body = _CMD.pack(
            MAGIC, VERSION, TYPE_COMMAND, self.robot_id & 0xFF, self.seq & 0xFFFF,
            _clamp(self.left, -1000, 1000), _clamp(self.right, -1000, 1000),
            _clamp(self.servo, 0, 180), led_bytes, flags,
        )
        return body + bytes([crc8(body)])


@dataclass
class Status:
    robot_id: int
    seq_ack: int
    battery_mv: int
    fw_version: int
    status: int
    rssi: int

    @property
    def watchdog_tripped(self) -> bool:
        return bool(self.status & STATUS_WATCHDOG)

    @property
    def low_battery(self) -> bool:
        return bool(self.status & STATUS_LOW_BATTERY)

    @classmethod
    def decode(cls, data: bytes) -> "Status":
        if len(data) != STATUS_SIZE:
            raise ValueError(f"status packet must be {STATUS_SIZE} bytes, got {len(data)}")
        if crc8(data[:-1]) != data[-1]:
            raise ValueError("bad CRC")
        magic, ver, typ, rid, seq_ack, mv, fw, st, rssi = _STATUS.unpack(data[:-1])
        if magic != MAGIC or ver != VERSION or typ != TYPE_STATUS:
            raise ValueError("not a v1 status packet")
        return cls(rid, seq_ack, mv, fw, st, rssi)

    def encode(self) -> bytes:
        """Used by the simulator and tests to fake a car."""
        body = _STATUS.pack(MAGIC, VERSION, TYPE_STATUS, self.robot_id, self.seq_ack,
                            self.battery_mv, self.fw_version, self.status, self.rssi)
        return body + bytes([crc8(body)])


def frame(packet: bytes) -> bytes:
    """Wrap a packet for the USB serial link to the gateway."""
    return FRAME_HEADER + bytes([len(packet)]) + packet


class FrameReader:
    """Incremental parser for the serial stream coming back from the gateway."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        packets = []
        while True:
            i = self._buf.find(FRAME_HEADER)
            if i < 0:
                del self._buf[:-1]  # keep a possible half header
                return packets
            del self._buf[:i]
            if len(self._buf) < 3:
                return packets
            n = self._buf[2]
            if len(self._buf) < 3 + n:
                return packets
            packets.append(bytes(self._buf[3:3 + n]))
            del self._buf[:3 + n]


def config_packet(new_id: int) -> bytes:
    """Set a car's robot id (send to that car's IP)."""
    body = bytes([MAGIC, VERSION, TYPE_CONFIG, new_id & 0xFF, 0])
    return body + bytes([crc8(body)])
