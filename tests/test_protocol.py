from robofield.comm.protocol import (COMMAND_SIZE, Command, FrameReader, Status, crc8, frame)


def test_crc8_known_value():
    assert crc8(b"123456789") == 0xF4   # standard CRC-8 check value


def test_command_size_and_clamping():
    pkt = Command(robot_id=2, seq=7, left=5000, right=-5000, servo=300).encode()
    assert len(pkt) == COMMAND_SIZE == 22
    assert int.from_bytes(pkt[6:8], "little", signed=True) == 1000
    assert int.from_bytes(pkt[8:10], "little", signed=True) == -1000
    assert pkt[10] == 180
    assert crc8(pkt[:-1]) == pkt[-1]


def test_status_roundtrip():
    s = Status(robot_id=3, seq_ack=99, battery_mv=3950, fw_version=1, status=0b10, rssi=-55)
    d = Status.decode(s.encode())
    assert d == s and d.low_battery and not d.watchdog_tripped


def test_frame_reader_handles_split_and_noise():
    a, b = Command(0, 1).encode(), Command(1, 2).encode()
    stream = b"\x00\x13" + frame(a) + frame(b)
    r = FrameReader()
    out = r.feed(stream[:10]) + r.feed(stream[10:])
    assert out == [a, b]
