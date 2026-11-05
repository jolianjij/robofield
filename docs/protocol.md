# Communication protocol (v1)

**Default transport: WiFi UDP.** Cars join the same 2.4 GHz network as the laptop.
- Cars broadcast a status packet to UDP port **4211** at 10 Hz; the laptop learns each car's IP from it.
- The laptop sends command packets to each car's IP on UDP port **4210** (unicast).
- A 6-byte config packet (type 0x03: magic, version, type, new_id, 0, crc) sent to a car's IP sets its id.
- WiFi is also used for wireless firmware updates (ArduinoOTA, port 3232).

**Alternative transport: ESP-NOW** through an ESP32 gateway on the laptop's USB (firmware/gateway),
with the same packets. Laptop ↔ gateway: USB serial, 921600 baud, framed.

All multi-byte fields are **little-endian**. Every packet ends with a CRC-8
(polynomial 0x07, init 0x00) computed over all previous bytes.

## Command packet — laptop → car (22 bytes)

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | magic | u8 | `0xA5` |
| 1 | version | u8 | `1` |
| 2 | type | u8 | `0x01` = command |
| 3 | robot_id | u8 | 0–19 (`255` = all robots) |
| 4 | seq | u16 | increments every packet |
| 6 | left | i16 | -1000..1000 (per-mille of max speed) |
| 8 | right | i16 | -1000..1000 |
| 10 | servo | u8 | gripper angle 0–180° |
| 11 | leds | 9 × u8 | R,G,B for LED 0, 1, 2 |
| 20 | flags | u8 | bit0 = enable motors, bit1 = brake |
| 21 | crc8 | u8 | |

## Status packet — car → laptop (12 bytes)

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | magic | u8 | `0xA5` |
| 1 | version | u8 | `1` |
| 2 | type | u8 | `0x02` = status |
| 3 | robot_id | u8 | |
| 4 | seq_ack | u16 | last command seq received |
| 6 | battery_mv | u16 | cell voltage in mV |
| 8 | fw_version | u8 | |
| 9 | status | u8 | bit0 = watchdog tripped, bit1 = low battery |
| 10 | rssi | i8 | dBm of last received command |
| 11 | crc8 | u8 | |

Cars send status at 10 Hz.

## Serial framing (laptop ↔ gateway)

```
0xAA 0x55 <len:u8> <packet bytes...>
```

The gateway forwards the packet bytes unchanged in both directions.

## Safety rules (firmware must implement)

- **Watchdog:** no valid command for 200 ms → motors off (STBY low), status bit0 set.
- **Low battery:** cell below 3.3 V → motors off, status bit1 set, LEDs blink red.
- Packets with a bad magic, version or CRC are dropped silently.
