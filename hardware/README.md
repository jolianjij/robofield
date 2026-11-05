# Car hardware (reference build v1)

Footprint: 70 × 50 mm. Tank drive.

## Parts list (per car)

| Part | Spec | Qty |
|---|---|---|
| Controller | ESP32-C3 Super Mini | 1 |
| Motors | N20 gear motor, 12V, 200 rpm | 2 |
| Motor driver | TB6612FNG module | 1 |
| Battery | 18650 cell, 3.5 Ah, high-drain | 1 |
| Battery protection | 1S protection board | 1 |
| Charging (optional) | 1S USB-C charge module | 1 |
| Boost converter | TPS61088 module, set to 12.0 V | 1 |
| Buck converter | 5 V, 2 A+ (e.g. Mini560) | 1 |
| Gripper servo | SG90 / MG90S | 1 |
| LEDs | WS2812B | 3 |
| Capacitors | 470 µF (boost out, servo), 100 µF (TB6612 VM), 100 nF (ADC) | 1 each |
| Resistors | 100 kΩ × 2 (battery divider), 330 Ω (LED data) | — |
| Switch, polyfuse (2 A) | | 1 each |
| ArUco marker | 35 mm, ID 10 + robot number | 1 |

## Power path

```
18650 ─ protection ─ switch ─ fuse ─┬─ boost 12V ─┬─ TB6612 VM (+100 µF)
                                    │             └─ buck 5V ─┬─ ESP32 5V pin
                                    │                         ├─ servo (+470 µF)
                                    │                         └─ LEDs
                                    └─ 100k/100k divider ─ GPIO0
ESP32 3V3 ─ TB6612 VCC        All grounds common.
```

## Pin map (ESP32-C3 Super Mini)

| GPIO | Function |
|---|---|
| 1 | PWMA (left speed) |
| 3 / 4 | AIN1 / AIN2 |
| 5 | PWMB (right speed) |
| 6 / 7 | BIN1 / BIN2 |
| 21 | STBY |
| 20 | Servo |
| 10 | LED data (via 330 Ω) |
| 0 | Battery voltage (divider) |

Avoid GPIO 2, 8, 9 (strapping pins).

## First power-up checklist

1. Set the boost to 12.0 V and the buck to 5.0 V with a multimeter **before** connecting loads.
2. Flash `firmware/car`, check the serial log shows "ready".
3. Run `examples/motor_test.py` with the car lifted off the table.
4. Stress test: both motors + servo at full load — the ESP32 must not reset.

Car CAD/STL files: `hardware/cad/` (coming in Phase 1).
Field tiles and pins: `hardware/field/` — see `docs/field.md`.
