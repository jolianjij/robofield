# Hardware

| Folder | Contents |
|---|---|
| [`car/`](car/README.md) | The full CAD assembly of the car |
| [`field/`](field/README.md) | The full CAD assembly of the field: floor, walls, towers, camera mast |

The numbers the software needs from the hardware (field size, tower positions and heights, the
car's track, wheel size and marker height) live in `config/field.yaml`; see [field](../docs/field.md)
and [kinematics](../docs/kinematics.md).

## The car

A small differential-drive robot: two driven wheels near the middle, a ball caster at the front,
and a 40 mm ArUco marker on top, centred over the axle. It has no sensors; the overhead camera is
its only sense.

| | |
|---|---|
| Footprint | 70 × 50 mm |
| Marker height | 84 mm above the floor |
| Wheel track | 56 mm (wheel centre to wheel centre) |
| Wheel diameter | 33 mm |
| Mass | about 220 g |
| Top speed | 340 mm/s measured; slowest steady speed about 100 mm/s |

### Parts (per car)

| Part | Notes |
|---|---|
| Wemos D1 Mini (ESP8266) | WiFi, runs `firmware/car_esp8266/` |
| DRV8833 dual motor driver | Two inputs per motor; has its own current limit and thermal shutdown |
| 2 × N20 gear motors with 33 mm wheels | |
| Ball caster | At the front |
| 2S Li-ion / LiPo pack, 7.4–8.4 V | Straight to the DRV8833's motor supply (VM, 10.8 V max) |
| 5 V buck converter | Powers the D1 Mini, the servo and the LEDs |
| Micro servo | Gripper |
| 3 × WS2812B RGB LEDs | Team colours, identify blink, e-stop red |
| 100 kΩ + 33 kΩ resistors | Battery voltage divider into A0 |
| ArUco marker, 40 mm, matte | ID 10 + robot number |

### Wiring

| Function | D1 Mini pin | Notes |
|---|---|---|
| Right motor IN1 / IN2 | D5 / D6 | DRV8833 |
| Left motor IN1 / IN2 | D7 / D3 | D3 is GPIO0, a boot pin: it works, but add nothing else to it |
| Gripper servo | D2 | Shares the on-board LED, which flickers with the servo (harmless) |
| WS2812B data | D1 | |
| Battery sense | A0 | 100k / 33k divider; reading switched off in the firmware for now |

```
2S pack ─ switch ─┬─ DRV8833 VM ── motors
                  ├─ 5 V buck ─┬─ D1 Mini 5V
                  │            ├─ servo
                  │            └─ LEDs
                  └─ 100k / 33k divider ─ A0
All grounds common.
```

### Lessons from the first build

- The first controller, an ESP32-C3, burned out, most likely from 5 V reaching a GPIO pin. Check
  every supply line with a multimeter before plugging the board in.
- The first motor driver, an L9110S, burned out from the current spike of reversing a motor at
  full speed. The DRV8833 handles it with its own current limiting. If a driver burns out again,
  re-enable the short pause before reversing in the firmware (`REVERSE_DEAD_MS`).
- Lift the car off the table for its first motor test (`python tools/cars.py test 0`).

## The field

A 1 × 1 m floor of 4 × 4 tiles with 100 mm walls, four tracking towers just outside the corners
(markers at 140–260 mm), and a mast holding the Raspberry Pi 5 and its Camera Module 3 Wide about
1.3 m above the centre. Everything the software needs is listed in [field](../docs/field.md).
