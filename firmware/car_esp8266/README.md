# Car firmware — Wemos D1 Mini (ESP8266) + DRV8833

This is the firmware the cars actually run (`fw 5`). It replaced the ESP32-C3 / PlatformIO
version in `firmware/car/` after the first ESP32-C3 and the first L9110S driver burned out during
bring-up; the radio protocol is identical, so the laptop side does not care which one a car runs.

The car is deliberately "dumb": it receives wheel commands over WiFi (UDP), drives the motors,
gripper servo and three WS2812 LEDs, and reports its status. All sensing (the overhead camera) and
all decisions (the laptop) live outside it.

## Wiring (config.h)

| Function | Pin | Notes |
|---|---|---|
| Right motor IN1 / IN2 | D5 / D6 | DRV8833, 2 inputs per motor (`DRIVER_DUAL_PWM`) |
| Left motor IN1 / IN2 | D7 / D3 | D3 is GPIO0 (boot strap): works, don't add load |
| Gripper servo | D2 | shares the on-board LED, which flickers with it |
| 3x WS2812B | D1 | |
| Battery sense | A0 | 100k/33k divider, monitor disabled (`ENABLE_BATTERY_MONITOR false`) |

Power: 2S Li-ion (7.4–8.4 V) straight to the DRV8833's VM, 5 V buck for the D1 Mini, servo and LEDs.

## Behaviour worth knowing

- **Dead zone:** every non-zero command is lifted to `DEADZONE + (1 - DEADZONE) * |u|` per wheel
  (`LEFT_DEADZONE 0.26`, `RIGHT_DEADZONE 0.20`), so the laptop must not compensate again
  (`robot_model.deadzone_* = 0` in `config/field.yaml`). Measured on 2026-11-06, these floors sit
  above where the motors really start: the smallest command already drives ~100 mm/s, and the
  unequal floors make the car curve at low speed. **Recommended next firmware change:** lower both,
  re-run `tools/calibrate_motors.py --write`, and the laptop gets much finer low-speed control.
- **Duty ramp:** `DUTY_RAMP_PER_S 6.0` — full power in ~0.17 s, which stops it popping a wheelie.
- **Watchdog:** motors off if no command for `WATCHDOG_MS 200`. The laptop sends at 30 Hz.
- **Reverse dead-time:** `REVERSE_DEAD_MS` is defined but deliberately unused (removed for
  responsiveness). If a motor driver ever burns out again, re-add an ~80 ms pause before reversing.
- **Brake flag:** the protocol has one, but it is only applied for the loop iteration a packet
  arrives in, so in practice the motors coast when stopped.

## Flashing (Arduino IDE 2.x)

1. Boards Manager URL: `https://arduino.esp8266.com/stable/package_esp8266com_index.json`,
   install **esp8266 by ESP8266 Community**, select **LOLIN(WEMOS) D1 R2 & mini**.
2. Library Manager: install **Adafruit NeoPixel**.
3. Copy `secrets.example.h` to `secrets.h` and put in your 2.4 GHz WiFi (it is git-ignored).
4. Open `robofield_car/robofield_car.ino`, plug in the D1 Mini, Upload.
5. Serial Monitor at 115200: type `id 0` (this car's number), Enter, then press RESET.
6. `python tools/cars.py list` should show it. Later updates can go over WiFi (ArduinoOTA,
   hostname `robofield-car-<id>`; change `OTA_PASSWORD` in `config.h` from its default).

Bump `FW_VERSION` on every change so `tools/cars.py list` confirms what each car is running.
