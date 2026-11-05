# Car firmware (ESP32-C3 Super Mini)

One firmware for every car. Cars join your WiFi, receive commands over UDP and are updated
**wirelessly** (OTA) after the first USB flash. Each car's robot id is stored in its flash.

## One-time setup on the computer
1. Install **VS Code** + the **PlatformIO** extension (or `pip install platformio`).
2. Copy `include/secrets.example.h` to `include/secrets.h` and enter your WiFi name and password
   (2.4 GHz network: the ESP32-C3 has no 5 GHz). Use the same network as the laptop and the Pi.
3. In `include/config.h` pick your motor driver: `DRIVER_DUAL_PWM` (2 inputs per motor:
   MX1508 / DRV8833 / L9110S) or `DRIVER_TB6612`. Change `OTA_PASSWORD` here and in
   `platformio.ini` (`--auth=`).

## Each new car (USB, once)
```bash
cd firmware/car
pio run -e usb -t upload          # hold BOOT while plugging in if the upload does not start
pio device monitor                # shows: RoboField car 0  ip 192.168.1.51
```
In the serial monitor type `id 1` (Enter) to give this car robot id 1, then `info` to check.
Restart the car (its OTA name becomes `robofield-car-1`).

## Every update after that (wireless)
```bash
pio run -e ota -t upload --upload-port robofield-car-1.local    # or the car's IP
```
The car's LEDs turn purple during the update and the motors are off.

## Manage the cars from the laptop
```bash
python tools/cars.py list                    # ids, IPs, battery, signal of all cars
python tools/cars.py set-id 192.168.1.52 2   # change a car's id without USB
python tools/cars.py test 1                  # forward, back, spin, gripper, LEDs
python tools/cars.py drive 1                 # keyboard driving
```

## LED meaning
| LEDs | Meaning |
|---|---|
| blue blinking | connecting to WiFi |
| dim green | connected, waiting for commands |
| colours from the laptop | driving |
| purple | wireless update in progress |
| red blinking | battery low (below 3.3 V): motors disabled |

## Safety built in
- no command for 200 ms -> motors off (watchdog)
- battery below 3.3 V -> motors off until above 3.5 V
- duty ramp 6 per second (2000 mm/s^2) -> no wheelies on hard starts
- lost WiFi -> motors off, reconnects automatically
