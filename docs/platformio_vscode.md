# PlatformIO with VS Code: from zero to a car running RoboField firmware

> **Legacy.** This guide is for the original ESP32-C3 firmware in `firmware/car/`. The cars now run
> the Wemos D1 Mini (ESP8266) firmware in `firmware/car_esp8266/`, flashed from the Arduino IDE; see
> [its README](../firmware/car_esp8266/README.md). The radio protocol is the same, so either works
> with the laptop software.

## 1. Install (once)
1. Install **VS Code**: https://code.visualstudio.com
2. In VS Code open **Extensions** (left bar, the four-squares icon, or `Ctrl+Shift+X`),
   search **PlatformIO IDE**, click **Install**. Wait until it finishes (it downloads its own
   Python and tools; the first time takes several minutes). Restart VS Code when asked.
3. An ant-head **PlatformIO icon** appears in the left bar, and a toolbar at the bottom:
   ✓ build, → upload, 🔌 serial monitor, 🗑 clean, and the environment selector (`env:...`).
4. Windows only: if the ESP32-C3 is not recognised, it uses the built-in USB of the chip
   (no driver usually needed on Windows 10/11). Try another USB cable: many are charge-only.

## 2. Open the car firmware
`File -> Open Folder...` and choose **`robofield/firmware/car`** (the folder that contains
`platformio.ini`, not the whole repo). PlatformIO reads `platformio.ini` and, the first time,
downloads the ESP32 platform, the Arduino framework and the NeoPixel library (a few minutes).

Folder layout:
```
firmware/car/
  platformio.ini        build settings: board, libraries, upload methods (usb / ota)
  include/config.h      pins, motor driver type, battery settings, OTA password
  include/secrets.h     your WiFi name and password (copy from secrets.example.h)
  src/main.cpp          the firmware
```

## 3. Before the first build
- `include/secrets.h`: put your **2.4 GHz** WiFi name and password.
- `include/config.h`: `#define DRIVER_DUAL_PWM` for a 2-input-per-motor board; battery
  settings are already for a 2S pack (100k / 33k divider).

## 4. Build
Bottom toolbar: pick environment **`env:usb`**, click **✓ (Build)**.
A terminal opens; it should end with `SUCCESS`. If there is an error, the terminal shows the file
and line; send me the red lines and I will fix them.

## 5. Upload by USB (first time for every car)
1. Plug the ESP32-C3 Super Mini in with a **data** USB-C cable.
2. Click **→ (Upload)** with `env:usb` selected.
3. If it says it cannot connect: hold the **BOOT** button, press and release **RESET**
   (or re-plug the cable while holding BOOT), release BOOT, upload again.

## 6. Serial monitor
Click **🔌 (Serial Monitor)**. You should see
`RoboField car 0  ip 192.168.x.x  fw 2`. Type in the monitor's input and press Enter:
- `info` - id, IP, WiFi signal, battery
- `id 1` - set this car's robot id to 1 (saved in flash); then press RESET

## 7. Wireless updates (every time after the first)
1. Bottom toolbar: choose **`env:ota`**.
2. Upload with the car's address: open the PlatformIO terminal (`Ctrl+Shift+P` ->
   *PlatformIO: New Terminal*) and run
   ```
   pio run -e ota -t upload --upload-port robofield-car-1.local
   ```
   (or use the car's IP address, which `python tools/cars.py list` shows).
3. The car's LEDs turn purple during the update, then it restarts with the new firmware.
If `.local` names do not resolve on your computer (some Windows setups), use the IP.

## 8. Useful
| Task | How |
|---|---|
| Rebuild everything | 🗑 Clean, then ✓ Build |
| Change upload port | `upload_port = COM5` (or `/dev/ttyACM0`) in `platformio.ini` under `[env:usb]` |
| See all cars | `python tools/cars.py list` (from the repo folder, in any terminal) |
| Test a car | `python tools/cars.py test 1` |
| Library missing | it is listed in `lib_deps` in `platformio.ini`; Build downloads it |
