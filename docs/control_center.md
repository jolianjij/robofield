# Control Center: the base layer that talks to the hardware

```
                       WiFi / Ethernet (same network)
 ┌───────────────────────┐   poses, UDP 5005 (JSON, ~30-50 Hz)    ┌──────────────────────────────┐
 │ Raspberry Pi 5        │ ─────────────────────────────────────► │ Laptop: Control Center       │
 │ pi/pi_tracker.py      │ ◄───────────────────────────────────── │ tools/control_center.py      │
 │  camera -> ArUco ->   │   goal + path overlay, UDP 5006        │  ControlCenter (30 Hz loop)  │
 │  field poses          │                                        │   controllers per car        │
 │  preview :8080 (MJPEG)│ ─── live camera view in the GUI ─────► │   Safety layer               │
 └───────────────────────┘                                        │   car registry (cars.json)   │
                                                                  │  web GUI :8000 (browser)     │
 ┌───────────────────────┐   commands, UDP 4210 (22 bytes, 30 Hz) │                              │
 │ Cars (ESP32-C3)       │ ◄───────────────────────────────────── │                              │
 │ firmware/car          │   status, UDP 4211 broadcast (10 Hz)   │                              │
 │  motors, gripper, LEDs│ ─────────────────────────────────────► │                              │
 │  watchdog, battery    │   firmware updates, OTA port 3232      │                              │
 └───────────────────────┘ ◄───────────────────────────────────── └──────────────────────────────┘
```

**On the Pi:** only the tracker runs (as a service): capture, marker detection, pose of every
robot in field coordinates, live preview with the coordinate grid, goal and path drawn on it.
**On the laptop:** the Control Center: all decisions, controllers, safety, the GUI.
**On each car:** executes wheel / gripper / LED commands, stops on its own if commands stop.

## Run it
```bash
python tools/control_center.py                                  # simulator, 2 cars, no hardware
python tools/control_center.py --real --pi robofield-pi.local   # real field
```
Opens http://localhost:8000 (also reachable from a phone on the same WiFi: http://<laptop-ip>:8000).

## What the GUI does
| Area | Use |
|---|---|
| Cars list | every car (found automatically): name, map colour, battery, signal, what it is doing, safety warnings |
| Per car | the colour of each of its 3 LEDs (or all the same), max speed, gripper, enable/disable, Identify (blinks white), Stop, Update firmware (over WiFi) |
| Map | live top view: field, grid, towers, every car with its LEDs, trail, goal and path |
| Go to pose | press where the car should go, drag toward where it should face, release |
| Draw path | draw any path with the mouse or a finger; the car follows it in real time (option: turn to the last direction at the end) |
| Keyboard | W A S D drive the selected car while held, Space stops it, Esc = emergency stop |
| Camera | live view from the Pi with the coordinate grid, goal and path drawn on the real image |
| Safety | speed limit, wall margin, keep-cars-apart; big STOP button |

## Safety (every command, every step)
1. **Emergency stop** (button / Esc): every motor off until released; LEDs turn red.
2. **Camera data lost** (tracker silent or no field lock for 0.6 s): everything stops.
3. **Car not seen** by the camera for 0.4 s: that car stops.
4. **Wall margin (geofence):** a car about to leave the safe area loses its forward speed but can still turn.
5. **Keep cars apart:** a car driving toward another closer than 130 mm stops moving toward it.
6. **Speed limit** for everything, plus a per-car max speed.
7. **Low battery** (< 6.6 V on 2S): car disabled.
8. **Manual driving is a dead-man control:** it stops 0.3 s after you release the keys.
9. **In the car firmware:** 200 ms command watchdog, battery cutoff, duty ramp, stop on WiFi loss / during updates.

## Flash and define your first car
1. Wire the car (`docs` wiring guide), 2S battery, 100k / 33k divider on GPIO0.
2. Flash by USB once (`docs/platformio_vscode.md`): `secrets.h` with your WiFi, `env:usb`, Upload.
3. Serial monitor: `id 0` + Enter, then RESET. It prints its IP.
4. `python tools/cars.py test 0`: forward, back, spin, gripper, LEDs. Fix a backwards wheel with
   `LEFT_REVERSED` / `RIGHT_REVERSED` in `config.h` and update over WiFi.
5. Stick robot marker **ID 10** (robot 0 = marker 10, robot 1 = 11, ...) on top, centred over the
   middle of the axle, its top edge pointing forward.
6. Start the Control Center (`--real`). The car appears in the list: give it a name, a map colour and
   LED colours. This is saved in `config/cars.json`, so it is remembered next time.
7. Click **Identify** to confirm which physical car it is, then try **Go to pose** and **Draw path**.

## Building on top of it
Everything else uses the same object:
```python
cc = ControlCenter(cfg, backend, "real").start()
cc.set_goal(0, 700, 300, 0.0)                 # go to a pose
cc.set_path(1, [(200, 200), (500, 800)])      # follow a path
cc.update_car(0, leds=[[255, 0, 0]] * 3)      # team colour
state = cc.state                              # live poses of every robot
```
Games, RL policies and multi-agent coordinators become new controller modes that write commands
into the same loop, so they automatically get the safety layer, the GUI and the hardware link.
