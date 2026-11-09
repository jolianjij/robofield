# RoboField documentation

Start at the top if you are new; jump to any page once you know your way around. The same pages,
searchable and with the API reference: [jolianjij.github.io/robofield](https://jolianjij.github.io/robofield/).

## New to the project

| Page | What you get from it |
|---|---|
| [Install](install.md) | One-click install scripts, what each library is for, the self-check, fixes for common problems |
| [Getting started](getting_started.md) | What RoboField is in five minutes, install, drive a simulated car, then bring up the real field step by step |
| [The RoboField app](app.md) | One window for everything: field, 3D view, training, tools, tuning |
| [Lab](lab.md) | Write your own tasks (to train), policies (one car) and systems (several cars) |
| [Control Center](control_center.md) | The web app: every panel, keyboard shortcuts, saved moves, the HTTP API |
| [Tuning guide](tuning.md) | Every parameter on the tuning page, what it does, and which one to change for a given symptom |

## How it works

| Page | What you get from it |
|---|---|
| [Architecture](architecture.md) | The three machines, every Python package and module, one control cycle from camera frame to wheel, threads, ports and files |
| [Kinematics and motor model](kinematics.md) | The differential-drive equations, the measured motor curve, trim, pivot turns, with worked numbers |
| [Motion control](control.md) | Delay prediction, the EKF, pure pursuit, pulse-and-wait turns, with the real-car measurements behind each choice |
| [Vision](vision.md) | ArUco tracking on the Pi: three pose estimators and why the default one is used |
| [Reinforcement learning](reinforcement_learning.md) | What RL is, how the race policy was trained, train your own, take it to the real car, and the plan for two-car football |
| [Testing and results](testing.md) | The automated test suite, the real-field measurement tools, and the measured results |

## Reference

| Page | What you get from it |
|---|---|
| [Coordinates](coordinates.md) | The one coordinate frame every module uses |
| [Field](field.md) | Field size, tower positions and heights, marker placement |
| [Markers](markers.md) | Which ArUco IDs are used for what, printing rules |
| [Protocol](protocol.md) | The UDP packets between the laptop and the cars, byte by byte |
| [Car firmware](../firmware/car_esp8266/README.md) | Wiring, flashing, and what the firmware does on its own |
| [Raspberry Pi tracker](../pi/README.md) | Setting up the Pi, calibrating the lens, running the tracker |
| [Hardware](../hardware/README.md) | Parts, wiring, and the CAD assemblies of the car and the field |
| [Legacy: PlatformIO / ESP32-C3](platformio_vscode.md) | The original firmware toolchain, kept for reference |
