# RoboField

An open-source framework for multi-robot games and AI experiments on a 1 × 1 m field.
Small tank-drive cars are tracked by an overhead camera using ArUco markers, and a laptop
runs all the intelligence, so the same code works in simulation and on real robots.

> Status: early development (Phase 0 — foundations). Name is a working title.

## How it works

```
camera ─► tracking ─► world state ─► game / RL policy ─► robot API ─► gateway ─► cars
                          ▲                                   │
                          └────────── simulator (same API) ◄──┘
```

## Repository layout

| Folder | What lives there |
|---|---|
| `docs/` | Specs: field, protocol, coordinate system, marker IDs, vision strategies |
| `config/` | YAML config for field, camera, and robots |
| `firmware/car/` | ESP32-C3 car firmware (PlatformIO) |
| `firmware/gateway/` | ESP32 USB ↔ ESP-NOW bridge (PlatformIO) |
| `firmware/common/` | Packet definitions shared by both firmwares |
| `robofield/comm/` | Python packet protocol and gateway driver |
| `robofield/vision/` | Marker detection and pose estimation |
| `robofield/core/` | World state, config loading, robot API |
| `robofield/control/` | Controllers (go-to-point, path following) |
| `robofield/sim/` | 2D simulator with the same API as the real field |
| `robofield/games/` | Game plugins and Gymnasium / PettingZoo environments |
| `tools/` | Marker generator, camera calibration, tracking demo |
| `hardware/` | Parts list, wiring, STL files |
| `examples/` | Small scripts showing how to use the framework |
| `tests/` | Automated tests |

## Control Center (start here)

```bash
pip install -e ".[dev]"
python tools/control_center.py                                 # simulator, no hardware
python tools/control_center.py --real --pi robofield-pi.local  # real field
```
Live map, every car's status and LED colours, go-to-pose, draw-a-path, keyboard driving,
safety layer, camera view. See [`docs/control_center.md`](docs/control_center.md).

## Quick start (vision only, no robots needed)

```bash
pip install -e ".[dev]"
python tools/generate_markers.py --out markers/      # print these at 100% scale
python tools/track_demo.py --camera 0                # live tracking overlay
pytest                                               # run tests
```

## Firmware

Both firmwares use [PlatformIO](https://platformio.org/):

```bash
cd firmware/car && pio run -t upload
cd firmware/gateway && pio run -t upload
```

## Reference hardware

ESP32-C3 Super Mini · 2× N20 12V 200 rpm · TB6612FNG · 1S 18650 3.5 Ah + protection ·
TPS61088 boost (12V) · 5V buck · gripper servo · 3× WS2812B · Raspberry Pi Camera Module 3 Wide.
Field: 16 printed 250 mm tiles, 100 mm walls, 4 corner towers — see [`docs/field.md`](docs/field.md).
Full list in [`hardware/README.md`](hardware/README.md).

## Contributing

Read the specs in `docs/` first: every module follows the same coordinate system and protocol.
Open an issue before large changes.

## License

MIT — see [`LICENSE`](LICENSE).
