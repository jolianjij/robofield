# RoboField

**Dumb robots, smart environment.** RoboField is a low-cost, open-source arena for testing robot
control and AI ideas on real robots, not just in simulation.

A 1 × 1 m field with four ArUco towers is watched by one overhead camera. Each car carries a single
ArUco sticker and no sensors or computer of its own: it only drives, grips and lights up. A laptop
sees every car through the camera, decides what each one should do, and sends the commands over
WiFi. The same code drives a MuJoCo simulation of the same field, so a controller or an RL policy
can be developed in simulation and then run on the real cars unchanged.

```
 Raspberry Pi 5 + Camera 3 Wide          Laptop (this repo)                   Cars (ESP8266)
 ArUco tracking, 30+ fps     --UDP-->    EKF + delay prediction     --UDP-->  motors, servo, LEDs
 live preview with overlay   <--UDP--    controllers, safety, games <--UDP--  status
                                         web Control Center  :8000
```

## What it can do today

| | |
|---|---|
| **Go to a pose** | click where a car should go and drag the direction it should face |
| **Follow a drawn path** | draw any curve on the map; the car follows it with pure pursuit |
| **Safety layer** | geofence (it never drives into a wall), collision avoidance, tracking-loss stop, e-stop |
| **Live tuning** | `/tuning`: every controller and motor parameter on a slider, saved to `config/tuning.yaml` |
| **Games / RL** | Gymnasium + PettingZoo environments (go-to, sumo, a 3-lap race), PPO training script |

Measured on the real car (2026-11-06, `tools/eval_motion.py`):

| Maneuver | Time | Result |
|---|---|---|
| Straight 350 mm | 2.4–3.1 s | stops 6–7 mm from the goal, within 19–30 mm of the line |
| Turn 120° in place | 3.5–4.2 s | final heading within 2–5°, no oscillation |
| Half circle, r = 170 mm | 3.3 s | stops 15 mm from the end of the drawn path |

## The control problem, and how it is solved

A robot that works perfectly in simulation meets friction, unequal motors and delay in the real
world. RoboField measures each of these on the real car and models it, instead of tuning around it.
Full write-up with the data: [`docs/control.md`](docs/control.md).

- **Delay.** From a command leaving the laptop to the camera seeing the result takes ~0.2 s
  (`tools/measure_latency.py`). Acting on the latest frame means always correcting where the robot
  *was*. A Smith predictor (`robofield/control/predictor.py`) replays the commands already in
  flight through the motor model, so the controllers steer on where the robot *will be*.
- **Unequal, non-linear motors.** The left wheel runs ~8% faster, and the slowest the car can move
  is ~100 mm/s. The measured speed-per-command curve and the left/right trim are part of the robot
  model, and every controller maps its speed requests through them ([kinematics](docs/kinematics.md)).
- **Turning precisely.** Turning in place needs a hard kick to break the wheels free, then coasts
  far past the target, so a continuous controller oscillates left and right. Final turns are
  instead single-wheel pivot pulses, each followed by a wait longer than the delay, sized by a
  turning model the car re-learns from every pulse.

## Quick start

No hardware needed to try it: the simulator runs the same code.

```bash
git clone https://github.com/jolianjij/robofield.git
cd robofield
pip install -e ".[dev,rl,sim]"
pytest                                                           # 18 tests, no hardware
python tools/control_center.py                                  # simulator
python tools/control_center.py --real --pi <pi-address>          # real field
```

Open `http://localhost:8000` (map, cars, go-to-pose, draw a path, safety, camera) and
`http://localhost:8000/tuning` (live parameters).

On the Pi: `pi/install.sh`, then `.venv/bin/python pi/pi_tracker.py --preview 8080`. Calibrate the
camera once with `pi/pi_calibrate.py` (see [`pi/README.md`](pi/README.md)).

## Documentation

New here? Read [Getting started](docs/getting_started.md): RoboField in five minutes, then a
simulated car, then the real field step by step. Everything else is listed in [`docs/`](docs/README.md):

| | |
|---|---|
| [Architecture](docs/architecture.md) | Every package and module, and one control cycle from camera frame to wheel |
| [Kinematics](docs/kinematics.md) | The differential-drive model and the measured motor model, with worked numbers |
| [Motion control](docs/control.md) | Delay prediction, the EKF, pure pursuit, pulse-and-wait turns |
| [Tuning guide](docs/tuning.md) | Every parameter, and what to change for a given symptom |
| [Control Center](docs/control_center.md) | The web app, the safety rules, the JSON API |
| [Reinforcement learning](docs/reinforcement_learning.md) | Train a policy in the simulator and run it on the real car, step by step |
| [Testing and results](docs/testing.md) | The test suite and the measured results on the real car |

## Tools

| Command | What it does |
|---|---|
| `tools/cars.py list` / `test 0` | find cars on the network; drive one through a self-test |
| `tools/measure_latency.py` | measure the command-to-camera delay on the real field |
| `tools/calibrate_motors.py --write` | measure top wheel speed and left/right trim, write them to the config |
| `tools/eval_motion.py` | score straight lines, turns and curves on the real field |
| `tools/demo.py wander\|figure8\|square` | demo drives |
| `tools/film_move.py hook` / `compare --style both` | film a saved move: camera video + Control Center screen recording + a score |
| `tools/record_camera.py` / `tools/record_screen.py` | record the Pi camera feed / the Control Center page to MP4 |
| `tools/real_goto.py` | minimal closed-loop go-to-point test |
| `tools/train_race.py` | train / evaluate a PPO policy for the 3-lap race |

## Repository layout

| Folder | What lives there |
|---|---|
| `robofield/control/` | kinematics + motor model, EKF, delay predictor, pose / path / turn controllers |
| `robofield/real/`, `robofield/sim/` | the real field and the MuJoCo simulator, behind one `Backend` interface |
| `robofield/app/` | Control Center: control loop, safety layer, web GUI and tuning page |
| `robofield/vision/`, `pi/` | ArUco field tracking (runs on the Raspberry Pi) |
| `robofield/comm/` | UDP protocol for the cars and for the Pi's pose stream |
| `robofield/games/`, `robofield/envs/` | game plugins, Gymnasium / PettingZoo environments |
| `firmware/car_esp8266/` | **the firmware the cars run** (Wemos D1 Mini + DRV8833, Arduino IDE) |
| `firmware/car/`, `firmware/gateway/` | earlier ESP32-C3 / PlatformIO versions, same protocol |
| `config/field.yaml` | every measurement: field, towers, camera, robot model, delay |
| `hardware/` | the car's parts and wiring; CAD assemblies of the car and the field |
| `docs/` | [all the documentation](docs/README.md): getting started, architecture, kinematics, control, tuning, testing |
| `tests/` | `pytest`: control, vision, protocol, safety, the full real-field stack against a simulated field |

## License

MIT — see [`LICENSE`](LICENSE).
