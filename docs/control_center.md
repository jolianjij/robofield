# Control Center: the web app that runs the field

The Control Center is the one program that talks to the cars (or to the simulator). It runs the
30 Hz control loop, keeps one controller per car, filters every command through the safety layer,
and serves a web page to watch and drive everything. How it works inside: [architecture](architecture.md).

## Run it

```bash
python tools/control_center.py                                   # simulator, 2 cars, no hardware
python tools/control_center.py --sim-robots 4                    # simulator, 4 cars
python tools/control_center.py --real --pi <pi-address>          # real field
```

| Option | Meaning |
|---|---|
| `--real` | Use the real field: poses from the Pi tracker, commands to the cars over WiFi |
| `--pi <host>` | The Pi's address: shows its camera view in the page and draws the goal and path on it |
| `--sim-robots N` | Number of simulated cars (simulator only) |
| `--port 8000` | Web page port |
| `--config config/field.yaml` | Another field / car configuration |
| `--no-browser` | Do not open a browser tab |

Open `http://localhost:8000`. From a phone or tablet on the same WiFi, use
`http://<laptop-address>:8000`. The tuning page is `http://localhost:8000/tuning` ([tuning guide](tuning.md)).

## The main page

| Area | What it does |
|---|---|
| **Top bar** | Mode (sim / real), camera status, global speed limit, **Stop all** (every car stops but stays ready), **STOP** (emergency stop: motors off until released) |
| **Cars** | One card per car, found automatically: name, map colour, battery, signal, firmware version, what it is doing, safety warnings |
| **Map** | Live top view: field, grid, towers, every car with its LEDs and trail, its goal and planned path, the safe area |
| **Camera** | The Pi's live view with the coordinate grid, heading arrows, goal and path drawn on the real image |
| **Moves** | Saved paths to replay: save, go to start, play |
| **Safety** | Wall margin, keep-cars-apart switch, keyboard help |
| **Activity** | The last few events: goals, arrivals, tuning changes, errors |

### Per-car card

| Control | Effect |
|---|---|
| Colour swatch, name | How the car appears on the map; saved in `config/cars.json` |
| LEDs (3 colours, or **same**) | The colour of the car's three LEDs, e.g. team colours |
| Max speed | A per-car speed limit on top of the global one; scales speed and turn rate together, so paths keep their shape |
| Gripper | Servo position, open (0) to closed (1) |
| Enabled | A disabled car always receives zero commands |
| **Identify** | Blinks the car's LEDs white for 2 s, to find which physical car is which |
| **Stop** | Stops this car |
| **Update firmware** | Builds and uploads the *legacy* ESP32-C3 PlatformIO firmware over WiFi. For the ESP8266 cars, flash from the Arduino IDE instead ([firmware README](../firmware/car_esp8266/README.md)) |

### Map tools

| Tool | How to use it |
|---|---|
| **Select** | Click a car to select it |
| **Go to pose** | Press where the car should go, drag toward the direction it should face, release. The car turns to the path, drives it, stops, then turns to the heading ([how](control.md#3-the-pose-controller)) |
| **Draw path** | Draw any curve; the car follows it. Tick **turn to the last direction at the end** to make it finish facing along your last stroke |
| **sketch only** | Draw a path without the car moving, to design a move and save it |

### Keyboard

| Key | Effect |
|---|---|
| W A S D | Drive the selected car while held (it stops 0.3 s after you let go) |
| Space | Stop the selected car |
| Esc | Emergency stop for everything |

## Saved moves and filming

Draw a path (tick **sketch only** to draw it without the car moving), type a name and press
**Save last path** in the **Moves** panel. Moves are kept in `config/moves/<name>.json`.

- **To start** drives the car to the move's first point, facing along it. It arrives in two legs
  (first to a point 120 mm behind the start, then straight in) so the last turn is tiny and the
  car really ends up on the start.
- **Play** drives the move, going to the start first if the car is not already there. A replayed
  move is followed from its first point, so a loop that ends where it began is driven all the way round.

`tools/film_move.py <move>` films one take: it parks the car (not recorded), records the Pi camera
(`record_camera.py`) and the Control Center page in a hidden Chrome (`record_screen.py`), plays the
move, and stops both. `--style before` uses the old controller (continuous turning, no delay
prediction), `--style after` the current one, `--style both` does the two from the same start. The
page gets a Before / After label (`/?caption=...&tone=bad|ok`), and each take writes a `.json` next
to its videos with the time, stop error, final heading error and wobble counts. The built-in
`compare` move is an S-curve across the field ending in a 120° turn.

## Safety

Every command, from every controller, every step, passes through `robofield/app/safety.py`:

| # | Rule | Default | What happens |
|---|---|---|---|
| 1 | Emergency stop | | STOP button or Esc: every motor off, LEDs red, until released |
| 2 | Camera data lost | 0.6 s | No pose message from the Pi, or the field is not locked: every car stops |
| 3 | Car not seen | 0.4 s | That car stops |
| 4 | Low battery | 6600 mV | That car stops. Only active when the car reports its battery; the current firmware has its battery monitor switched off |
| 5 | Speed limit | 70% | Caps forward speed and turn rate for everything |
| 6 | Geofence (wall margin) | 63 mm | A car predicted (0.35 s ahead) to leave the safe area loses its forward speed but can still turn, so it can face away and leave the edge |
| 7 | Keep cars apart | 130 mm | A car driving toward another closer than this (plus its stopping distance) stops moving toward it |
| 8 | Manual driving | 0.3 s | Keyboard driving is a dead-man switch: it stops when the commands stop |

The car firmware adds its own last line of defence: a 200 ms command watchdog (motors off if the
laptop goes silent) and a duty ramp.

## The JSON API

The page uses these, and so can your own scripts ([example](architecture.md#7-extending-it)).

| Method and path | Body | Does |
|---|---|---|
| `GET /api/state` | | Everything the page shows: robots (pose, mode, phase, goal, path, battery...), tracker, safety, log |
| `POST /api/goal` | `{"id", "x", "y", "theta"}` | Go to a pose (mm, mm, rad) |
| `POST /api/path` | `{"id", "points": [[x, y], ...], "theta": null or rad}` | Follow a path |
| `POST /api/drive` | `{"id", "left", "right"}` | Manual wheel commands (−1..1); repeat at ~10 Hz |
| `POST /api/stop` | `{"id"}` or `{}` | Stop one car or all |
| `POST /api/estop` | `{"on": true or false}` | Emergency stop |
| `POST /api/car` | `{"id", "name"?, "color"?, "leds"?, "max_speed"?, "gripper"?, "enabled"?}` | Change a car's profile |
| `POST /api/identify` | `{"id"}` | Blink the car's LEDs |
| `POST /api/settings` | `{"speed_limit"?, "geofence_margin_mm"?, "avoid_collisions"?, ...}` | Safety settings |
| `GET /api/tuning` | | Tuning values and their descriptions |
| `POST /api/tuning` | `{param: value, ...}` | Change tuning; applies from the next goal or path |
| `POST /api/tuning/save` | | Write the values to `config/tuning.yaml` |
| `POST /api/tuning/reset` | | Back to the built-in defaults (not saved until you save) |
| `POST /api/policy` | `{"id", "model", "speed"?, "no_reverse"?, "place"?}` | A trained policy from `models/` drives the car for one episode of its game |
| `POST /api/sim/place` | `{"id", "x", "y", "theta"}` | Simulator only: put a car at a pose |
| `GET /api/scene` | | The fixed geometry of the field and cars (for the 3D view) |
| `GET /api/moves` | | The saved moves |
| `POST /api/moves/save` | `{"id", "name"}` or `{"name", "points", "theta"}` | Save the car's last path, or the given points |
| `POST /api/moves/start` | `{"id", "name"}` | Park at the move's start |
| `POST /api/moves/play` | `{"id", "name"}` | Play the move |

Every POST answers `{"ok": true or false, "msg": "..."}`. The RoboField app adds routes for switching
mode, training and tools; all routes are listed at the top of `robofield/app/server.py`.

## Adding a car

1. Build and flash it ([firmware README](../firmware/car_esp8266/README.md)), and give it the next
   free robot id (`id 1` in the serial monitor, then reset).
2. Stick its marker on top: ArUco **ID 10 + robot id**, 40 mm, centred over the middle of the axle,
   top edge pointing forward ([markers](markers.md)).
3. `python tools/cars.py list` should show it; `python tools/cars.py test 1` runs a short self-test.
   If a wheel runs backwards, set `LEFT_REVERSED` / `RIGHT_REVERSED` in its `config.h`. If it turns
   the wrong way, add it to `robots:` in `config/field.yaml` with `wheels_swapped: true`.
4. Start the Control Center with `--real`. The car appears in the list: name it and give it colours.
5. Press **Identify** to confirm which car it is, then try **Go to pose** and **Draw path**.
6. If its motors differ from the first car's, measure them ([kinematics](kinematics.md#7-updating-the-model-for-a-new-or-rebuilt-car)).
   The robot model is shared by all cars for now, so very different cars need the same treatment.
