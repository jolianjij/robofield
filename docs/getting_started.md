# Getting started

This page takes you from nothing to a car driving itself on the real field. You can do the first
half (the simulator) on any laptop, with no hardware at all.

## 1. RoboField in five minutes

RoboField is a small arena for testing robot control and AI on real robots. It has three parts:

- **The field.** A 1 × 1 m floor with low walls and four tall towers just outside the corners.
  Each tower carries an ArUco marker (a printed black-and-white square a camera can find and
  measure). Because the towers' positions are known, the camera can work out exactly where it is.
- **The cars.** Small two-wheeled robots with one ArUco marker on top. They have no camera and make
  no decisions: they drive their two wheels, open and close a gripper, and light three LEDs, as
  told over WiFi.
- **The brain.** A Raspberry Pi with a camera looks down at the field and measures every car's
  position and heading about 40 times a second. A laptop receives those positions, decides what
  every car should do, and sends wheel commands back, 30 times a second.

A few words you will see everywhere:

| Word | Meaning |
|---|---|
| **Pose** | Where a car is and which way it faces: (x, y, θ), in millimetres and radians |
| **Field frame** | The coordinate system: origin at the inner bottom-left corner, x along the bottom wall, y up the left wall ([coordinates](coordinates.md)) |
| **Marker** | An ArUco square. IDs 0–3 are the towers, 10 + N is robot N ([markers](markers.md)) |
| **Control Center** | The program on the laptop that runs everything, with a web page to watch and drive |
| **Backend** | Either the real field or the simulator; the rest of the code cannot tell which |

The same code drives a MuJoCo simulation of the same field, so you can try everything below
before building anything.

## 2. Install

You need Python 3.10 or newer and git.

```bash
git clone https://github.com/jolianjij/robofield.git
cd robofield
python -m venv .venv
```

Activate the environment (`.venv\Scripts\activate` on Windows, `source .venv/bin/activate` on
macOS and Linux), then install RoboField with the simulator, the RL environments and the tests:

```bash
pip install -e ".[dev,rl,sim]"
```

Add `train` to the list (`".[dev,rl,sim,train]"`) if you want to train RL policies; it installs
PyTorch and Stable-Baselines3.

Check that everything works:

```bash
pytest
```

All tests run without hardware.

## 3. Drive a simulated car

```bash
pip install -e ".[train,app]"
python tools/robofield_app.py
```

The RoboField window opens on the **Field** tab, with two simulated cars ([the app](app.md)). Try, in order:

1. **Go to pose.** Choose **Go to pose** above the map, press on the field where the car should go,
   drag toward where it should face, release. Watch it turn to face the path, drive, stop, then turn
   to the heading. The car's card shows the phase: TURN, FOLLOW, ALIGN.
2. **Draw a path.** Choose **Draw path** and draw any curve. The car follows it.
3. **Drive by hand.** Click a car, then hold W A S D. Space stops it; Esc is the emergency stop.
4. **Change how it drives.** Open the **Tuning** tab, raise the cruise speed, set a goal on that
   page's map, and watch the difference ([tuning guide](tuning.md)).
5. **See it in 3D.** The **3D view** tab shows the same field; press **Follow** to ride behind a car.
6. **Let a trained policy drive.** On the **Training** tab, press **Drive** next to a policy, or
   train your own in a minute with *Quick test* ([reinforcement learning](reinforcement_learning.md)).

Every panel is described in [Control Center](control_center.md).

## 4. Bring up the real field

Each step has a check. Do not move on until it passes.

### Step 0: the parts

- The field, the towers, the camera mast and a car, built from the CAD in [`hardware/`](../hardware/README.md).
- Markers printed at exact size on matte paper: `python tools/generate_markers.py --out markers/ --dpi 300`,
  printed at 100% scale. Measure them with a ruler. Towers: IDs 0–3, 80 mm. Car: ID 10 for robot 0, 40 mm.
- Tower positions and marker heights measured and entered in `config/field.yaml` ([field](field.md)).

### Step 1: the Raspberry Pi and the lens (about 20 minutes)

Set up the Pi and the camera as in [`pi/README.md`](../pi/README.md). Then calibrate the lens with a
printed chessboard. Count the **inner** corners (where four squares meet) and measure one square;
the board used for the reference build has 9 × 6 squares, so 8 × 5 inner corners, 28 mm squares:

```bash
.venv/bin/python pi/pi_calibrate.py --cols 8 --rows 5 --square 28
```

Move the board slowly under the camera: centre, all four corners, the edges, tilted different ways.

**Check:** RMS error under 0.5 px.

### Step 2: tracking (about 15 minutes)

```bash
.venv/bin/python pi/pi_tracker.py --preview 8080
```

Open `http://<pi-address>:8080` on the laptop.

**Check:** the preview says **FIELD LOCKED**, shows 4 towers, the camera at about (500, 500, 1300) mm,
and a reprojection error under 1 px. Put the car at a few measured spots, for example (250, 250),
(500, 500) and (750, 750): the reported position should be within about ±5 mm.

### Step 3: the coordinate grid on the camera image

The preview draws a 100 mm grid (thick white lines every 250 mm, on the tile seams), the X (red)
and Y (green) axes at the origin, and a heading arrow on every car.

**Check:** the thick lines sit on the real tile seams and walls everywhere in the image.
Off at the edges only: redo the lens calibration. Shifted or rotated: check the tower positions and
heights in the config, and that each tower has the right marker with its top edge toward +y.

### Step 4: the car

Flash the firmware and set the car's id as in the [firmware README](../firmware/car_esp8266/README.md). Then:

```bash
python tools/cars.py list        # the car, its IP, signal, firmware version
python tools/cars.py test 0      # forward, back, spin, gripper, LEDs at 40% speed (lift it first)
```

**Check:** both wheels turn the right way. If one runs backwards, set `LEFT_REVERSED` or
`RIGHT_REVERSED` in `config.h` and reflash.

### Step 5: measure the car

```bash
python tools/measure_latency.py --robot 0            # loop delay → control.latency_s
python tools/calibrate_motors.py --robot 0 --write   # top speed and left/right trim
```

Put the car in the middle of the field facing the centre first, and keep the Control Center closed
while these run. Details: [kinematics](kinematics.md#7-updating-the-model-for-a-new-or-rebuilt-car).

### Step 6: first closed-loop drive

```bash
python tools/robofield_app.py --real
```

The app finds the Pi on the network by itself. The car appears in the Cars list on the Field tab. Press **Identify** to make sure it is the right one, then use
**Go to pose**.

**Check:** it arrives within about 12 mm and 4° of the goal without swinging back and forth. If it
turns the wrong way, add it to `robots:` in `config/field.yaml` with `wheels_swapped: true`.

### Step 7: score it

```bash
python tools/eval_motion.py --robot 0
```

Compare with the reference results in [testing](testing.md#results-on-the-real-car). If something is
off, the [tuning guide](tuning.md) has a symptom → fix table.

## 5. Where to go next

- How the pieces fit together: [architecture](architecture.md).
- Why the car drives the way it does: [kinematics](kinematics.md) and [motion control](control.md).
- Reinforcement learning: the games in `robofield/games/`, the Gymnasium environment
  `robofield.envs.RoboFieldEnv`, and `python tools/train_race.py` to train a policy that races three
  laps around the field in the simulator.
