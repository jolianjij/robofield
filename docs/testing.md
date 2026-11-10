# Testing and results

RoboField is tested in two ways: an automated test suite that needs no hardware, and measurement
tools that score the real car on the real field. This page describes both, and the results.

## Automated tests

```bash
pytest                     # everything (18 tests, about 15 s)
pytest tests/test_safety.py
pytest -k plane            # tests whose name contains "plane"
```

Tests that need the simulator skip themselves if MuJoCo is not installed (`pip install -e ".[sim]"`).

| File | What it checks | Needs MuJoCo |
|---|---|---|
| `test_protocol.py` | Car packets: CRC-8 against the standard check value, command size and clamping, status encode/decode, serial framing with split and noisy input | |
| `test_safety.py` | E-stop and lost tracking stop everything; the geofence blocks driving out but still allows turning; collision avoidance | |
| `test_vision.py` | Homography estimator on a synthetic camera image: recovers poses, parallax correction, no poses before the field is known | |
| `test_plane.py` | The plane estimator on a simulated Pi Camera 3 Wide at 1.3 m with pixel noise: accuracy, and still working with two towers hidden | |
| `test_pose_controller.py` | Straight paths stay inside the margin; six random goals in the simulator with the real car's motor curve, delay and prediction: each ends within 40 mm and 6° | yes |
| `test_race.py` | The race track geometry is consistent; a scripted lap completes without leaving the track | yes |
| `test_real_stack.py` | The **real-field code** (`CarLink`, `PoseReceiver`, `RealBackend`) against a simulated world that speaks the car firmware's UDP protocol and the Pi tracker's pose format: car discovery, setting an id, driving to points | yes |

Two helpers make the hardware-free tests possible:

- `tests/fake_world.py`: `FakeCar` answers on the car's UDP ports exactly like the firmware and
  drives a simulated robot; `FakeTracker` sends that robot's pose in the Pi's JSON format. The real
  backend cannot tell the difference.
- `tests/synthetic.py`: renders a top-down image of the field with markers, for the vision tests.

## Measuring the real car

These tools run against the real field. Each prints numbers you can compare before and after a change.

| Tool | Needs the Control Center running | Measures |
|---|---|---|
| `tools/measure_latency.py --robot 0` | no (close it) | Time from a command to the camera seeing the car react, over 6 trials |
| `tools/calibrate_motors.py --robot 0` | no (close it) | Wheel speed per command from pivot tests, and straight-line drift → trim. `--write` saves them |
| `tools/eval_motion.py --robot 0` | yes | Straight line, 120° turn in place, half-circle path: time, distance from the line, stop error, heading error, wobbles |
| `tools/film_move.py <move>` | yes | Plays a saved move while recording camera and screen; writes the time, stop error, heading error and wobble counts to a `.json` next to the videos |

A **wobble** is one reversal of the heading's direction of change while settling. A car that swings
left, right, left before stopping has two. A clean turn has none.

## Results on the real car

All measured on 2026-11-06 with car 0 (Wemos D1 Mini, DRV8833, N20 motors, 2S battery).

### Loop delay

| | median | range |
|---|---|---|
| Command sent → motion seen by the laptop | 234 ms | 157–422 ms |

### Motion quality (`tools/eval_motion.py`)

| Maneuver | Time | Result |
|---|---|---|
| Straight 350 mm | 2.4–3.1 s | stops 6–7 mm from the goal, within 19–30 mm of the line |
| Turn 120° in place | 3.5–4.2 s | final heading within 2–5°, no oscillation |
| Half circle, r = 170 mm | 3.3 s | stops 15 mm from the end of the drawn path |

### Before and after: the old controller against the current one

The turn in place, old continuous controller against pulse-and-wait with delay prediction:

| | Old: continuous turn, no prediction | Current: pulse-and-wait, prediction |
|---|---|---|
| Swings past the target | 14 in 5 s | none |
| Final heading error | ~19° | 2–5° |

The `compare` move (an S-curve across the field ending in a 120° turn), filmed with
`tools/film_move.py compare --style both`:

| | Before (`--style before`) | After (`--style after`) |
|---|---|---|
| Distance from the drawn path, mean / max | 13 / 40 mm | 8 / 21 mm |
| Final heading error | 7.1° | 2.8° |

On a gentle S-curve the difference is modest; it is largest in turns on the spot, where the delay
does the most damage.

## Experiments

The controlled comparisons for the paper (baseline vs PPO vs PPO with domain randomization, in sim and
on the real field) are described in [experiments](experiments.md).

## Photos and videos of the tests

Real-world photos and videos of the field, the car and these tests will go in
[`docs/images/`](images/README.md) and be linked from here.
