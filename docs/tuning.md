# Tuning guide

How to make a car drive the way you want: what each parameter does, what to change for a given
symptom, and how to check that a change helped. If you have not read it yet, [motion control](control.md)
explains the controllers these parameters belong to.

## The rule: measure first, tune second

Most "tuning problems" are measurement problems. Before touching a slider, make sure the model of
the car is right, in this order:

| Step | Tool | Fixes |
|---|---|---|
| 1. Camera calibrated, field locked | `pi/pi_calibrate.py`, Pi preview | Poses that jump or are off near the edges |
| 2. Loop delay measured | `tools/measure_latency.py` | Oscillation and overshoot everywhere |
| 3. Motors measured | `tools/calibrate_motors.py`, speed curve | Drifting on straight lines, wrong speeds |
| 4. Scored | `tools/eval_motion.py` | Gives you numbers to compare against |

Then change **one** parameter at a time, run `eval_motion.py` again, and keep the change only if
the numbers got better.

## Where parameters live

| Place | What | When it applies |
|---|---|---|
| Tuning page (`http://localhost:8000/tuning`) | The 16 controller parameters below | Immediately, from the next goal or path. **Save to config** writes `config/tuning.yaml` |
| `config/tuning.yaml` | Saved tuning values | Loaded on top of the defaults at startup. Delete the file to go back to the defaults |
| Main page → Safety, top bar | Speed limit, wall margin, keep-cars-apart | Immediately; not saved |
| `config/field.yaml` | Measurements: robot model, speed curve, delay, field, towers | At startup |
| Car firmware `config.h` | Dead zones, duty ramp, watchdog, PWM | After reflashing the car |

## Every parameter on the tuning page

Defaults are the values the controllers start with when there is no `config/tuning.yaml`.

### Lines and curves (FOLLOW phase, pure pursuit)

| Parameter | Default | What it does | Raise it when | Lower it when |
|---|---|---|---|---|
| Cruise speed `v_path` | 200 mm/s | Top speed along the path | You want faster runs and it follows well | It wobbles at speed, or cuts curves |
| Acceleration / braking `accel` | 400 mm/s² | How fast it speeds up, and how early it brakes for the goal (v = √(2·accel·distance left)) | It crawls into every goal | It overshoots the goal or pops a wheelie on starts |
| Curve speed limit `lat_accel` | 500 mm/s² | Slows down in tight bends (v = √(lat_accel/curvature)) | It is too slow in gentle curves | It runs wide or skids in tight bends |
| Lookahead `lookahead` | 120 mm | How far ahead on the path it aims | It weaves left and right on straight lines | It cuts corners or misses small details of a drawn path |
| Arrive within `pos_tol` | 12 mm | Distance to the goal that counts as arrived | It hunts back and forth around the goal | It stops visibly short |
| Accept a just-passed goal within `pass_tol` | 20 mm | A goal already behind the car and this close counts as reached | It reverses to chase goals it just missed | It stops noticeably past the goal |
| Stop and re-face if off by `reface_angle_deg` | 70° | If the path ahead is this far off the nose, stop and turn to it | It stops and turns in the middle of curvy paths | It drives big loops to get back onto a path |

### Final orientation (TURN and ALIGN phases, `TurnTo`)

| Parameter | Default | What it does | Raise it when | Lower it when |
|---|---|---|---|---|
| Face the path within `face_tol_deg` | 20° | How precisely it faces the path before driving | It spends long nudging before every start | It sets off at an angle and swerves |
| Final heading within `ang_tol_deg` | 4° | How precisely it reaches the final heading | It keeps nudging around the target heading | The final heading is not precise enough |
| Wait between nudges `wait_s` | 0.35 s | Pause after each turn pulse before measuring again | The final turn still swings left and right | Turns are slow and the car clearly stops early |
| Nudge wheel speed `pulse_speed` | 150 mm/s | Speed of the driving wheel during a turn pulse | Pulses often do not move the car (heavier car, grippy floor) | Each pulse overshoots |
| New pulse turns (1) / old continuous (0) `turn_style` | 1 | 1 = pulse-and-wait pivots, 0 = the old continuous turn | | Only set 0 to film or show the difference |

### Delay prediction

| Parameter | Default | What it does | Notes |
|---|---|---|---|
| Prediction on (1) / off (0) `predict` | 1 | Controllers steer on the pose 0.2 s ahead instead of the camera's old pose | Keep it on. Off is only for comparison |
| Loop delay to predict over `latency_s` | 0.20 s | How far ahead to predict | Set it from `tools/measure_latency.py`, not by feel. Too small: overshoot remains. Too large: it stops early and turns too little |

### Motors

| Parameter | Default | What it does | Notes |
|---|---|---|---|
| Left trim `trim_left` | 0.923 | Multiplies the left wheel's speed | The faster wheel gets a trim below 1 |
| Right trim `trim_right` | 1.0 | Multiplies the right wheel's speed | Prefer `calibrate_motors.py` over guessing |

## Symptom → what to change

| You see | Most likely cause | Try, in this order |
|---|---|---|
| The car turns the wrong way or spins without stopping | Wheels wired the other way round | `wheels_swapped: true` for that car in `config/field.yaml`; a single backwards wheel: `LEFT_REVERSED` / `RIGHT_REVERSED` in the firmware |
| It curves when driving "straight" | Unequal motors | Run `calibrate_motors.py --write`, or lower the trim of the faster wheel by 0.01–0.02 |
| It swings left and right at the end of every turn | Delay not compensated | `predict` = 1, `turn_style` = 1, `latency_s` from a fresh measurement, then raise `wait_s` to 0.45 |
| It weaves on straight lines | Lookahead too short, or wrong delay | Raise `lookahead` to 150–180; check `latency_s` |
| It cuts the corners of drawn paths | Lookahead too long | Lower `lookahead` to 80–100; lower `lat_accel` |
| It overshoots the goal and comes back | Brakes too late | Lower `accel` to 300; raise `pass_tol` to 25–30 |
| It stops short and creeps back and forth | It cannot move slower than ~100 mm/s | Raise `pos_tol` to 15–20 and `pass_tol` to 25 |
| The final turn takes many tiny nudges | Tolerance tight, or pulses too weak | Raise `ang_tol_deg` to 5–6; raise `pulse_speed` |
| The car shifts sideways during a big final turn | Pivot geometry: a pivot moves the pose point ([why](kinematics.md#4-pivot-turns)) | Expected. Arrive already facing the right way (saved moves do this), or accept the shift |
| It stops dead mid-path | Safety | Look at the car's card: geofence, too close to another car, not seen by the camera |
| Everything stops at once | Camera data lost | Check the Pi preview says FIELD LOCKED and the Pi is on the network |
| Poses jump around | Tracking | Matte markers, steady light, exposure (`--exposure-us`), redo the lens calibration |

## Checking a change with numbers

```bash
python tools/control_center.py --real --pi <pi-address>         # terminal 1
python tools/eval_motion.py --robot 0                            # terminal 2: straight, turn, curve
python tools/eval_motion.py --robot 0 --tests turn --repeat 3    # just turns, three times
```

For each maneuver it prints the time taken, how far the car strayed from the line or path, how far
from the goal it stopped, the final heading error, and the number of **wobbles** (how often the
heading reversed while settling). Compare those before and after a change. The 2026-11-06 baseline
is in [testing](testing.md#results-on-the-real-car).

## Firmware constants (change with care)

These live in `firmware/car_esp8266/robofield_car/config.h` and need a reflash.

| Constant | Value | Effect | Before changing it |
|---|---|---|---|
| `LEFT_DEADZONE`, `RIGHT_DEADZONE` | 0.26, 0.20 | Minimum duty each wheel gets for any non-zero command | Lowering them would allow slower driving, but the speed curve must then be measured again |
| `DUTY_RAMP_PER_S` | 6.0 | How fast the duty may change (full power in ~0.17 s) | Faster ramps make starts jerkier and can lift the front |
| `WATCHDOG_MS` | 200 | Motors off if no command for this long | Keep it well above the 33 ms command period |
| `REVERSE_DEAD_MS` | 80 (unused) | A pause before reversing a wheel, removed for responsiveness | If a motor driver burns out, re-enable it |
| `ENABLE_BATTERY_MONITOR` | false | Battery reading and low-battery cut-off | Check the divider reading against a multimeter before enabling |
