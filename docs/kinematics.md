# Kinematics and the motor model

How the software describes the car's motion, and how it turns "drive along this arc at this speed"
into the two numbers sent to the wheels. All of it lives in `robofield/control/kinematics.py`
(`DiffDrive`), with the car's measurements in `config/field.yaml` → `robot_model`.

## 1. The car as the software sees it

```
            front
        ┌───────────┐
        │     ●     │  ← ball caster (front)
        │           │
   ▐█▌  │     ✚     │  ▐█▌   ✚ = the pose point (x, y): midway between the wheels,
  left  │   marker  │  right      right under the centre of the ArUco marker
 wheel  └───────────┘  wheel
        |←── T ──→|   T = track = 56 mm (wheel centre to wheel centre)
```

| Symbol | Meaning | Value on the current car | Config key |
|---|---|---|---|
| T | track: distance between the wheel centres | 56 mm | `track_mm` |
| r | wheel radius | 16.5 mm | `wheel_radius_mm` |
| v | forward speed of the pose point | mm/s | |
| ω | turn rate, counter-clockwise positive | rad/s | |
| v_L, v_R | left and right wheel rim speeds | mm/s | |
| θ | heading: 0 along +x, counter-clockwise positive | rad | |

The pose `(x, y, θ)` is the point midway between the wheels, because that is the point a
differential-drive robot turns around. The marker is mounted right above it, top edge toward the
front, so the camera measures this point directly.

## 2. Body motion and wheel speeds

A differential-drive car has no steering: it turns by running its wheels at different speeds.

**Forward kinematics** (wheel speeds → body motion), `DiffDrive.forward()`:

```
v = (v_R + v_L) / 2
ω = (v_R − v_L) / T
```

**Inverse kinematics** (body motion → wheel speeds), `DiffDrive.inverse()`:

```
v_L = v − ω · T / 2
v_R = v + ω · T / 2
```

Driving along a circle of radius R means ω = v / R (curvature κ = 1 / R). Three useful cases:

| Motion | v | ω | v_L | v_R |
|---|---|---|---|---|
| Straight at 150 mm/s | 150 | 0 | 150 | 150 |
| Arc of radius 300 mm, counter-clockwise, at 200 mm/s | 200 | 0.667 | 181 | 219 |
| Spin in place at 4 rad/s | 0 | 4 | −112 | +112 |
| Pivot about the still left wheel | s / 2 | s / T | 0 | s |

> The sign convention matters. `inverse()` must be the exact inverse of `forward()`; when it once
> had v_L and v_R swapped, every heading correction turned the car the wrong way and the controller
> oscillated without end. The docstring of `inverse()` records this. If a real car turns the wrong
> way, its wiring is mirrored: set `wheels_swapped: true` for it in `config/field.yaml` (the swap
> happens at the radio, in `RealBackend`) instead of changing the equations.

### Moving the pose forward in time

`integrate(x, y, θ, v, ω, dt)` moves a pose along the exact arc a constant (v, ω) produces:

```
θ' = θ + ω·dt
x' = x + (v/ω)·(sin θ' − sin θ)
y' = y − (v/ω)·(cos θ' − cos θ)          (straight line when ω ≈ 0)
```

## 3. The motor model: what a command really does

A command is a number from −1 to 1 per wheel. On an ideal motor, speed would be proportional to
it. On the real car it is not, so the measured curve is stored and every controller goes through it.

### The measured speed curve

Measured on the real car on 2026-11-06 (straight-line speed per command, from the camera):

| Command | 0 | 0.001 | 0.03 | 0.08 | 0.15 | 0.25 | 0.35 | 0.5 | 1.0 |
|---|---|---|---|---|---|---|---|---|---|
| Speed, mm/s | 0 | 100 | 116 | 152 | 185 | 200 | 255 | 282 | 340 |

Two things stand out:

- **There is no slow.** The firmware lifts any non-zero command above each wheel's dead zone, so
  the smallest command already drives about 100 mm/s. That is `DiffDrive.min_wheel_speed`.
- **It is far from a straight line.** Half the command range covers 100–282 mm/s.

`wheel_speed(u)` looks the speed up by linear interpolation between the points;
`wheel_command(speed)` does the reverse. The top speed, 340 mm/s, is `max_wheel_speed`.

### From (v, ω) to two commands: what `inverse()` does, in order

1. **Ideal wheel speeds:** v_L = v − ωT/2, v_R = v + ωT/2.
2. **Too fast?** If either is above 340 mm/s, both are scaled down by the same factor. The turn
   ratio, and so the curvature of the path, is kept; clipping only the fast wheel would bend the path.
3. **Too slow?** If the faster wheel is below 100 mm/s, both are scaled *up* by the same factor
   until it reaches 100 mm/s. A controller asking for "slowly" still moves the car on the same arc,
   just faster, instead of stalling short of every goal.
4. **Trim:** each wheel speed is multiplied by its trim (`trim_left = 0.923`, `trim_right = 1.0`).
   With equal commands the left wheel ran about 8% faster and the car curved on a 0.6–0.9 m radius;
   the trim slows the left wheel so equal requests drive straight.
5. **To a command:** each speed goes through `wheel_command()`. A wheel asked for less than half
   the minimum (under 50 mm/s) stops; anything between 50 and 100 mm/s runs at the minimum.

`forward()` undoes the trim, so `forward(*inverse(v, ω))` gives back the motion that was asked for.

### Worked example: a gentle curve

Pure pursuit asks for v = 200 mm/s on a 300 mm radius (ω = 0.667 rad/s):

```
v_L = 200 − 0.667 × 28 = 181.3 mm/s      v_R = 200 + 0.667 × 28 = 218.7 mm/s
neither above 340, neither below 100      → no scaling
trim:   181.3 × 0.923 = 167.4 mm/s         218.7 × 1.0 = 218.7 mm/s
curve:  167.4 lies between (0.08, 152) and (0.15, 185) → u_L = 0.08 + 0.07 × 15.4/33 = 0.113
        218.7 lies between (0.25, 200) and (0.35, 255) → u_R = 0.25 + 0.10 × 18.7/55 = 0.284
```

The right command is 2.5 times the left one, for wheels that differ by only 20%. A linear model
would have sent 0.53 and 0.64 and the car would have gone much faster and turned much less.

### Worked example: why it cannot spin slowly

Asking to spin at 2 rad/s gives v_L = −56, v_R = +56 mm/s. Both are under the 100 mm/s minimum, so
both are scaled up to ±100 mm/s, and the car spins at 2 × 100 / 56 = **3.6 rad/s**, the slowest it
can. On the real car, spinning both wheels in opposite directions also scrubs them sideways, needs
a hard kick to start, then coasts for up to 0.9 s after the motors stop. That is why final turns
are pivots, below.

## 4. Pivot turns

A pivot drives one wheel and holds the other still. The car rotates about the still wheel:

```
ω = s / T            (s = speed of the driving wheel)
the pose point moves on a circle of radius T/2 = 28 mm around the still wheel
```

At the turn controller's nudge speed of 150 mm/s, ω = 150 / 56 = 2.7 rad/s, half the spin rate,
and the car breaks free gently and stops sooner.

The price is that the pose point moves. Turning by Δθ about one wheel shifts it by a chord of
T · sin(Δθ / 2):

| Pivot | Shift of the pose point |
|---|---|
| 10° | 5 mm |
| 45° | 21 mm |
| 120° | 48 mm |

So `TurnTo` alternates which wheel does the work from one pulse to the next, and each pivot's shift
is mostly undone by the next. Saved moves go one step further and arrive at their start already
facing the right way, so the final turn is tiny (`ControlCenter._park()`).

## 5. The motion model inside the filter and the predictor

**EKF** (`robofield/control/estimator.py`). The state is `[x, y, θ, v, ω]`. Between camera frames it
assumes constant speed and turn rate:

```
x' = x + v cos θ · dt      y' = y + v sin θ · dt      θ' = θ + ω · dt      v' = v      ω' = ω
```

Changes in v and ω are modelled as noise (acceleration 1500 mm/s², angular acceleration 40 rad/s²).
Each frame corrects x, y, θ with the camera's measurement (1 mm, 0.015 rad). The filter's v and ω
are what the controllers use as the car's measured speed and turn rate.

**Delay predictor** (`robofield/control/predictor.py`). It starts from the filter's state and steps
through the last 0.2 s in 10 ms steps. At each step the wheels are assumed to follow the command
that was being sent at that moment, through `forward()` (so through the measured curve), with a
60 ms first-order lag for the motors and the firmware ramp:

```
v ← v + (dt / τ) · (v_cmd − v)        ω ← ω + (dt / τ) · (ω_cmd − ω)        τ = 0.06 s
x, y, θ advanced along (v, ω), with the mid-step heading
```

Turning in place is capped at 6 rad/s in the prediction, the fastest the car really turns while
scrubbing.

How much the delay matters, in numbers: at 200 mm/s the camera's position is 40 mm behind the car;
turning at 3 rad/s, its heading is 34° behind.

## 6. Keeping clear of the walls

`safety_margin_mm()` is the clearance used by path planning and the geofence: half the car's
diagonal (so a corner never touches a wall, even mid-turn) plus `control.clearance_mm`:

```
½ · √(70² + 50²) + 20 = 43 + 20 = 63 mm
```

## 7. Updating the model for a new or rebuilt car

1. Measure the track and wheel radius with calipers; put them in `robot_model`.
2. Stop the Control Center (the tool talks to the car and the Pi itself), put the car mid-field
   facing the centre, and run `python tools/calibrate_motors.py --robot 0 --write`. It spins the
   car at three commands and drives it forward and back, measures everything with the camera, and
   writes `trim_left`, `trim_right` and `max_wheel_rpm`.
3. Update `speed_curve` by hand: drive the car straight at a few commands (0.03, 0.08, 0.15, 0.25,
   0.5, 1.0), read its speed from the Control Center (`/api/state` → `v`) or a video, and enter the
   pairs. Keep the `[0.0, 0]` and `[0.001, <slowest speed>]` points at the start.
4. Run `python tools/measure_latency.py --robot 0` and put the result in `control.latency_s`.
5. Check with `python tools/eval_motion.py --robot 0` ([testing](testing.md)).

If you change `LEFT_DEADZONE` / `RIGHT_DEADZONE` in the firmware, the curve changes completely:
redo steps 2 and 3.
