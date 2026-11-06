# Motion control on the real car

How RoboField drives a car with no sensors of its own, using only an overhead camera that is
always a little behind. Every number here was measured on the real field on 2026-11-06; the tools
that measured them are in `tools/`, so they can be re-measured for any new car.

## 1. Measure the car

### The loop delay — `tools/measure_latency.py`

From standstill the car is sent a short in-place turn, and the time until the camera sees its
heading change is recorded (6 trials, alternating direction).

| | median | range |
|---|---|---|
| command sent → motion seen by the laptop | **234 ms** | 157–422 ms |

This covers WiFi to the car, the firmware's duty ramp, the motors spinning up, camera exposure,
ArUco detection on the Pi (~23 ms) and the UDP hop back. The system had assumed 70 ms.
`control.latency_s` is 0.20 (the part of it that is pure delay rather than spin-up).

What a 0.2 s delay does: at the car's turning speed (3–5 rad/s), the heading the controller sees is
40–60° behind the real one. A controller correcting that stale heading keeps turning after the
robot has already arrived, overshoots, then corrects back — **the left-right oscillation at the
end of every move.**

### The motors — `tools/calibrate_motors.py`

Straight-line speed per command (`robot_model.speed_curve`):

| command | 0.03 | 0.08 | 0.15 | 0.25 | 0.50 |
|---|---|---|---|---|---|
| speed, mm/s | 116 | 152 | 185 | 200 | 282 |

- **There is no slow.** The firmware lifts every non-zero command over each wheel's dead zone, and
  those floors sit above where these motors actually start, so the smallest command already drives
  ~100 mm/s. The laptop was additionally compensating the dead zone itself, on top of the firmware,
  which made the smallest correction ~30% power; that double compensation is removed
  (`robot_model.deadzone_* = 0`).
- **Unequal motors.** With equal commands the left wheel ran 259 mm/s against the right's 239 mm/s,
  curving with a 0.6–0.9 m radius. `trim_left = 0.923` equalises them.
- **Turning in place is hard.** Spinning both wheels in opposite directions scrubs them sideways:
  it needs a big command to break free at all, then turns at 3–6 rad/s regardless of the command,
  and coasts for up to ~0.9 s after the motors stop.

Every controller maps the speed it wants through the measured curve (`DiffDrive.inverse`), so
asking for 150 mm/s gives 150 mm/s; speeds below the minimum are scaled up together so the turn
ratio, and therefore the path's curvature, is kept.

## 2. Predict, don't react — `robofield/control/predictor.py`

Each robot's pose is reported to the controllers **0.2 s in the future**: the state the robot will
be in when the command computed now reaches it. This is a Smith predictor. It starts from the
filtered camera measurement and replays the commands that were already sent during the delay window
— on their way to the wheels, not yet visible to the camera — through the motor model, with a short
lag for the motors and the firmware ramp.

Simply extrapolating the last measured speed does not work: when the controller reverses its
steering, extrapolation keeps swinging the old way and invents an overshoot that isn't there (seen
on the real car as a predicted heading 115° away from the real one). Replaying the actual commands
knows the steering already changed.

Switch it off on the tuning page (`Prediction on (1) / off (0)`) to see the difference.

## 3. Lines and curves — pure pursuit (`robofield/control/pose_controller.py`)

The car aims at a point `lookahead` (120 mm) ahead on the path and drives the circular arc that
reaches it: curvature = 2 sin(α) / distance. It is geometry, not a tuned heading loop, so straight
lines do not weave and curves are followed as curves. Speed ramps up at `accel`, is capped in tight
curves so the sideways acceleration stays under `lat_accel`, and ramps down into the goal like a
braking distance (v = √(2 · accel · distance left)). It all runs on the predicted pose.

## 4. Turning precisely — pulse, wait, measure (`TurnTo` in `robofield/control/goto.py`)

Because the car cannot turn slowly and coasts after stopping, no continuous controller can fine-tune
its heading: every correction is too big and arrives too late. So every turn is a sequence of moves:

1. **Pulse** — one wheel drives, the other stays still: a pivot about the still wheel. It breaks
   free far more gently than spinning both wheels, turns half as fast and coasts less. The wheel
   doing the work alternates between pulses, so each pivot's small sideways shift is undone by the next.
2. **Wait** — longer than the delay, until the camera shows the robot at rest. No decision is ever
   made on data older than the move before it.
3. **Measure and learn** — the pulse length comes from a two-number model of this car: how long it
   takes to start turning from a standstill, and how fast it then turns. Each pulse's measured
   result updates both. A pulse that did not move the car lengthens the start time.

Result on the real car: a 120° turn takes ~4 s and ends within 2–5°. The old continuous controller
(still available, `New pulse turns (1) / old continuous (0)` on the tuning page) swung past the
target 14 times in 5 s and ended ~19° off.

## 5. Tuning it yourself

`http://localhost:8000/tuning` shows every parameter above as a live slider, a map to set test
goals and live telemetry. `Save to config` writes `config/tuning.yaml`, loaded on top of the
defaults at startup (delete it to go back). `tools/eval_motion.py` scores straight lines, turns and
curves with numbers, so a change can be judged by data rather than by eye.

## Known limits / next steps

- Lowering `LEFT_DEADZONE` / `RIGHT_DEADZONE` in the car firmware would give the laptop real
  low-speed control (see `firmware/car_esp8266/README.md`), then re-run `calibrate_motors.py --write`.
- The firmware's brake flag only lasts one loop iteration; a real brake would stop turns sooner.
- The tracker on the Pi and the laptop have unsynchronised clocks; synchronising them (NTP) would
  let the predictor use each frame's true capture time instead of a fixed delay.
