# RoboField / RLCR Framework — project context for Claude Code

## Current state (2026-11-06) — read this first; the older notes below are kept as history

- **Firmware is in the repo:** `firmware/car_esp8266/robofield_car/` is the sketch the car runs
  (ESP8266 D1 Mini + DRV8833, fw 5). `secrets.h` is git-ignored; copy `secrets.example.h`.
  `firmware/car/` (ESP32-C3, PlatformIO) is the old version — `ControlCenter.flash()` still targets it.
- **Motion control was rebuilt from measurements** — see `docs/control.md` before touching it:
  loop delay ~0.2 s (Smith predictor, `robofield/control/predictor.py`); the firmware already
  compensates the dead zone, so the laptop's is 0; measured `speed_curve` and `trim_left 0.923`
  in `config/field.yaml`; lines/curves use pure pursuit; final turns are single-wheel pivot
  pulses with waits (`TurnTo`), not a continuous PID. The PID-based plan described further down
  was tried and replaced.
- **This car's wheels are wired swapped** relative to the software (`robots[0].wheels_swapped`),
  handled in `RealBackend._send`. `DiffDrive.inverse()`'s sign fix is correct — never revert it.
- **Tuning:** `http://localhost:8000/tuning`, saved to `config/tuning.yaml`. Score changes with
  `tools/eval_motion.py`; re-measure with `tools/measure_latency.py`, `tools/calibrate_motors.py`.
- Real-car results: straight 350 mm stops 6–7 mm from goal; 120° turn ends within 2–5° in ~4 s.
- **Next:** lower the firmware dead-zone floors (`LEFT/RIGHT_DEADZONE`) for finer low-speed control
  and re-run `calibrate_motors.py --write`; build a second car; retrain the race RL policy (the
  saved one was trained on the old, linear motor model and does not transfer).
- On Windows, call the local server as `127.0.0.1`, not `localhost` (IPv6-first adds ~2 s per request).

Read this fully before making changes. It reflects the REAL current state as of 2026-11-05, including several deviations from the original design that happened during hardware bring-up. Put this file at the repo root as `CLAUDE.md`.
What this project is
An open-source, low-cost multi-robot research platform:

* A 1×1 m 3D-printed arena with 4 ArUco towers at the corners.
* An overhead camera (Raspberry Pi 5 + Pi Camera Module 3 Wide) that tracks every robot's position and heading, dozens of times a second.
* Small, "dumb" robot cars (no onboard compute/vision) that only drive, grip, and light up.
* One laptop ("Control Center") that sees the whole field through the camera and decides what every robot should do — manual driving, go-to-pose, drawn-path following, games, and eventually trained RL policies and multi-agent coordination — then sends commands over WiFi.
* The same backend interface runs the simulator (MuJoCo) and the real field, so controllers and RL policies can be developed and tested in sim first.

Three machines, three roles

```
Raspberry Pi 5 (pi/)          Laptop (robofield/, tools/)         Cars (firmware)
  camera -> ArUco detect  --UDP-->  ControlCenter (30 Hz loop)  --UDP-->  ESP8266 D1 Mini
  -> field poses                     controllers, safety,                 motors, servo, LEDs
  live preview + overlay  <--UDP--   web GUI (:8000)             <--UDP--  battery/status

```

* Pi: only runs the tracker (`pi/pi_tracker.py`), calibration (`pi/pi_calibrate.py`), and serves a live MJPEG preview with a coordinate-grid overlay on :8080.
* Laptop: everything else — `robofield/app/control_center.py` is the single hub everything else (games, RL, multi-agent) should plug into.
* Cars: WiFi UDP, OTA-updatable, no onboard intelligence.

⚠️ Hardware deviated from the original design — read this before touching firmware
The project was originally designed around an ESP32-C3 Super Mini. During bring-up:

1. The first ESP32-C3 burned out (likely a short or 5V backfeed onto a GPIO).
2. It was replaced with a Wemos D1 Mini (ESP8266) — a different chip family entirely (different WiFi stack, software PWM only, EEPROM instead of Preferences/NVS, Servo library instead of LEDC). The car currently running is an ESP8266, not an ESP32.
3. The first motor driver (an L9110S) also burned out from a reversal current spike. It was replaced with a DRV8833 (same 2-pin-per-motor wiring style, no firmware interface change needed).
4. The firmware was developed and iterated entirely in the Arduino IDE, as a flat sketch folder (`robofield_car.ino`, `config.h`, `secrets.h`, `protocol.h` all in one folder), NOT in the PlatformIO project that still exists at `firmware/car/` in this repo. That PlatformIO ESP32-C3 firmware is now STALE and does not reflect the real hardware.

First task for Claude Code: reconcile this. The live Arduino sketch currently lives outside the repo (on the Desktop, `robofield_car/` folder) and needs to be brought into version control, e.g. as `firmware/car_esp8266/`, alongside (or replacing) the stale PlatformIO version. Ask the user for the current contents of that folder if they're not already copied in — the version below is the last one confirmed working, but may have since been hand-edited further on the real file.
Confirmed final, working pin mapping (ESP8266 / Wemos D1 Mini)

```c
#define PIN_R_IN1 D5     // right motor, driver IN1
#define PIN_R_IN2 D6     // right motor, driver IN2
#define PIN_L_IN1 D7     // left motor, driver IN1
#define PIN_L_IN2 D3     // left motor, driver IN2  (GPIO0 — boot-sensitive; works but don't add more load here)
#define PIN_SERVO D2     // gripper servo (shares the onboard LED pin — flickers with gripper, harmless)
#define PIN_LEDS  D1     // 3x WS2812B addressable RGB
#define PIN_VBAT  A0     // battery sense — wired but DISABLED in firmware (see below)

#define LEFT_DEADZONE  0.26f   // measured: left wheel's real minimum duty to move at all
#define RIGHT_DEADZONE 0.20f   // measured: right wheel's real minimum duty to move at all
#define ENABLE_BATTERY_MONITOR false   // battery divider on A0 not yet exercised in firmware — set true once trusted
#define FW_VERSION 5+   // bump on every firmware change so `cars.py list` confirms what's actually flashed

```

Board in Arduino IDE: "LOLIN(WEMOS) D1 R2 & mini" under ESP8266 Boards. Driver mode: `DRIVER_DUAL_PWM` (2 inputs per motor — matches the DRV8833). Motor PWM frequency is 1 kHz (ESP8266 software PWM can't do the 20 kHz the C3 could — audible whine is expected/normal).
Power: 2S LiPo/18650 pack (7.4–8.4 V) → DRV8833 VM directly (no boost converter needed, driver's 10.8 V max covers it) + 5V buck for the ESP8266/servo/LEDs. Battery divider: 100kΩ/33kΩ into A0 (not yet enabled in firmware — see above).
Known open items on the firmware (do not silently "fix" without flagging to the user)

* Reverse dead-time protection was deliberately REMOVED at the user's request (to make direction changes feel more responsive). This was the exact mechanism added after the L9110S burned out from a reversal spike. The DRV8833 has its own current limiting/thermal shutdown, so this is a reasonable trade-off, but if a driver burns out again, re-add a short (~80 ms) zero-pause before reversing each wheel.
* Per-wheel speed trim for a forward-driving drift was proposed but never finished — the user never confirmed which direction the real car drifts, so no `LEFT_TRIM`/`RIGHT_TRIM` values were ever set. If drift is reported, measure it and add trim constants (pattern discussed in-chat: scale `target[i]` by a trim multiplier before the duty ramp).
* Camera/vision config now reflects the real car, already applied in `config/field.yaml`: `robot_size_mm: 40`, `robot_marker_height_mm: 84` (the car is 84 mm tall — taller than the original 45 mm default design assumed). Robot 0's marker is ArUco ID 10, 40 mm black square, centred over the axle, top edge toward the front.

Software control stack — just upgraded, NOT yet hardware-tested
A PID-based control upgrade was just handed to the user as exact code (new file `robofield/control/pid.py`, plus rewrites of `GoToPoint` and `TurnTo` in `robofield/control/goto.py`, and `PoseController` in `robofield/control/pose_controller.py`). It replaces plain proportional heading control with PID (derivative on the EKF's measured turn rate, not on raw error) to fix:

1. Violent oscillation on sharp turns,
2. Shaking while settling into a final target orientation (caused by dead-zone compensation always boosting any nonzero command to ~20–26% power, even for a 1° error),
3. Curve smoothness while following a drawn path.

It keeps a "creep-pulse" mechanism (short power pulses + pauses, using `time.monotonic()`) for the final few degrees of a turn-in-place, since continuous PID still can't finely control a motor with a large dead zone — pulsing is what actually lets it settle precisely instead of oscillating.
Status: code was handed over in the previous session; NOT yet confirmed working on the real car. First task after the firmware reconciliation above: test this on hardware (`tools/real_goto.py` or `tools/control_center.py --real`) and tune `kd`/`ki` per the guidance already given to the user (raise `kd` if still oscillating, lower if sluggish, only enable `ki` if a steady curve persists after firmware trim).
Also applied earlier, should already be in place: a sign flip in `DiffDrive.inverse()` (`robofield/control/kinematics.py`) to fix a rotate-the-wrong-way bug — verify this is actually in the repo file, it was given as an instruction, not necessarily re-synced here.
Repo layout

```
robofield/
  app/            ControlCenter (the hub), Safety layer, web GUI (static/index.html), HTTP server
  control/        kinematics (DiffDrive), EKF, goto.py (GoToPoint/TurnTo), pose_controller.py,
                  path.py (straight-line planner), pid.py (new)
  sim/            MuJoCo scene + SimBackend
  real/           RealBackend — same interface as SimBackend, talks to the real field
  games/          goto.py, sumo.py, pushbox.py (cooperative: box too heavy for one car)
  envs/           Gymnasium + PettingZoo wrappers
  rl/             policies.py (MLPPolicy/SB3Policy), cem.py (numpy-only trainer), marl.py (shared-policy MARL)
  comm/           protocol.py (packet formats), udp_link.py (laptop<->cars), pose_link.py (Pi<->laptop)
pi/               pi_tracker.py (with coordinate-grid overlay), pi_calibrate.py (interactive, live
                  preview — rewritten once already because the original had no display), install.sh
firmware/car/     STALE PlatformIO ESP32-C3 project — superseded by the real Arduino/ESP8266 sketch,
                  see the hardware deviation note above
tools/            control_center.py (main entry point), cars.py (list/set-id/test/drive),
                  keyboard_drive.py (arrow keys, 1/2 speed, hold-to-move), real_goto.py,
                  train.py, train_team.py, evaluate.py
config/field.yaml Every measurement: field, towers, robot (now 40mm/84mm marker), control, battery
docs/             control_center.md, tomorrow.md, platformio_vscode.md — tomorrow.md's checklist is
                  mostly done (through "drive to a point"); PID retest is the new next step

```

How to run things

```bash
python tools/control_center.py                                  # simulator, no hardware
python tools/control_center.py --real --pi <pi-address>          # real field, opens :8000
python tools/cars.py list                                        # find cars, battery, signal
python tools/cars.py test 0                                      # forward/back/spin/gripper/LEDs
python tools/keyboard_drive.py --robot 0                         # arrow-key manual driving
python tools/real_goto.py --robot 0                              # simple closed-loop point test
python tools/train.py --game goto --algo cem                     # CEM training, no PyTorch needed

```

On the Pi: `.venv/bin/python pi/pi_tracker.py --preview 8080`, calibrate first with `pi/pi_calibrate.py --cols 8 --rows 5 --square 28` (the real chessboard used was 9×6 squares, i.e. 8×5 inner corners, 28 mm squares — not the 9×6/25mm originally assumed).
Immediate next steps (in order)

1. Bring the real Arduino/ESP8266 firmware into the repo (currently only on the Desktop).
2. Test the new PID controller on the real car — tune `kd`/`ki` against real behavior.
3. Re-verify the coordinate-grid overlay still lines up (camera/marker config changed since it was last checked: 40mm/84mm marker).
4. Build a second car (wiring and firmware are now known-good, should be much faster).
5. Try the cooperative push-box game with 2 cars via the Control Center's Games panel.
6. Start drafting a paper skeleton (architecture, cost comparison vs Duckietown/ TurtleBot/Kilobots, measured results).

Things to NOT silently change

* Don't re-add firmware safety features (reverse dead-time, battery monitor) without flagging it to the user first — both were deliberately disabled/removed by explicit request, with trade-offs the user accepted knowingly.
* Don't assume the PlatformIO `firmware/car/` project is current — it isn't.
* Don't change `config/field.yaml`'s marker/height values without re-confirming real measurements — the tracking math is sensitive to these.
