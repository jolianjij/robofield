# Bring-up plan: calibration -> coordinate system -> drive to a pose

Each step has a pass check. Do not move on until it passes.

## 0. Before you start
- Pi 5 set up (`pi/README.md`), camera on the beam over the centre, towers in place (ID 0 FL,
  1 FR, 2 BR, 3 BL, notches toward the back wall), marker heights measured in `config/field.yaml`.
- One car flashed (`docs/platformio_vscode.md`), id set, `python tools/cars.py list` shows it,
  `python tools/cars.py test 0` moves it correctly (if a wheel runs backwards, set
  `LEFT_REVERSED` / `RIGHT_REVERSED` in `config.h` and update over WiFi).
- 2S battery wiring: divider 100k / 33k; `cars.py list` shows ~7.4-8.4 V.

## 1. Camera calibration (~20 min)
```bash
.venv/bin/python pi/pi_calibrate.py --cols 9 --rows 6 --square 25
```
Move the chessboard slowly: centre, all four corners, edges, tilted.
**Pass:** RMS < 0.5 px.

## 2. Tracking check (~15 min)
```bash
.venv/bin/python pi/pi_tracker.py --preview 8080
```
Open `http://robofield-pi.local:8080`.
**Pass:** FIELD LOCKED, 4 towers, camera ~(500, 500, 1300) mm, reprojection < 1 px.
Put the car at measured spots (250,250) (500,500) (750,750): **pass** within about +-5 mm.

## 3. Coordinate system on the live image
The preview now draws a 100 mm grid (tile seams every 250 mm in thick white), the X (red) and
Y (green) axes at the front-left corner, and every robot's heading arrow.
**Pass:** the thick lines sit on the real tile seams and the walls, everywhere in the image.
If they are off at the edges: redo the calibration. If shifted or rotated: check the tower
positions/heights in the config and that each marker is on the right tower, top toward the notch.

## 4. Drive to a point
```bash
python tools/real_goto.py --robot 0
```
**Pass:** visits the 4 points, each within ~10 mm.

## 5. Drive to a position + orientation, with a planned path
```bash
python tools/goal_gui.py --real --robot 0 --pi robofield-pi.local
```
Click where the car should go, drag toward where it should face, release. The orange path also
appears on the Pi preview. The car: turns to face the path -> follows it -> stops -> turns to
the heading. **Pass:** within ~10 mm and ~3 deg.

Try it first without hardware: `python tools/goal_gui.py` (simulator).

## If something is off
| Symptom | Fix |
|---|---|
| Car turns the wrong way / spins | a motor is reversed: `LEFT_REVERSED` / `RIGHT_REVERSED` |
| Car stops short or hunts at the end | raise `deadzone` in `config/field.yaml` (0.08 -> 0.12) |
| Overshoots corners / oscillates | lower `k_heading` in PoseController (5 -> 3) or `v_path` |
| Pose jumps | lighting/glare on markers; lower `--exposure-us`, add diffuse light |
| Final heading slow | raise TurnTo `k` (4 -> 5) |
| Car too slow / too fast | measure top speed, set `max_wheel_rpm` in the config |
