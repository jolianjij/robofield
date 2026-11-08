# Raspberry Pi 5: camera tracker

The Pi captures the overhead camera, finds the ArUco markers, computes every robot's field pose
(mm, rad) and sends it to the laptop over the network about 30-50 times per second.

## 1. Prepare the SD card
1. Raspberry Pi Imager -> **Raspberry Pi OS (64-bit)**.
2. In the Imager's settings (gear icon): hostname `robofield-pi`, user `pi`, your WiFi,
   **enable SSH**. Write the card, boot the Pi.
3. From the laptop: `ssh pi@robofield-pi.local`

## 2. Connect the camera
Pi off. Camera Module 3 **Wide** -> Pi 5 **CAM/DISP 0** with the 22-pin-to-15-pin cable
(contacts facing the right way: the cable's metal side toward the connector's contacts).
Test: `rpicam-hello -t 5000` (a preview should open, or no error appears over SSH).

## 3. Install the tracker
```bash
git clone <your RoboField repo> robofield     # or copy the folder with scp
cd robofield
bash pi/install.sh
```

## 4. Calibrate the lens (once)
Print a chessboard, tape it flat to cardboard, and measure one square. `--cols` and `--rows` count
the **inner** corners (where four squares meet), not the squares. The reference build used a board
of 9 × 6 squares (8 × 5 inner corners) with 28 mm squares:
```bash
.venv/bin/python pi/pi_calibrate.py --cols 8 --rows 5 --square 28
```
Move the board slowly under the camera, covering the whole image, especially the corners,
tilted different ways. After 30 captures it saves `pi/camera.npz`. RMS under 0.5 px is good.

## 5. Run the tracker
```bash
.venv/bin/python pi/pi_tracker.py --preview 8080
```
- Open `http://robofield-pi.local:8080` on the laptop: live view with detected markers.
- It must say **FIELD LOCKED** and show the camera at about (500, 500, 1300) mm.
- If the image is too dark or blurry: `--exposure-us 6000`, `--gain 6`, `--focus 0.75`
  (focus is in dioptres = 1 / distance in metres).
- By default poses are broadcast on UDP port 5005; use `--laptop <ip>` to send to one computer.

## 6. Start automatically at boot
```bash
sudo cp pi/robofield-tracker.service /etc/systemd/system/
sudo systemctl enable --now robofield-tracker
journalctl -u robofield-tracker -f
```

## Checks
| Check | Expected |
|---|---|
| Towers seen | 4 (at least 2 needed) |
| Camera position | about (500, 500, 1300) mm |
| Tower reprojection error | under 1 px |
| Detection time | 20-40 ms per frame on a Pi 5 |
| Robot standing still | position jitter under ~1 mm |
