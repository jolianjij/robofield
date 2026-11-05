# Vision and pose estimation strategies

Every estimator implements `robofield.vision.PoseEstimator` and returns `Pose(x, y, theta)`
in the field frame, so they can be swapped in `config/field.yaml` (`vision.estimator`) and compared.

## A. Planar homography — `homography` (flat corner setups only)

Corner marker centres → `cv2.findHomography` → every robot marker's corners mapped to field mm.
Heading = direction from the marker's bottom edge to its top edge.

- No calibration required (it still helps at the image edges).
- Very stable heading: uses all 4 corners in field space.
- **Parallax:** car tops sit ~45 mm above the floor, so they appear farther from the centre
  than they are (≈13 mm error at 400 mm off-centre with the camera at 1.4 m).
  Two fixes, both supported:
  1. Mount the corner markers at the same height as the car tops (`corner_plane_height_mm`).
  2. Correct in software using camera and marker heights (`parallax_correct`).
- Synthetic test (1 px/mm, no noise): mean error 0.7 mm, 0.4°.

## B. Per-marker 3D pose — `pnp`

`solvePnP` with `SOLVEPNP_IPPE_SQUARE` gives each marker's 6-DoF pose relative to the camera;
the corner markers give the camera's pose in the field frame.

- Needs a good camera calibration (`tools/calibrate_camera.py`).
- Handles height and tilt naturally; no parallax step.
- Small markers seen from straight above suffer the **IPPE ambiguity**: orientation can flip
  between two solutions, making heading noisy. Quality = how flat the marker looks.
- Best for comparison experiments and anything leaving the plane.

## C. Tower pose + ray-plane intersection (default) — `plane`

1. The corners of every visible tower marker (known 3D positions at 140–260 mm) go into
   `solvePnP` → camera pose in the field frame. Non-coplanar points make it very stable,
   and any 2 towers are enough if the others are blocked.
2. Each robot marker corner pixel becomes a 3D ray, intersected with the plane z = 45 mm.

- No parallax error (the robot height is in the geometry).
- No per-marker PnP, so no IPPE orientation flips on small robot markers.
- Needs a camera calibration, which a 120° wide lens requires anyway.
- Synthetic test (Pi Camera 3 Wide at 1.3 m, 2° mount tilt, 0.3 px noise): mean error ~1 mm.

**Resolution check:** at 2304 × 1296 a 35 mm robot marker is only ~26 px wide, about 4.3 px
per cell, which is borderline once lens blur and edge distortion are added. A **40 mm** robot
marker gives ~30 px. Test both on the real field before printing many.

**Where detection runs:** the Pi Camera needs a Raspberry Pi. Two options:
- **Detect on the Pi (recommended):** a Pi 5 runs ArUco + the estimator and sends poses
  (a few hundred bytes) to the laptop over UDP. Lowest latency, no video stream.
- **Stream video to the laptop:** simpler code, but H.264 encoding adds ~50–150 ms latency.

## D. Planned improvements

| Idea | Why |
|---|---|
| Kalman / EKF filter with a unicycle model + commanded wheel speeds | Smooths noise, estimates velocity, bridges dropped frames |
| Latency compensation | Predict the pose ~50–100 ms ahead to match "now" |
| AprilTag 36h11 dictionary | Fewer false detections, better corner accuracy |
| ROI tracking | Search only near last known positions → higher fps |
| Two markers per robot | Redundancy and better heading (space permitting) |
| Marker-free tracking (colour / YOLO-OBB) | Research track; compare against ArUco ground truth |
| Multi-camera fusion | Larger fields, occlusion robustness |

## How to compare estimators

Place a car at measured positions (grid of points + known angles), log poses from each estimator,
and report mean / max position error, heading error, jitter (std over 2 s still), and fps.
