# The field

What the software needs to know about the field, and where each number goes in
`config/field.yaml`. The full CAD assembly of the field is in [`hardware/field/`](../hardware/field/README.md).

## Playing area

| | Value | Config key |
|---|---|---|
| Size | 1000 × 1000 mm | `field.width_mm`, `field.height_mm` |
| Floor | 4 × 4 tiles of 250 mm | `field.tile_size_mm` |
| Walls | 100 mm above the playing surface, 10 mm thick | `field.wall_height_mm`, `field.wall_thickness_mm` |

The origin (0, 0) is the inner bottom-left corner of the playing area, x along the bottom wall,
y along the left wall ([coordinates](coordinates.md)). Game zones are defined in code, not printed
on the floor; the race, for example, uses the outer ring of tiles as its track.

## Tracking towers

Four towers stand just outside the corners. Each carries an 80 mm ArUco marker facing up, at a
different height. The tracker uses them to find the camera's exact position and angle every frame;
markers at different heights pin that down far better than four markers on the floor would, and
any two visible towers are enough if the others are blocked.

| Tower | Marker ID | Centre (x, y) mm | Marker height |
|---|---|---|---|
| T3 (bottom-left) | 0 | (−71, −71) | 220 mm |
| T4 (bottom-right) | 1 | (1071, −71) | 260 mm |
| T2 (top-right) | 2 | (1071, 1071) | 180 mm |
| T1 (top-left) | 3 | (−71, 1071) | 140 mm |

Each marker lies flat with its **top edge toward +y**; the estimator relies on it. These values
are in `markers.corners` as `[x, y, z]`.

**Measure your build.** Tower positions and marker heights are what the whole coordinate system
hangs on. Measure each marker's centre position and its height above the playing surface after
assembly, and put the real values in the config. If the grid in the Pi preview does not sit on the
tile seams, these numbers are the first thing to check.

## Camera

| | Value | Config key |
|---|---|---|
| Camera | Raspberry Pi Camera Module 3 Wide (120° diagonal) | `camera.model` |
| Height | about 1300 mm above the playing surface, over the centre, facing down | `camera.height_mm` |
| Mode | 2304 × 1296 at 56 fps, focus and exposure locked | `camera.width`, `camera.height`, `camera.fps` |

All four towers fit in the image with a margin. The tracker measures the camera's real position
every frame, so the height in the config only needs to be approximate; the Pi preview prints the
measured one (it should read about (500, 500, 1300) mm).

## Car marker height

The tracker intersects each car marker with a horizontal plane at the marker's height above the
floor. On the current car that is **84 mm** (`vision.robot_marker_height_mm`). A wrong value shifts
every car's position away from or toward the camera, more so near the edges of the field, so
re-measure it whenever the car's build changes.
