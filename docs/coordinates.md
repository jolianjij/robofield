# Coordinate system

Every module uses this frame. No exceptions.

- **Origin:** the inner bottom-left corner of the playing area (tower T3 / marker 0 is just outside it).
- **X axis:** along the bottom wall, toward tower T4 (marker 1).
- **Y axis:** along the left wall, toward tower T1 (marker 3).
- **Z axis:** up from the playing surface (right-handed).
- **Units:** millimetres for position, radians for angles, seconds for time.
- **Heading (theta):** 0 along +X, counter-clockwise positive, wrapped to (-pi, pi].
- **Field size:** set in `config/field.yaml` (reference build: 1000 × 1000 mm).

```
 y
 ▲   3 ┌──────────────┐ 2
 │     │              │
 │     │    field     │
 │     │              │
     0 └──────────────┘ 1
   (0,0) ─────────────► x
```

A robot's pose is `(x, y, theta)` of its marker centre, measured on the plane at the marker's
height (`vision.robot_marker_height_mm`, 84 mm on the current car). The marker sits right above
the midpoint between the wheels, so this is also the point the car turns around ([kinematics](kinematics.md)).
