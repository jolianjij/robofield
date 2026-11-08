# Car CAD

The full Fusion 360 assembly of the car goes here. Parts list and wiring: [`hardware/README.md`](../README.md).

| File | Format | Use |
|---|---|---|
| `robofield_car.f3z` | Fusion 360 archive | The editable assembly with every component |
| `robofield_car.step` | STEP | Opens in any CAD program (FreeCAD, Onshape, SolidWorks...) |
| `stl/` | STL, one per printed part | Ready to slice and print |
| `robofield_car.png` | Image | A render, shown on this page |

When you change the car, re-measure what the software uses and update `config/field.yaml`: the
wheel track, the wheel radius and the marker height above the floor.
