# Field CAD

The full Fusion 360 assembly of the field (floor tiles, walls, towers, camera mast) goes here.
What the software needs from it: [`docs/field.md`](../../docs/field.md).

| File | Format | Use |
|---|---|---|
| `robofield_field.f3z` | Fusion 360 archive | The editable assembly with every component |
| `robofield_field.step` | STEP | Opens in any CAD program |
| `stl/` | STL, one per printed part | Ready to slice and print |
| `robofield_field.png` | Image | A render, shown on this page |

After building, measure each tower marker's centre position and its height above the playing
surface, and put the real values in `config/field.yaml` → `markers.corners`.
