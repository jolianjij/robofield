# Marker IDs

Dictionary: `DICT_4X4_50` (small grid = detectable at small sizes; 50 IDs is enough for v1).

| IDs | Use | Printed size |
|---|---|---|
| 0–3 | Corner towers (see docs/field.md for positions and heights) | 80 mm |
| 10–29 | Robots (robot N uses marker 10 + N) | 35 mm |
| 30–49 | Objects (balls, boxes, flags) | 30–40 mm |
| 4–9 | Reserved | — |

Rules:

- Print at 100% scale and measure with a ruler; the size goes into `config/field.yaml`.
- Keep a white border (quiet zone) of at least one cell width around each marker.
- Matte paper only. Glossy prints cause glare and dropped detections.
- Robot markers face "forward": the marker's top edge points the way the car drives.
- Tower markers lie flat with their top edge pointing toward +Y. The plane and PnP estimators rely on this.
