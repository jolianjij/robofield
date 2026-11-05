"""Generate printable ArUco markers at exact physical size.

    python tools/generate_markers.py --out markers/ --dpi 300

Print the PNGs at 100% scale (no "fit to page") and check the size with a ruler.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

SETS = {  # name: (ids, black-square size in mm) — see docs/markers.md
    "corner": (range(0, 4), 80),
    "robot": (range(10, 16), 35),
    "object": (range(30, 36), 35),
}


def marker_png(dictionary, marker_id, size_mm, dpi):
    px = round(size_mm / 25.4 * dpi)
    tag = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    border = round(px / 6)                     # white quiet zone
    tag = cv2.copyMakeBorder(tag, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    label = np.full((round(px / 4), tag.shape[1]), 255, np.uint8)
    cv2.putText(label, f"ID {marker_id}  {size_mm} mm", (border, round(px / 6)),
                cv2.FONT_HERSHEY_SIMPLEX, px / 400, 0, max(1, px // 200))
    return np.vstack([tag, label])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="markers")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--dictionary", default="DICT_4X4_50")
    args = ap.parse_args()
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, (ids, size) in SETS.items():
        for i in ids:
            path = out / f"{name}_{i:02d}.png"
            cv2.imwrite(str(path), marker_png(d, i, size, args.dpi))
    print(f"Saved markers to {out}/ — print at 100% scale, {args.dpi} DPI.")


if __name__ == "__main__":
    main()
