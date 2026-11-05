"""Floor tile + tile joiner for the RoboField floor (16 tiles, 4 x 4 = 1 x 1 m).

    pip install cadquery
    python hardware/field/generate_field_tile.py

Tiles join from UNDERNEATH with dovetail (bowtie) keys - no screws at all in the floor.
A key is pressed into matching dovetail pockets in the underside of two neighbouring tiles.
The dovetail shape locks the tiles together so they cannot pull apart or slide, and the pocket
depth keeps them flush. To take the floor apart, lever a key out with a flat screwdriver in the
pry notch at its end. The wall ring (bolted to the tile edges) holds the whole floor together.
Every outer edge has 3 horizontal M3 inserts for bolting on a wall module.

Print orientation: playing face down on the bed, ribs and pockets facing up.
In this file z = 0 is the PLAYING FACE and z = H is the underside that rests on the table.
"""
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent

S = 250.0          # tile size
H = 12.0           # thickness (12 mm leaves room for a socket head + insert under the face)
SKIN = 1.2         # playing-face skin
WALL = 1.2         # perimeter wall
RIB = 1.2          # rib thickness
NCELL = 6          # 6 x 6 cells -> 5 x 5 ribs
GUSSET = 20.0

# ---- dovetail keys (tile to tile), 2 per shared edge ----
JOIN_POS = [62.5, 187.5]           # along every edge
KEY_NARROW, KEY_WIDE = 10.0, 20.0  # key width at the seam / at its ends
KEY_HALF = 25.0                    # key reaches this far into each tile
KEY_T = 6.0                        # key thickness
KEY_CLEAR = 0.10                   # per side: firm press fit; tune with the test strip
POCKET_DEPTH = KEY_T + 0.2         # key ends 0.2 mm above the underside, never touches the table
PRY_W, PRY_D = 5.0, 2.5            # pry notch at the outer end of each pocket
JOIN_BOSS_W, JOIN_BOSS_D = 26.0, 30.0

# ---- wall screws (tile to wall module), 3 per edge ----
INSERT_POS = [30.0, 125.0, 220.0]
INS_D, INS_DEPTH = 4.0, 7.0        # M3 heat-set insert, 4 mm hole
SCREW_D, SCREW_DEPTH = 3.2, 10.0   # clearance pocket beyond the insert
INS_BOSS_W, INS_BOSS_D = 10.0, 12.0
zc = H / 2


def box(x, y, sx, sy, z0=0.0, sz=H):
    return cq.Workplane("XY").box(sx, sy, sz, centered=False).translate((x, y, z0))


def edge_copies(solid):
    """The same feature on all four edges (rotate about the tile centre)."""
    ctr = cq.Vector(S / 2, S / 2, 0)
    return [solid.rotate(ctr, ctr + cq.Vector(0, 0, 1), a) for a in (0, 90, 180, 270)]


def half_key_prism(c, clear, z0, height):
    """Half of the bowtie (from the seam y = 0 into the tile), centred on x = c."""
    n, w = KEY_NARROW / 2 + clear, KEY_WIDE / 2 + clear
    pts = [(c - n, -1.0), (c + n, -1.0), (c + w, KEY_HALF + clear), (c - w, KEY_HALF + clear)]
    return cq.Workplane("XY").polyline(pts).close().extrude(height).translate((0, 0, z0))


def make_tile():
    t = box(0, 0, S, S, sz=SKIN)
    for b in [box(0, 0, S, WALL), box(0, S - WALL, S, WALL), box(0, 0, WALL, S), box(S - WALL, 0, WALL, S)]:
        t = t.union(b)
    p = S / NCELL
    for k in range(1, NCELL):
        t = t.union(box(k * p - RIB / 2, 0, RIB, S)).union(box(0, k * p - RIB / 2, S, RIB))
    for (cx, cy, dx, dy) in [(0, 0, 1, 1), (S, 0, -1, 1), (0, S, 1, -1), (S, S, -1, -1)]:
        t = t.union(cq.Workplane("XY").polyline([(cx, cy), (cx + dx * GUSSET, cy), (cx, cy + dy * GUSSET)])
                    .close().extrude(H))
    # bosses (built on the y = 0 edge, copied to all edges)
    for c in JOIN_POS:
        for b in edge_copies(box(c - JOIN_BOSS_W / 2, 0, JOIN_BOSS_W, JOIN_BOSS_D)):
            t = t.union(b)
    for c in INSERT_POS:
        for b in edge_copies(box(c - INS_BOSS_W / 2, 0, INS_BOSS_W, INS_BOSS_D)):
            t = t.union(b)
    # dovetail pockets: open on the underside (z = H) and at the edge
    z_floor = H - POCKET_DEPTH
    for c in JOIN_POS:
        pocket = half_key_prism(c, KEY_CLEAR, z_floor, POCKET_DEPTH + 1)
        pry = box(c - PRY_W / 2, KEY_HALF + KEY_CLEAR - 0.5, PRY_W, PRY_D + 0.5, z0=z_floor, sz=POCKET_DEPTH + 1)
        for cut in edge_copies(pocket.union(pry)):
            t = t.cut(cut)
    # wall-screw insert holes, horizontal at mid-height (inserts are melted in, so print sag does not matter)
    for c in INSERT_POS:
        ins = cq.Solid.makeCylinder(INS_D / 2, INS_DEPTH + 1, cq.Vector(c, -1, zc), cq.Vector(0, 1, 0))
        scr = cq.Solid.makeCylinder(SCREW_D / 2, SCREW_DEPTH + 1, cq.Vector(c, -1, zc), cq.Vector(0, 1, 0))
        for cut in edge_copies(cq.Workplane("XY").add(ins.fuse(scr))):
            t = t.cut(cut)
    return t


def make_joiner():
    """Bowtie key. Local: seam along y at x = 0... built along y here: seam at y = 0.
    Print flat (6 mm tall). Small chamfers on top help it start into the pockets."""
    n, w = KEY_NARROW / 2, KEY_WIDE / 2
    pts = [(-w, -KEY_HALF), (w, -KEY_HALF), (n, 0), (w, KEY_HALF), (-w, KEY_HALF), (-n, 0)]
    k = cq.Workplane("XY").polyline(pts).close().extrude(KEY_T)
    return k.faces(">Z").edges().chamfer(0.5)


if __name__ == "__main__":
    tile, joiner = make_tile(), make_joiner()
    strip = tile.intersect(box(0, 0, S, 32.0))
    for name, part in (("field_tile_250mm", tile), ("field_tile_key", joiner),
                       ("field_tile_edge_test_strip", strip)):
        cq.exporters.export(part, str(HERE / f"{name}.step"))
        cq.exporters.export(part, str(HERE / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        s = part.val(); bb = s.BoundingBox()
        print(f"{name:28s} valid={s.isValid()} {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm  "
              f"PLA ~{s.Volume() / 1000 * 1.24:.0f} g")
