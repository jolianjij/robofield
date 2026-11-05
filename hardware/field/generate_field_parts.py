"""Parametric generator for the field walls, corner towers and camera mast.
Every printed part fits a Bambu Lab A1 (256 x 256 x 256 mm).

    pip install cadquery
    python hardware/field/generate_field_parts.py

Writes STEP (for Fusion 360) and STL (for printing) into hardware/field/out/,
plus field_assembly.step with everything placed.

Frames: x, y as in docs/coordinates.md (origin = inner bottom-left playing-area corner).
z is measured from the table; the playing surface (tile top) is at z = TILE_H.
"""
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"

# ---------------- field (tiles are NOT modified) ----------------
FIELD = 1000.0
TILE, TILE_H = 250.0, 12.0
TILE_INSERT_POS = [30.0, 125.0, 220.0]   # 3 horizontal M3 inserts on every tile edge
TILE_FEATURE_Z = TILE_H / 2        # insert sits at tile mid-height

# ---------------- printed wall modules ----------------
# One module per outer tile edge (16, all identical). Each is 250 mm long, so it fits the A1.
# The bottom 10 mm sits beside the tile; the top is 100 mm above the playing surface.
WALL_T = 10.0                      # thin wall (saves ~55 % filament vs 20 mm)
WALL_TOP = TILE_H + 100.0          # from the table = 100 mm above the field
M3_CLEAR, M3_CBORE_D = 3.4, 6.5
M3_INTO_TILE = 6.5                 # wall material under the screw head (3.5 mm counterbore); M3 x 12-16
M3_INSERT_D, M3_INSERT_DEPTH = 4.0, 7.0   # same M3 heat-set inserts as the tiles (4 mm hole)
JOINT_X = 20.0                     # insert distance from each module end
JOINT_Z = [35.0, 85.0]             # insert heights (from the table)

# ---------------- splice plates (join modules end to end) ----------------
SPLICE_T, SPLICE_W = 6.0, 80.0
SPLICE_Z = (20.0, 100.0)

# ---------------- towers ----------------
EXT = 20.0                         # 2020 aluminium extrusion
SOCK_IN, SOCK_OUT = 20.4, 36.0
PLATE_T = 6.0
TOWER_OFFSET = 61.0                # tower centre from the outer wall faces (-71 = -10 - 61)
TOWER_COL = 30.0                   # printed tower column, 30 x 30 mm
T_SOCK_IN, T_SOCK_OUT = TOWER_COL + 0.4, TOWER_COL + 14.0
FLANGE_REACH, FLANGE_H = 90.0, 100.0
CAP_PLATE, CAP_SOCKET_H = 100.0, 30.0
CLAMP_INSERT_D, CLAMP_INSERT_DEPTH = 4.0, 7.0   # M3 heat-set insert (printed tower columns only)
# 2020 aluminium joints use M3 T-nuts (slot nuts) in the extrusion's slot: the screw goes through a
# clearance hole in the printed wall, centred on the extrusion face (= on the slot), into the T-nut.
TNUT_MIN_CBORE_WALL = 7.0          # walls at least this thick get a counterbore for the screw head


def tnut_hole(part, p, axis, wall):
    """M3 clearance hole for a 2020 T-nut screw, entering at p along axis through `wall` mm.
    Counterbored (head flush) when the wall is thick enough; otherwise the head sits on the surface."""
    if wall >= TNUT_MIN_CBORE_WALL:
        return m3_cbore(part, p, axis, wall)
    return cyl_hole(part, p, M3_CLEAR, axis, wall + 2)
SCREW_D, CSK_D = 4.2, 8.5          # optional screws into a table / board
# (name, marker id, outer wall corner, rotation, marker height above the playing surface)
TOWERS = [("T3", 0, (-WALL_T, -WALL_T), 0, 220.0),
          ("T4", 1, (FIELD + WALL_T, -WALL_T), 90, 260.0),
          ("T2", 2, (FIELD + WALL_T, FIELD + WALL_T), 180, 180.0),
          ("T1", 3, (-WALL_T, FIELD + WALL_T), 270, 140.0)]


def extrusion_length(marker_z):
    """2020 length so the marker (cap top) lands at marker_z above the playing surface."""
    return marker_z + TILE_H - PLATE_T - PLATE_T   # base socket floor + cap plate


# ---------------- camera mast ----------------
CAMERA_Z = 1300.0                  # lens height above the playing surface
CAM_STACK = 20.5                   # arm underside -> lens front (sleeve wall 3 + standoffs 6 + board 1 + lens 10.5)
ARM_BOTTOM = CAMERA_Z + TILE_H + CAM_STACK
UPRIGHT_L = ARM_BOTTOM + EXT       # arm top flush with the upright top (both inside the elbow)
ARM_L = 580.0
BRACE_L = 540.0                    # 2020 diagonal, 45-degree cut ends
MAST_Y = FIELD / 2                 # on the left wall, over the joint between two modules
MAST_FOOT = 120.0
MAST_SOCK_OUT = 30.0               # narrow socket: screwdriver clearance for the wall screws


# ======================= helpers =======================
def box(x0, y0, z0, x1, y1, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_hole(part, p, d, axis, depth):
    """Cut a cylinder starting at point p going along axis (unit tuple) for depth."""
    return part.cut(cq.Workplane("XY").add(
        cq.Solid.makeCylinder(d / 2, depth, cq.Vector(*p), cq.Vector(*axis))))


def m3_cbore(part, p, axis, thickness):
    """M3 clearance hole through a plate of given thickness, counterbored on the entry side."""
    part = cyl_hole(part, p, M3_CLEAR, axis, thickness + 2)
    return cyl_hole(part, p, M3_CBORE_D, axis, 3.0 + 1)


def csk_hole(part, x, y, z_top, depth, d=SCREW_D, csk=CSK_D):
    """Vertical countersunk hole entering from z_top downward."""
    part = cyl_hole(part, (x, y, z_top + 1), d, (0, 0, -1), depth + 2)
    cone = cq.Solid.makeCone(csk / 2 + 0.01, d / 2, (csk - d) / 2 + 0.01,
                             cq.Vector(x, y, z_top + 0.01), cq.Vector(0, 0, -1))
    return part.cut(cq.Workplane("XY").add(cone))


# ======================= wall module =======================
def wall_module():
    """Local frame: tile edge along x (0..250) at y = 0, tile on y >= 0, outer face at y = -20.
    Stands on the table beside the tile (so tops line up automatically) and is pulled tight
    against the tile edge by three M3 screws. Print lying on its outer face."""
    p = box(0, -WALL_T, 0, TILE, 0, WALL_TOP)
    # 3 x M3 x 16 from the outer face into the tile's edge inserts
    z = TILE_FEATURE_Z
    for x in TILE_INSERT_POS:
        p = cyl_hole(p, (x, 1, z), M3_CLEAR, (0, -1, 0), WALL_T + 2)
        p = cyl_hole(p, (x, -WALL_T - 1, z), M3_CBORE_D, (0, 1, 0), WALL_T - M3_INTO_TILE + 1)
    # M3 heat-set inserts on the outer face near both ends (splice plates, tower bases, mast clamp)
    for x in (JOINT_X, TILE - JOINT_X):
        for z in JOINT_Z:
            p = cyl_hole(p, (x, -WALL_T - 1, z), M3_INSERT_D, (0, 1, 0), M3_INSERT_DEPTH + 1)
    return p


# ======================= splice plate =======================
def splice_plate():
    """Bridges two modules on their outer face. Local: joint at x = 0, wall outer face at y = 0,
    plate on y < 0. Print flat."""
    h = SPLICE_W / 2
    p = box(-h, -SPLICE_T, SPLICE_Z[0], h, 0, SPLICE_Z[1])
    for x in (-JOINT_X, JOINT_X):
        for z in JOINT_Z:
            p = m3_cbore(p, (x, -SPLICE_T - 1, z), (0, 1, 0), SPLICE_T)
    return p


# ======================= tower base =======================
def tower_base():
    """Local frame: outer wall corner at (0,0), walls occupy x >= 0, y >= 0 (outside is negative).
    Includes the corner block that closes the wall ring. One part fits all corners.
    The socket takes the printed 30 x 30 mm tower column."""
    c = -TOWER_OFFSET
    h = T_SOCK_OUT / 2
    far = c - h - 19
    p = box(far, far, 0, 0, 0, PLATE_T)                                   # floor pad
    p = p.union(box(far, -PLATE_T, 0, FLANGE_REACH, 0, FLANGE_H))         # flange on the y = 0 wall face
    p = p.union(box(-PLATE_T, far, 0, 0, FLANGE_REACH, FLANGE_H))         # flange on the x = 0 wall face
    p = p.union(box(0, 0, 0, WALL_T, WALL_T, WALL_TOP))                   # corner block
    p = p.union(box(c - h, c - h, 0, c + h, c + h, 50))                   # column socket
    p = p.union(box(c - 3, c + h, 0, c + 3, -PLATE_T, 50))                # ribs socket -> flanges
    p = p.union(box(c + h, c - 3, 0, -PLATE_T, c + 3, 50))
    p = p.cut(box(c - T_SOCK_IN / 2, c - T_SOCK_IN / 2, PLATE_T, c + T_SOCK_IN / 2, c + T_SOCK_IN / 2, 51))
    # M3 into the first insert pair of each neighbouring module (module end = WALL_T from the corner)
    x = WALL_T + JOINT_X
    for z in JOINT_Z:
        p = m3_cbore(p, (x, -PLATE_T - 1, z), (0, 1, 0), PLATE_T)
        p = m3_cbore(p, (-PLATE_T - 1, x, z), (1, 0, 0), PLATE_T)
    # optional: screw down into a table or board
    for px, py in ((-90.0, -20.0), (-20.0, -90.0), (-90.0, -90.0), (-25.0, -25.0)):
        p = csk_hole(p, px, py, PLATE_T, PLATE_T)
    # 4 M3 clamp screws (2 heights x 2 faces) lock the column in its socket
    for z in (18.0, 38.0):
        p = cyl_hole(p, (c - h - 1, c, z), CLAMP_INSERT_D, (1, 0, 0), CLAMP_INSERT_DEPTH + 1)
        p = cyl_hole(p, (c, c - h - 1, z), CLAMP_INSERT_D, (0, 1, 0), CLAMP_INSERT_DEPTH + 1)
    return p


# ======================= tower cap =======================
def tower_cap():
    """Marker plate on top, socket underneath. Print plate-down. Notch marks the +Y edge."""
    s, h = CAP_PLATE / 2, T_SOCK_OUT / 2
    p = box(-s, -s, CAP_SOCKET_H, s, s, CAP_SOCKET_H + PLATE_T)
    p = p.union(box(-h, -h, 0, h, h, CAP_SOCKET_H))
    p = p.cut(box(-T_SOCK_IN / 2, -T_SOCK_IN / 2, -1, T_SOCK_IN / 2, T_SOCK_IN / 2, CAP_SOCKET_H))
    p = p.cut(box(-4, s - 3, CAP_SOCKET_H - 1, 4, s + 1, CAP_SOCKET_H + 2))
    p = cyl_hole(p, (-h - 1, 0, 15.0), CLAMP_INSERT_D, (1, 0, 0), CLAMP_INSERT_DEPTH + 1)
    p = cyl_hole(p, (0, -h - 1, 15.0), CLAMP_INSERT_D, (0, 1, 0), CLAMP_INSERT_DEPTH + 1)
    return p


# ======================= tower column (printed) =======================
def tower_column(length):
    """Printed 30 x 30 mm column between tower base and cap. Print lying flat (the 260 mm one
    fits diagonally on the A1 bed). Chamfered ends slide into the sockets easily."""
    c = cq.Workplane("XY").rect(TOWER_COL, TOWER_COL).extrude(length)
    return c.edges("|Z").fillet(2.0).faces(">Z or <Z").edges().chamfer(0.8)


# ======================= mast clamp =======================
def mast_clamp():
    """Holds the camera upright on the outer face of the left wall, centred on a module joint
    (it doubles as that joint's splice plate). Local: wall outer face at x = 0, wall at x >= 0.
    The socket is kept narrow (30 mm) so a straight screwdriver reaches the 4 wall screws
    at y = +-20 past its sides."""
    h = MAST_SOCK_OUT / 2
    cx = -PLATE_T - h
    p = box(-PLATE_T, -45, 0, 0, 45, WALL_TOP)
    p = p.union(box(cx - h, -h, 0, cx + h, h, WALL_TOP))
    p = p.union(box(cx - h - MAST_FOOT, -50, 0, -PLATE_T, 50, PLATE_T))
    p = p.union(box(cx - h - MAST_FOOT, -3, 0, cx - h, 3, 60))
    p = p.cut(box(cx - SOCK_IN / 2, -SOCK_IN / 2, -1, cx + SOCK_IN / 2, SOCK_IN / 2, WALL_TOP + 1))
    for y in (-JOINT_X, JOINT_X):
        for z in JOINT_Z:
            p = m3_cbore(p, (-PLATE_T - 1, y, z), (1, 0, 0), PLATE_T)
    for z in (75.0, 100.0):            # T-nut screws, centred on the slot, above the foot rib
        p = tnut_hole(p, (cx - h - 1, 0, z), (1, 0, 0), h - SOCK_IN / 2)
    for x, y in ((cx - h - MAST_FOOT + 16, 35.0), (cx - h - MAST_FOOT + 16, -35.0)):
        p = csk_hole(p, x, y, PLATE_T, PLATE_T)
    return p


# ======================= mast joints =======================
JOINT_SOCKET_DEPTH = 40.0
BRACE_FLOOR = 30.0                 # brace socket floor, measured from the clamp's centre along the brace
BRACE_SOCKET_DEPTH = 35.0


def mast_elbow():
    """Top corner of the mast: the upright goes up into it, the arm goes in sideways.
    Local: upright axis = z, upright top at z = 0; arm along +x, arm top flush with the upright top.
    Print standing on the arm-socket face (x = max) or on its side; no supports needed lying on y."""
    h, si = SOCK_OUT / 2, SOCK_IN / 2
    cap = PLATE_T
    p = box(-h, -h, -JOINT_SOCKET_DEPTH - 5, h, h, cap)                       # upright collar
    p = p.union(box(h, -h, -EXT - 6, si + 2 + JOINT_SOCKET_DEPTH + 10, h, cap))   # arm collar
    p = p.union(cq.Workplane("XZ").polyline([(h, -EXT - 6), (h + 30, -EXT - 6), (h, -EXT - 36)])
                .close().extrude(6).translate((0, 3, 0)))                    # gusset under the arm collar
    p = p.cut(box(-si, -si, -JOINT_SOCKET_DEPTH - 6, si, si, 0))             # upright socket
    p = p.cut(box(si + 2, -si, -EXT - 0.2, si + 2 + JOINT_SOCKET_DEPTH + 11, si, 0))   # arm socket
    p = tnut_hole(p, (-h - 1, 0, -25.0), (1, 0, 0), h - si)                     # upright, T-nut in its slot
    p = tnut_hole(p, (si + 2 + JOINT_SOCKET_DEPTH / 2 + 5, 0, cap + 1), (0, 0, -1), cap)   # arm, from the top
    return p


def brace_clamp():
    """Collar that slides on a 2020 extrusion (axis z) with a 45-degree socket for the brace.
    The same part is used on the upright and (rotated) on the arm."""
    h, si = SOCK_OUT / 2, SOCK_IN / 2
    p = box(-h, -h, -25, h, h, 25)
    reach = BRACE_FLOOR + BRACE_SOCKET_DEPTH
    arm = box(0, -h, -h, reach, h, h).rotate((0, 0, 0), (0, 1, 0), -45)      # +x -> up-and-out at 45 deg
    p = p.union(arm)
    p = p.cut(box(-si, -si, -200, si, si, 200))                              # slides on the extrusion
    hole = box(BRACE_FLOOR, -si, -si, reach + 1, si, si).rotate((0, 0, 0), (0, 1, 0), -45)
    p = p.cut(hole)
    p = cyl_hole(p, (-h - 1, 0, 0), CLAMP_INSERT_D, (1, 0, 0), CLAMP_INSERT_DEPTH + 1)   # clamp on extrusion
    u = BRACE_FLOOR + BRACE_SOCKET_DEPTH / 2                                 # clamp the brace
    c = (u / 2 ** 0.5 - h / 2 ** 0.5, 0, u / 2 ** 0.5 + h / 2 ** 0.5)       # point on the socket's top face
    p = cyl_hole(p, (c[0] - 0.7, 0, c[2] + 0.7), CLAMP_INSERT_D, (2 ** -0.5, 0, -(2 ** -0.5)), CLAMP_INSERT_DEPTH + 1)
    return p


# ======================= camera holder =======================
# Raspberry Pi Camera Module 3 (Wide): 25 x 24 mm board, 4 x M2 holes Ø2.2 spaced 21 x 12.5 mm,
# top holes 2 mm from the top edge, lens centre ~9.5 mm from the top edge, 15-pin ribbon at the
# bottom edge. Check these with calipers against your camera before printing.
CAM_BOARD = (25.0, 24.0, 1.0)
CAM_HOLES = [(-10.5, 10.0), (10.5, 10.0), (-10.5, -2.5), (10.5, -2.5)]   # from the board centre
CAM_LENS_FROM_TOP = 9.5
CAM_LENS_BLOCK, CAM_LENS_BLOCK_H = 8.5, 4.5
CAM_LENS_D, CAM_LENS_H = 7.5, 6.0
HOLDER_PLATE = (40.0, 36.0, 3.0)   # sleeve length, width, wall under the arm
STANDOFF_H = 6.0                   # clearance for parts on the back of the board
CUP_WALL, CUP_CLEAR, CUP_BELOW_BOARD = 2.5, 0.6, 2.0
RIBBON_SLOT_W = 18.0
# board centre offset so the LENS sits exactly under the holder centre
CAM_DY = -(CAM_BOARD[1] / 2 - CAM_LENS_FROM_TOP)
CAM_BOARD_BACK_Z = -HOLDER_PLATE[2] - STANDOFF_H
CAM_LENS_FRONT_Z = CAM_BOARD_BACK_Z - CAM_BOARD[2] - CAM_LENS_BLOCK_H - CAM_LENS_H


def camera_holder():
    """Sleeve that slides along the 2020 arm and locks with 2 M3 screws into T-nuts in its top slot.
    Local: arm along x, arm underside at z = 0. The camera hangs lens-down in a protective cup;
    the lens sits under the origin. Slide it on from the free end of the arm.
    Print standing on one sleeve end (x face down)."""
    L, W, T = HOLDER_PLATE
    si, h = SOCK_IN / 2, W / 2
    p = box(-L / 2, -h, -T, L / 2, h, EXT + T)                          # sleeve around the arm
    p = p.cut(box(-L / 2 - 1, -si, -0.2, L / 2 + 1, si, EXT + 0.2))     # arm passage
    for x in (-10.0, 10.0):                                             # T-nut screws into the arm's top slot
        p = tnut_hole(p, (x, 0, EXT + T + 1), (0, 0, -1), T)
    bw, bh, bt = CAM_BOARD
    ix, iy = bw / 2 + CUP_CLEAR, bh / 2 + CUP_CLEAR
    z_cup = CAM_BOARD_BACK_Z - bt - CUP_BELOW_BOARD
    cup = box(-ix - CUP_WALL, CAM_DY - iy - CUP_WALL, z_cup, ix + CUP_WALL, CAM_DY + iy + CUP_WALL, -T)
    cup = cup.cut(box(-ix, CAM_DY - iy, z_cup - 1, ix, CAM_DY + iy, -T + 0.01))
    cup = cup.cut(box(-RIBBON_SLOT_W / 2, CAM_DY - iy - CUP_WALL - 1, z_cup - 1,
                      RIBBON_SLOT_W / 2, CAM_DY - iy + 0.01, -T + 0.01))
    p = p.union(cup)
    for hx, hy in CAM_HOLES:
        # standoff + locating peg through the board's Ø2.2 hole; melt the peg tip (heat-stake)
        # or add a drop of hot glue to hold the board, so no M2 screws are needed
        p = p.union(cq.Workplane("XY").circle(2.4).extrude(STANDOFF_H)
                    .translate((hx, CAM_DY + hy, CAM_BOARD_BACK_Z)))
        p = p.union(cq.Workplane("XY").circle(0.95).extrude(CAM_BOARD[2] + 1.5)
                    .translate((hx, CAM_DY + hy, CAM_BOARD_BACK_Z - CAM_BOARD[2] - 1.5)))
    return p


def camera_dummy():
    """Placeholder of the camera (not printed) for checking fit in the assembly."""
    bw, bh, bt = CAM_BOARD
    d = box(-bw / 2, CAM_DY - bh / 2, CAM_BOARD_BACK_Z - bt, bw / 2, CAM_DY + bh / 2, CAM_BOARD_BACK_Z)
    for hx, hy in CAM_HOLES:
        d = cyl_hole(d, (hx, CAM_DY + hy, CAM_BOARD_BACK_Z - bt - 1), 2.2, (0, 0, 1), bt + 2)
    s = CAM_LENS_BLOCK / 2
    z0 = CAM_BOARD_BACK_Z - bt
    d = d.union(box(-s, -s, z0 - CAM_LENS_BLOCK_H, s, s, z0))
    d = d.union(cq.Workplane("XY").circle(CAM_LENS_D / 2).extrude(CAM_LENS_H)
                .translate((0, 0, z0 - CAM_LENS_BLOCK_H - CAM_LENS_H)))
    return d


PRINTED = {"wall_module": wall_module, "splice_plate": splice_plate, "tower_base": tower_base,
           "tower_cap": tower_cap, "mast_clamp": mast_clamp, "mast_elbow": mast_elbow,
           "brace_clamp": brace_clamp, "camera_holder": camera_holder}


# ======================= assembly =======================
def placed(part, rot_deg, dx, dy, dz=0.0):
    return part.rotate((0, 0, 0), (0, 0, 1), rot_deg).translate((dx, dy, dz))


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    parts = {k: f() for k, f in PRINTED.items()}
    for name, part in parts.items():
        cq.exporters.export(part, str(OUT / f"{name}.step"))
        cq.exporters.export(part, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(camera_dummy(), str(OUT / "camera_pi_module3_placeholder.step"))
    for name, mid, _, _, mz in TOWERS:
        col = tower_column(extrusion_length(mz))
        stem = f"tower_column_{name}_{extrusion_length(mz):.0f}mm"
        cq.exporters.export(col, str(OUT / f"{stem}.step"))
        cq.exporters.export(col, str(OUT / f"{stem}.stl"), tolerance=0.02, angularTolerance=0.1)

    mod, spl, base, cap, clamp, elbow, bclamp, cam = (parts[k] for k in PRINTED)
    asm = cq.Assembly(name="robofield_field")
    for i in range(4):
        for j in range(4):
            asm.add(box(i * TILE, j * TILE, 0, (i + 1) * TILE - 0.5, (j + 1) * TILE - 0.5, TILE_H),
                    name=f"tile_{i}{j}", color=cq.Color(0.92, 0.92, 0.9))
    wc, pc = cq.Color(0.95, 0.55, 0.15), cq.Color(0.2, 0.45, 0.8)
    for k in range(4):
        asm.add(placed(mod, 0, k * TILE, 0), name=f"wall_bottom_{k}", color=wc)
        asm.add(placed(mod, 180, (k + 1) * TILE, FIELD), name=f"wall_top_{k}", color=wc)
        asm.add(placed(mod, -90, 0, (k + 1) * TILE), name=f"wall_left_{k}", color=wc)
        asm.add(placed(mod, 90, FIELD, k * TILE), name=f"wall_right_{k}", color=wc)
    for k in (1, 2, 3):
        j = k * TILE
        asm.add(placed(spl, 0, j, -WALL_T), name=f"splice_bottom_{k}", color=pc)
        asm.add(placed(spl, 180, j, FIELD + WALL_T), name=f"splice_top_{k}", color=pc)
        asm.add(placed(spl, 90, FIELD + WALL_T, j), name=f"splice_right_{k}", color=pc)
        if j != MAST_Y:
            asm.add(placed(spl, -90, -WALL_T, j), name=f"splice_left_{k}", color=pc)
    grey = cq.Color(0.7, 0.7, 0.72)
    lengths = {}
    for name, mid, (cx, cy), rot, mz in TOWERS:
        L = extrusion_length(mz)
        lengths[name] = (mid, mz, L)
        asm.add(placed(base, rot, cx, cy), name=f"{name}_base", color=pc)
        a = math.radians(rot)
        tx = cx - TOWER_OFFSET * math.cos(a) + TOWER_OFFSET * math.sin(a)
        ty = cy - TOWER_OFFSET * math.sin(a) - TOWER_OFFSET * math.cos(a)
        asm.add(tower_column(L).translate((tx, ty, PLATE_T)), name=f"{name}_column_{L:.0f}mm", color=pc)
        asm.add(placed(cap, rot, tx, ty, PLATE_T + L - CAP_SOCKET_H), name=f"{name}_cap", color=pc)
    asm.add(placed(clamp, 0, -WALL_T, MAST_Y), name="mast_clamp", color=pc)
    ux = -WALL_T - PLATE_T - MAST_SOCK_OUT / 2
    asm.add(box(ux - EXT / 2, MAST_Y - EXT / 2, 0, ux + EXT / 2, MAST_Y + EXT / 2, UPRIGHT_L),
            name=f"mast_upright_2020_{UPRIGHT_L:.0f}mm", color=grey)
    ax0 = ux + SOCK_IN / 2 + 2                     # arm end sits in the elbow's arm socket
    asm.add(box(ax0, MAST_Y - EXT / 2, ARM_BOTTOM, ax0 + ARM_L, MAST_Y + EXT / 2, UPRIGHT_L),
            name=f"mast_arm_2020_{ARM_L:.0f}mm", color=grey)
    asm.add(elbow.translate((ux, MAST_Y, UPRIGHT_L)), name="mast_elbow", color=pc)
    # brace: 45 deg between a clamp on the upright and a clamp on the arm
    D = (BRACE_L + 2 * BRACE_FLOOR) / 2 ** 0.5     # horizontal = vertical offset between clamp centres
    arm_z = ARM_BOTTOM + EXT / 2
    asm.add(bclamp.translate((ux, MAST_Y, arm_z - D)), name="brace_clamp_upright", color=pc)
    asm.add(bclamp.rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (0, 0, 1), 180)
            .translate((ux + D, MAST_Y, arm_z)), name="brace_clamp_arm", color=pc)
    mid = (ux + D / 2, MAST_Y, arm_z - D / 2)
    brace = (cq.Workplane("XY").box(BRACE_L, EXT, EXT).rotate((0, 0, 0), (0, 1, 0), -45).translate(mid))
    asm.add(brace, name=f"mast_brace_2020_{BRACE_L:.0f}mm", color=grey)
    asm.add(placed(cam, 0, FIELD / 2, MAST_Y, ARM_BOTTOM), name="camera_holder", color=pc)
    asm.add(placed(camera_dummy(), 0, FIELD / 2, MAST_Y, ARM_BOTTOM), name="camera_pi_module3_placeholder",
            color=cq.Color(0.1, 0.5, 0.2))
    asm.export(str(OUT / "field_assembly.step"))

    for name, part in parts.items():
        s = part.val()
        bb = s.BoundingBox()
        fits = all(d <= 256 for d in (bb.xlen, bb.ylen, bb.zlen))
        print(f"{name:13s} valid={s.isValid()} {bb.xlen:.0f}x{bb.ylen:.0f}x{bb.zlen:.0f} mm "
              f"solid PLA {s.Volume() / 1000 * 1.24:.0f} g  fits A1: {fits}")
    print("\nTower columns (printed, 30 x 30 mm):")
    for name, (mid, mz, L) in lengths.items():
        print(f"  {name} (marker {mid}): marker at {mz:.0f} mm -> column {L:.0f} mm")
    print(f"Mast: upright {UPRIGHT_L:.0f} mm, arm {ARM_L:.0f} mm, brace {BRACE_L:.0f} mm (square-cut ends)")
    print(f"Brace clamps: on the upright {D:.0f} mm below the arm centre line, on the arm {D:.0f} mm from the upright")
    return asm


if __name__ == "__main__":
    build()
