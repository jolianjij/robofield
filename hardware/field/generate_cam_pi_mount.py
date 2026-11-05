"""Camera holder (under the beam) and Raspberry Pi 5 top plate (on top of the beam).

    python hardware/field/generate_cam_pi_mount.py

Both parts fix to the 2020 beam with 2 x M3 screws into T-nuts in the beam's slot (bottom slot
for the camera holder, top slot for the plate). The screw holes are slots, so each part slides
+-8 mm along the beam for fine adjustment, and the T-nuts slide anywhere along the beam.
Side lips hug the beam so the parts stay square to it.
"""
from pathlib import Path

import cadquery as cq

import generate_field_parts as g

OUT = g.OUT
BEAM = 20.0
LIP_CLEAR, LIP_T, LIP_H = 0.25, 3.0, 8.0
SLOT_LEN, SLOT_W = 16.0, 3.4            # +-6.3 mm travel for an M3 screw
HEAD_SLOT_W, HEAD_DEPTH = 6.8, 3.0      # counterbored slot so the screw head sits flush

# ---- camera holder ----
CAM_PLATE = (70.0, 2 * (BEAM / 2 + LIP_CLEAR + LIP_T), 6.0)   # length along the beam, width, thickness
CAM_SCREW_X = 25.0

# ---- Raspberry Pi 5 plate ----
TOP_PLATE = (210.0, 74.0, 5.0)
TOP_SCREW_X = 30.0
PI_CENTER_X = -55.0                     # Pi 5 on the left half, power module on the right half
PI_HOLES = (58.0, 49.0)                 # Pi 5 mounting-hole rectangle (M2.5)
PI_BOARD = (85.0, 56.0)
STANDOFF_D, STANDOFF_H, PI_SCREW_D = 6.5, 6.0, 2.2      # 2.2 mm = M2.5 self-tapping
POWER_AREA_X = (2.0, 103.0)
STRAP_X, STRAP_Y, STRAP = (25.0, 80.0), 31.0, (22.0, 3.5)    # slots for 20 mm velcro straps / zip ties


def lips(x0, x1, z0, z1):
    """Two walls hugging the beam's sides."""
    a = BEAM / 2 + LIP_CLEAR
    return g.box(x0, a, z0, x1, a + LIP_T, z1).union(g.box(x0, -a - LIP_T, z0, x1, -a, z1))


def screw_slot(part, x, z_entry, direction, thickness):
    """Slot along x through a plate (M3 clearance) plus a counterbored slot for the screw head on
    the entry face. direction +1: screw enters at z_entry going up; -1: going down."""
    z_lo = z_entry if direction > 0 else z_entry - thickness
    part = part.cut(cq.Workplane("XY").slot2D(SLOT_LEN, SLOT_W).extrude(thickness + 2).translate((x, 0, z_lo - 1)))
    head = cq.Workplane("XY").slot2D(SLOT_LEN + HEAD_SLOT_W - SLOT_W, HEAD_SLOT_W).extrude(HEAD_DEPTH + 0.01)
    hz = z_entry - 0.01 if direction > 0 else z_entry - HEAD_DEPTH
    return part.cut(head.translate((x, 0, hz)))


def camera_holder_beam():
    """Local: beam along x, beam bottom face at z = 0. Plate below the beam, lips up its sides,
    Pi Camera Module 3 (Wide) lens-down in a cup under the beam centre line.
    Print lying on its +y side (plate edge and lip form one flat 70 x 14 mm face): no supports."""
    L, W, T = CAM_PLATE
    p = g.box(-L / 2, -W / 2, -T, L / 2, W / 2, 0)
    p = p.union(lips(-L / 2, L / 2, 0, LIP_H))
    for x in (-CAM_SCREW_X, CAM_SCREW_X):
        p = screw_slot(p, x, -T, +1, T)                          # screws go UP into the beam's bottom slot
    bw, bh, bt = g.CAM_BOARD
    back_z = -T - g.STANDOFF_H
    ix, iy = bw / 2 + g.CUP_CLEAR, bh / 2 + g.CUP_CLEAR
    dy = g.CAM_DY
    z_cup = back_z - bt - g.CUP_BELOW_BOARD
    cup = g.box(-ix - g.CUP_WALL, dy - iy - g.CUP_WALL, z_cup, ix + g.CUP_WALL, dy + iy + g.CUP_WALL, -T)
    cup = cup.cut(g.box(-ix, dy - iy, z_cup - 1, ix, dy + iy, -T + 0.01))
    cup = cup.cut(g.box(-g.RIBBON_SLOT_W / 2, dy - iy - g.CUP_WALL - 1, z_cup - 1, g.RIBBON_SLOT_W / 2, dy - iy + 0.01, -T + 0.01))
    p = p.union(cup)
    for hx, hy in g.CAM_HOLES:                                   # standoffs + locating pegs (heat-stake or glue)
        p = p.union(cq.Workplane("XY").circle(2.4).extrude(g.STANDOFF_H).translate((hx, dy + hy, back_z)))
        p = p.union(cq.Workplane("XY").circle(0.95).extrude(bt + 1.5).translate((hx, dy + hy, back_z - bt - 1.5)))
    return p


def camera_dummy_beam():
    bw, bh, bt = g.CAM_BOARD
    T = CAM_PLATE[2]
    back_z = -T - g.STANDOFF_H
    d = g.box(-bw / 2, g.CAM_DY - bh / 2, back_z - bt, bw / 2, g.CAM_DY + bh / 2, back_z)
    for hx, hy in g.CAM_HOLES:
        d = g.cyl_hole(d, (hx, g.CAM_DY + hy, back_z - bt - 1), 2.2, (0, 0, 1), bt + 2)
    s = g.CAM_LENS_BLOCK / 2
    z0 = back_z - bt
    d = d.union(g.box(-s, -s, z0 - g.CAM_LENS_BLOCK_H, s, s, z0))
    return d.union(cq.Workplane("XY").circle(g.CAM_LENS_D / 2).extrude(g.CAM_LENS_H)
                   .translate((0, 0, z0 - g.CAM_LENS_BLOCK_H - g.CAM_LENS_H)))


def pi_plate():
    """Local: beam along x, beam top face at z = 0. Plate on top, lips down the beam's sides.
    Left half: Raspberry Pi 5 on 4 separate printed spacers (so the top face stays flat).
    Right half: power module held by 2 straps or 4 M3 screws.
    Print upside down: top face on the bed, the two lips pointing up. No supports."""
    L, W, T = TOP_PLATE
    p = g.box(-L / 2, -W / 2, 0, L / 2, W / 2, T)
    p = p.edges("|Z").fillet(6)
    p = p.union(lips(-45, 45, -LIP_H, 0))
    for x in (-TOP_SCREW_X, TOP_SCREW_X):
        p = screw_slot(p, x, T, -1, T)                           # screws go DOWN into the beam's top slot
    hx, hy = PI_HOLES[0] / 2, PI_HOLES[1] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):                                       # M2.5 self-tapping into the plate
            p = g.cyl_hole(p, (PI_CENTER_X + sx * hx, sy * hy, -1), PI_SCREW_D, (0, 0, 1), T + 2)
    for i in range(5):                                           # vents under the Pi (it runs hot)
        for j in range(3):
            x = PI_CENTER_X - 24 + i * 12
            y = -12 + j * 12
            if abs(x - (-TOP_SCREW_X)) < 9 and abs(y) < 6:
                continue
            p = g.cyl_hole(p, (x, y, -1), 6.0, (0, 0, 1), T + 2)
    for sy in (-1, 1):                                           # cable notches on both long edges, beside the Pi
        p = p.cut(g.box(PI_CENTER_X - 15, sy * (W / 2) - 8, -1, PI_CENTER_X + 15, sy * (W / 2) + 8, T + 1)
                  if sy > 0 else g.box(PI_CENTER_X - 15, -W / 2 - 8, -1, PI_CENTER_X + 15, -W / 2 + 8, T + 1))
    for x in STRAP_X:                                            # strap slots around the power module
        for sy in (-1, 1):
            p = p.cut(g.box(x - STRAP[0] / 2, sy * STRAP_Y - STRAP[1] / 2, -1, x + STRAP[0] / 2, sy * STRAP_Y + STRAP[1] / 2, T + 1))
    for x in (12.0, 93.0):                                       # 4 x M3 holes for modules with mounting holes
        for y in (-20.0, 20.0):
            p = g.cyl_hole(p, (x, y, -1), g.M3_CLEAR, (0, 0, 1), T + 2)
    return p


def pi_spacer():
    """6 mm spacer between the Pi 5 and the plate (print 4). M2.5 screw passes through."""
    c = cq.Workplane("XY").circle(STANDOFF_D / 2).extrude(STANDOFF_H)
    return c.faces(">Z").workplane().hole(2.8)


def pi5_dummy():
    b = g.box(-PI_BOARD[0] / 2, -PI_BOARD[1] / 2, 0, PI_BOARD[0] / 2, PI_BOARD[1] / 2, 1.6)
    b = b.union(g.box(-PI_BOARD[0] / 2 + 5, -20, 1.6, -PI_BOARD[0] / 2 + 40, 20, 16))      # USB / Ethernet block
    b = b.union(g.box(0, -15, 1.6, 30, 15, 12))                                              # active cooler
    return b.translate((PI_CENTER_X, 0, TOP_PLATE[2] + STANDOFF_H))


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    cam, plate = camera_holder_beam(), pi_plate()
    for n, part in (("camera_holder_beam", cam), ("pi5_top_plate", plate), ("pi5_spacer", pi_spacer())):
        cq.exporters.export(part, str(OUT / f"{n}.step"))
        cq.exporters.export(part, str(OUT / f"{n}.stl"), tolerance=0.02, angularTolerance=0.1)
    beam = g.box(-140, -BEAM / 2, 0, 140, BEAM / 2, BEAM)
    asm = cq.Assembly(name="camera_pi_mount")
    asm.add(beam, name="beam_2020", color=cq.Color(0.72, 0.72, 0.75))
    asm.add(cam, name="camera_holder_beam", color=cq.Color(0.2, 0.45, 0.8))
    asm.add(camera_dummy_beam(), name="pi_camera_module3", color=cq.Color(0.1, 0.55, 0.25))
    asm.add(plate.translate((0, 0, BEAM)), name="pi5_top_plate", color=cq.Color(0.95, 0.6, 0.15))
    for sx in (-1, 1):
        for sy in (-1, 1):
            asm.add(pi_spacer().translate((PI_CENTER_X + sx * PI_HOLES[0] / 2, sy * PI_HOLES[1] / 2, BEAM + TOP_PLATE[2])),
                    name=f"pi5_spacer_{sx}_{sy}", color=cq.Color(0.95, 0.6, 0.15))
    asm.add(pi5_dummy().translate((0, 0, BEAM)), name="raspberry_pi5_placeholder", color=cq.Color(0.15, 0.5, 0.2))
    asm.add(g.box(8, -27, BEAM + TOP_PLATE[2], 98, 27, BEAM + TOP_PLATE[2] + 25), name="power_module_placeholder",
            color=cq.Color(0.3, 0.3, 0.3))
    asm.export(str(OUT / "camera_pi_mount_assembly.step"))
    return asm


if __name__ == "__main__":
    build()
    for n, f in (("camera_holder_beam", camera_holder_beam), ("pi5_top_plate", pi_plate)):
        s = f().val(); bb = s.BoundingBox()
        print(f"{n}: valid={s.isValid()} {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")
