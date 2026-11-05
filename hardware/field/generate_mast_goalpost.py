"""Goalpost camera mast using 120 cm 2020 extrusions (no cutting of the uprights).

    python hardware/field/generate_mast_goalpost.py

New part: mast_clamp_tall (x2, same part on the left and right wall). It lifts each uncut
1200 mm upright 152 mm off the table, so the upright tops are at 1352 mm, as before.
Reuses mast_elbow (x2) and camera_holder from generate_field_parts.py unchanged.
The diagonal brace and brace clamps are no longer needed.
"""
from pathlib import Path

import cadquery as cq

import generate_field_parts as g

OUT = g.OUT
BAR = 1200.0                                   # shop length of the 2020 extrusion
UPRIGHT_TOP = g.UPRIGHT_L                      # 1352 mm from the table (camera stays at 1.3 m)
BASE_Z = UPRIGHT_TOP - BAR                     # the upright stands on this floor (152 mm)
GRIP = 100.0                                   # socket depth gripping the extrusion
CLAMP_H = BASE_Z + GRIP                        # 252 mm, fits the A1 (256 mm)
SI = g.SOCK_IN / 2                             # 20.4 mm socket
LOW_HALF = g.MAST_SOCK_OUT / 2                 # 15: narrow next to the wall screws (screwdriver room)
HIGH_HALF = 20.0                               # 40 mm wide above the wall top, for stiffness
LIP = 4.0                                      # hook over the wall top, down its inner face
LIP_DOWN = 20.0
SCREW_OFF = 7.0                                # clamp screws press on the extrusion's lip, not the T-slot


def mast_clamp_tall():
    """Local frame: wall outer face at x = 0, wall at 0 <= x <= WALL_T, wall top at z = WALL_TOP.
    Upright axis at (cx, 0). Print foot-down (252 mm tall)."""
    P, W, top = g.PLATE_T, g.WALL_T, g.WALL_TOP
    cx = -P - LOW_HALF
    p = g.box(-P, -45, 0, 0, 45, top)                                   # plate on the wall's outer face
    p = p.union(g.box(-P, -45, top, W + LIP, 45, top + P))              # cap over the wall top
    p = p.union(g.box(W, -45, top - LIP_DOWN, W + LIP, 45, top + P))    # lip down the inner face
    p = p.union(g.box(cx - LOW_HALF, -LOW_HALF, 0, -P, LOW_HALF, top))  # narrow column beside the wall
    p = p.union(g.box(cx - HIGH_HALF, -HIGH_HALF, top, cx + HIGH_HALF, HIGH_HALF, CLAMP_H))   # wide socket body
    p = p.union(g.box(cx + LOW_HALF, -HIGH_HALF, top, 0, HIGH_HALF, top + 60))   # web joining body to the cap
    foot = 120.0
    p = p.union(g.box(cx - LOW_HALF - foot, -55, 0, -P, 55, P))          # foot on the table
    for y in (-30.0, 30.0):                                              # foot ribs up to the socket body
        rib = (cq.Workplane("XZ").polyline([(cx - LOW_HALF - foot, P), (cx - LOW_HALF, P),
                                            (cx - LOW_HALF, top)]).close().extrude(6).translate((0, y + 3, 0)))
        p = p.union(rib)
    p = p.cut(g.box(cx - SI, -SI, BASE_Z, cx + SI, SI, CLAMP_H + 1))     # socket, floor at BASE_Z
    # 4 x M3 into the wall inserts (same holes as a splice plate), counterbored on the outside
    for y in (-g.JOINT_X, g.JOINT_X):
        for z in g.JOINT_Z:
            p = g.m3_cbore(p, (-P - 1, y, z), (1, 0, 0), P)
    # 3 clamp screws on the outward face, 7 mm off-centre so they press on the solid lip
    for y, z in ((SCREW_OFF, BASE_Z + 20), (-SCREW_OFF, BASE_Z + 55), (SCREW_OFF, BASE_Z + 90)):
        p = g.cyl_hole(p, (cx - HIGH_HALF - 1, y, z), g.CLAMP_INSERT_D, (1, 0, 0), g.CLAMP_INSERT_DEPTH + 1)
    for x, y in ((cx - LOW_HALF - foot + 15, 40.0), (cx - LOW_HALF - foot + 15, -40.0)):   # optional table screws
        p = g.csk_hole(p, x, y, P, P)
    return p


def extrusion(L):
    return cq.Workplane("XY").rect(20, 20).extrude(L).faces(">Z").workplane().hole(4.2)


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    clamp = mast_clamp_tall()
    cq.exporters.export(clamp, str(OUT / "mast_clamp_tall.step"))
    cq.exporters.export(clamp, str(OUT / "mast_clamp_tall.stl"), tolerance=0.02, angularTolerance=0.1)

    cx_local = -g.PLATE_T - LOW_HALF
    xL = -g.WALL_T + cx_local                       # upright axis, left wall
    xR = g.FIELD + g.WALL_T - cx_local              # upright axis, right wall
    arm_floor = g.SOCK_IN / 2 + 2                   # elbow's arm socket starts this far from the upright axis
    beam = round((xR - xL) - 2 * arm_floor - 2)     # 1 mm clearance at each end
    for name, L in ((f"2020_goalpost_upright_{BAR:.0f}mm", BAR), (f"2020_goalpost_beam_{beam}mm", beam)):
        cq.exporters.export(extrusion(L), str(OUT / f"{name}.step"))

    Y = g.MAST_Y
    pc, grey, wc = cq.Color(0.85, 0.35, 0.25), cq.Color(0.72, 0.72, 0.75), cq.Color(0.95, 0.55, 0.15)
    asm = cq.Assembly(name="goalpost_mast")
    mod = g.wall_module()
    asm.add(g.placed(mod, -90, 0, g.MAST_Y + 250), name="wall_left_ref_a", color=wc)     # same placement
    asm.add(g.placed(mod, -90, 0, g.MAST_Y), name="wall_left_ref_b", color=wc)           # as the field
    asm.add(g.placed(mod, 90, g.FIELD, g.MAST_Y - 250), name="wall_right_ref_a", color=wc)
    asm.add(g.placed(mod, 90, g.FIELD, g.MAST_Y), name="wall_right_ref_b", color=wc)
    asm.add(g.placed(clamp, 0, -g.WALL_T, Y), name="mast_clamp_tall_left", color=pc)
    asm.add(g.placed(clamp, 180, g.FIELD + g.WALL_T, Y), name="mast_clamp_tall_right", color=pc)
    for nm, x in (("left", xL), ("right", xR)):
        asm.add(g.box(x - 10, Y - 10, BASE_Z, x + 10, Y + 10, UPRIGHT_TOP), name=f"upright_{nm}_{BAR:.0f}mm", color=grey)
    elbow = g.mast_elbow()
    asm.add(elbow.translate((xL, Y, UPRIGHT_TOP)), name="mast_elbow_left", color=cq.Color(0.55, 0.3, 0.75))
    asm.add(elbow.rotate((0, 0, 0), (0, 0, 1), 180).translate((xR, Y, UPRIGHT_TOP)), name="mast_elbow_right",
            color=cq.Color(0.55, 0.3, 0.75))
    asm.add(g.box(xL + arm_floor + 1, Y - 10, g.ARM_BOTTOM, xR - arm_floor - 1, Y + 10, UPRIGHT_TOP),
            name=f"beam_{beam}mm", color=grey)
    asm.add(g.placed(g.camera_holder(), 0, g.FIELD / 2, Y, g.ARM_BOTTOM), name="camera_holder", color=cq.Color(0.2, 0.45, 0.8))
    asm.add(g.placed(g.camera_dummy(), 0, g.FIELD / 2, Y, g.ARM_BOTTOM), name="camera_placeholder", color=cq.Color(0.1, 0.55, 0.25))
    asm.export(str(OUT / "mast_goalpost_assembly.step"))

    s = clamp.val(); bb = s.BoundingBox()
    print(f"mast_clamp_tall valid={s.isValid()} {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, "
          f"fits A1: {max(bb.xlen, bb.ylen, bb.zlen) <= 256}")
    print(f"uprights: 2 x {BAR:.0f} mm (uncut), standing at z = {BASE_Z:.0f} mm, tops at {UPRIGHT_TOP:.0f} mm")
    print(f"beam: 1 x {beam} mm (cut from a third 1200 mm bar)")
    return asm, (xL, xR, beam)


if __name__ == "__main__":
    build()
