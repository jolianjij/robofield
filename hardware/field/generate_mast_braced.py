"""Braced goalpost camera mast, cut from one 6 m 2020 bar.

    python hardware/field/generate_mast_braced.py

  2 x upright 1352 mm, standing on the table in the original mast clamps (left + right wall)
  1 x beam 1036 mm between two mast elbows
  4 x brace 505 mm at ~65.5 deg, sideways (perpendicular to the beam): from a double brace clamp
      on the upright (~609 mm up) down to a brace anchor on the wall joint at y = 250 / 750
Nothing is screwed to the table: the brace anchors replace the 4 splice plates at those wall
joints and bolt into the same wall inserts.
New printed parts: brace_anchor (x4) and brace_clamp_double (x2). Everything else reuses parts from generate_field_parts.py.
"""
import math

import cadquery as cq

import generate_field_parts as g

OUT = g.OUT
UPRIGHT = g.UPRIGHT_L                  # 1352
BRACE = 505.0                          # braces already cut to 505 mm: geometry follows from this
H_OFF = 250.0                          # horizontal offset: upright (y = 500) -> wall joint (y = 250 / 750)
ANCHOR_Z = 60.0                        # brace axis point at the anchor, height above the table
FLOOR = 44.0                           # anchor socket floor, from its reference point
FLOOR_C = 59.0                         # clamp socket floor: leaves a 4.7 mm wall between socket and upright bore
SOCK_DEPTH = 35.0
COLLAR_HALF = 25.0                     # collar height 50 mm
COLLAR_SCREW_Z = (15.0, -15.0)         # two T-nut screws on the outward face
PLAY = 1.0                             # total end play for the brace
AXIS_L = BRACE + FLOOR + FLOOR_C + PLAY                         # 603 mm between reference points
ANGLE = math.degrees(math.acos(H_OFF / AXIS_L))                 # ~65.5 deg above horizontal
CLAMP_Z = ANCHOR_Z + H_OFF * math.tan(math.radians(ANGLE))      # ~609 mm
FOOT_HALF = 18.0                       # keeps the foot clear of the wall (3 mm gap)
SI = g.SOCK_IN / 2


def brace_anchor():
    """Splice plate + brace socket in one part. Replaces the splice plate at a wall joint and
    bolts into the same 4 wall inserts. Local: joint at x = 0, wall outer face at y = 0, plate on
    y < 0, z from the table. The brace rises toward +x at ANGLE; its axis passes through
    (0, -Y_AX, ANCHOR_Z), Y_AX = distance of the upright axis from the wall face."""
    hw = g.SPLICE_W / 2
    y_ax = g.PLATE_T + g.MAST_SOCK_OUT / 2                       # 21 mm, same line as the upright
    h, reach = 18.0, FLOOR + SOCK_DEPTH
    p = g.box(-hw, -g.SPLICE_T, g.SPLICE_Z[0], hw, 0, g.SPLICE_Z[1])
    tilt = lambda w: w.rotate((0, 0, 0), (0, 1, 0), -ANGLE)      # +x -> up at ANGLE
    block = tilt(g.box(FLOOR - 10, -h, -h, reach, h, h)).translate((0, -y_ax, ANCHOR_Z))
    p = p.union(block)
    p = p.union(g.box(-hw, -y_ax - h, g.SPLICE_Z[0], hw, -g.SPLICE_T + 0.01, g.SPLICE_Z[0] + 8))   # bottom web
    p = p.cut(tilt(g.box(FLOOR, -SI, -SI, reach + 1, SI, SI)).translate((0, -y_ax, ANCHOR_Z)))
    p = p.cut(g.box(-200, -200, -100, 200, 200, g.SPLICE_Z[0]))   # nothing below the plate bottom
    for x in (-g.JOINT_X, g.JOINT_X):
        for z in g.JOINT_Z:
            # M3 x 12 through the 6 mm plate into the wall insert; a 7 mm channel through anything
            # in front keeps the screw head and a straight screwdriver reachable from outside
            p = g.cyl_hole(p, (x, -g.SPLICE_T - 1, z), g.M3_CLEAR, (0, 1, 0), g.SPLICE_T + 2)
            p = g.cyl_hole(p, (x, -80, z), 7.0, (0, 1, 0), 80 - g.SPLICE_T)
    u = FLOOR + SOCK_DEPTH / 2
    c, sa = math.cos(math.radians(ANGLE)), math.sin(math.radians(ANGLE))
    n = (-sa, 0, c)                                              # outward normal of the socket's top face
    pt = (u * c + h * n[0], -y_ax, ANCHOR_Z + u * sa + h * n[2])                # centre of the top face
    p = g.tnut_hole(p, (pt[0] + n[0], pt[1], pt[2] + n[2]), (-n[0], 0, -n[2]), h - SI)   # T-nut in the brace
    return p


def brace_clamp_double():
    """Collar that slides on an upright (axis z) with two sockets pointing DOWN at ANGLE to opposite
    sides (+x and -x), one per brace. Everything locks with M3 screws into 2020 T-nuts:
      - 2 screws through the +y face into the upright's slot (the +y face is mounted facing OUTWARD,
        away from the wall: the side toward the wall has only ~3 mm of room)
      - 1 screw through each socket's top face into the brace's slot
    Print lying on the -y face (all square holes horizontal, 20 mm bridges, no supports)."""
    h, si = g.SOCK_OUT / 2, SI
    wall = h - si
    reach = FLOOR_C + SOCK_DEPTH
    p = g.box(-h, -h, -COLLAR_HALF, h, h, COLLAR_HALF)
    for side in (0, 180):
        arm = g.box(0, -h, -h, reach, h, h).rotate((0, 0, 0), (0, 1, 0), ANGLE)    # +x and down
        hole = g.box(FLOOR_C, -si, -si, reach + 1, si, si).rotate((0, 0, 0), (0, 1, 0), ANGLE)
        p = p.union(arm.rotate((0, 0, 0), (0, 0, 1), side))
        p = p.cut(hole.rotate((0, 0, 0), (0, 0, 1), side))
    p = p.cut(g.box(-si, -si, -300, si, si, 300))                                   # slides on the upright
    for z in COLLAR_SCREW_Z:                                                        # into the upright's slot
        p = g.tnut_hole(p, (0, h + 1, z), (0, -1, 0), wall)
    u = FLOOR_C + SOCK_DEPTH / 2
    ca, sa = math.cos(math.radians(ANGLE)), math.sin(math.radians(ANGLE))
    for sx in (1, -1):                                                              # into each brace's slot
        n = (sx * sa, 0, ca)                                                        # socket top-face normal
        c = (sx * u * ca + h * n[0], 0.0, -u * sa + h * n[2])                       # centre of that face
        p = g.tnut_hole(p, (c[0] + n[0], 0.0, c[2] + n[2]), (-n[0], 0, -n[2]), wall)
    return p


def extrusion(L):
    return cq.Workplane("XY").rect(20, 20).extrude(L).faces(">Z").workplane().hole(4.2)


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    anchor = brace_anchor()
    dbl = brace_clamp_double()
    for n, part in (("brace_anchor", anchor), ("brace_clamp_double", dbl)):
        cq.exporters.export(part, str(OUT / f"{n}.step"))
        cq.exporters.export(part, str(OUT / f"{n}.stl"), tolerance=0.02, angularTolerance=0.1)
    clamp, elbow, holder = g.mast_clamp(), g.mast_elbow(), g.camera_holder()

    xL = -g.WALL_T - g.PLATE_T - g.MAST_SOCK_OUT / 2
    xR = g.FIELD - xL
    arm_floor = g.SOCK_IN / 2 + 2
    beam = round((xR - xL) - 2 * arm_floor - 2)
    for n, L in ((f"2020_upright_{UPRIGHT:.0f}mm", UPRIGHT), (f"2020_beam_{beam}mm", beam), (f"2020_brace_{BRACE:.0f}mm", BRACE)):
        cq.exporters.export(extrusion(L), str(OUT / f"{n}.step"))

    Y = g.MAST_Y
    red, grey, purple, orange, blue, green, wood = (cq.Color(0.85, 0.35, 0.25), cq.Color(0.72, 0.72, 0.75),
        cq.Color(0.55, 0.3, 0.75), cq.Color(0.95, 0.6, 0.15), cq.Color(0.2, 0.45, 0.8), cq.Color(0.1, 0.55, 0.25),
        cq.Color(0.92, 0.75, 0.5))
    asm = cq.Assembly(name="braced_goalpost_mast")
    mod = g.wall_module()
    for k, (rot, x, y) in enumerate([(-90, 0, 250), (-90, 0, Y), (-90, 0, Y + 250), (-90, 0, 1000),
                                     (90, g.FIELD, 0), (90, g.FIELD, 250), (90, g.FIELD, Y), (90, g.FIELD, 750)]):
        asm.add(g.placed(mod, rot, x, y), name=f"wall_ref_{k}", color=wood)
    asm.add(g.placed(clamp, 0, -g.WALL_T, Y), name="mast_clamp_left", color=red)
    asm.add(g.placed(clamp, 180, g.FIELD + g.WALL_T, Y), name="mast_clamp_right", color=red)
    for nm, x in (("left", xL), ("right", xR)):
        asm.add(g.box(x - 10, Y - 10, 0, x + 10, Y + 10, UPRIGHT), name=f"upright_{nm}_{UPRIGHT:.0f}mm", color=grey)
    asm.add(elbow.translate((xL, Y, UPRIGHT)), name="mast_elbow_left", color=purple)
    asm.add(elbow.rotate((0, 0, 0), (0, 0, 1), 180).translate((xR, Y, UPRIGHT)), name="mast_elbow_right", color=purple)
    asm.add(g.box(xL + arm_floor + 1, Y - 10, g.ARM_BOTTOM, xR - arm_floor - 1, Y + 10, UPRIGHT), name=f"beam_{beam}mm", color=grey)
    asm.add(g.placed(holder, 0, g.FIELD / 2, Y, g.ARM_BOTTOM), name="camera_holder", color=blue)
    asm.add(g.placed(g.camera_dummy(), 0, g.FIELD / 2, Y, g.ARM_BOTTOM), name="camera_placeholder", color=green)
    # braces: s = -1 -> anchor at y = 250, s = +1 -> anchor at y = 750 (wall joints)
    ca, sa = math.cos(math.radians(ANGLE)), math.sin(math.radians(ANGLE))
    for nm, x, wall_x, rot in (("left", xL, -g.WALL_T, -90), ("right", xR, g.FIELD + g.WALL_T, 90)):
        # local +y (screw face) must point away from the wall: -x on the left, +x on the right
        asm.add(dbl.rotate((0, 0, 0), (0, 0, 1), 90 if nm == "left" else -90).translate((x, Y, CLAMP_Z)),
                name=f"brace_clamp_double_{nm}", color=orange)
        for s_ in (-1, 1):
            jy = Y + s_ * H_OFF
            # anchor local: joint at x=0, wall face y=0 (plate y<0), brace rises toward +x.
            # left wall: local y<0 -> global x<-10 ; brace must rise toward the upright (y = 500)
            a_ = anchor if (s_ == -1) == (nm == "left") else anchor.mirror("YZ")
            asm.add(g.placed(a_, rot, wall_x, jy), name=f"brace_anchor_{nm}_y{jy:.0f}", color=orange)
            p0 = (x, jy + s_ * -1 * -FLOOR * ca * -1, ANCHOR_Z)
            mid_u = FLOOR + PLAY / 2 + BRACE / 2
            dirv = (0, -s_ * ca, sa)                              # from anchor toward the upright
            cx_, cy_, cz_ = x, jy + dirv[1] * mid_u, ANCHOR_Z + dirv[2] * mid_u
            br = (cq.Workplane("XY").box(20, BRACE, 20)
                  .rotate((0, 0, 0), (1, 0, 0), (ANGLE if s_ == -1 else -ANGLE))
                  .translate((cx_, cy_, cz_)))
            asm.add(br, name=f"brace_{nm}_y{jy:.0f}_{BRACE:.0f}mm", color=grey)
    asm.export(str(OUT / "mast_braced_assembly.step"))
    return asm, (xL, xR, beam)


if __name__ == "__main__":
    asm, (xL, xR, beam) = build()
    for n, f in (("brace_anchor", brace_anchor), ("brace_clamp_double", brace_clamp_double)):
        s = f().val(); bb = s.BoundingBox()
        print(f"{n} valid={s.isValid()} {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")
    print(f"cut list: 2 x {UPRIGHT:.0f}, 1 x {beam}, 4 x {BRACE:.0f} mm; angle {ANGLE:.1f} deg; clamp centre at {CLAMP_Z:.0f} mm")
