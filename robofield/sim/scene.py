"""Build the MuJoCo scene (MJCF) from config/field.yaml.

The field is generated, not imported from CAD: change the config and the simulated field changes.
MuJoCo works in metres; the rest of RoboField in mm. Everything here converts once.
"""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np

MM = 0.001
CACHE = Path.home() / ".cache" / "robofield" / "textures"


def marker_texture(dictionary: str, marker_id: int, quiet_ratio: float = 0.125, px: int = 256) -> str:
    """PNG of an ArUco marker with a white quiet zone; returns its path (cached)."""
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE / f"{dictionary}_{marker_id}_{quiet_ratio:.3f}_{px}.png"
    if not path.exists():
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        inner = int(px / (1 + 2 * quiet_ratio))
        tag = cv2.aruco.generateImageMarker(d, marker_id, inner)
        pad = (px - inner) // 2
        tag = cv2.copyMakeBorder(tag, pad, px - inner - pad, pad, px - inner - pad, cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(str(path), cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR))
    return str(path)


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def robot_xml(i: int, m: dict, marker_tex: str | None) -> str:
    r = m["wheel_radius_mm"] * MM
    half_track = m["track_mm"] / 2 * MM
    ww = m["wheel_width_mm"] / 2 * MM
    L, W = m["length_mm"] * MM, m["width_mm"] * MM
    cx = -m["axle_offset_mm"] * MM                     # body centre, measured from the axle
    caster_x = cx + m["caster_offset_mm"] * MM
    cr = m["caster_radius_mm"] * MM
    clear = 0.005                                      # chassis bottom above the floor
    top = m["height_mm"] * MM
    chassis_h = top - 0.004 - clear
    mass = m["mass_g"] / 1000
    marker = ""
    if marker_tex:   # textured slab mesh (planes are only allowed on static bodies)
        marker = (f'<geom name="r{i}_marker" type="mesh" mesh="marker_slab" pos="{_f(0, 0, top - r)}" '
                  f'material="mat_r{i}" contype="0" conaffinity="0" mass="0.0001" group="1"/>')
    return f"""
    <body name="robot{i}" pos="{_f(0.2 + 0.1 * i, 0.5, r)}">
      <freejoint name="r{i}_free"/>
      <geom name="r{i}_chassis" type="box" size="{_f(L / 2, W / 2 - 2 * ww - 0.001, chassis_h / 2)}"
            pos="{_f(cx, 0, clear + chassis_h / 2 - r)}" mass="{mass * 0.85:.4f}" rgba="0.2 0.45 0.8 1"/>
      <geom name="r{i}_top" type="box" size="{_f(L / 2, W / 2, 0.002)}" pos="{_f(cx, 0, top - r - 0.002)}"
            mass="0.005" rgba="0.95 0.95 0.95 1"/>
      {marker}
      <geom name="r{i}_caster" type="sphere" size="{cr}" pos="{_f(caster_x, 0, cr - r)}" condim="1" priority="1"
            mass="0.005" rgba="0.6 0.6 0.6 1"/>
      <body name="r{i}_wl" pos="{_f(0, half_track, 0)}">
        <joint name="r{i}_left" type="hinge" axis="0 1 0" damping="0.00005" armature="0.0001" frictionloss="0.0005"/>
        <geom type="cylinder" size="{_f(r, ww)}" euler="90 0 0" mass="0.02" friction="1.2 0.02 0.001"
              rgba="0.1 0.1 0.1 1"/>
      </body>
      <body name="r{i}_wr" pos="{_f(0, -half_track, 0)}">
        <joint name="r{i}_right" type="hinge" axis="0 1 0" damping="0.00005" armature="0.0001" frictionloss="0.0005"/>
        <geom type="cylinder" size="{_f(r, ww)}" euler="90 0 0" mass="0.02" friction="1.2 0.02 0.001"
              rgba="0.1 0.1 0.1 1"/>
      </body>
    </body>"""


def build_scene(cfg: dict, n_robots: int = 1, objects: tuple = (), markers: bool = True) -> str:
    """MJCF string. objects: sequence of ("ball" | "box", id)."""
    f, m, cam, mk = cfg["field"], cfg["robot_model"], cfg["camera"], cfg["markers"]
    W, H = f["width_mm"] * MM, f["height_mm"] * MM
    wt, wh = f["wall_thickness_mm"] * MM, f["wall_height_mm"] * MM
    dt = cfg.get("sim", {}).get("timestep_s", 0.002)
    dic = mk["dictionary"]
    assets, towers = [], []
    if markers:
        quiet = 10.0 / mk["corner_size_mm"]
        for tid, (x, y, z) in mk["corners"].items():
            assets.append(f'<texture name="tex_c{tid}" type="2d" file="{marker_texture(dic, tid, quiet)}"/>'
                          f'<material name="mat_c{tid}" texture="tex_c{tid}" texrepeat="1 1" specular="0" shininess="0"/>')
        half = mk["robot_size_mm"] * 1.2 / 2 * MM          # marker + 10 % quiet zone each side
        v = [(-half, -half, 0.0005), (half, -half, 0.0005), (half, half, 0.0005), (-half, half, 0.0005),
             (-half, -half, 0), (half, -half, 0), (half, half, 0), (-half, half, 0)]
        # image 'up' = robot forward (+x), image 'right' = robot right (-y): tracker heading = theta
        uv = [(1, 1), (1, 0), (0, 0), (0, 1), (1, 1), (1, 0), (0, 0), (0, 1)]
        faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
                 (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
        assets.append(f'<mesh name="marker_slab" vertex="{_f(*[c for p in v for c in p])}" '
                      f'texcoord="{_f(*[c for p in uv for c in p])}" face="{" ".join(str(k) for fc in faces for k in fc)}"/>')
        for i in range(n_robots):
            rid = mk["robot_id_offset"] + i
            q = 0.1                                       # robot marker quiet zone (fraction of marker)
            assets.append(f'<texture name="tex_r{i}" type="2d" file="{marker_texture(dic, rid, q)}"/>'
                          f'<material name="mat_r{i}" texture="tex_r{i}" texrepeat="1 1" specular="0" shininess="0"/>')
    for tid, (x, y, z) in mk["corners"].items():
        x, y, z = x * MM, y * MM, z * MM
        plate = (mk["corner_size_mm"] + 20) / 2 * MM
        towers.append(f'<geom type="box" size="{_f(0.015, 0.015, (z - 0.006) / 2)}" pos="{_f(x, y, (z - 0.006) / 2)}" '
                      f'rgba="0.45 0.6 0.85 1"/>'
                      f'<geom type="box" size="{_f(plate, plate, 0.003)}" pos="{_f(x, y, z - 0.003)}" rgba="1 1 1 1"/>')
        if markers:
            towers.append(f'<geom type="plane" size="{_f(plate, plate, 0.001)}" pos="{_f(x, y, z + 0.0002)}" '
                          f'material="mat_c{tid}" contype="0" conaffinity="0" group="1"/>')
    walls = [
        ((-wt / 2, H / 2), (wt / 2, H / 2 + wt)), ((W + wt / 2, H / 2), (wt / 2, H / 2 + wt)),
        ((W / 2, -wt / 2), (W / 2, wt / 2)), ((W / 2, H + wt / 2), (W / 2, wt / 2)),
    ]
    wall_xml = "".join(f'<geom type="box" size="{_f(sx, sy, wh / 2)}" pos="{_f(px, py, wh / 2)}" '
                       f'rgba="0.95 0.6 0.25 1" friction="0.4 0.005 0.0001"/>' for (px, py), (sx, sy) in walls)
    obj_xml = []
    for kind, oid in objects:
        if kind == "ball":
            obj_xml.append(f'<body name="obj{oid}" pos="{_f(0.5, 0.5, 0.02)}"><freejoint/>'
                           f'<geom type="sphere" size="0.02" mass="0.01" rgba="1 0.9 0.1 1" friction="0.6 0.01 0.001"/></body>')
        else:
            obj_xml.append(f'<body name="obj{oid}" pos="{_f(0.5, 0.5, 0.0205)}"><freejoint/>'
                           f'<geom type="box" size="0.02 0.02 0.02" mass="0.03" rgba="0.9 0.2 0.3 1" friction="0.5 0.01 0.001"/></body>')
    fovy = 2 * math.degrees(math.atan(math.tan(math.radians(51)) * cam["height"] / cam["width"]))
    cx, cy, cz = W / 2, H / 2, cam["height_mm"] * MM
    return f"""<mujoco model="robofield">
  <compiler angle="degree"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic"/>
  <visual><global offwidth="{cam['width']}" offheight="{cam['height']}"/><quality shadowsize="0"/>
          <headlight ambient="0.45 0.45 0.45" diffuse="0.55 0.55 0.55" specular="0 0 0"/></visual>
  <default><geom friction="0.8 0.01 0.001" solref="0.01 1"/></default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.86 0.86 0.84" rgb2="0.82 0.82 0.8" width="512" height="512"/>
    <material name="floor" texture="grid" texrepeat="4 4" specular="0"/>
    {''.join(assets)}
  </asset>
  <worldbody>
    <light pos="{_f(cx, cy, 2.0)}" dir="0 0 -1" diffuse="0.4 0.4 0.4" castshadow="false"/>
    <geom name="floor" type="plane" size="1.2 1.2 0.1" pos="{_f(cx, cy, 0)}" material="floor" friction="0.9 0.01 0.001"/>
    {wall_xml}
    {''.join(towers)}
    <camera name="overhead" pos="{_f(cx, cy, cz)}" fovy="{fovy:.4f}"/>
    {''.join(robot_xml(i, m, f'tex_r{i}' if markers else None) for i in range(n_robots))}
    {''.join(obj_xml)}
  </worldbody>
  <actuator>
    {''.join(f'<general name="r{i}_{s}_motor" joint="r{i}_{s}" ctrlrange="-1 1" biastype="affine" gainprm="{m["stall_torque_nm"]}" biasprm="0 0 {-m["stall_torque_nm"] / (m["max_wheel_rpm"] * 2 * math.pi / 60):.6g}"/>' for i in range(n_robots) for s in ("left", "right"))}
  </actuator>
</mujoco>"""
