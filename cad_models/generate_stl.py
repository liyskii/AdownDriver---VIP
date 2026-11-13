#!/usr/bin/env python3
"""
Parametric source of every 3D-printed part of AdownDriver.

    pip install manifold3d trimesh matplotlib
    python3 cad_models/generate_stl.py            # writes the STL files + parts_preview.png

All dimensions in millimetres, every part is exported in its print orientation
(flat face on the bed, z up). Change the parameters below and re-run; import the
STL into Fusion 360 if you need to edit further.

Design rules (ISE "3D Printing" lecture, week 13, and the M-124 printer guide):
  * MakerBot Sketch Large Classroom, max build 220 x 220 x 250 mm  -> checked in main()
  * no overhang steeper than 45 deg from vertical without support -> gussets / teardrops
  * walls >= 0.8 mm                                                -> thinnest wall is 1.6 mm
  * avoid large flat sharp-cornered plates (warping)               -> rounded corners

Parts
-----
pi_camera_v2_mount_25deg.stl    Node 2  Pi Camera V2 (M2 holes 21 x 12.5 mm) on a plate tilted 25 deg
                                        down; base with two 16 mm M3 slots for the TurtleBot waffle plate.
vl53l1x_tof_bumper_bracket.stl  Node 2  VL53L1X on an upright tilted TOF_PITCH_UP_DEG upwards, so the
                                        27 deg cone does not hit the floor in front of the robot.
cockpit_webcam_stand.stl        Node 1  Desk stand for the Logitech C270 (no tripod thread - it hangs by
                                        its own clip on the ledge); top-open ballast pocket.
node2_status_pod_box.stl        Node 2  Box for the E-STOP / RESET buttons, RGB LED and buzzer on the robot
node2_status_pod_lid.stl                (replaces the loose breadboard on a moving robot).
node1_cockpit_panel_box.stl     Node 1  Cockpit panel: 0.96" SSD1306 OLED window, CALIBRATE / ACK buttons,
node1_cockpit_panel_lid.stl             RGB LED and buzzer.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from manifold3d import CrossSection, Manifold

OUT = Path(__file__).resolve().parent
SEG = 48
BUILD_VOLUME = (220.0, 220.0, 250.0)   # MakerBot Sketch Large Classroom (ISE M-124)

# ------------------------------ parameters ------------------------------------
CAM_PITCH_DEG = 25.0        # downward tilt of the road camera (config: camera.pitch_deg)
CAM_LENS_HEIGHT = 60.0      # approx. lens centre above the mount face
TOF_CENTER_HEIGHT = 45.0    # VL53L1X optical centre above the mount face (before tilt)
TOF_PITCH_UP_DEG = 10.0     # upward tilt: lower cone edge = 13.5 - 10 = 3.5 deg below horizontal
WEBCAM_COLUMN = 120.0       # stand height (C270 clip ledge)
M2, M25, M3 = 2.3, 2.8, 3.4  # clearance hole diameters

# enclosures (measure your parts and adjust)
BTN_HOLE = 12.4             # 12 mm panel push button or the cap of a 12x12 tactile switch
LED_HOLE = 5.2              # 5 mm RGB LED (the 5.8 mm flange stops it from inside)
BUZZER_D = 12.4             # 12 mm active buzzer, held in a ring under the lid
OLED_PCB = (27.5, 28.0)     # 0.96" SSD1306 module board
OLED_HOLES = (23.5, 23.8)   # M2 hole pattern of the module (varies by vendor: 23-24 mm)
OLED_WINDOW = (25.0, 14.0)  # opening over the glass (active area 21.7 x 10.9 mm)
OLED_WINDOW_DY = 1.5        # glass centre is above the board centre
WALL, FLOOR, LID_T = 2.4, 2.4, 2.5
LID_CLEAR = 0.3             # clearance of the lid lip inside the box (per side)


# ------------------------------ helpers ---------------------------------------
def box(x, y, z, center_xy=True):
    m = Manifold.cube([x, y, z])
    return m.translate([-x / 2, -y / 2, 0]) if center_xy else m


def cyl(d, h, z0=0.0):
    return Manifold.cylinder(h, d / 2, d / 2, SEG).translate([0, 0, z0])


def rbox(x, y, z, r):
    """Box with rounded vertical edges, centred in XY, z from 0."""
    r = min(r, x / 2 - 0.01, y / 2 - 0.01)
    pts = [cyl(2 * r, z).translate([sx * (x / 2 - r), sy * (y / 2 - r), 0]) for sx in (-1, 1) for sy in (-1, 1)]
    return Manifold.batch_hull(pts)


def slot(d, length, h, z0=-1.0):
    """Slot along X."""
    return (box(length, d, h).translate([0, 0, z0]) + cyl(d, h, z0).translate([-length / 2, 0, 0]) +
            cyl(d, h, z0).translate([length / 2, 0, 0]))


def prism_yz(points, width):
    """Extrude a polygon drawn in the (y, z) plane along X, centred on x = 0."""
    m = CrossSection([points]).extrude(width)          # (u, v, w)
    m = m.rotate([90, 0, 0]).rotate([0, 0, 90])          # -> u = Y, v = Z, w = X
    return m.translate([-width / 2, 0, 0])


def teardrop_y(d, length, y0, z):
    """Horizontal hole along Y that prints without support (45 deg roof)."""
    r = d / 2
    pts = [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(math.radians(-45), math.radians(225), 40)]
    pts.append((0.0, -r * math.sqrt(2)))      # flipped to the top by the rotation below
    m = CrossSection([pts]).extrude(length)              # x, z(profile) , y(extrude)
    m = m.rotate([-90, 0, 0])                            # profile y -> z, extrusion -> +Y
    return m.translate([0, y0, z])


def base_with_slots(w, d, t, slot_len=16.0, spacing=None, r=3.0):
    spacing = spacing or w - 14
    b = rbox(w, d, t, r)
    for sx in (-spacing / 2, spacing / 2):
        b = b - slot(M3, slot_len - M3, t + 2).rotate([0, 0, 90]).translate([sx, 0, 0])
    return b


def above_bed(m: Manifold) -> Manifold:
    return m ^ box(400, 400, 400)


# ------------------------------ Node 2 mounts ---------------------------------
def pi_camera_mount() -> Manifold:
    t = 4.0
    base = base_with_slots(44, 34, t)
    # camera plate 30 x 3 x 30, rotated about X so the lens looks CAM_PITCH_DEG down
    pw, ph, pt = 30.0, 30.0, 3.0
    plate = box(pw, pt, ph)
    # Pi Camera V2 holes: 21 mm apart horizontally, 12.5 mm vertically; board 25 x 24 mm,
    # upper hole pair 2 mm below the board top edge; 3 mm bosses clear the rear components.
    top = ph - 3.0
    for hx in (-10.5, 10.5):
        for hz in (top - 2.0, top - 14.5):
            plate = plate + cyl(5.0, 3.0).rotate([90, 0, 0]).translate([hx, -pt / 2, hz])
            plate = plate - cyl(M2, 12).rotate([90, 0, 0]).translate([hx, 6, hz])
    stem_h = CAM_LENS_HEIGHT - ph / 2
    stem = box(pw, 6.0, stem_h)
    # front face (-Y, lens side) points CAM_PITCH_DEG below horizontal; plate sits on top of the stem.
    # (rev. 2 lost this translate in a comment and printed the plate at the base.)
    plate = plate.rotate([CAM_PITCH_DEG, 0, 0]).translate([0, 0, stem_h - 2])
    gusset = prism_yz([(0, t), (10, t), (0, t + 10)], 3.0)            # 45 deg rear gusset
    body = base + stem.translate([0, 4, 0]) + plate.translate([0, 4, 0])
    for gx in (-12, 12):
        body = body + gusset.translate([gx, 7, 0])
    return above_bed(body)


def tof_bracket() -> Manifold:
    """VL53L1X on the outer (+Y = robot front) face. The upright leans back by TOF_PITCH_UP_DEG, so the
    sensor looks slightly up: with a 27 deg cone (13.5 deg half angle) the lower edge is only 3.5 deg below
    horizontal and the floor enters the cone at h / tan(3.5 deg) ~ 0.75 m (h = 45 mm) instead of ~0.19 m,
    i.e. far beyond the 20 cm EMERGENCY_STOP threshold."""
    t = 4.0
    base = base_with_slots(40, 24, t)
    up_h = TOF_CENTER_HEIGHT + 12
    upright = box(32, 3.5, up_h)                       # centred on y = 0, pivot at the bed
    for side in (-1, 1):                               # two horizontal M2.5 slots, 13-25 mm hole spacing
        s = slot(M25, 6.0, 12).rotate([90, 0, 0]).translate([side * 9.5, 6, TOF_CENTER_HEIGHT])
        upright = upright - s
    # clearance window behind the sensor (rear components of the breakout)
    upright = upright - box(8, 12, 6).translate([0, 0, TOF_CENTER_HEIGHT - 3])
    upright = upright.rotate([TOF_PITCH_UP_DEG, 0, 0]).translate([0, 10, 0])
    a = math.tan(math.radians(TOF_PITCH_UP_DEG))
    gusset = prism_yz([(-1, 0), (10, 0), (10 - 20 * a, 20)], 3.0)    # follows the tilted upright
    body = base + upright + gusset.translate([-12, 0, 0]) + gusset.translate([12, 0, 0])
    return above_bed(body)


# ------------------------------ Node 1 stand ----------------------------------
def webcam_stand() -> Manifold:
    bt = 8.0
    base = rbox(90, 90, bt, 8)
    # ballast pocket open from the TOP (rev. 2 opened it downwards = 60 mm unsupported bridge)
    pocket = rbox(70, 70, bt, 6).translate([0, 0, 3]) - box(48, 50, 20).translate([0, 9, 0])
    base = base - pocket
    column = box(40, 16, WEBCAM_COLUMN).translate([0, 0, bt])         # as wide as the ledge: no side overhang
    ledge = box(40, 30, 6).translate([0, 7, bt + WEBCAM_COLUMN])           # C270 clip hooks over this
    lip = box(40, 3, 10).translate([0, -6.5, bt + WEBCAM_COLUMN - 4])      # keeps the clip from sliding off
    ztop = bt + WEBCAM_COLUMN
    ledge_gusset = prism_yz([(8, ztop - 14), (22, ztop), (8, ztop)], 40)  # 45 deg under the 14 mm overhang
    fillet = Manifold.cube([30, 20, 20]).translate([-15, 8, bt]) - cyl(40, 30).rotate([0, 90, 0]).translate([-16, 28, bt + 20])
    body = base + column + ledge + lip + ledge_gusset + fillet
    # USB cable pass-through big enough for the USB-A plug (12 x 4.5 mm), teardrop roof
    body = body - teardrop_y(14, 60, -30, bt + 22)
    return above_bed(body)


# ------------------------------ enclosures ------------------------------------
def enclosure_box(inner_x, inner_y, inner_h, cable_side="+x", floor_slots=True) -> Manifold:
    ox, oy = inner_x + 2 * WALL, inner_y + 2 * WALL
    h = FLOOR + inner_h
    b = rbox(ox, oy, h, 4) - rbox(inner_x, inner_y, h, 4 - WALL * 0.5).translate([0, 0, FLOOR])
    # cable notch for the Dupont bundle to the Pi header (open at the top, lid closes it)
    nw, nd = 16.0, 10.0                     # leaves ~7 mm under the lid lip: two rows of Dupont leads
    if cable_side == "+x":
        b = b - box(WALL * 3, nw, nd + 1).translate([ox / 2, 0, h - nd])
    else:
        b = b - box(nw, WALL * 3, nd + 1).translate([0, -oy / 2, h - nd])
    if floor_slots:  # M3 slots in the floor for the TurtleBot waffle plate (screw heads inside)
        for sx in (-(inner_x / 2 - 10), inner_x / 2 - 10):
            b = b - slot(M3, 12, FLOOR + 2).rotate([0, 0, 90]).translate([sx, 0, 0])
    return b


def enclosure_lid(inner_x, inner_y) -> Manifold:
    """Printed top face down: plate + friction lip that drops inside the box walls."""
    ox, oy = inner_x + 2 * WALL, inner_y + 2 * WALL
    plate = rbox(ox, oy, LID_T, 4)
    lx, ly = inner_x - 2 * LID_CLEAR, inner_y - 2 * LID_CLEAR
    lip = rbox(lx, ly, 3.0, 3) - rbox(lx - 3.2, ly - 3.2, 5.0, 2).translate([0, 0, -1.0])
    return plate + lip.translate([0, 0, LID_T])


def lid_features(lid: Manifold, buttons, led, buzzer, inner_x, inner_y) -> Manifold:
    for (bx, by) in buttons:
        lid = lid - cyl(BTN_HOLE, 20, -1).translate([bx, by, 0])
    lx, ly = led
    lid = lid - cyl(LED_HOLE, 20, -1).translate([lx, ly, 0])
    zx, zy = buzzer
    ring = cyl(BUZZER_D + 3.2, 5.0) - cyl(BUZZER_D, 6.0, -0.5)
    lid = lid + ring.translate([zx, zy, LID_T])
    for k in range(6):                                  # sound holes
        a = 2 * math.pi * k / 6
        lid = lid - cyl(2.0, 20, -1).translate([zx + 3.2 * math.cos(a), zy + 3.2 * math.sin(a), 0])
    lid = lid - cyl(2.0, 20, -1).translate([zx, zy, 0])
    # keep features clear of the lip
    return lid


def mirror_x(m: Manifold) -> Manifold:
    return m.mirror([1, 0, 0])


POD_IN = (60.0, 44.0, 24.0)


def node2_pod_box() -> Manifold:
    return enclosure_box(*POD_IN, cable_side="+x")


def node2_pod_lid() -> Manifold:
    ix, iy, _ = POD_IN
    lid = enclosure_lid(ix, iy)
    # printed upside down -> layout mirrored in X so it reads correctly when fitted
    lid = lid_features(lid, buttons=[(-17, 9), (-17, -11)], led=(3, 9), buzzer=(15, -8), inner_x=ix, inner_y=iy)
    # raised marks next to the buttons: one dot = E-STOP (A), two dots = RESET (B) - legible by touch
    for (x, y) in [(-6, 9), (-6, -8), (-6, -14)]:
        lid = lid - cyl(2.4, 0.8, -0.01).translate([x, y, 0])
    return mirror_x(lid)


PANEL_IN = (84.0, 56.0, 26.0)


def node1_panel_box() -> Manifold:
    return enclosure_box(*PANEL_IN, cable_side="-y", floor_slots=False)


def node1_panel_lid() -> Manifold:
    ix, iy, _ = PANEL_IN
    lid = enclosure_lid(ix, iy)
    ox_, oy_ = -18.0, 4.0                                # OLED board centre
    wx, wy = OLED_WINDOW
    lid = lid - box(wx, wy, 20).translate([ox_, oy_ + OLED_WINDOW_DY, -1])
    hx, hy = OLED_HOLES
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = [ox_ + sx * hx / 2, oy_ + sy * hy / 2]
            lid = lid + cyl(5.0, 2.2).translate([p[0], p[1], LID_T])     # stand-offs: glass touches the lid
            lid = lid - cyl(M2 - 0.4, 10, LID_T - 1.0).translate([p[0], p[1], 0])  # M2 self-tapping
    lid = lid_features(lid, buttons=[(12, 10), (30, 10)], led=(12, -12), buzzer=(28, -12), inner_x=ix, inner_y=iy)
    for (x, y) in [(12, 20), (28, 20), (32, 20)]:       # 1 dot = CALIBRATE (A), 2 dots = ACK (B)
        lid = lid - cyl(2.4, 0.8, -0.01).translate([x, y, 0])
    return mirror_x(lid)


# ------------------------------ export ---------------------------------------
def to_trimesh(m: Manifold):
    import trimesh
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


def preview(meshes: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    n = len(meshes)
    cols = 4
    rows = math.ceil(n / cols)
    fig = plt.figure(figsize=(16, 4.4 * rows))
    for i, (name, tm) in enumerate(meshes.items()):
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d")
        tris = tm.vertices[tm.faces]
        light = np.clip(np.abs(tm.face_normals @ np.array([0.4, -0.6, 0.7])), 0.25, 1)
        ax.add_collection3d(Poly3DCollection(tris, facecolors=plt.cm.Greys(0.35 + 0.5 * light), edgecolor="none"))
        lo, hi = tm.bounds
        span = (hi - lo).max() / 2
        c = (hi + lo) / 2
        ax.set_xlim(c[0] - span, c[0] + span)
        ax.set_ylim(c[1] - span, c[1] + span)
        ax.set_zlim(c[2] - span, c[2] + span)
        ax.view_init(24, -60)
        ax.set_title(f"{name}\n{(hi - lo).round(1)} mm", fontsize=9)
        ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=100)


PARTS = {
    "pi_camera_v2_mount_25deg": pi_camera_mount,
    "vl53l1x_tof_bumper_bracket": tof_bracket,
    "cockpit_webcam_stand": webcam_stand,
    "node2_status_pod_box": node2_pod_box,
    "node2_status_pod_lid": node2_pod_lid,
    "node1_cockpit_panel_box": node1_panel_box,
    "node1_cockpit_panel_lid": node1_panel_lid,
}


def main() -> None:
    meshes = {}
    for name, fn in PARTS.items():
        m = fn()
        assert len(m.decompose()) == 1, f"{name}: more than one body"
        tm = to_trimesh(m)
        assert tm.is_watertight, name
        lo, hi = tm.bounds
        assert abs(lo[2]) < 1e-6, f"{name}: not on the bed (z_min = {lo[2]:.2f})"
        assert all(e <= b for e, b in zip(sorted(tm.extents), sorted(BUILD_VOLUME))), f"{name}: too big"
        tm.export(OUT / f"{name}.stl")
        meshes[name] = tm
        print(f"{name}.stl  {len(tm.faces)} triangles, {tm.volume / 1000:.1f} cm^3, bbox {tm.extents.round(1)} mm")
    preview(meshes, OUT / "parts_preview.png")


if __name__ == "__main__":
    main()
