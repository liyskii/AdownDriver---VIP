#!/usr/bin/env python3
"""
Parametric source of the 3D-printed parts (replaces the placeholder box STLs of
revision 1, which were 12-triangle cuboids with no holes).

    pip install manifold3d trimesh
    python3 cad_models/generate_stl.py            # writes the STL files + preview PNG

All dimensions in millimetres. Change the parameters below and re-run; import
the STL into Fusion 360 if you need to edit further.

Parts
-----
pi_camera_v2_mount_25deg.stl   Pi Camera V2 (25 x 24 mm, M2 holes 21 x 12.5 mm) on a 25 deg
                               down-tilted plate; base with two 16 mm M3 slots for the TurtleBot
                               waffle plate (slots tolerate the hole pitch, fine-tune yaw).
vl53l1x_tof_bumper_bracket.stl Upright plate with two horizontal M2.5 slots (fits 13-25 mm wide
                               VL53L1X breakouts), sensor centre 45 mm above the mount face,
                               base with two M3 slots.
cockpit_webcam_stand.stl       Desk stand for the Logitech C270. NOTE: the C270 has *no* 1/4"
                               tripod thread (revision 1 assumed one) - it hangs by its own
                               monitor clip on the 6 mm ledge at the top. Base has a pocket for
                               ballast (M3 nuts / coins) so the stand does not tip.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from manifold3d import Manifold

OUT = Path(__file__).resolve().parent
SEG = 48

# ------------------------------ parameters ------------------------------------
CAM_PITCH_DEG = 25.0        # downward tilt of the road camera
CAM_LENS_HEIGHT = 60.0      # lens centre above the mount face (plus TurtleBot top plate height)
TOF_CENTER_HEIGHT = 45.0    # VL53L1X optical centre above the mount face
WEBCAM_COLUMN = 120.0       # stand height (C270 clip ledge)
M2, M25, M3 = 2.3, 2.8, 3.4  # clearance hole diameters


def box(x, y, z, center_xy=True):
    m = Manifold.cube([x, y, z])
    return m.translate([-x / 2, -y / 2, 0]) if center_xy else m


def cyl(d, h, z0=0.0):
    return Manifold.cylinder(h, d / 2, d / 2, SEG).translate([0, 0, z0])


def slot(d, length, h, z0=-1.0):
    """Slot along X."""
    return (box(length, d, h).translate([0, 0, z0]) + cyl(d, h, z0).translate([-length / 2, 0, 0]) +
            cyl(d, h, z0).translate([length / 2, 0, 0]))


def base_with_slots(w, d, t, slot_len=16.0, spacing=None):
    spacing = spacing or w - 14
    b = box(w, d, t)
    for sx in (-spacing / 2, spacing / 2):
        b = b - slot(M3, slot_len - M3, t + 2).rotate([0, 0, 90]).translate([sx, 0, 0])
    return b


# ------------------------------ parts ----------------------------------------
def pi_camera_mount() -> Manifold:
    t = 4.0
    base = base_with_slots(44, 34, t)
    # tilted camera plate: 30 x 30 x 3, rotated about X so the lens looks 25 deg down
    pw, ph, pt = 30.0, 30.0, 3.0
    plate = box(pw, pt, ph)
    # Pi Camera V2 holes: 21 mm apart horizontally, 12.5 mm vertically; board 25 x 24 mm,
    # upper hole pair 2 mm below the board top edge.
    top = ph - 3.0
    for hx in (-10.5, 10.5):
        for hz in (top - 2.0, top - 14.5):
            boss = cyl(5.0, 3.0).rotate([90, 0, 0]).translate([hx, -pt / 2, hz])  # standoff bosses (front)
            plate = plate + boss
            plate = plate - cyl(M2, 12).rotate([90, 0, 0]).translate([hx, 6, hz])
    # ribbon cable relief at the bottom
    plate = plate - box(17, 10, 4).translate([0, 0, -1])
    stem_h = CAM_LENS_HEIGHT - ph / 2
    stem = box(pw, 6.0, stem_h)
    plate = plate.rotate([CAM_PITCH_DEG, 0, 0])  # front face (-Y, lens side) now points 25 deg below horizontal.translate([0, 0, stem_h - 2])
    gusset = (Manifold.cube([3, 14, 14]).rotate([45, 0, 0]).translate([-1.5, 0, t - 10]) ^ box(3, 30, 40))
    body = base + stem.translate([0, 4, 0]) + plate.translate([0, 4, 0])
    for gx in (-12, 12):
        body = body + gusset.translate([gx, 7, 0])
    return body


def tof_bracket() -> Manifold:
    t = 4.0
    base = base_with_slots(40, 24, t)
    up_h = TOF_CENTER_HEIGHT + 12
    upright = box(32, 3.5, up_h).translate([0, 10, 0])
    # two horizontal M2.5 slots, centred on the sensor height, 13-25 mm hole spacing
    for side in (-1, 1):
        s = slot(M25, 6.0, 12).rotate([90, 0, 0]).translate([side * 9.5, 16, TOF_CENTER_HEIGHT])
        upright = upright - s
    # aperture behind the sensor so the emitter/receiver are never shadowed on thin boards
    upright = upright - box(8, 12, 6).translate([0, 10, TOF_CENTER_HEIGHT - 3])
    rib = box(3, 10, 18).translate([0, 4, 0])
    return base + upright + rib.translate([-12, 0, 0]) + rib.translate([12, 0, 0])


def webcam_stand() -> Manifold:
    base = box(90, 90, 8) - box(60, 60, 5).translate([0, 0, -1])     # ballast pocket, open from below
    column = box(30, 16, WEBCAM_COLUMN).translate([0, 0, 8])
    ledge = box(40, 30, 6).translate([0, 7, 8 + WEBCAM_COLUMN])          # C270 clip hooks over this
    lip = box(40, 3, 10).translate([0, -6.5, 8 + WEBCAM_COLUMN - 4])     # keeps the clip from sliding off
    cable = cyl(6, 40).rotate([90, 0, 0]).translate([0, 20, 20])        # USB cable pass-through
    fillet = Manifold.cube([30, 20, 20]).translate([-15, 8, 8]) - cyl(40, 30).rotate([0, 90, 0]).translate([-16, 28, 28])
    return base + column + ledge + lip + fillet - cable


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
    fig = plt.figure(figsize=(12, 4.6))
    for i, (name, tm) in enumerate(meshes.items()):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        tris = tm.vertices[tm.faces]
        light = np.clip(np.abs(tm.face_normals @ np.array([0.4, -0.6, 0.7])), 0.25, 1)
        ax.add_collection3d(Poly3DCollection(tris, facecolors=plt.cm.Greys(0.35 + 0.5 * light), edgecolor="none"))
        lo, hi = tm.bounds
        span = (hi - lo).max() / 2
        c = (hi + lo) / 2
        ax.set_xlim(c[0] - span, c[0] + span)
        ax.set_ylim(c[1] - span, c[1] + span)
        ax.set_zlim(c[2] - span, c[2] + span)
        ax.view_init(22, -60)
        ax.set_title(f"{name}\n{(hi - lo).round(1)} mm", fontsize=9)
        ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> None:
    parts = {"pi_camera_v2_mount_25deg": pi_camera_mount(), "vl53l1x_tof_bumper_bracket": tof_bracket(),
             "cockpit_webcam_stand": webcam_stand()}
    meshes = {}
    for name, m in parts.items():
        tm = to_trimesh(m)
        assert tm.is_watertight, name
        tm.export(OUT / f"{name}.stl")
        meshes[name] = tm
        print(f"{name}.stl  {len(tm.faces)} triangles, {tm.volume / 1000:.1f} cm^3, bbox {tm.extents.round(1)} mm")
    preview(meshes, OUT / "parts_preview.png")


if __name__ == "__main__":
    main()
