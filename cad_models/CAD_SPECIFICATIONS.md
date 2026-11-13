# 3D-printed parts (`cad_models/`)

![parts](parts_preview.png)

The STL files are generated from **`generate_stl.py`** (parametric, watertight meshes).
Edit the parameters at the top and run `python3 cad_models/generate_stl.py`
(`pip install manifold3d trimesh`). Import the STL into Fusion 360 for further edits.

| File | Node | Size (mm) | Fastening | Purpose |
|---|---|---|---|---|
| `pi_camera_v2_mount_25deg.stl` | Node 2 top plate | 44 × 34 × 46 | 2 × M3 in 16 mm slots (+ M3 standoffs to set height) | Holds the Pi Camera V2 (4 × M2, 21 × 12.5 mm pattern, on 3 mm bosses for rear-component clearance) at a fixed **25° down-tilt**. Ribbon relief at the bottom. The homography in `camera_model.py` assumes this angle. |
| `vl53l1x_tof_bumper_bracket.stl` | Node 2 front | 40 × 24 × 57 | base: 2 × M3 slots; sensor: 2 × M2.5 horizontal slots (13–25 mm hole spacing) | VL53L1X optical centre **45 mm** above the mount face, pitch 0°, so the mat is outside the 27° cone at < 1.3 m. |
| `cockpit_webcam_stand.stl` | Node 1 desk | 90 × 90 × 134 | — | C270 hangs on the 6 mm ledge with its own clip (the C270 has no tripod thread). Fill the base pocket with M3 nuts/coins as ballast. USB cable pass-through. |

## Print settings (MakerBot Sketch Large, room M-124)

* PLA 1.75 mm black (matte black reduces stray reflections into the cameras), 0.20 mm layers, 3 walls, 20 % gyroid infill.
* Camera mount: print standing on its base; supports only under the tilted plate (> 50° overhang).
* ToF bracket and stand: no supports needed.
* Holes are modelled with clearance (M2 → 2.3, M2.5 → 2.8, M3 → 3.4 mm); drill out if your printer over-extrudes.

## After mounting

1. Measure the lens height above the mat and the actual tilt with a phone inclinometer → `camera.height_m`, `camera.pitch_deg`.
2. Run `tools/calibrate_lane.py` (4-point homography) – this compensates any mounting error.
