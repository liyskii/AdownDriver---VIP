# 📐 3D CAD Models & Optical Mounting Specifications (`cad_models/`)

> **CAD Software:** Autodesk Fusion 360 (Inha University VIP Team License)  
> **Slicer & Printer:** UltiMaker Digital Factory -> **MakerBot Sketch Large** (SGCS Office, Room M-124)  
> **Material:** 1.75 mm PLA Filament (Matte Black & White)

---

## 1. Included 3D Printable Models (`.stl`)

| File Name | Target Node | Dimensions (mm) | Mounting Interface | Optical / Mechanical Purpose |
| :--- | :--- | :---: | :--- | :--- |
| **`pi_camera_v2_mount_25deg.stl`** | **Node 2 (TurtleBot 3)** | `36 x 28 x 42 mm` | 4x M3 Brass Standoffs (Top Waffle Plate) | Locks the Raspberry Pi Camera V2 at a fixed **25.0° downward pitch angle** so the OpenCV bird's-eye homography matrix (`M_warp`) remains invariant under chassis acceleration. |
| **`vl53l1x_tof_bumper_bracket.stl`** | **Node 2 (TurtleBot 3)** | `30 x 20 x 25 mm` | 2x M2.5 Screws (Front Lower Bumper) | Aligns the **VL53L1X Time-of-Flight laser sensor** parallel to the track surface (`height = 45 mm`, `pitch = 0.0°`) to prevent false floor reflections while guaranteeing `< 20 cm` obstacle detection. |
| **`cockpit_webcam_stand.stl`** | **Node 1 (Cockpit DMS)** | `65 x 65 x 110 mm` | Weighted Desktop Base + 1/4" Tripod Slot | Holds the **Logitech C270 HD Webcam** at a steady eye-level framing distance (`45-55 cm` from volunteer) for stable 468-point MediaPipe facial landmark tracking. |

---

## 2. MakerBot Sketch Large Slicing Profile (Room M-124)
* **Layer Height:** `0.20 mm`
* **Infill Density:** `20%` (Gyroid / Grid pattern for vibration damping)
* **Wall Line Count:** `3 shells` (`1.2 mm` wall thickness)
* **Support Structure:** Enabled (` overhang > 50° `)
* **Build Plate Adhesion:** Raft / Brim enabled
