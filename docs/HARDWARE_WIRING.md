# Hardware & wiring

Only parts from the team inventory are used. Pin numbers are **BCM GPIO** numbers
(physical header pin in brackets). All pins are configurable in `config/default.yaml → hardware.pins`.

## Bill of materials (as purchased)

| Part | Qty | Used for |
|---|---|---|
| TurtleBot 3 Burger (RPi 4B + OpenCR 1.0 + 2× XL430) | 1 | Node 2 – mobile base |
| Raspberry Pi 4B (IoT kit) + case with fan + heatsinks | 1 | Node 1 – cockpit DMS station |
| Logitech C270 HD USB webcam | 1 | Driver camera (Node 1) |
| Raspberry Pi Camera Module V2 (8 MP) + 30 cm CSI ribbon | 1 | Road camera (Node 2) |
| VL53L1X ToF laser distance sensor | 1 | Forward obstacle range (Node 2, I²C) |
| 0.96" SSD1306 OLED (I²C) | 1 | Cockpit status display (Node 1) |
| Active buzzer + NPN 2N2222 / S8050 | 1 + 1 per node | Acoustic alerts |
| RGB LED (common cathode) + 220 Ω resistors | 1 per node | State indication |
| Push buttons | 2 per node | Node 1: CALIBRATE / ACK – Node 2: E-STOP / RESET |
| SanDisk Extreme 64 GB microSDXC ×3 | 3 | Node 1, Node 2, spare image |
| Official 15 W USB-C PSU (5.1 V / 3 A) | 1 | Node 1 |
| Travel Wi-Fi router | 1 | Dedicated local network (docs/NETWORK.md) |
| Matte black EVA mats + white/yellow non-reflective tape | — | Test track |
| LED desk lamp (dimmable) | 1 | Lighting scenarios S12/S13 |
| PLA 1.75 mm, M2 / M2.5 / M3 screws & standoffs | — | 3D-printed mounts and enclosures (cad_models/): Node 2 status pod (buttons, LED, buzzer), Node 1 cockpit panel (OLED, buttons, LED, buzzer) |

> The HC-SR04 ultrasonic sensor mentioned in revision 1 of the spec is **not** in the inventory and is not needed — the VL53L1X covers 4–130 cm in short mode at 30 Hz.

## Node 1 – cockpit station

```
 Raspberry Pi 4B (Node 1)                     breadboard
 ───────────────────────                      ──────────
 3V3  (pin 1) ─────────────── OLED VCC
 GPIO2/SDA (pin 3) ────────── OLED SDA
 GPIO3/SCL (pin 5) ────────── OLED SCL
 GND  (pin 9) ─────────────── OLED GND

 GPIO17 (pin 11) ──[220Ω]──── LED R ┐
 GPIO27 (pin 13) ──[220Ω]──── LED G ├─ RGB LED, common cathode ── GND (pin 14)
 GPIO22 (pin 15) ──[220Ω]──── LED B ┘

 5V   (pin 2) ─────────────── buzzer (+)
                              buzzer (−) ── C  ┐
 GPIO18 (pin 12) ─[220Ω]─[220Ω]─────────── B  ├ 2N2222 / S8050 (flat side towards you: E B C for 2N2222,
 GND  (pin 6) ──────────────────────────── E  ┘                  E B C for S8050 TO-92 as well)

 GPIO23 (pin 16) ── button A (CALIBRATE) ── GND (pin 20)     internal pull-ups, active low
 GPIO24 (pin 18) ── button B (ACK/RESET) ── GND (pin 20)
 USB ──────────────── Logitech C270
```

* **Why the transistor?** An active buzzer draws ~25–35 mA; a Pi GPIO is rated for 16 mA. Two 220 Ω in series (440 Ω) give ≈6 mA base current → the transistor saturates and switches the buzzer from 5 V.
* Button A held 2 s = calibrate (also possible from the dashboard). Button B = driver ACK: after a CONTROLLED_STOP, once the driver has been alert for 1.5 s, ACK returns the robot to STANDBY.

## Node 2 – TurtleBot 3

```
 TurtleBot RPi 4B header                      
 3V3 (pin 1) ──── VL53L1X VIN        GPIO17/27/22 ─[220Ω]─ RGB LED (state, visible from outside)
 SDA (pin 3) ──── VL53L1X SDA        GPIO18 ─[2×220Ω]─ NPN ─ buzzer (same circuit as Node 1)
 SCL (pin 5) ──── VL53L1X SCL        GPIO23 ── button A = E-STOP  (latched, needs RESET)
 GND (pin 9) ──── VL53L1X GND        GPIO24 ── button B = RESET
 CSI ──────────── Pi Camera V2 (30 cm ribbon, contacts towards the HDMI ports)
 USB ──────────── OpenCR 1.0  (motors)   USB ── LDS (unused by AdownDriver)
```

* The VL53L1X sits on the printed bracket at the front of the robot, optical centre ≈ 44 mm above the mount face, **tilted 10° up**: with the 27° field of view the floor enters the cone only at ≈ 0.7 m instead of ≈ 0.19 m (level mount), so it cannot trigger the 20 cm EMERGENCY_STOP. Details: `cad_models/CAD_SPECIFICATIONS.md`.
* The Pi Camera V2 sits on the printed **25° down-tilt** mount on the top plate, lens ≈ 19.5 cm above the mat (measure and set `camera.height_m`).
* The TurtleBot Pi is powered from the OpenCR 5 V rail (battery). Use the aluminium heatsinks — the Pi 4 throttles at 80 °C, which would break the 100 ms latency budget. The dashboard shows the CPU temperature.

## Power

| Node | Supply | Note |
|---|---|---|
| Node 1 | Official 15 W USB-C PSU 5.1 V/3 A | case fan on 5 V |
| Node 2 | TurtleBot LiPo 11.1 V → OpenCR → 5 V | charge before every arena session; ~ 2 h of driving |
| Router | USB from a power bank or laptop | keep it ≤ 3 m from the track |

## Self-test

```bash
python -m adowndriver check --role node1   # camera fps, MediaPipe model, OLED/GPIO libraries, I²C
python -m adowndriver check --role node2   # road camera, ToF reading, rclpy, ports
i2cdetect -y 1                             # expect 0x29 (VL53L1X) on Node 2, 0x3C (OLED) on Node 1
```
