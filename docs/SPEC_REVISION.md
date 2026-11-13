# Specification revision 2 — what changed and why

Revision 1 = *VIP_TeamMatigoat_AdownDriver_MIDTERM.docx* (15 Oct 2026). This document lists every
issue found while implementing it on the real hardware inventory, and the fix now in the code.
Paste the relevant parts into the final report (section "Design iteration").

## A. Safety-logic defects in Algorithm 1

| # | Rev. 1 behaviour | Problem | Rev. 2 (implemented in `vehicle/fsm.py`) |
|---|---|---|---|
| A1 | `t_drowsy += dt` while EAR < TH, reset only after 1.5 s of open eyes | Blinks **accumulate**: a driver blinking 0.3 s every 1.2 s reaches T_WARN after 4 blinks and SPEED_REDUCTION after 8 → contradicts the 0 % blink false-alarm target | `t_closed` measures **continuous** closure; it resets once the eyes are open ≥ 0.2 s (tolerance against one noisy frame). Blinks < 0.4 s can never reach 1.2 s. Recovery timer ignores blinks. |
| A2 | `mar_filtered > TH_MAR` → WARNING immediately | Talking / laughing produces single-frame MAR spikes → false warnings | Yawn = MAR > 0.60 for ≥ 0.8 s. `yawn_count` (declared but unused in rev. 1) now works: 3 yawns / 60 s → SPEED_REDUCTION |
| A3 | `c_lane < 0.65` on one frame → EMERGENCY | Single-frame dropouts (glare, motion blur) cause random emergency stops | 0.3 s debounce (≈ 5 cm of travel at 0.16 m/s), then EMERGENCY_STOP (latched) |
| A4 | `c_driver < 0.65` → SPEED_REDUCTION forever | No debounce; driver who leaves the seat lets the robot drive on at 50 % indefinitely | 0.5 s debounce → SPEED_REDUCTION; after 5 s → CONTROLLED_STOP (`DRIVER_NOT_MONITORABLE`) |
| A5 | No exit from CONTROLLED_STOP / EMERGENCY | Robot can only be restarted by rebooting | Explicit rules: obstacle and link faults auto-clear after 1 s of healthy signal; lane loss, ToF fault, manual E-STOP and CONTROLLED_STOP are **latched** and need RESET (dashboard, Node 2 button B, or driver ACK on Node 1 once alert ≥ 1.5 s) |
| A6 | Robot starts driving on power-up | Unsafe in a lab | New **STANDBY** state; START is accepted only when the readiness checklist is empty (driver visible, lane visible, path clear, link up) |
| A7 | `heartbeat_ok` undefined | Metric M5 (≤ 200 ms) not achievable without a defined timeout | 50 Hz heartbeat, timeout **150 ms**; measured brake latency is logged per event (`BRAKE` events) |
| A8 | EMERGENCY v = 0, CONTROLLED_STOP "ramp", but the ramp formula used the state-entry speed only | v jumps between states (0.16 → 0.08 instantly) | One velocity profiler with slew limits (A_ACCEL 0.10, A_DECEL 0.06 m/s²) for every transition; only EMERGENCY bypasses it |
| A9 | Obstacle handling binary at 20 cm | Hard stop from full speed every time | Slow-down zone 45 → 20 cm (linear speed cap), emergency below 20 cm |
| A10 | Looking to the side lowers EAR → false closure | Mentioned as a problem in the spec but not handled | EAR ignored while \|yaw\| > 35°; looking away ≥ 2 s → WARNING (`DRIVER_DISTRACTED`) |
| A11 | Fixed TH_EAR = 0.21 for every person | Open-eye EAR varies 0.24–0.36 between people → misses or false alarms | 5-s per-driver calibration: TH_EAR = 0.72 × open-eye EAR (clamped 0.15–0.28), saved as a driver profile |
| A12 | — | Long-term fatigue with short closures is invisible | PERCLOS (60 s window) ≥ 25 % → WARNING |

## B. Perception & control defects

| # | Rev. 1 | Problem | Rev. 2 |
|---|---|---|---|
| B1 | Lane centre = mean x of *all* white/yellow pixels | With one visible line (every curve, any drift) the "centre" is that line → robot steers into it | Separate yellow (left) and white (right) tracking with sliding windows; joint parallel polynomial fit; one visible line is offset by the measured lane width |
| B2 | `e_y_cm = e_px × 0.045` (unexplained constant) | Not metric; wrong after any mount change | Homography from a pinhole model of the 25° mount (or 4-point calibration with `tools/calibrate_lane.py`): the bird's-eye image has a known 400 px/m scale |
| B3 | PID only, on pixel error | Steady-state error in curves (radius 0.65 m needs ω = 0.25 rad/s) | Curvature feed-forward + PID on axle offset + heading term, gain-scheduled by v so the path is identical while ramping down (centred stop, no spinning at v = 0) |
| B4 | No glare handling | LED-lamp reflections on the mat look like white tape | Adaptive thresholds + morphological blob rejection (scenario S13) |
| B5 | `mp.solutions.face_mesh` with silent fallback to constant fake values (EAR 0.28) | `mp.solutions` was **removed** from current MediaPipe wheels → the midterm detector produced fake data without warning | MediaPipe Tasks **FaceLandmarker** (478 landmarks + head-pose matrix); legacy API only as fallback; a missing backend is a hard error |
| B6 | C_driver = 0.95 constant when a face is found | Not a confidence | Product of face size, head pose and exposure factors |
| B7 | SQLite connection opened/closed + CSV re-opened every tick on the control thread | Several ms per tick on a microSD card | Background writer thread, WAL, batched commits; control loop only enqueues |
| B8 | Motor commands only when the loop runs | A crashed loop leaves the last /cmd_vel active | Motor thread republishes at 20 Hz and sends zero if the command is older than 250 ms (software watchdog); SIGINT/SIGTERM zero the motors |

## C. Hardware / documentation corrections

* HC-SR04 is not in the inventory → removed; VL53L1X short mode (≤ 1.3 m, 30 Hz, best in ambient light).
* "468 landmarks" → FaceLandmarker returns 478 (468 + 10 iris).
* The Logitech C270 has **no tripod thread**; the printed stand now has a clip ledge.
* The placeholder STL files were 12-triangle boxes; replaced by parametric parts (`cad_models/generate_stl.py`).
* CAD rev 3: camera mount plate was generated at the base (lost `translate`) → fixed; a level VL53L1X sees the floor at ≈ 0.19 m (27° cone) → bracket tilted 10° up; webcam stand pocket was a 60 mm unsupported bridge → opened at the top; added the Node 2 status pod and Node 1 cockpit panel enclosures; all parts checked against the M-124 printer (220 × 220 × 250 mm, no supports).
* Active buzzer must be driven through the 2N2222/S8050 (GPIO max 16 mA) — wiring in `docs/HARDWARE_WIRING.md`.
* Buzzer/LED on **both** nodes: the cockpit buzzer wakes the driver; the robot LED shows the state to observers.

## D. Measurability of the five metrics

| Metric | How it is measured now |
|---|---|
| M1 F1 ≥ 0.90 | Guided protocol (`config/protocols/f1_standard.yaml`) shows cues to the volunteer and writes the ground-truth label into every logged tick; episode-level matching with 2.5 s grace (`audit/metrics.py`) |
| M2 0 % blink false alarms | Count of driver-caused SPEED_REDUCTION / CONTROLLED_STOP episodes outside labelled drowsy/yawn episodes |
| M3 ≤ 100 ms per frame | p95 of per-frame processing time on each node (both logged per tick) + RTT via packet echo |
| M4 ±5 cm during stop | max \|e_y at axle\| while CONTROLLED_STOP and v > 0 (camera estimate); verify physically with a tape mark — SIL uses ground truth |
| M5 ≤ 200 ms braking | `BRAKE` event: link age at trigger (heartbeat) or ToF sample age + one ToF period (obstacle) + processing |

## E. Frozen parameters (rev. 2)

`TH_MAR 0.60 · TH_CONF_MIN 0.65 · T_WARN 1.2 s · T_SLOW 2.5 s · T_STOP 4.0 s · T_RECOVER 1.5 s · T_BLINK_MAX 0.4 s ·
T_YAWN_MIN 0.8 s · 3 yawns/60 s · PERCLOS 25 % · yaw 35° / 2 s · C_driver debounce 0.5 s / lost 5 s · lane-loss 0.3 s ·
heartbeat 150 ms · D_SAFE 20 cm · slow zone 45 cm · V 0.16 / 0.08 m/s · A_DECEL 0.06 m/s²` — see `adowndriver/config.py`.
