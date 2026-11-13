# Architecture

## System overview

```mermaid
flowchart LR
  subgraph N1["Node 1 · Cockpit DMS (RPi 4B, IoT kit)"]
    CAM1[C270 webcam<br/>640×480 MJPG 30 fps] --> FL[MediaPipe<br/>FaceLandmarker]
    FL --> AN[EAR · MAR · yaw<br/>C_driver · EMA α=0.35]
    AN --> TX[Heartbeat sender<br/>50 Hz UDP]
    CAL[Calibration /<br/>guided protocol] --> AN
    RXS[vehicle_state rx] --> IND1[Buzzer · RGB LED · OLED]
    LFSM[local FSM<br/>monitor-only fallback] --> IND1
  end
  subgraph N2["Node 2 · TurtleBot 3 Burger"]
    CAM2[Pi Camera V2<br/>320×240 @30] --> LANE[Bird's-eye warp<br/>yellow/white windows<br/>joint fit → e, ψ, κ, C_lane]
    TOF[VL53L1X 30 Hz] --> FSM
    RX[driver rx<br/>+ link age] --> FSM[Safety FSM<br/>6 states]
    LANE --> FSM
    FSM --> VP[Velocity profiler<br/>A_ACCEL / A_DECEL]
    LANE --> LKC[Lane keeping<br/>FF + PID, v-scheduled]
    VP --> MOT[Motor thread 20 Hz<br/>+ 250 ms watchdog]
    LKC --> MOT
    MOT --> ROS[/cmd_vel → turtlebot3_bringup → OpenCR/]
    FSM --> REC[(Audit recorder<br/>SQLite WAL + CSV)]
    FSM --> IND2[RGB LED · buzzer · E-STOP/RESET buttons]
  end
  TX -- "udp/5005 driver 50 Hz" --> RX
  FSM -- "udp/5006 state 20 Hz" --> RXS
  N1 -. "telemetry + JPEG" .-> DASH
  N2 -. "telemetry + JPEG" .-> DASH
  DASH[Dashboard FastAPI<br/>WebSocket 10 Hz] -- "commands udp/5020-5021" --> N2
  DASH -- commands --> N1
  BROWSER[Operator browser<br/>laptop / phone] <--> DASH
```

The dashboard is **outside** the safety loop: losing it never stops the robot unexpectedly,
and every stop function also exists in hardware (buttons) and in the FSM itself.

## Supervisory FSM

```mermaid
stateDiagram-v2
  [*] --> STANDBY
  STANDBY --> NORMAL_CRUISE: START and ready
  NORMAL_CRUISE --> WARNING: closed ≥1.2 s · yawn ≥0.8 s · PERCLOS ≥25 % · look-away ≥2 s
  WARNING --> SPEED_REDUCTION: closed ≥2.5 s · 3 yawns/min · C_driver low ≥0.5 s
  NORMAL_CRUISE --> SPEED_REDUCTION
  SPEED_REDUCTION --> CONTROLLED_STOP: closed ≥4 s · driver lost ≥5 s
  WARNING --> NORMAL_CRUISE: alert ≥1.5 s
  SPEED_REDUCTION --> NORMAL_CRUISE: alert ≥1.5 s
  CONTROLLED_STOP --> STANDBY: RESET / ACK (driver alert ≥1.5 s)
  NORMAL_CRUISE --> EMERGENCY_STOP: obstacle <20 cm · heartbeat >150 ms · lane lost >0.3 s · ToF fault · E-STOP
  WARNING --> EMERGENCY_STOP
  SPEED_REDUCTION --> EMERGENCY_STOP
  CONTROLLED_STOP --> EMERGENCY_STOP
  EMERGENCY_STOP --> NORMAL_CRUISE: obstacle / link auto-clear (1 s)
  EMERGENCY_STOP --> STANDBY: RESET (latched faults)
```

Priority every tick: **commands → faults (EMERGENCY) → latched CONTROLLED_STOP → driver level**.

## Timing budget (Node 2, 30 Hz loop)

| Step | Typical (Pi 4) | Notes |
|---|---|---|
| frame grab | 0 ms (thread) | latest frame only, buffer size 1 |
| bird's-eye warp + segmentation + windows + fit | 4–8 ms | 240×180 BEV |
| FSM + profiler + PID | < 0.2 ms | |
| logging | < 0.05 ms | queue only |
| **total** | **< 10 ms** | 100 ms budget |

Node 1: FaceLandmarker on a Pi 4 at 640×480 ≈ 35–50 ms per frame → 20–28 fps (within the 100 ms
budget; reduce `driver.camera_width` to 480 if needed).

## Source layout

```
adowndriver/
  config.py                 all parameters (dataclasses) + YAML loading + runtime-tunable list
  common/protocol.py        UDP packets, endpoints   common/filters.py  EMA, median, persistence, PERCLOS window
  driver_monitor/           Node 1: analyzer.py (EAR/MAR/pose/C_driver/calibration), backend.py (MediaPipe), node.py
  vehicle/                  Node 2: lane.py, camera_model.py, control.py, fsm.py, core.py (one control step), io.py, node.py
  hardware/                 peripherals.py (LED/buzzer/buttons/OLED with mock fallback), selftest.py
  audit/                    recorder.py (SQLite/CSV), metrics.py (M1–M5)
  sim/                      world.py (track, robot, ToF, renderer), synthetic_driver.py, scenarios.py, runner.py
  dashboard/                server.py (FastAPI), static/ (HTML/CSS/JS, no CDN)
tests/                      53 pytest tests (FSM, lane, control, analyzer, protocol/audit, SIL)
tools/calibrate_lane.py     homography + colour calibration
cad_models/generate_stl.py  parametric 3D-printed parts
deploy/                     setup scripts + systemd units
```

## Team ownership

| Module | Owner |
|---|---|
| Architecture, `vehicle/fsm.py`, integration | Li Vladislav |
| `driver_monitor/analyzer.py`, `backend.py` | Li Andrey |
| `vehicle/lane.py`, `camera_model.py`, `control.py`, `tools/calibrate_lane.py` | Pak Nikita |
| `common/protocol.py`, `vehicle/io.py`, `deploy/` | Vadim Lifrov |
| `dashboard/` | Hsu Shwe Sin Htun |
| `hardware/peripherals.py`, wiring | Egamberdiev Saidumar |
| `audit/recorder.py` | Husan Tulaev |
| `sim/`, `audit/metrics.py`, `tests/` | Dilshod Obidov |
| `cad_models/` | Fazliddin Igamberdiev |
| Confidence matrix & watchdog (`fsm.py` faults, motor watchdog) | Arsen Baktybek uulu |
| `docs/`, demo video | Sevinchxon Khoshimova |
