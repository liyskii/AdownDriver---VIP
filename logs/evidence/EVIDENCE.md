# AdownDriver - evidence pack

Generated 2026-11-13 20:45:56 by `tools/evidence_pack.py` (Team #Matigoat).

## 1. Environment

| Item | Value |
|---|---|
| OS | Linux 6.18.44-fc-v445 (x86_64) |
| Python | 3.13.18 |
| numpy | 2.5.3 |
| cv2 | 5.0.0 |
| fastapi | 0.162.5 |
| yaml | 6.0.3 |
| mediapipe | 0.10.32 |
| rclpy | not installed on this machine |
| git commit | n/a (exported copy) |

## 2. Unit tests - 59/59 passed (25.2 s)

| Test file | Tests | Failed |
|---|---|---|
| test_analyzer.py | 5 | 0 |
| test_control.py | 3 | 0 |
| test_fsm.py | 27 | 0 |
| test_lane.py | 10 | 0 |
| test_launcher.py | 6 | 0 |
| test_protocol_audit.py | 4 | 0 |
| test_sil.py | 4 | 0 |

## 3. Software-in-the-loop validation - 16/16 scenarios passed

| Metric | Value | Target | Result |
|---|---|---|---|
| f1 | 1.0 | >= 0.9 | PASS |
| blink_false_alarms | 0 | == 0 | PASS |
| lag_p95_ms | 4.5 | <= 100.0 | PASS |
| lane_dev_stop_cm | 1.15 | <= 5.0 | PASS |
| brake_latency_ms | 177.5 | <= 200.0 | PASS |

| ID | Scenario | Final state | Max abs e (cm) | Result |
|---|---|---|---|---|
| S01 | Baseline cruise, natural blinking | NORMAL_CRUISE | 3.27 | PASS |
| S02 | Rapid blinking (300 ms every ~2 s) | NORMAL_CRUISE | 3.27 | PASS |
| S03 | Talking / reading aloud | NORMAL_CRUISE | 3.27 | PASS |
| S04 | Single yawn | NORMAL_CRUISE | 3.27 | PASS |
| S05 | Repeated yawning (3 in 30 s) | NORMAL_CRUISE | 3.27 | PASS |
| S06 | Microsleep 1.8 s | NORMAL_CRUISE | 3.27 | PASS |
| S07 | Microsleep 3.2 s | NORMAL_CRUISE | 3.16 | PASS |
| S08 | Sustained closure -> graduated safe stop, then reset | NORMAL_CRUISE | 3.03 | PASS |
| S09 | Looking away 3 s (distraction) | NORMAL_CRUISE | 3.27 | PASS |
| S10 | Driver camera occluded 2 s | NORMAL_CRUISE | 3.65 | PASS |
| S11 | Driver leaves the seat 7 s | CONTROLLED_STOP | 0.88 | PASS |
| S12 | Dim lighting (lamp at 20 %) | NORMAL_CRUISE | 3.44 | PASS |
| S13 | Glare (lamp reflection on mat + bright face) | NORMAL_CRUISE | 5.75 | PASS |
| S14 | Static obstacle on the lane | NORMAL_CRUISE | 3.28 | PASS |
| S15 | Wi-Fi / heartbeat loss 1 s | NORMAL_CRUISE | 2.9 | PASS |
| S16 | Lane markings lost (tape gap) | EMERGENCY_STOP | 0.28 | PASS |

## 4. Traceability (SHA-256, first 16 hex)

| File | SHA-256 |
|---|---|
| `config/default.yaml` | `5ecccb2b6b2fe170` |
| `adowndriver/__init__.py` | `d9d7db361e7bd057` |
| `adowndriver/__main__.py` | `39c74bdb6e4f53fe` |
| `adowndriver/audit/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/audit/metrics.py` | `94cbcffb2aea98b8` |
| `adowndriver/audit/recorder.py` | `cb943cb2352e94cb` |
| `adowndriver/common/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/common/filters.py` | `308b519d5439460d` |
| `adowndriver/common/protocol.py` | `b6e47a7c98abb0e7` |
| `adowndriver/config.py` | `24ee6894fd834af0` |
| `adowndriver/dashboard/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/dashboard/server.py` | `33436007619e9145` |
| `adowndriver/driver_monitor/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/driver_monitor/analyzer.py` | `fc49959a469e672e` |
| `adowndriver/driver_monitor/backend.py` | `b1f039e1d64b5a1c` |
| `adowndriver/driver_monitor/node.py` | `7a3bdeb025808190` |
| `adowndriver/hardware/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/hardware/peripherals.py` | `cecaedca64cd45c4` |
| `adowndriver/hardware/selftest.py` | `fd141a8a6c5a66cf` |
| `adowndriver/launcher/__init__.py` | `2ff9c6786c4066bd` |
| `adowndriver/launcher/catalog.py` | `2d3e0586352a82c9` |
| `adowndriver/launcher/jobs.py` | `0c77b98df6736040` |
| `adowndriver/launcher/menu.py` | `14afc8fef39495f5` |
| `adowndriver/sim/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/sim/runner.py` | `5ddb0c3814f1ea23` |
| `adowndriver/sim/scenarios.py` | `41fd19c88afcffe7` |
| `adowndriver/sim/synthetic_driver.py` | `a654e2f976d27677` |
| `adowndriver/sim/world.py` | `8ce6f53b719cc512` |
| `adowndriver/vehicle/__init__.py` | `01ba4719c80b6fe9` |
| `adowndriver/vehicle/camera_model.py` | `5b875302f5ea044a` |
| `adowndriver/vehicle/control.py` | `bd7b381fb3413177` |
| `adowndriver/vehicle/core.py` | `65d8994e661ea3f6` |
| `adowndriver/vehicle/fsm.py` | `f3bb12f855e32b19` |
| `adowndriver/vehicle/io.py` | `07c6389e5e392c04` |
| `adowndriver/vehicle/lane.py` | `3115190fa7a4f123` |
| `adowndriver/vehicle/node.py` | `7c3b06130342865c` |

## 5. Not covered by this pack

Arena trials on the real TurtleBot, the lane homography calibration and the Node 1 frame rate on the Raspberry Pi are measured on hardware with the Trials tab (`logs/runs/`) and are reported separately.

_Built in 27 s._
