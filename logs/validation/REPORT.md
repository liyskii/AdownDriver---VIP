# AdownDriver - SIL validation report

Generated 2026-11-13 20:45:18 - **16/16 scenarios passed**

## Acceptance metrics

| Metric | Value | Target | Result |
|---|---|---|---|
| f1 | 1.0 | >= 0.9 | PASS |
| blink_false_alarms | 0 | == 0 | PASS |
| lag_p95_ms | 4.5 | <= 100.0 | PASS |
| lane_dev_stop_cm | 1.15 | <= 5.0 | PASS |
| brake_latency_ms | 177.5 | <= 200.0 | PASS |

## Scenarios

| ID | Scenario | States | Lane dev. in stop (cm) | Result |
|---|---|---|---|---|
| S01 | Baseline cruise, natural blinking | STANDBY → NORMAL_CRUISE | - | PASS |
| S02 | Rapid blinking (300 ms every ~2 s) | STANDBY → NORMAL_CRUISE | - | PASS |
| S03 | Talking / reading aloud | STANDBY → NORMAL_CRUISE | - | PASS |
| S04 | Single yawn | STANDBY → NORMAL_CRUISE → WARNING → NORMAL_CRUISE | - | PASS |
| S05 | Repeated yawning (3 in 30 s) | STANDBY → NORMAL_CRUISE → WARNING → NORMAL_CRUISE → WARNING → NORMAL_CRUISE → SPEED_REDUCTION → NORMAL_CRUISE | - | PASS |
| S06 | Microsleep 1.8 s | STANDBY → NORMAL_CRUISE → WARNING → NORMAL_CRUISE | - | PASS |
| S07 | Microsleep 3.2 s | STANDBY → NORMAL_CRUISE → WARNING → SPEED_REDUCTION → NORMAL_CRUISE | - | PASS |
| S08 | Sustained closure -> graduated safe stop, then reset | STANDBY → NORMAL_CRUISE → WARNING → SPEED_REDUCTION → CONTROLLED_STOP → STANDBY → NORMAL_CRUISE | 1.15 | PASS |
| S09 | Looking away 3 s (distraction) | STANDBY → NORMAL_CRUISE → WARNING → NORMAL_CRUISE | - | PASS |
| S10 | Driver camera occluded 2 s | STANDBY → NORMAL_CRUISE → SPEED_REDUCTION → NORMAL_CRUISE | - | PASS |
| S11 | Driver leaves the seat 7 s | STANDBY → NORMAL_CRUISE → SPEED_REDUCTION → CONTROLLED_STOP | 0.88 | PASS |
| S12 | Dim lighting (lamp at 20 %) | STANDBY → NORMAL_CRUISE | - | PASS |
| S13 | Glare (lamp reflection on mat + bright face) | STANDBY → NORMAL_CRUISE | - | PASS |
| S14 | Static obstacle on the lane | STANDBY → NORMAL_CRUISE → EMERGENCY_STOP → NORMAL_CRUISE | - | PASS |
| S15 | Wi-Fi / heartbeat loss 1 s | STANDBY → NORMAL_CRUISE → EMERGENCY_STOP → NORMAL_CRUISE | - | PASS |
| S16 | Lane markings lost (tape gap) | STANDBY → NORMAL_CRUISE → EMERGENCY_STOP | - | PASS |

## Plots

![S01](S01.png)
![S02](S02.png)
![S03](S03.png)
![S04](S04.png)
![S05](S05.png)
![S06](S06.png)
![S07](S07.png)
![S08](S08.png)
![S09](S09.png)
![S10](S10.png)
![S11](S11.png)
![S12](S12.png)
![S13](S13.png)
![S14](S14.png)
![S15](S15.png)
![S16](S16.png)
