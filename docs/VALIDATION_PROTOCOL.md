# Validation protocol (W12–W13, milestone M4)

Two layers, scored by the same code (`adowndriver/audit/metrics.py`):

1. **Software-in-the-loop (SIL)** – `python -m adowndriver validate --plot` (or *Validation* tab).
   Real lane detector on rendered images + real FSM + real controller, simulated driver / ToF / Wi-Fi.
   Run after every code or parameter change. Report: `logs/validation/REPORT.md` + one plot per scenario.
2. **Arena trials** on the EVA-mat track with a human volunteer at Node 1.

## Arena set-up

* Track: stadium oval, 2.0 m straights, inner radius ≈ 0.5 m, lane 30 cm between tape centres
  (yellow = left/inner, white = right/outer, matte tape only).
* Mark a 2 cm tape tick every 50 cm on the lane centre to read the physical stop position (M4).
* LED lamp on a stand 1 m from the track for S12/S13; room lights on for everything else.
* Volunteer 45–55 cm from the C270, face evenly lit, calibrate once per person (Calibrate button).
* Robot always starts from the same mark, in STANDBY; operator presses START.

## Procedure per trial

1. *Trials* tab → choose the scenario (procedure text appears) → driver/operator names, notes (lamp level, mat) → **Start recording**.
2. For driver scenarios start the guided protocol if applicable (the cue appears on the cockpit HUD/OLED).
3. Execute the scenario; never touch the robot unless it is about to leave the mat (then E-STOP; note it).
4. **Stop & score**. The run is saved (SQLite + `logs/runs/run_XXXX.csv`) and scored.
5. After a CONTROLLED_STOP: measure the robot's lateral offset to the centre mark with a ruler and write it in *Notes*.

## Matrix (≥ 50 logged trials for M4)

| ID | Scenario | Repeats | Pass condition |
|---|---|---|---|
| S01 | Baseline, natural blinking 40 s | 4 | NORMAL only |
| S02 | Rapid blinking | 3 | no SPEED_REDUCTION / STOP |
| S03 | Talking | 3 | NORMAL only |
| S04 | Single yawn | 3 | WARNING → back to NORMAL |
| S05 | 3 yawns in 30 s | 3 | SPEED_REDUCTION (REPEATED_YAWNING) |
| S06 | Eyes closed 1.8 s | 4 | WARNING only |
| S07 | Eyes closed 3.2 s | 4 | SPEED_REDUCTION, no stop |
| S08 | Eyes closed until stop | 5 | CONTROLLED_STOP, \|e_y\| ≤ 5 cm, RESET works |
| S09 | Look away 3 s | 3 | WARNING (DRIVER_DISTRACTED) |
| S10 | Lens covered 2 s | 3 | SPEED_REDUCTION, recovers |
| S11 | Driver leaves 7 s | 3 | CONTROLLED_STOP (DRIVER_NOT_MONITORABLE) |
| S12 | Dim light | 3 | NORMAL, no lane loss |
| S13 | Glare | 3 | no EMERGENCY |
| S14 | Obstacle | 4 | EMERGENCY ≤ 200 ms, no contact, resume |
| S15 | Wi-Fi drop 1 s | 3 | EMERGENCY ≤ 200 ms, resume |
| S16 | Tape gap / lens blocked | 2 | EMERGENCY (LANE_LOST), latched |
| | **F1 protocol** (`f1_standard`, 3 min) | 5 volunteers | F1 ≥ 0.90 over all runs |

Total 52 + 5 protocol runs.

## Reporting

* `python -m adowndriver report <run_id>` prints the metrics JSON and exports CSV.
* The *Trials* tab shows per-run metrics; download CSVs for the final report figures.
* Aggregate across runs: pooled TP/FP/FN for F1, worst case for the other metrics (`audit.metrics.aggregate`).
