"""
AdownDriver Visual HUD Demo Simulator for Midterm Screen Recording
Simulates Node 1 (Driver EAR/MAR) + Node 2 (Bird's-Eye Lane + PID + 5-State FSM) in a live OpenCV window.
Press 'q' to exit early, or let it run through all 5 scenarios automatically.
"""
import os
import sys
import cv2
import time
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from src.vehicle_control.controller import SupervisorySafetyFSM, StructuredAuditLogger
from src.lane_following.lane_detector import PIDSteeringController

STATE_COLORS = {
    "NORMAL_CRUISE": (80, 220, 100),       # Green
    "WARNING": (0, 215, 255),              # Yellow
    "SPEED_REDUCTION": (0, 140, 255),      # Orange
    "CONTROLLED_STOP": (60, 60, 255),      # Red
    "EMERGENCY_SAFE_STOP": (40, 40, 255)   # Bright Red
}

def draw_hud():
    logger = StructuredAuditLogger()
    fsm = SupervisorySafetyFSM(logger=logger)
    pid = PIDSteeringController()
    dt = 0.1

    scenarios = [
        ("S1: NORMAL CRUISE & BLINK (<0.4s)", 45, lambda t: (0.16 if 15 <= t <= 18 else 0.31, 0.14, 0.96, 0.94, 85.0, 1.2 * np.sin(t*0.2))),
        ("S2: DRIVER YAWNING (MAR > 0.60)", 35, lambda t: (0.29, 0.68 if t > 8 else 0.15, 0.95, 0.92, 80.0, 0.8 * np.cos(t*0.2))),
        ("S3: MICROSLEEP -> GRADUATED SAFE STOP", 65, lambda t: (0.14, 0.18, 0.94, 0.91, 90.0, max(0.1, 2.5 - t*0.04))),
        ("S4: CAMERA OCCLUSION (Low C_driver)", 35, lambda t: (0.28, 0.15, 0.40, 0.93, 75.0, 1.0)),
        ("S5: TOF OBSTACLE (<20cm) EMERGENCY", 35, lambda t: (0.31, 0.14, 0.96, 0.95, 14.2 if t > 10 else 60.0, 0.4))
    ]

    cv2.namedWindow("AdownDriver - Dual-Node Supervisory Safety HUD (Team Matigoat)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("AdownDriver - Dual-Node Supervisory Safety HUD (Team Matigoat)", 1000, 560)

    for scen_name, steps, gen_fn in scenarios:
        fsm.reset()
        for step in range(steps):
            ear, mar, c_drv, c_ln, d_obs, e_y_cm = gen_fn(step)
            omega = pid.compute(e_y=e_y_cm * 22.0, dt=dt)
            v_cmd, state, reason = fsm.update(ear, mar, c_drv, c_ln, d_obs, True, dt, omega, e_y_cm, scen_name.split(":")[0])

            canvas = np.full((560, 1000, 3), (30, 23, 15), dtype=np.uint8)
            color = STATE_COLORS.get(state, (255, 255, 255))

            # Top Banner
            cv2.rectangle(canvas, (0, 0), (1000, 65), (50, 40, 28), -1)
            cv2.putText(canvas, "AdownDriver: Dual-Node Drowsiness-Aware Autonomous Safety System", (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (248, 189, 56), 2)
            cv2.putText(canvas, f"Active Scenario: {scen_name}", (20, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 220, 220), 1)

            # Left Panel: Node 1 Cockpit DMS
            cv2.rectangle(canvas, (20, 80), (480, 360), (60, 48, 35), 2)
            cv2.putText(canvas, "NODE 1: COCKPIT DMS (RPi 4B IoT Kit)", (35, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (248, 189, 56), 2)
            cv2.putText(canvas, f"EAR (Filtered): {ear:.3f}  [Threshold: 0.21]", (35, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"MAR (Filtered): {mar:.3f}  [Threshold: 0.60]", (35, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"Driver Confidence (C_driver): {c_drv:.2f}", (35, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"Fatigue Timer (t_drowsy): {fsm.t_drowsy:.2f} s", (35, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)

            # Eye Graphic Indicator
            eye_h = int(max(3, min(28, (ear / 0.32) * 28)))
            cv2.ellipse(canvas, (160, 315), (45, eye_h), 0, 0, 360, color, 2)
            cv2.circle(canvas, (160, 315), max(2, eye_h - 4), color, -1)
            cv2.ellipse(canvas, (320, 315), (45, eye_h), 0, 0, 360, color, 2)
            cv2.circle(canvas, (320, 315), max(2, eye_h - 4), color, -1)

            # Right Panel: Node 2 TurtleBot 3 Lane & ToF
            cv2.rectangle(canvas, (510, 80), (980, 360), (60, 48, 35), 2)
            cv2.putText(canvas, "NODE 2: TURTLEBOT 3 LANE & TOF SAFETY", (525, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (120, 220, 120), 2)
            cv2.putText(canvas, f"Lane Confidence (C_lane): {c_ln:.2f}", (525, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"Lateral Error (e_y): {e_y_cm:+.2f} cm [Limit: +/-5 cm]", (525, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"ToF Obstacle Dist (d_obs): {d_obs:.1f} cm [Safe >=20 cm]", (525, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1)
            cv2.putText(canvas, f"Commanded Speed (v_cmd): {v_cmd:.3f} m/s", (525, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2)

            # Mini Bird's-Eye Lane Graphic
            cv2.rectangle(canvas, (640, 275), (850, 350), (20, 20, 20), -1)
            cv2.line(canvas, (665, 275), (665, 350), (255, 255, 255), 3)
            cv2.line(canvas, (825, 275), (825, 350), (0, 220, 255), 3)
            cv2.line(canvas, (745, 275), (745, 350), (90, 90, 90), 1)
            car_x = int(745 + e_y_cm * 12)
            cv2.circle(canvas, (car_x, 315), 12, color, -1)

            # Bottom Supervisory FSM Status Banner
            cv2.rectangle(canvas, (20, 385), (980, 535), color, 3)
            cv2.putText(canvas, f"SUPERVISORY FSM STATE: {state}", (40, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 2)
            cv2.putText(canvas, f"Buzzer: {fsm.buzzer}   |   LED Status: {fsm.led}   |   Stop Reason: {reason}", (40, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (240, 240, 240), 1)
            cv2.putText(canvas, "Live SQLite & CSV Telemetry Logging: ACTIVE (logs/validation_trial_results.csv)", (40, 512), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 230, 160), 1)

            cv2.imshow("AdownDriver - Dual-Node Supervisory Safety HUD (Team Matigoat)", canvas)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                return
    cv2.destroyAllWindows()

if __name__ == "__main__":
    draw_hud()
