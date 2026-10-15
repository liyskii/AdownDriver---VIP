"""
Node 1: Cockpit DMS Station (Raspberry Pi 4B IoT Kit)
Real-time Driver Drowsiness & Yawn Detector (EAR, MAR, C_driver) with EMA Smoothing & UDP Telemetry.
Team Matigoat - AdownDriver VIP Project
"""
import cv2
import time
import json
import socket
import argparse
import numpy as np

try:
    import mediapipe as mp
    MP_AVAILABLE = True
except ImportError:
    MP_AVAILABLE = False

# MediaPipe 468 Face Mesh Landmark Indices
LEFT_EYE_IDX  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]
INNER_LIP_IDX = [78, 81, 13, 311, 308, 402, 14, 178]

def euclidean_dist(p1, p2):
    return float(np.linalg.norm(np.array(p1, dtype=np.float32) - np.array(p2, dtype=np.float32)))

def compute_ear(landmarks, eye_indices):
    p1, p2, p3, p4, p5, p6 = [landmarks[i] for i in eye_indices]
    vert1 = euclidean_dist(p2, p6)
    vert2 = euclidean_dist(p3, p5)
    horiz = euclidean_dist(p1, p4)
    if horiz < 1e-6:
        return 0.30
    return (vert1 + vert2) / (2.0 * horiz)

def compute_mar(landmarks, lip_indices):
    m1, m2, m3, m4, m5, m6, m7, m8 = [landmarks[i] for i in lip_indices]
    vert1 = euclidean_dist(m2, m8)
    vert2 = euclidean_dist(m3, m7)
    vert3 = euclidean_dist(m4, m6)
    horiz = euclidean_dist(m1, m5)
    if horiz < 1e-6:
        return 0.10
    return (vert1 + vert2 + vert3) / (2.0 * horiz)

class EMAFilter:
    def __init__(self, alpha=0.35, initial_value=None):
        self.alpha = alpha
        self.value = initial_value

    def update(self, measurement):
        if self.value is None:
            self.value = float(measurement)
        else:
            self.value = self.alpha * float(measurement) + (1.0 - self.alpha) * self.value
        return self.value

class DriverMonitoringNode:
    def __init__(self, udp_ip="127.0.0.1", udp_port=5005, alpha=0.35):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ear_ema = EMAFilter(alpha=alpha, initial_value=0.30)
        self.mar_ema = EMAFilter(alpha=alpha, initial_value=0.15)
        self.seq_id = 0

        self.face_mesh = None
        if MP_AVAILABLE:
            try:
                self.mp_face_mesh = mp.solutions.face_mesh
                self.face_mesh = self.mp_face_mesh.FaceMesh(
                    max_num_faces=1,
                    refine_landmarks=True,
                    min_detection_confidence=0.65,
                    min_tracking_confidence=0.65
                )
            except AttributeError:
                print("[Warning] mp.solutions not found in this MediaPipe build. Running in simulation/fallback mode.")
                self.face_mesh = None

    def process_frame(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        ear_raw, mar_raw, c_driver = 0.30, 0.15, 0.0

        if self.face_mesh is not None:
            results = self.face_mesh.process(rgb)
            if results.multi_face_landmarks:
                face_lms = results.multi_face_landmarks[0].landmark
                pts = [(lm.x * w, lm.y * h) for lm in face_lms]
                ear_l = compute_ear(pts, LEFT_EYE_IDX)
                ear_r = compute_ear(pts, RIGHT_EYE_IDX)
                ear_raw = (ear_l + ear_r) / 2.0
                mar_raw = compute_mar(pts, INNER_LIP_IDX)
                c_driver = 0.95
            else:
                c_driver = 0.10
        else:
            # Fallback simulation for testing without physical webcam or if solutions is missing
            ear_raw = 0.28
            mar_raw = 0.12
            c_driver = 0.90

        ear_filt = self.ear_ema.update(ear_raw) if c_driver >= 0.65 else self.ear_ema.value
        mar_filt = self.mar_ema.update(mar_raw) if c_driver >= 0.65 else self.mar_ema.value

        self.seq_id += 1
        packet = {
            "seq_id": self.seq_id,
            "timestamp_ms": int(time.time() * 1000),
            "ear_filtered": round(ear_filt, 4),
            "mar_filtered": round(mar_filt, 4),
            "c_driver": round(c_driver, 3),
            "heartbeat_ok": True
        }
        try:
            self.sock.sendto(json.dumps(packet).encode("utf-8"), (self.udp_ip, self.udp_port))
        except Exception:
            pass
        return packet

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Node 1: AdownDriver DMS Vision Station")
    parser.add_argument("--camera", type=int, default=0, help="USB Webcam index")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Target Node 2 IP address")
    parser.add_argument("--port", type=int, default=5005, help="Target UDP Port")
    args = parser.parse_args()

    node = DriverMonitoringNode(udp_ip=args.ip, udp_port=args.port)
    cap = cv2.VideoCapture(args.camera)
    print(f"[Node 1 DMS] Streaming UDP telemetry to {args.ip}:{args.port} ... Press 'q' to exit.")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("[Info] Camera frame not detected. Running simulation tick...")
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        pkt = node.process_frame(frame)
        cv2.putText(frame, f"EAR: {pkt['ear_filtered']:.2f} | MAR: {pkt['mar_filtered']:.2f} | C_drv: {pkt['c_driver']:.2f}",
                    (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Node 1 - Cockpit DMS Station", frame)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
