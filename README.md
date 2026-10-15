# 🚗 AdownDriver: Drowsiness-Aware Autonomous Vehicle Safety System

[![VIP Coursework](https://img.shields.io/badge/Inha_University-VIP_Project_2026-005baa?style=for-the-badge)](https://www.inha.ac.kr)
[![Track T4](https://img.shields.io/badge/Track_T4-Autonomous_Vehicle_%26_Deep_Learning-success?style=for-the-badge)]()
[![Platform](https://img.shields.io/badge/Hardware-Dual_Raspberry_Pi_4B_%2B_TurtleBot_3-orange?style=for-the-badge)]()
[![Python](https://img.shields.io/badge/Python-3.10%2B_%7C_ROS_2_%7C_MediaPipe_%7C_OpenCV-blue?style=for-the-badge)]()

> **Course:** Vertically Integrated Projects (VIP) — Fall 2026, School of Global Convergence Studies, Inha University  
> **Academic Advisor:** Prof. Mehdi Pirahandeh[cite: 3]  
> **Team:** **#Matigoat** (11 Members | Team Leader: **Li Vladislav**, `12250245`)[cite: 5]

---

## 📌 1. Project Overview & Core Innovations

**AdownDriver** is a distributed, dual-node drowsiness-aware supervisory control and autonomous safety prototype built on a **TurtleBot 3** mobile robot platform and a dedicated **Raspberry Pi 4B IoT Cockpit Station**[cite: 5].

Conventional low-cost Driver Monitoring Systems (DMS) operate as isolated *open-loop* alarms. **AdownDriver** closes the control loop by executing a verifiable **Graduated Safe-Stop Envelope**[cite: 5]:

1. **Distributed Dual-Pi Edge Architecture:** Splits high-FPS facial landmark deep learning (**Node 1**) from real-time road lane segmentation and motor control (**Node 2**) over a 50 Hz low-latency UDP telemetry link[cite: 5].
2. **Volunteer / Synchronized Cockpit Testbed:** A live human operator sits at the stationary **Node 1 Cockpit DMS Station**, streaming real-time alertness states to the moving **Node 2 TurtleBot 3** on the test track[cite: 4].
3. **Decoupled Confidence Metrics & Hysteresis:** Separates driver landmark visibility confidence ($C_{	ext{driver}}$) from road lane visibility confidence ($C_{	ext{lane}}$) and enforces strict persistence timers to achieve a **0% false-positive rate** during normal blinking ($< 400	ext{ ms}$)[cite: 4, 5].
4. **Mandatory SQLite & CSV Audit Logging:** Every state transition, `EAR`/`MAR` trace, lateral offset ($e_y$), obstacle distance ($d_{	ext{obs}}$), and stop reason code is automatically recorded to `logs/validation_trial_results.csv` and `logs/adown_audit.db`[cite: 4, 5].

---

## 🏗️ 2. System Architecture & Hardware Stack

| Subsystem Layer | Hardware Components | Software & Frameworks | Primary Engineering Function |
| :--- | :--- | :--- | :--- |
| **Node 1: Cockpit DMS Station** | Raspberry Pi 4B (IoT Kit), Logitech C270 HD USB Webcam, 0.96\" OLED Display | Python 3, MediaPipe Face Landmarker (TFLite), NumPy, UDP Socket | 30 FPS facial landmark extraction (468 points), bilateral `EAR` & `MAR` calculation, EMA filtering ($lpha=0.35$), and $C_{	ext{driver}}$ streaming[cite: 4, 5]. |
| **Node 2: Mobile Vehicle Base** | TurtleBot 3 (RPi 4B + OpenCR 1.0 + Dynamixel XL430), Pi Camera V2 (CSI) | ROS 2, OpenCV (`warpPerspective`, HSV/Canny), PID Steering Controller | Bird's-eye lane segmentation, lateral centroid offset ($e_y$) tracking, $C_{	ext{lane}}$ evaluation, and velocity profiling[cite: 4, 5]. |
| **Active Safety Peripherals** | VL53L1X ToF Laser Sensor, Active/Passive Buzzer, RGB/Status LEDs, Push-Button Reset | RPi.GPIO / I2C Interface, Hardware Watchdog Timer | Independent collision override ($d_{	ext{obs}} < 20	ext{ cm}$), multi-stage acoustic/LED warnings, and heartbeat loss braking ($\le 200	ext{ ms}$)[cite: 4, 5]. |
| **Telemetry, Logging & HMI** | SanDisk Extreme 64GB microSDXC, Custom 3D-Printed Brackets (SGCS MakerBot M-124) | FastAPI, Uvicorn, SQLite3, CSV Audit Logger, Autodesk Fusion 360 | Real-time web dashboard visualization (`http://127.0.0.1:8000`) and reproducible experimental validation logging[cite: 4, 5]. |

---

## 👥 3. Team #Matigoat — Member Roles & Responsibilities

| Student ID | Name | Assigned Engineering Subsystem & Role |
| :---: | :--- | :--- |
| **`12250245`** | **Li Vladislav** *(Leader)* | System Architecture, Supervisory Safety FSM & Distributed Dual-Pi Integration[cite: 5] |
| **`12250249`** | **Li Andrey** | Driver Monitoring Vision Pipeline (MediaPipe Face Mesh, `EAR`/`MAR` & EMA Filtering)[cite: 5] |
| **`12250247`** | **Pak Nikita** | Forward Road Perception, OpenCV Perspective Warp, Lane Centroid & $C_{	ext{lane}}$[cite: 5] |
| **`12240192`** | **Vadim Lifrov** | Embedded Systems, ROS 2 TurtleBot 3 Actuation & Inter-Pi UDP Telemetry[cite: 5] |
| **`12240198`** | **Hsu Shwe Sin Htun** | Real-Time Operator Web Dashboard & HMI Visualization (FastAPI)[cite: 5] |
| **`12240221`** | **Egamberdiev Saidumar** | Hardware Assembly, ToF Sensor / Buzzer / LED Wiring & Power Regulation[cite: 5] |
| **`12240236`** | **Husan Tulaev** | Backend Telemetry Aggregation & Structured SQLite / CSV Audit Logging[cite: 5] |
| **`12225243`** | **Dilshod Obidov** | Experimental Validation, 16-Scenario Fault-Injection Testing & Latency Auditing[cite: 4, 5] |
| **`12240234`** | **Fazliddin Igamberdiev** | 3D CAD Design (Autodesk Fusion 360) & 3D-Printed Mounts (SGCS MakerBot M-124)[cite: 5, 6] |
| **`12225262`** | **Arsen Baktybek uulu** | Safety Fallback Logic, Decoupled Confidence Matrix ($C_{	ext{driver}}$, $C_{	ext{lane}}$) & Watchdog[cite: 5] |
| **`12240216`** | **Sevinchxon Khoshimova** | Technical Documentation, Quantitative Metric Verification & Demo Video Production[cite: 5] |
