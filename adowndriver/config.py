"""
Central configuration for AdownDriver.

All tunable parameters live here as typed dataclasses. Values can be overridden
by a YAML file (``config/default.yaml`` or ``--config my.yaml``) and, for a
subset of safe parameters, at run time from the operator dashboard.

The defaults are the *frozen MVP parameters* of the project specification
(revision 2, see docs/SPEC_REVISION.md).
"""
from __future__ import annotations

import copy
import dataclasses
import os
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - yaml is in requirements
    yaml = None

REPO_ROOT = Path(__file__).resolve().parent.parent


# --------------------------------------------------------------------------- #
# Network
# --------------------------------------------------------------------------- #
@dataclass
class NetworkConfig:
    # Static IPs on the dedicated travel router (see docs/NETWORK.md)
    node1_ip: str = "127.0.0.1"          # Cockpit DMS station
    node2_ip: str = "127.0.0.1"          # TurtleBot 3
    dashboard_ip: str = "127.0.0.1"      # where the dashboard process runs
    driver_port: int = 5005              # Node1 -> Node2 driver telemetry (50 Hz)
    vehicle_state_port: int = 5006       # Node2 -> Node1 state feedback (20 Hz)
    telemetry_port: int = 5010           # Node1/Node2 -> dashboard telemetry
    preview_port: int = 5011             # Node1/Node2 -> dashboard JPEG previews
    node2_cmd_port: int = 5020           # dashboard -> Node2 commands
    node1_cmd_port: int = 5021           # dashboard -> Node1 commands
    driver_send_hz: float = 50.0
    vehicle_state_hz: float = 20.0
    telemetry_hz: float = 10.0
    preview_hz: float = 6.0
    preview_width: int = 320
    preview_jpeg_quality: int = 60


# --------------------------------------------------------------------------- #
# Node 1 - driver monitoring
# --------------------------------------------------------------------------- #
@dataclass
class DriverMonitorConfig:
    camera_index: int = 0
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    mirror: bool = True
    model_path: str = "models/face_landmarker.task"
    model_url: str = ("https://storage.googleapis.com/mediapipe-models/face_landmarker/"
                      "face_landmarker/float16/1/face_landmarker.task")
    ema_alpha: float = 0.35
    default_ear_threshold: float = 0.21
    default_mar_threshold: float = 0.60
    ear_threshold_ratio: float = 0.72     # calibrated TH_EAR = ratio * open-eye baseline
    ear_threshold_min: float = 0.15
    ear_threshold_max: float = 0.28
    calibration_seconds: float = 5.0
    min_face_width_px: float = 90.0       # face narrower than this -> confidence drops
    max_reliable_yaw_deg: float = 30.0
    frame_stale_s: float = 0.30           # frame older than this -> c_driver = 0


# --------------------------------------------------------------------------- #
# Safety FSM (runs on Node 2)
# --------------------------------------------------------------------------- #
@dataclass
class FsmConfig:
    TH_MAR: float = 0.60                  # yawn threshold (EAR threshold comes from Node 1 calibration)
    TH_CONF_MIN: float = 0.65             # minimum perception confidence (C_driver, C_lane)
    T_WARN: float = 1.2                   # continuous eye closure -> WARNING
    T_SLOW: float = 2.5                   # continuous eye closure -> SPEED_REDUCTION
    T_STOP: float = 4.0                   # continuous eye closure -> CONTROLLED_STOP
    T_RECOVER: float = 1.5                # continuous alertness needed to de-escalate
    T_BLINK_MAX: float = 0.40             # closures shorter than this are blinks (never interrupt recovery)
    T_OPEN_TOLERANCE: float = 0.20        # eyes must be open this long to end a closure episode
    T_YAWN_MIN: float = 0.80              # MAR above threshold this long = one yawn
    YAWN_WINDOW_S: float = 60.0
    YAWN_COUNT_SLOW: int = 3              # yawns inside window -> SPEED_REDUCTION
    PERCLOS_WINDOW_S: float = 60.0
    PERCLOS_WARN: float = 0.25            # fraction of time eyes closed -> WARNING
    YAW_DISTRACT_DEG: float = 35.0
    T_DISTRACT: float = 2.0               # looking away this long -> WARNING
    T_CONF_DEBOUNCE: float = 0.50         # low C_driver this long -> SPEED_REDUCTION
    T_DRIVER_LOST: float = 5.0            # low C_driver this long -> CONTROLLED_STOP
    T_LANE_LOSS: float = 0.30             # low C_lane this long -> EMERGENCY_STOP
    HEARTBEAT_TIMEOUT: float = 0.15       # no driver packet this long -> EMERGENCY_STOP
    T_LINK_RESTORE: float = 1.0           # link healthy this long -> auto-clear
    D_SAFE_CM: float = 20.0               # obstacle closer than this -> EMERGENCY_STOP
    D_CLEAR_HYST_CM: float = 5.0
    T_OBSTACLE_CLEAR: float = 1.0
    D_SLOW_CM: float = 45.0               # adaptive slow-down zone start
    T_TOF_FAULT: float = 0.20             # invalid ToF this long -> EMERGENCY_STOP
    V_NORMAL: float = 0.16                # m/s, TurtleBot3 Burger max is 0.22
    V_REDUCED: float = 0.08               # 50 % of nominal
    V_CREEP: float = 0.03                 # minimum speed in the slow-down zone
    A_ACCEL: float = 0.10                 # m/s^2 ramp up
    A_DECEL: float = 0.06                 # m/s^2 controlled ramp down (graduated safe stop)
    auto_resume_obstacle: bool = True
    auto_resume_link: bool = True
    require_driver_for_start: bool = True


# --------------------------------------------------------------------------- #
# Node 2 perception & control
# --------------------------------------------------------------------------- #
@dataclass
class CameraModelConfig:
    """Pinhole model of the road camera; used to build the default bird's-eye
    homography and by the simulator. Refine with tools/calibrate_lane.py."""
    width: int = 320
    height: int = 240
    hfov_deg: float = 62.2                # Pi Camera V2
    vfov_deg: float = 48.8
    height_m: float = 0.195               # lens height above the mat
    pitch_deg: float = 25.0               # downward tilt (3D printed mount)
    forward_offset_m: float = 0.03        # lens ahead of the wheel axle


@dataclass
class LaneConfig:
    lane_width_m: float = 0.30            # distance between yellow and white tape centres
    left_color: str = "yellow"
    right_color: str = "white"
    bev_lateral_m: float = 0.30           # +/- lateral half-width covered by bird's-eye view
    bev_near_m: float = 0.20
    bev_far_m: float = 0.65
    bev_px_per_m: float = 400.0
    lookahead_m: float = 0.35
    src_points: list = field(default_factory=list)   # optional manual homography: 4 [u, v] pixels...
    calib_ground_points: list = field(default_factory=lambda: [   # ...of these ground points [f, r] (m)
        [0.25, -0.15], [0.25, 0.15], [0.55, 0.15], [0.55, -0.15]])
    white_l_min: int = 120
    white_s_max: int = 70
    yellow_h_lo: int = 15
    yellow_h_hi: int = 40
    yellow_s_min: int = 70
    yellow_v_min: int = 90
    n_windows: int = 9
    window_margin_m: float = 0.04
    min_window_px: int = 12
    ema_alpha: float = 0.6


@dataclass
class SteeringConfig:
    kp: float = 3.2                       # rad/s per metre of lateral error
    ki: float = 0.4
    kd: float = 0.25
    kh: float = 1.1                       # rad/s per rad of heading error
    kff: float = 1.0                      # curvature feed-forward gain (0 disables)
    max_omega: float = 1.5                # rad/s (Burger max 2.84)
    integral_limit: float = 0.15


@dataclass
class VehicleConfig:
    loop_hz: float = 30.0
    camera_source: str = "auto"           # auto | picamera2 | v4l2:0 | gstreamer | file:path.mp4 | sim
    motor_driver: str = "auto"            # auto | ros2 | null | sim
    cmd_vel_topic: str = "/cmd_vel"
    cmd_vel_stamped: bool = False         # True for ROS 2 Jazzy TurtleBot3 images
    motor_watchdog_s: float = 0.25        # stale command -> publish zero
    lane_keeping: bool = True             # False = demo without a taped track: drive straight, no LANE_LOST
    tof_driver: str = "auto"              # auto | vl53l1x | lidar | sim | none  (auto: VL53L1X, else TurtleBot LDS /scan)
    tof_i2c_bus: int = 1
    tof_address: int = 0x29
    tof_mode: int = 1                     # 1 short (<=1.3 m, best in ambient light), 2 medium, 3 long
    tof_mount_offset_cm: float = 0.0      # sensor-to-bumper offset
    # TurtleBot3 LDS (LDS-01/02/03) used as the forward obstacle sensor instead of the VL53L1X
    lidar_topic: str = "/scan"
    lidar_sector_deg: float = 20.0        # +/- half-angle of the forward cone
    lidar_front_offset_cm: float = 7.0    # lidar centre -> front edge of the robot (Burger ~7 cm)
    lidar_max_cm: float = 300.0           # nothing closer in the cone = clear path at this distance
    lidar_stale_s: float = 1.0            # no scan for this long -> invalid (FSM raises TOF_FAULT)


@dataclass
class GpioPins:
    led_r: int = 17
    led_g: int = 27
    led_b: int = 22
    buzzer: int = 18                      # drives 2N2222/S8050 base through 2x220R
    button_a: int = 23                    # Node1: CALIBRATE | Node2: E-STOP
    button_b: int = 24                    # Node1: ACK / RESET | Node2: RESET
    common_anode: bool = False


@dataclass
class HardwareConfig:
    gpio_enabled: str = "auto"            # auto | yes | no
    pins: GpioPins = field(default_factory=GpioPins)
    oled_enabled: str = "auto"
    oled_i2c_bus: int = 1
    oled_address: int = 0x3C
    buzzer_enabled: bool = True
    # Buzzer patterns through a speaker on the 3.5 mm jack / USB audio (aplay). auto = Node 1 only.
    audio_buzzer: str = "auto"            # auto | yes | no
    audio_device: str = "auto"            # auto (Pi headphone jack if present) | ALSA name, e.g. plughw:1,0
    audio_volume: float = 0.6             # 0..1
    # Custom alert sounds: WAV files in this folder replace the tones (missing file -> tone):
    #   warning.wav (WARNING), slowing.wav (SPEED_REDUCTION), stop.wav (CONTROLLED/EMERGENCY STOP), ack.wav (button chirp)
    audio_dir: str = "sounds"
    audio_gap_s: float = 0.4              # silence between repeats of a looping alert sound


@dataclass
class AuditConfig:
    log_dir: str = "logs"
    db_name: str = "adown_audit.db"
    tick_log_hz: float = 20.0
    export_csv_on_stop: bool = True


@dataclass
class DashboardConfig:
    host: str = "0.0.0.0"
    port: int = 8000
    node_offline_s: float = 1.5


@dataclass
class AppConfig:
    network: NetworkConfig = field(default_factory=NetworkConfig)
    driver: DriverMonitorConfig = field(default_factory=DriverMonitorConfig)
    fsm: FsmConfig = field(default_factory=FsmConfig)
    camera: CameraModelConfig = field(default_factory=CameraModelConfig)
    lane: LaneConfig = field(default_factory=LaneConfig)
    steering: SteeringConfig = field(default_factory=SteeringConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    audit: AuditConfig = field(default_factory=AuditConfig)
    dashboard: DashboardConfig = field(default_factory=DashboardConfig)

    # ------------------------------------------------------------------ #
    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    def get(self, dotted: str) -> Any:
        obj: Any = self
        for part in dotted.split("."):
            obj = getattr(obj, part)
        return obj

    def set(self, dotted: str, value: Any) -> Any:
        """Set a parameter by dotted path with type coercion. Returns the new value."""
        parts = dotted.split(".")
        obj: Any = self
        for part in parts[:-1]:
            obj = getattr(obj, part)
        name = parts[-1]
        if not hasattr(obj, name):
            raise KeyError(dotted)
        current = getattr(obj, name)
        new = _coerce(value, current)
        setattr(obj, name, new)
        return new

    def copy(self) -> "AppConfig":
        return copy.deepcopy(self)


#: Parameters the dashboard is allowed to change while the robot runs.
RUNTIME_TUNABLE = {
    "fsm": ["TH_MAR", "TH_CONF_MIN", "T_WARN", "T_SLOW", "T_STOP", "T_RECOVER", "T_YAWN_MIN",
            "YAWN_COUNT_SLOW", "PERCLOS_WARN", "YAW_DISTRACT_DEG", "T_DISTRACT", "D_SAFE_CM",
            "D_SLOW_CM", "V_NORMAL", "V_REDUCED", "A_DECEL", "A_ACCEL"],
    "steering": ["kp", "ki", "kd", "kh", "kff", "max_omega"],
    "lane": ["lane_width_m", "lookahead_m", "white_l_min", "yellow_s_min"],
}

#: Hard limits that protect the robot from typos in the dashboard.
PARAM_LIMITS = {
    "fsm.V_NORMAL": (0.0, 0.22), "fsm.V_REDUCED": (0.0, 0.22), "fsm.A_DECEL": (0.02, 1.0),
    "fsm.A_ACCEL": (0.02, 1.0), "fsm.D_SAFE_CM": (10.0, 100.0), "fsm.D_SLOW_CM": (10.0, 150.0),
    "fsm.TH_CONF_MIN": (0.2, 0.95), "steering.max_omega": (0.1, 2.84),
}


def _coerce(value: Any, like: Any) -> Any:
    if isinstance(like, bool):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)
    if isinstance(like, int) and not isinstance(like, bool):
        return int(float(value))
    if isinstance(like, float):
        return float(value)
    if isinstance(like, list):
        return list(value)
    return value


def _merge(dc: Any, data: dict) -> None:
    for f in fields(dc):
        if f.name not in data:
            continue
        cur = getattr(dc, f.name)
        val = data[f.name]
        if is_dataclass(cur) and isinstance(val, dict):
            _merge(cur, val)
        else:
            setattr(dc, f.name, _coerce(val, cur) if cur is not None else val)


def load_config(path: str | os.PathLike | None = None) -> AppConfig:
    """Load defaults, then ``config/default.yaml``, then ``config/local.yaml`` (per-machine, git-ignored),
    then ``$ADOWN_CONFIG``, then *path*."""
    cfg = AppConfig()
    candidates = [REPO_ROOT / "config" / "default.yaml", REPO_ROOT / "config" / "local.yaml"]
    env = os.environ.get("ADOWN_CONFIG")
    if env:
        candidates.append(Path(env))
    if path:
        candidates.append(Path(path))
    for p in candidates:
        if p and Path(p).is_file() and yaml is not None:
            with open(p, "r", encoding="utf-8") as fh:
                data = yaml.safe_load(fh) or {}
            _merge(cfg, data)
    return cfg


def resolve_path(p: str) -> Path:
    path = Path(p)
    return path if path.is_absolute() else REPO_ROOT / path
