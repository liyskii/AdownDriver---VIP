"""
Inter-node UDP protocol (version 2).

Every datagram is a compact UTF-8 JSON object with a ``type`` field, except
preview frames which are binary (``b"ADPV" + node_id + JPEG``).

    Node1 --driver (50 Hz)----------------------> Node2
    Node2 --vehicle_state (20 Hz)---------------> Node1      (echo for RTT)
    Node1/Node2 --status/telemetry (10 Hz)------> Dashboard
    Node1/Node2 --preview JPEG (6 Hz)-----------> Dashboard
    Dashboard --command-------------------------> Node1 / Node2
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Optional

PROTOCOL_VERSION = 2
PREVIEW_MAGIC = b"ADPV"
MAX_DATAGRAM = 60000


def now_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class DriverPacket:
    """What Node 1 knows about the driver, sent at 50 Hz."""
    seq: int = 0
    t_send_ms: int = 0              # Node 1 wall clock at send
    frame_seq: int = 0
    frame_age_ms: float = 0.0       # age of the analysed frame when sent
    face: bool = False
    ear: float = 0.30               # EMA-filtered, bilateral average
    mar: float = 0.10               # EMA-filtered
    ear_raw: float = 0.30
    ear_th: float = 0.21            # per-driver calibrated threshold
    c_driver: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    proc_ms: float = 0.0            # Node 1 per-frame processing time
    fps: float = 0.0
    gt: str = ""                    # ground-truth label during a guided protocol
    rtt_ms: float = -1.0            # Node1<->Node2 round trip measured via vehicle_state echo
    ack: bool = False               # driver pressed ACK button (one-shot)
    type: str = "driver"
    v: int = PROTOCOL_VERSION


@dataclass
class VehicleStatePacket:
    """Node 2 -> Node 1 feedback (drives cockpit buzzer, LED, OLED)."""
    state: str = "STANDBY"
    reason: str = "NONE"
    v_cmd: float = 0.0
    buzzer: str = "OFF"
    led: str = "BLUE"
    echo_seq: int = 0
    echo_t_send_ms: int = 0
    hold_ms: float = 0.0            # time between receiving echo_seq and sending this
    d_obs_cm: float = -1.0
    c_lane: float = 0.0
    type: str = "vehicle_state"
    v: int = PROTOCOL_VERSION


def encode(obj) -> bytes:
    d = asdict(obj) if hasattr(obj, "__dataclass_fields__") else obj
    return json.dumps(d, separators=(",", ":")).encode("utf-8")


def decode(data: bytes) -> Optional[dict]:
    try:
        msg = json.loads(data.decode("utf-8"))
        return msg if isinstance(msg, dict) and "type" in msg else None
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


def driver_from_dict(d: dict) -> DriverPacket:
    known = DriverPacket.__dataclass_fields__.keys()
    return DriverPacket(**{k: v for k, v in d.items() if k in known})


def encode_preview(node: int, jpeg: bytes) -> bytes:
    return PREVIEW_MAGIC + bytes([node]) + jpeg


def decode_preview(data: bytes):
    if len(data) > 5 and data[:4] == PREVIEW_MAGIC:
        return data[4], data[5:]
    return None


# --------------------------------------------------------------------------- #
class UdpEndpoint:
    """Small helper around a UDP socket with an optional receive thread."""

    def __init__(self, bind_port: Optional[int] = None, bind_ip: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:  # low-latency DSCP (EF) - harmless if not honoured
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xB8)
        except OSError:
            pass
        if bind_port is not None:
            self.sock.bind((bind_ip, bind_port))
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def send(self, payload: bytes, ip: str, port: int) -> bool:
        if len(payload) > MAX_DATAGRAM:
            return False
        try:
            self.sock.sendto(payload, (ip, port))
            return True
        except OSError:
            return False

    def start_receiver(self, handler: Callable[[bytes, tuple], None]) -> None:
        self._running = True
        self.sock.settimeout(0.2)

        def loop():
            while self._running:
                try:
                    data, addr = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    handler(data, addr)
                except Exception as exc:  # never kill the receive thread
                    print(f"[udp] handler error: {exc}")

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass


@dataclass
class LatestValue:
    """Thread-safe holder of the latest value plus its local arrival time."""
    value: object = None
    t_mono: float = 0.0
    count: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def put(self, value) -> None:
        with self._lock:
            self.value = value
            self.t_mono = time.monotonic()
            self.count += 1

    def get(self):
        with self._lock:
            return self.value, self.t_mono, self.count
