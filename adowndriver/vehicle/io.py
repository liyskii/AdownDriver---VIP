"""
Node 2 hardware I/O: road camera, VL53L1X ToF sensor, motor command sink.
Every device runs in its own thread and exposes only its *latest* value with a
timestamp, so a slow device can never block the 30 Hz control loop.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

from ..common.filters import MedianFilter
from ..config import VehicleConfig


# --------------------------------------------------------------------------- #
# Camera
# --------------------------------------------------------------------------- #
class FrameGrabber:
    """Background grabber; ``read()`` returns (frame, t_mono, seq)."""

    def __init__(self, source: str, width: int = 320, height: int = 240, fps: int = 30):
        self.source, self.w, self.h, self.fps = source, width, height, fps
        self.frame: Optional[np.ndarray] = None
        self.t = 0.0
        self.seq = 0
        self.backend = "none"
        self._lock = threading.Lock()
        self._running = True
        self._open()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self) -> None:
        src = self.source
        self.picam = self.cap = None
        tried = []
        order = [src] if src != "auto" else ["picamera2", "v4l2:0", "gstreamer"]
        for s in order:
            try:
                if s == "picamera2":
                    from picamera2 import Picamera2
                    cam = Picamera2()
                    cam.configure(cam.create_video_configuration(
                        main={"size": (self.w, self.h), "format": "RGB888"},  # RGB888 = BGR byte order
                        controls={"FrameRate": self.fps}))
                    cam.start()
                    self.picam, self.backend = cam, "picamera2"
                    return
                if s.startswith("v4l2"):
                    idx = int(s.split(":")[1]) if ":" in s else 0
                    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    if cap.isOpened() and cap.read()[0]:
                        self.cap, self.backend = cap, s
                        return
                    cap.release()
                if s == "gstreamer":
                    pipe = (f"libcamerasrc ! video/x-raw,width={self.w},height={self.h},framerate={self.fps}/1 ! "
                            "videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1")
                    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
                    if cap.isOpened() and cap.read()[0]:
                        self.cap, self.backend = cap, "gstreamer"
                        return
                if s.startswith("file:"):
                    cap = cv2.VideoCapture(s[5:])
                    if cap.isOpened():
                        self.cap, self.backend = cap, s
                        return
            except Exception as exc:
                tried.append(f"{s}: {exc}")
        raise RuntimeError(f"No road camera available (tried {order}). {tried}")

    def _loop(self) -> None:
        period = 1.0 / self.fps
        while self._running:
            if self.picam is not None:
                frame = self.picam.capture_array()
            else:
                ok, frame = self.cap.read()
                if not ok:
                    if self.backend.startswith("file:"):
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    time.sleep(0.01)
                    continue
                if self.backend.startswith("file:"):
                    time.sleep(period)
            if frame.shape[1] != self.w:
                frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
            with self._lock:
                self.frame, self.t, self.seq = frame, time.monotonic(), self.seq + 1

    def read(self):
        with self._lock:
            return self.frame, self.t, self.seq

    def close(self) -> None:
        self._running = False
        try:
            if self.picam is not None:
                self.picam.stop()
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# ToF
# --------------------------------------------------------------------------- #
class TofSensor:
    """VL53L1X (Pimoroni ``vl53l1x`` library) with a 3-sample median filter.

    ``read()`` -> (distance_cm or None, t_mono). None means *invalid*: sensor
    error, 0 mm (no target / saturated) or out of range; the FSM treats a
    continuously invalid sensor as TOF_FAULT after 200 ms.
    """
    max_valid_cm = {1: 130.0, 2: 300.0, 3: 400.0}

    def __init__(self, cfg: VehicleConfig):
        import VL53L1X  # pip install vl53l1x
        self.cfg = cfg
        self.dev = VL53L1X.VL53L1X(i2c_bus=cfg.tof_i2c_bus, i2c_address=cfg.tof_address)
        self.dev.open()
        try:
            self.dev.set_timing(33000, 33)        # 33 ms budget, 33 ms period -> 30 Hz
        except Exception:
            pass
        self.dev.start_ranging(cfg.tof_mode)
        self.med = MedianFilter(3)
        self.value: Optional[float] = None
        self.t = 0.0
        self.enabled = True
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        limit = self.max_valid_cm.get(self.cfg.tof_mode, 130.0)
        while self._running:
            try:
                mm = self.dev.get_distance()
                cm = mm / 10.0 - self.cfg.tof_mount_offset_cm
                if mm <= 0:
                    val = None
                elif cm > limit:
                    val = limit                   # nothing within range = clear path
                else:
                    val = self.med.update(cm)
            except Exception:
                val = None
            self.value, self.t = val, time.monotonic()
            time.sleep(0.005)

    def read(self):
        return self.value, self.t

    def close(self) -> None:
        self._running = False
        try:
            self.dev.stop_ranging()
        except Exception:
            pass


class NoTof:
    enabled = False

    def read(self):
        return None, time.monotonic()

    def close(self):
        pass


# --------------------------------------------------------------------------- #
# Motors
# --------------------------------------------------------------------------- #
class MotorSink:
    """Publishes (v, omega) at 20 Hz from its own thread and zeroes the command
    if the control loop has not refreshed it within ``watchdog_s`` (software
    watchdog: a hung control loop cannot leave the robot driving)."""

    name = "null"

    def __init__(self, watchdog_s: float = 0.25):
        self.watchdog_s = watchdog_s
        self.cmd = (0.0, 0.0)
        self.t_cmd = 0.0
        self.watchdog_trips = 0
        self.last_publish_t = 0.0
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def command(self, v: float, w: float) -> float:
        with self._lock:
            self.cmd, self.t_cmd = (v, w), time.monotonic()
        if v == 0.0:
            self._publish(0.0, 0.0)          # stop commands go out immediately
        return time.monotonic()

    def _loop(self) -> None:
        while self._running:
            with self._lock:
                (v, w), t = self.cmd, self.t_cmd
            if time.monotonic() - t > self.watchdog_s:
                if v != 0 or w != 0:
                    self.watchdog_trips += 1
                v = w = 0.0
            self._publish(v, w)
            time.sleep(0.05)

    def _publish(self, v: float, w: float) -> None:
        self.last_publish_t = time.monotonic()

    def close(self) -> None:
        self._running = False
        self._publish(0.0, 0.0)


class Ros2Motor(MotorSink):
    """geometry_msgs/Twist (Humble) or TwistStamped (Jazzy) on /cmd_vel for turtlebot3_bringup."""
    name = "ros2"

    def __init__(self, cfg: VehicleConfig):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("adown_vehicle")
        self.stamped = cfg.cmd_vel_stamped
        if self.stamped:
            from geometry_msgs.msg import TwistStamped as Msg
        else:
            from geometry_msgs.msg import Twist as Msg
        self.Msg = Msg
        self.pub = self.node.create_publisher(Msg, cfg.cmd_vel_topic,
                                              QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._plock = threading.Lock()
        super().__init__(cfg.motor_watchdog_s)

    def _publish(self, v: float, w: float) -> None:
        msg = self.Msg()
        tw = msg.twist if self.stamped else msg
        if self.stamped:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        tw.linear.x = float(v)
        tw.angular.z = float(w)
        with self._plock:
            self.pub.publish(msg)
        self.last_publish_t = time.monotonic()

    def close(self) -> None:
        super().close()
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass


def make_motor(cfg: VehicleConfig, sim_world=None) -> MotorSink:
    want = cfg.motor_driver
    if sim_world is not None and want in ("auto", "sim"):
        return SimMotor(sim_world, cfg.motor_watchdog_s)
    if want in ("auto", "ros2"):
        try:
            return Ros2Motor(cfg)
        except Exception as exc:
            if want == "ros2":
                raise
            print(f"[node2] ROS 2 not available ({exc.__class__.__name__}: {exc}); motor commands are NOT sent")
    return MotorSink(cfg.motor_watchdog_s)


class SimMotor(MotorSink):
    name = "sim"

    def __init__(self, world, watchdog_s: float):
        self.world = world
        self.applied = (0.0, 0.0)
        super().__init__(watchdog_s)

    def _publish(self, v: float, w: float) -> None:
        self.applied = (v, w)
        self.last_publish_t = time.monotonic()
