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

    def __init__(self, source: str, width: int = 320, height: int = 240, fps: int = 30,
                 optional: bool = False):
        self.source, self.w, self.h, self.fps = source, width, height, fps
        self.frame: Optional[np.ndarray] = None
        self.t = 0.0
        self.seq = 0
        self.backend = "none"
        self._lock = threading.Lock()
        self._running = True
        self.optional = optional
        try:
            self._open()
        except RuntimeError as exc:
            if not optional:
                raise
            print(f"[node2] road camera not found - running WITHOUT it, will keep looking ({exc})", flush=True)
            self.picam = self.cap = None
            self.backend = "none"
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self) -> None:
        src = self.source
        self.picam = self.cap = None
        tried = []
        order = [src] if src != "auto" else ["picamera2", "v4l2:0", "gstreamer"]
        if src.startswith("v4l2"):                        # a USB camera may come up as video0..video3
            order += [f"v4l2:{i}" for i in range(4) if f"v4l2:{i}" != src]
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
            if self.picam is None and self.cap is None:      # optional camera missing: retry every 3 s
                time.sleep(3.0)
                try:
                    self._open()
                    print(f"[node2] road camera connected ({self.backend})", flush=True)
                except Exception:
                    pass
                continue
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


def front_distance_cm(ranges, angle_min: float, angle_increment: float, sector_deg: float,
                      front_offset_cm: float, max_cm: float, range_min: float = 0.05) -> float:
    """Closest obstacle (cm, from the robot's front edge) inside +/-sector_deg of straight ahead.

    0 / inf / NaN / below range_min are "no return" and ignored. The 3rd-smallest valid beam is
    used instead of the minimum so one noisy beam cannot trigger an EMERGENCY_STOP. If the cone
    has no valid return the path is clear -> ``max_cm``.
    """
    r = np.asarray(ranges, dtype=np.float32)
    if r.size == 0:
        return max_cm
    ang = angle_min + angle_increment * np.arange(r.size, dtype=np.float32)
    ang = (ang + np.pi) % (2 * np.pi) - np.pi                     # wrap to [-pi, pi)
    cone = np.abs(ang) <= np.deg2rad(sector_deg)
    vals = r[cone]
    vals = vals[np.isfinite(vals) & (vals >= range_min)]
    if vals.size == 0:
        return max_cm
    k = min(2, vals.size - 1)
    d = float(np.partition(vals, k)[k]) * 100.0 - front_offset_cm
    return float(min(max(d, 0.0), max_cm))


class LidarRange:
    """TurtleBot3 LDS on ``/scan`` used as the forward range sensor (drop-in for TofSensor).

    ``read()`` -> (distance_cm or None, t_mono); None when no scan arrived for ``lidar_stale_s``
    (lidar unplugged / bringup not running) so the FSM raises TOF_FAULT exactly as for the VL53L1X.
    Runs its own rclpy context + node + executor thread; needs ``turtlebot3_bringup`` to be running.
    If no scan arrives for ``RESUBSCRIBE_S`` the node is rebuilt (recovers after a bringup restart
    or a lost DDS discovery), and ``stats()`` reports what is happening for the dashboard.
    """

    RESUBSCRIBE_S = 3.0

    def __init__(self, cfg: VehicleConfig):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self.cfg = cfg
        self.rclpy = rclpy
        self._Exec, self._qos, self._Msg = SingleThreadedExecutor, qos_profile_sensor_data, LaserScan
        try:                                              # own DDS participant, isolated from the motor node
            self.ctx = Context()
            rclpy.init(context=self.ctx)
        except Exception:                                 # fall back to the shared default context
            self.ctx = None
            if not rclpy.ok():
                rclpy.init()
        self.value: Optional[float] = None
        self.t_scan = 0.0
        self.scans = 0
        self.rebuilds = 0
        self.last_error = ""
        self.enabled = True
        self._running = True
        self.node = None
        self.exec = None
        self._build()
        self.t_built = time.monotonic()
        threading.Thread(target=self._spin, daemon=True).start()

    def _build(self) -> None:
        kw = {} if self.ctx is None else {"context": self.ctx}
        self.node = self.rclpy.create_node("adown_lidar_range", **kw)
        self.node.create_subscription(self._Msg, self.cfg.lidar_topic, self._on_scan, self._qos)
        self.exec = self._Exec(**kw)
        self.exec.add_node(self.node)

    def _rebuild(self) -> None:
        try:
            self.exec.shutdown(timeout_sec=0.2)
            self.node.destroy_node()
        except Exception as exc:
            self.last_error = f"{exc.__class__.__name__}: {exc}"[:120]
        self._build()
        self.rebuilds += 1
        self.t_built = time.monotonic()
        print(f"[node2] lidar: no {self.cfg.lidar_topic} for {self.RESUBSCRIBE_S:.0f} s - resubscribed "
              f"(#{self.rebuilds}, scans so far {self.scans})", flush=True)

    def _on_scan(self, msg) -> None:
        try:
            self.value = front_distance_cm(msg.ranges, msg.angle_min, msg.angle_increment,
                                           self.cfg.lidar_sector_deg, self.cfg.lidar_front_offset_cm,
                                           self.cfg.lidar_max_cm, max(0.05, msg.range_min))
            self.t_scan = time.monotonic()
            self.scans += 1
        except Exception as exc:
            self.last_error = f"{exc.__class__.__name__}: {exc}"[:120]

    def _spin(self) -> None:
        while self._running:
            try:
                self.exec.spin_once(timeout_sec=0.1)
            except Exception as exc:
                self.last_error = f"{exc.__class__.__name__}: {exc}"[:120]
                time.sleep(0.05)
            now = time.monotonic()
            if now - max(self.t_scan, self.t_built) > self.RESUBSCRIBE_S:
                try:
                    self._rebuild()
                except Exception as exc:
                    self.last_error = f"{exc.__class__.__name__}: {exc}"[:120]
                    self.t_built = now

    def read(self):
        now = time.monotonic()
        if self.t_scan == 0.0 or now - self.t_scan > self.cfg.lidar_stale_s:
            return None, now
        return self.value, self.t_scan

    def stats(self) -> dict:
        age = None if self.t_scan == 0.0 else round((time.monotonic() - self.t_scan) * 1000)
        return {"src": "lidar", "topic": self.cfg.lidar_topic, "scans": self.scans, "age_ms": age,
                "rebuilds": self.rebuilds, "error": self.last_error}

    def close(self) -> None:
        self._running = False
        time.sleep(0.15)                                  # let the spin thread leave spin_once
        try:
            self.exec.shutdown(timeout_sec=0.5)
            self.node.destroy_node()
            if self.ctx is not None:
                self.rclpy.shutdown(context=self.ctx)
        except Exception:
            pass


def make_range_sensor(cfg: VehicleConfig):
    """VL53L1X if present, otherwise the TurtleBot LDS (``tof_driver``: auto | vl53l1x | lidar | none)."""
    want = cfg.tof_driver
    if want == "none":
        return NoTof()
    if want in ("auto", "vl53l1x"):
        try:
            return TofSensor(cfg)
        except Exception as exc:
            if want == "vl53l1x":
                raise
            print(f"[node2] VL53L1X not available ({exc.__class__.__name__}); trying the LDS lidar on {cfg.lidar_topic}")
    try:
        return LidarRange(cfg)
    except Exception as exc:
        if want == "lidar":
            raise
        print(f"[node2] lidar not available ({exc.__class__.__name__}: {exc}); obstacle detection DISABLED")
        return NoTof()


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
