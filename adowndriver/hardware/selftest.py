"""`python -m adowndriver check --role node1|node2` - verifies every dependency and device
before an arena session and prints a PASS/WARN/FAIL checklist."""
from __future__ import annotations

import importlib
import os
import socket
import time

from ..config import AppConfig, resolve_path


def _line(status: str, name: str, info: str = "") -> bool:
    print(f"  [{status:^4}] {name:<34} {info}")
    return status != "FAIL"


def run_selftest(cfg: AppConfig, role: str) -> None:
    print(f"AdownDriver self-test ({role})")
    ok = True
    for mod in ("numpy", "cv2", "yaml"):
        try:
            m = importlib.import_module(mod)
            _line("OK", f"python: {mod}", getattr(m, "__version__", ""))
        except ImportError as exc:
            ok &= _line("FAIL", f"python: {mod}", str(exc))
    for mod, why in (("gpiozero", "LED / buzzer / buttons"), ("luma.oled", "OLED (node1)")):
        try:
            importlib.import_module(mod)
            _line("OK", f"python: {mod}", why)
        except ImportError:
            _line("WARN", f"python: {mod}", f"missing -> {why} disabled")
    for dev in ("/dev/i2c-1",):
        _line("OK" if os.path.exists(dev) else "WARN", dev, "I2C bus (ToF / OLED)" if os.path.exists(dev) else
              "enable I2C (raspi-config / dtparam=i2c_arm=on)")

    if role == "node1":
        try:
            from ..driver_monitor.backend import create_backend
            b = create_backend(cfg.driver)
            _line("OK", "MediaPipe face landmarker", b.name)
        except Exception as exc:
            ok &= _line("FAIL", "MediaPipe face landmarker", str(exc)[:80])
        import cv2
        cap = cv2.VideoCapture(cfg.driver.camera_index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        good = cap.isOpened() and cap.read()[0]
        if good:
            t0, n = time.time(), 0
            while time.time() - t0 < 1.0:
                n += cap.read()[0]
            _line("OK", "USB webcam (C270)", f"{n} fps")
        else:
            ok &= _line("FAIL", "USB webcam (C270)", f"index {cfg.driver.camera_index} not readable")
        cap.release()
    else:
        try:
            from ..vehicle.io import FrameGrabber
            g = FrameGrabber(cfg.vehicle.camera_source, cfg.camera.width, cfg.camera.height)
            time.sleep(1.0)
            f, _, seq = g.read()
            _line("OK" if f is not None else "FAIL", "road camera", f"{g.backend}, {seq} frames/s")
            g.close()
        except Exception as exc:
            ok &= _line("FAIL", "road camera", str(exc)[:90])
        try:
            from ..vehicle.io import make_range_sensor
            t = make_range_sensor(cfg.vehicle)
            name = type(t).__name__
            time.sleep(1.5)
            d, _ = t.read()
            if name == "NoTof":
                ok &= _line("FAIL", "obstacle sensor", "no VL53L1X and no /scan (is turtlebot3_bringup running?)")
            else:
                _line("OK" if d is not None else "WARN", f"obstacle sensor ({name})", f"{d} cm ahead")
            t.close()
        except Exception as exc:
            ok &= _line("FAIL", "obstacle sensor", str(exc)[:80])
        try:
            import rclpy  # noqa: F401
            _line("OK", "ROS 2 rclpy", "remember: ros2 launch turtlebot3_bringup robot.launch.py")
        except ImportError:
            ok &= _line("FAIL", "ROS 2 rclpy", "source /opt/ros/humble/setup.bash")
    model = resolve_path(cfg.driver.model_path)
    if role == "node1":
        _line("OK" if model.is_file() else "WARN", "face_landmarker.task", str(model))
    for name, port in (("driver port", cfg.network.driver_port), ("cmd port", cfg.network.node2_cmd_port)):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
            _line("OK", f"udp/{port} free", name)
        except OSError:
            _line("WARN", f"udp/{port} in use", "is a node already running?")
        finally:
            s.close()
    print("RESULT:", "PASS" if ok else "FAIL")
