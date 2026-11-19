"""
Node 2 - TurtleBot 3 vehicle process.

    python -m adowndriver node2                 # on the TurtleBot (ROS 2 bringup running)
    python -m adowndriver node2 --sim           # laptop: simulated track, ToF and motors

Threads: camera grabber, ToF reader, driver-packet receiver, command receiver,
motor publisher (with watchdog), indicator player; the main thread runs the
fixed-rate control loop from vehicle/core.py.
"""
from __future__ import annotations

import collections
import json
import math
import signal
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from ..audit.metrics import compute_metrics
from ..audit.recorder import AuditRecorder
from ..common.protocol import (LatestValue, UdpEndpoint, VehicleStatePacket, decode, driver_from_dict, encode,
                               encode_preview, now_ms)
from ..common.filters import FpsMeter, RateTimer
from ..config import PARAM_LIMITS, RUNTIME_TUNABLE, AppConfig, resolve_path
from ..hardware.peripherals import Buttons, StatusIndicators
from .core import StepInput, VehicleCore
from .io import FrameGrabber, make_motor, make_range_sensor


def cpu_temp_c() -> Optional[float]:
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (OSError, ValueError):
        return None


def apply_param(cfg: AppConfig, path: str, value) -> tuple[bool, str]:
    section, _, name = path.partition(".")
    if name not in RUNTIME_TUNABLE.get(section, []):
        return False, f"{path} is not runtime-tunable"
    try:
        v = cfg.set(path, value)
    except (KeyError, ValueError, TypeError) as exc:
        return False, str(exc)
    lo_hi = PARAM_LIMITS.get(path)
    if lo_hi and not lo_hi[0] <= v <= lo_hi[1]:
        cfg.set(path, min(max(v, lo_hi[0]), lo_hi[1]))
        return True, f"{path} clamped to {cfg.get(path)}"
    return True, f"{path} = {v}"


class VehicleNode:
    def __init__(self, cfg: AppConfig, sim: bool = False):
        self.cfg = cfg
        self.sim = sim
        self.core = VehicleCore(cfg)
        n = cfg.network
        self.world = None
        if sim:
            from ..sim.world import SimWorld
            self.world = SimWorld(cfg.camera, cfg.lane)
            self.cam = None
            self.tof = None
        else:
            # with lane keeping off the road camera is only a preview -> the robot must still run without it
            self.cam = FrameGrabber(cfg.vehicle.camera_source, cfg.camera.width, cfg.camera.height,
                                    optional=not cfg.vehicle.lane_keeping)
            self.tof = self._make_tof()
        self.motor = make_motor(cfg.vehicle, self.world)
        self.ind = StatusIndicators(cfg.hardware, "node2")
        self.buttons = Buttons(cfg.hardware, on_a=lambda: self.push_cmd("estop", "button"),
                               on_b=lambda: self.push_cmd("reset", "button"))
        self.recorder = AuditRecorder(resolve_path(cfg.audit.log_dir), cfg.audit.db_name, cfg.audit.tick_log_hz)

        self.driver_rx = LatestValue()
        self.drv_ep = UdpEndpoint(n.driver_port)
        self.drv_ep.start_receiver(self._on_driver)
        self.cmd_ep = UdpEndpoint(n.node2_cmd_port)
        self.cmd_ep.start_receiver(self._on_command)
        self.tx = UdpEndpoint()
        self.node1_addr = (n.node1_ip, n.vehicle_state_port)

        self.cmd_queue: collections.deque = collections.deque()
        self.recent_events: collections.deque = collections.deque(maxlen=40)
        self.last_ack: dict = {}
        self.link_drop_until = 0.0
        self.running = True
        self.t_start = time.monotonic()
        self.fps = FpsMeter()

    def _make_tof(self):
        return make_range_sensor(self.cfg.vehicle)

    # ------------------------------------------------------------------ #
    def _on_driver(self, data: bytes, addr) -> None:
        if time.monotonic() < self.link_drop_until:      # simulated Wi-Fi drop (dashboard sim tools)
            return
        msg = decode(data)
        if msg and msg.get("type") == "driver":
            pkt = driver_from_dict(msg)
            self.driver_rx.put((pkt, addr))

    def _on_command(self, data: bytes, addr) -> None:
        msg = decode(data)
        if msg and msg.get("type") == "command":
            self.cmd_queue.append(msg)

    def push_cmd(self, cmd: str, source: str = "local") -> None:
        self.cmd_queue.append({"type": "command", "cmd": cmd, "source": source})

    def _handle_commands(self) -> list[str]:
        fsm_cmds = []
        while self.cmd_queue:
            m = self.cmd_queue.popleft()
            cmd = m.get("cmd", "")
            ok, info = True, ""
            if cmd in ("start", "stop", "estop", "reset", "ack"):
                fsm_cmds.append(cmd)
                if cmd == "start" and self.recorder.run_id is None:
                    self.recorder.start_run("FREE_DRIVE", source="sim" if self.sim else "live",
                                            config=self.cfg.to_dict())
            elif cmd == "set_param":
                ok, info = apply_param(self.cfg, m.get("path", ""), m.get("value"))
                if ok and m.get("path", "").startswith("lane."):
                    self.core.lane.recompute()
            elif cmd == "run_start":
                rid = self.recorder.start_run(m.get("scenario", "FREE_DRIVE"), m.get("operator", ""),
                                              m.get("driver", ""), m.get("notes", ""),
                                              "sim" if self.sim else "live", self.cfg.to_dict())
                info = f"run {rid} started"
            elif cmd == "run_stop":
                info = self._finish_run()
            elif cmd == "sim" and self.world is not None:
                info = self._sim_action(m.get("action"), m.get("value"))
            else:
                ok, info = False, f"unknown command {cmd}"
            self.last_ack = {"cmd": cmd, "ok": ok, "info": info, "t_ms": now_ms(), "id": m.get("id")}
            self.recorder.event("COMMAND", json.dumps({"cmd": cmd, "source": m.get("source", "dashboard"),
                                                       "info": info}))
        return fsm_cmds

    def _finish_run(self) -> str:
        rid = self.recorder.run_id
        if rid is None:
            return "no active run"
        self.recorder.flush()
        _, ticks, events = AuditRecorder.load_run(self.recorder.db_path, rid)
        m = compute_metrics(ticks, events)
        self.recorder.stop_run({"metrics": {k: v["value"] for k, v in m["metrics"].items()},
                                "pass": {k: v["pass"] for k, v in m["metrics"].items()}})
        if self.cfg.audit.export_csv_on_stop:
            AuditRecorder.export_csv(self.recorder.db_path, rid,
                                     self.recorder.dir / "runs" / f"run_{rid:04d}.csv")
        return f"run {rid} saved"

    def _sim_action(self, action: str, value) -> str:
        w = self.world
        if action == "obstacle":
            w.place_obstacle_ahead(float(value or 0.6))
        elif action == "clear":
            w.obstacles.clear()
        elif action == "brightness":
            w.brightness = float(value)
        elif action == "glare":
            w.glare = (210, 150, 22) if value else None
        elif action == "lens":
            w.lens_blocked = bool(value)
        elif action == "link_drop":
            self.link_drop_until = time.monotonic() + float(value or 1.0)
        elif action == "respawn":
            x, y, th = w.track.pose_at(0.3)
            w.robot.x, w.robot.y, w.robot.th, w.robot.v, w.robot.w = x, y, th, 0, 0
        else:
            return f"unknown sim action {action}"
        return f"sim {action} {value}"

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        cfg = self.cfg
        period = 1.0 / cfg.vehicle.loop_hz
        tel_t, state_t, prev_t = RateTimer(cfg.network.telemetry_hz), RateTimer(cfg.network.vehicle_state_hz), \
            RateTimer(cfg.network.preview_hz)
        last = time.monotonic()
        last_seq = -1
        print(f"[node2] running ({'SIMULATION' if self.sim else 'robot'}); motor={self.motor.name}, "
              f"camera={'sim' if self.sim else self.cam.backend}, tof={'sim' if self.sim else type(self.tof).__name__}")
        while self.running:
            loop_start = time.monotonic()
            dt = loop_start - last
            last = loop_start

            # ---- sensors ----------------------------------------------------
            if self.sim:
                v_app, w_app = self.motor.applied
                self.world.step(v_app, w_app, dt)
                frame, f_seq = self.world.render(), last_seq + 1
                d_cm, tof_t, tof_on = self.world.tof_cm(), loop_start, True
            else:
                frame, _, f_seq = self.cam.read()
                d_cm, tof_t = self.tof.read()
                tof_on = self.tof.enabled
            new_frame = frame is not None and f_seq != last_seq
            last_seq = f_seq

            (drv, drv_t, _) = self.driver_rx.get()
            pkt = drv[0] if drv else None
            age = max(0.0, loop_start - drv_t) if drv else float("inf")
            want_preview = prev_t.ready(loop_start)

            out = self.core.step(StepInput(
                dt=dt, t_ms=now_ms(), frame=frame if new_frame else None, d_obs_cm=d_cm,
                tof_age_s=max(0.0, loop_start - tof_t), tof_enabled=tof_on, driver=pkt, driver_age_s=age,
                rtt_ms=pkt.rtt_ms if pkt and pkt.rtt_ms >= 0 else None,
                commands=tuple(self._handle_commands()), want_debug=want_preview and new_frame))

            self.motor.command(out.v_cmd, out.omega)
            self.ind.set(out.fsm.led, out.fsm.buzzer)
            for k, detail in out.events:
                self.recorder.event(k, detail, out.fsm.state.value, out.fsm.reason)
                self.recent_events.append({"t_ms": now_ms(), "kind": k, "detail": detail,
                                           "state": out.fsm.state.value})
                if k in ("TRANSITION", "FAULT", "BRAKE"):
                    print(f"[node2] {k:<10} {detail}")
            self.recorder.tick(out.tick_row)
            loop_fps = self.fps.tick(loop_start)

            # ---- feedback to Node 1 ------------------------------------------
            if state_t.ready(loop_start):
                vs = VehicleStatePacket(out.fsm.state.value, out.fsm.reason, round(out.v_cmd, 3), out.fsm.buzzer,
                                        out.fsm.led, pkt.seq if pkt else 0, pkt.t_send_ms if pkt else 0,
                                        round(age * 1000, 1) if drv else 0.0,
                                        -1.0 if d_cm is None else round(d_cm, 1), round(out.lane.c_lane, 3))
                addr = (drv[1][0], self.cfg.network.vehicle_state_port) if drv else self.node1_addr
                self.tx.send(encode(vs), *addr)

            # ---- dashboard ---------------------------------------------------
            if tel_t.ready(loop_start):
                tel = dict(out.telemetry)
                tel.update({
                    "type": "node2", "t_ms": now_ms(), "sim": self.sim, "loop_hz": round(loop_fps, 1),
                    "uptime_s": round(loop_start - self.t_start, 1), "cpu_temp": cpu_temp_c(),
                    "motor": self.motor.name, "watchdog_trips": self.motor.watchdog_trips,
                    "camera": "sim" if self.sim else self.cam.backend,
                    "range_sensor": ({"src": "sim"} if self.sim else
                                     self.tof.stats() if hasattr(self.tof, "stats") else
                                     {"src": type(self.tof).__name__}),
                    "run_id": self.recorder.run_id, "run": self.recorder.run_meta if self.recorder.run_id else None,
                    "events": list(self.recent_events)[-12:], "ack": self.last_ack,
                    "pose": None if not self.sim else {"x": round(self.world.robot.x, 3),
                                                       "y": round(self.world.robot.y, 3),
                                                       "th": round(self.world.robot.th, 3)},
                })
                self.tx.send(json.dumps(tel, separators=(",", ":"), default=str).encode(),
                             self.cfg.network.dashboard_ip, self.cfg.network.telemetry_port)
            if want_preview and frame is not None and out.lane.debug is not None:
                self._send_preview(frame, out)

            sleep = period - (time.monotonic() - loop_start)
            if sleep > 0:
                time.sleep(sleep)

    def _send_preview(self, frame: np.ndarray, out) -> None:
        cam = frame.copy()
        h = cam.shape[0]
        bev = cv2.resize(out.lane.debug, (int(out.lane.debug.shape[1] * h / out.lane.debug.shape[0]), h))
        img = np.hstack([cam, bev])
        cv2.putText(img, f"C_lane {out.lane.c_lane:.2f}  e {out.lane.e_axle_m * 100:+.1f}cm  {out.lane.mode}",
                    (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.cfg.network.preview_jpeg_quality])
        if ok:
            self.tx.send(encode_preview(2, jpg.tobytes()), self.cfg.network.dashboard_ip,
                         self.cfg.network.preview_port)

    def close(self) -> None:
        self.running = False
        self.motor.close()
        self.ind.close()
        if self.recorder.run_id is not None:
            self._finish_run()
        self.recorder.close()
        for c in (self.cam, self.tof):
            if c is not None:
                c.close()
        for ep in (self.drv_ep, self.cmd_ep, self.tx):
            ep.close()


def main(cfg: AppConfig, sim: bool = False) -> None:
    node = VehicleNode(cfg, sim)

    def stop(*_):
        node.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node.run()
    finally:
        node.close()
        print("[node2] stopped, motors zeroed")
