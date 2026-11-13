"""
Node 1 - Cockpit DMS station (Raspberry Pi 4B + Logitech C270 + OLED + buzzer).

    python -m adowndriver node1                      # webcam 0, HUD window
    python -m adowndriver node1 --headless           # no monitor attached
    python -m adowndriver node1 --source sim         # no webcam: synthetic driver controlled from the dashboard
    python -m adowndriver node1 --driver andrey      # load calibrated profile config/drivers/andrey.json

Buttons: A = calibrate (5 s, look at the camera with open eyes), B = ACK (driver
acknowledges an alarm / resets a latched CONTROLLED_STOP once alert).
If Node 2 stops answering, Node 1 keeps warning the driver locally with its own
copy of the safety FSM (monitor-only mode).
"""
from __future__ import annotations

import json
import signal
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from ..common.filters import FpsMeter, RateTimer
from ..common.protocol import (DriverPacket, LatestValue, UdpEndpoint, decode, encode, encode_preview, now_ms)
from ..config import AppConfig, REPO_ROOT, resolve_path
from ..hardware.peripherals import Buttons, Oled, StatusIndicators
from ..vehicle.fsm import FsmInputs, SafetyFSM
from .analyzer import (INNER_LIP_IDX, LEFT_EYE_IDX, RIGHT_EYE_IDX, Calibrator, DriverProfile, DriverSignalProcessor,
                       observe)

STATE_BGR = {"STANDBY": (230, 160, 60), "NORMAL_CRUISE": (90, 200, 90), "WARNING": (0, 215, 255),
             "SPEED_REDUCTION": (0, 140, 255), "CONTROLLED_STOP": (60, 60, 240), "EMERGENCY_STOP": (40, 40, 255),
             "OFFLINE": (140, 140, 140)}
PROFILE_DIR = REPO_ROOT / "config" / "drivers"
PROTOCOL_DIR = REPO_ROOT / "config" / "protocols"


class Camera:
    def __init__(self, index: int, w: int, h: int, fps: int):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2) if hasattr(cv2, "CAP_V4L2") else cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # C270: 30 fps needs MJPG
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open webcam {index}")
        self.latest = LatestValue()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                self.latest.put(frame)
            else:
                time.sleep(0.01)

    def close(self):
        self.running = False
        self.cap.release()


class Protocol:
    """Guided trial: on-screen cues with ground-truth labels for the F1 metric."""

    def __init__(self, name: str):
        data = yaml.safe_load((PROTOCOL_DIR / f"{name}.yaml").read_text(encoding="utf-8"))
        self.name = name
        self.steps = data["steps"]
        self.t0 = time.monotonic()
        self.idx = -1

    def current(self):
        t = time.monotonic() - self.t0
        acc = 0.0
        for i, st in enumerate(self.steps):
            if t < acc + st["duration"]:
                return i, st, acc + st["duration"] - t
            acc += st["duration"]
        return None, None, 0.0

    @staticmethod
    def available() -> list[str]:
        return sorted(p.stem for p in PROTOCOL_DIR.glob("*.yaml"))


class DriverMonitorNode:
    def __init__(self, cfg: AppConfig, source: str = "webcam", headless: bool = False, driver: str = "default"):
        self.cfg = cfg
        d = cfg.driver
        self.source = source
        self.headless = headless
        self.proc = DriverSignalProcessor(d)
        self.profile = DriverProfile.load(PROFILE_DIR, driver) or DriverProfile(driver, ear_th=d.default_ear_threshold)
        self.proc.ear_th = self.profile.ear_th
        self.backend = None
        self.cam = None
        self.synth = None
        if source == "sim":
            from ..sim.synthetic_driver import Segment, SyntheticDriver
            self.synth = SyntheticDriver(d, [Segment(0, 1e9, "alert", "")], seed=int(time.time()))
            self.sim_behaviour = "alert"
        else:
            from .backend import create_backend
            self.backend = create_backend(d)
            self.cam = Camera(int(source) if source.isdigit() else d.camera_index, d.camera_width,
                              d.camera_height, d.camera_fps)

        n = cfg.network
        self.tx = UdpEndpoint()
        self.state_rx = LatestValue()
        self.state_ep = UdpEndpoint(n.vehicle_state_port)
        self.state_ep.start_receiver(self._on_state)
        self.cmd_ep = UdpEndpoint(n.node1_cmd_port)
        self.cmd_ep.start_receiver(self._on_cmd)

        self.ind = StatusIndicators(cfg.hardware, "node1")
        self.oled = Oled(cfg.hardware)
        self.buttons = Buttons(cfg.hardware, on_a=self.start_calibration, on_b=self.ack)
        self.local_fsm = SafetyFSM(cfg.fsm.__class__(**{**cfg.fsm.__dict__, "require_driver_for_start": False}))
        self.local_fsm.update(FsmInputs(dt=0.0, commands=("start",)))

        self.lock = threading.Lock()
        self.packet = DriverPacket(ear_th=self.proc.ear_th)
        self.frame_t = 0.0
        self.seq = 0
        self.rtt_ms = -1.0
        self.ack_until = 0.0
        self.calib: Optional[Calibrator] = None
        self.calib_msg = ""
        self.protocol: Optional[Protocol] = None
        self.vis_frame = None
        self.vis_obs = None
        self.vis_vals: dict = {}
        self.cue = None
        self.fps = FpsMeter()
        self.running = True

    # ---------------- network callbacks ----------------------------------
    def _on_state(self, data, addr):
        m = decode(data)
        if m and m.get("type") == "vehicle_state":
            self.state_rx.put(m)
            if m.get("echo_t_send_ms"):
                rtt = now_ms() - m["echo_t_send_ms"] - m.get("hold_ms", 0.0)
                if 0 <= rtt < 2000:
                    self.rtt_ms = rtt if self.rtt_ms < 0 else 0.8 * self.rtt_ms + 0.2 * rtt

    def _on_cmd(self, data, addr):
        m = decode(data)
        if not m or m.get("type") != "command":
            return
        cmd = m.get("cmd")
        if cmd == "calibrate":
            self.start_calibration(m.get("driver") or self.profile.name)
        elif cmd == "load_profile":
            p = DriverProfile.load(PROFILE_DIR, m.get("driver", ""))
            if p:
                self.profile, self.proc.ear_th = p, p.ear_th
        elif cmd == "protocol_start":
            try:
                self.protocol = Protocol(m.get("name", "f1_standard"))
            except (OSError, KeyError, yaml.YAMLError) as exc:
                self.calib_msg = f"protocol error: {exc}"
        elif cmd == "protocol_stop":
            self.protocol = None
        elif cmd == "sim_behaviour" and self.synth is not None:
            from ..sim.synthetic_driver import Segment
            b = m.get("value", "alert")
            t0 = time.monotonic()
            # a yawn is an event (~4 s), everything else lasts until the next click
            self.synth.timeline = [Segment(t0, t0 + (4.0 if b == "yawn" else 1e9), b, "")]
            self.sim_behaviour = b
        elif cmd == "ack":
            self.ack()
        elif cmd == "mute":
            self.ind.muted = bool(m.get("value"))

    def start_calibration(self, name: Optional[str] = None):
        self.calib = Calibrator(self.cfg.driver, name or self.profile.name)
        self.calib_msg = "Calibrating: look at the camera, eyes open"
        self.ind.chirp()

    def ack(self):
        self.ack_until = time.monotonic() + 0.4
        self.ind.chirp()

    # ---------------- sender thread (50 Hz heartbeat) --------------------
    def _sender(self):
        n = self.cfg.network
        period = 1.0 / n.driver_send_hz
        while self.running:
            t0 = time.monotonic()
            with self.lock:
                p = self.packet
                self.seq += 1
                p.seq = self.seq
                p.t_send_ms = now_ms()
                age = (t0 - self.frame_t) * 1000.0 if self.frame_t else 1e6
                p.frame_age_ms = round(age, 1)
                stale = age > self.cfg.driver.frame_stale_s * 1000
                p.ack = t0 < self.ack_until
                p.rtt_ms = round(self.rtt_ms, 1)
                payload = encode(p) if not stale else encode(DriverPacket(**{**p.__dict__, "face": False,
                                                                               "c_driver": 0.0}))
            self.tx.send(payload, n.node2_ip, n.driver_port)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    # ---------------- main processing loop -------------------------------
    def _process_once(self) -> bool:
        t0 = time.perf_counter()
        if self.synth is not None:
            time.sleep(1 / 30)
            if self.sim_behaviour == "yawn" and time.monotonic() > self.synth.timeline[0].t1:
                self.sim_behaviour = "alert"
            t0 = time.perf_counter()
            obs = self.synth.observe(time.monotonic())
            frame = None
        else:
            frame, ft, cnt = self.cam.latest.get()
            if frame is None or cnt == getattr(self, "_last_cnt", -1):
                time.sleep(0.003)
                return False
            self._last_cnt = cnt
            if self.cfg.driver.mirror:
                frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pts, mat = self.backend.detect(rgb)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            obs = observe(pts, gray, mat)
        vals = self.proc.update(obs)
        proc_ms = (time.perf_counter() - t0) * 1000.0
        fps = self.fps.tick()

        if self.calib is not None:
            self.calib.add(obs, vals["c_driver"])
            if self.calib.done():
                prof = self.calib.result()
                if prof:
                    prof.save(PROFILE_DIR)
                    self.profile, self.proc.ear_th = prof, prof.ear_th
                    self.calib_msg = f"Calibrated {prof.name}: EAR open {prof.ear_open:.3f} -> TH {prof.ear_th:.3f}"
                else:
                    self.calib_msg = "Calibration failed: face not visible enough"
                self.calib = None
                self.ind.chirp()

        gt = ""
        cue = None
        if self.protocol is not None:
            i, step, remaining = self.protocol.current()
            if step is None:
                self.protocol = None
            else:
                gt = step.get("label", "alert")
                cue = (step["cue"], remaining, i + 1, len(self.protocol.steps))
                if i != self.protocol.idx:
                    self.protocol.idx = i
                    self.ind.chirp()
        with self.lock:
            self.packet = DriverPacket(frame_seq=self.packet.frame_seq + 1, face=vals["face"], ear=vals["ear"],
                                       mar=vals["mar"], ear_raw=vals["ear_raw"], ear_th=vals["ear_th"],
                                       c_driver=vals["c_driver"], yaw_deg=vals["yaw_deg"],
                                       pitch_deg=vals["pitch_deg"], proc_ms=round(proc_ms, 2), fps=round(fps, 1),
                                       gt=gt)
            self.frame_t = time.monotonic()
        self.vis_frame, self.vis_obs, self.vis_vals, self.cue = frame, obs, vals, cue
        return True

    def _vehicle_state(self) -> dict:
        st, t, _ = self.state_rx.get()
        if st is None or time.monotonic() - t > 1.0:
            return {"state": "OFFLINE", "reason": "NODE2_OFFLINE", "v_cmd": 0.0, "buzzer": "OFF", "led": "MAGENTA"}
        return st

    def run(self):
        threading.Thread(target=self._sender, daemon=True).start()
        tel_t, prev_t = RateTimer(self.cfg.network.telemetry_hz), RateTimer(self.cfg.network.preview_hz)
        last = time.monotonic()
        print(f"[node1] streaming to Node 2 at {self.cfg.network.node2_ip}:{self.cfg.network.driver_port} "
              f"({self.backend.name if self.backend else 'synthetic driver'}); profile '{self.profile.name}' "
              f"TH_EAR={self.proc.ear_th:.3f}")
        while self.running:
            if not self._process_once():
                continue
            now = time.monotonic()
            dt, last = now - last, now
            vs = self._vehicle_state()
            # local monitor-only FSM keeps alarming the driver if Node 2 is offline
            lout = self.local_fsm.update(FsmInputs(dt=dt, driver=self.packet, driver_age_s=0.0, tof_enabled=False))
            if vs["state"] == "OFFLINE":
                buzz, led = lout.buzzer, "MAGENTA" if lout.buzzer == "OFF" else lout.led
            else:
                buzz, led = vs["buzzer"], vs["led"]
            self.ind.set(led, buzz)
            self._oled(vs)
            if tel_t.ready(now):
                self._telemetry(vs)
            if prev_t.ready(now):
                img = self._render(vs, small=True)
                if img is not None:
                    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.cfg.network.preview_jpeg_quality])
                    if ok:
                        self.tx.send(encode_preview(1, jpg.tobytes()), self.cfg.network.dashboard_ip,
                                     self.cfg.network.preview_port)
            if not self.headless:
                img = self._render(vs, small=False)
                if img is not None:
                    cv2.imshow("AdownDriver - Cockpit DMS (Node 1)", img)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    self.running = False
                elif key == ord("c"):
                    self.start_calibration()
                elif key == ord(" "):
                    self.ack()

    def _telemetry(self, vs):
        v = self.vis_vals
        msg = {"type": "node1", "t_ms": now_ms(), "fps": self.packet.fps, "proc_ms": self.packet.proc_ms,
               "backend": self.backend.name if self.backend else "synthetic", "rtt_ms": round(self.rtt_ms, 1),
               "profile": self.profile.__dict__, "calibrating": None if self.calib is None else round(self.calib.progress, 2),
               "calib_msg": self.calib_msg, "protocols": Protocol.available(),
               "protocol": None if self.cue is None else {"name": self.protocol.name if self.protocol else "",
                                                          "cue": self.cue[0], "remaining": round(self.cue[1], 1),
                                                          "step": self.cue[2], "of": self.cue[3]},
               "driver": v, "vehicle": vs.get("state"), "sim": self.synth is not None,
               "sim_behaviour": getattr(self, "sim_behaviour", None), "muted": self.ind.muted,
               "oled": self.oled.available, "gpio": self.ind.backend}
        self.tx.send(json.dumps(msg, default=str).encode(), self.cfg.network.dashboard_ip,
                     self.cfg.network.telemetry_port)

    def _oled(self, vs):
        if not self.oled.available:
            return
        v = self.vis_vals
        if self.cue:
            self.oled.show(f"STEP {self.cue[2]}/{self.cue[3]} {self.cue[1]:.0f}s", [self.cue[0][i:i + 21] for i in range(0, 63, 21)])
            return
        self.oled.show(vs["state"].replace("_", " "),
                       [f"EAR {v['ear']:.2f}/{v['ear_th']:.2f}", f"MAR {v['mar']:.2f} C {v['c_driver']:.2f}",
                        f"v {vs.get('v_cmd', 0):.2f} m/s", f"RTT {self.rtt_ms:.0f} ms"],
                       invert=vs["state"] not in ("NORMAL_CRUISE", "STANDBY"))

    def _render(self, vs, small: bool):
        frame, obs, v = self.vis_frame, self.vis_obs, self.vis_vals
        if frame is None:
            frame = np.full((240, 320, 3), 30, np.uint8)
            cv2.putText(frame, f"SYNTHETIC: {getattr(self, 'sim_behaviour', '')}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        img = frame.copy()
        if obs is not None and obs.pts:
            for idx, col in ((LEFT_EYE_IDX, (255, 220, 0)), (RIGHT_EYE_IDX, (255, 220, 0)), (INNER_LIP_IDX, (0, 200, 255))):
                pts = np.array([obs.pts[i] for i in idx], np.int32)
                cv2.polylines(img, [pts], True, col, 1, cv2.LINE_AA)
            if obs.bbox:
                cv2.rectangle(img, obs.bbox[:2], obs.bbox[2:], (120, 120, 120), 1)
        if small:
            img = cv2.resize(img, (self.cfg.network.preview_width, int(img.shape[0] * self.cfg.network.preview_width / img.shape[1])))
        h, w = img.shape[:2]
        col = STATE_BGR.get(vs["state"], (200, 200, 200))
        s = w / 640
        cv2.rectangle(img, (0, 0), (w, int(34 * s)), col, -1)
        cv2.putText(img, vs["state"].replace("_", " "), (int(8 * s), int(24 * s)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75 * s, (20, 20, 20), max(1, int(2 * s)), cv2.LINE_AA)
        txt = f"EAR {v['ear']:.2f}/{v['ear_th']:.2f}  MAR {v['mar']:.2f}  C {v['c_driver']:.2f}  {self.packet.fps:.0f}fps"
        cv2.putText(img, txt, (int(8 * s), h - int(10 * s)), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (255, 255, 255),
                    1, cv2.LINE_AA)
        if self.cue and not small:
            cv2.rectangle(img, (0, h // 2 - 40), (w, h // 2 + 40), (0, 0, 0), -1)
            cv2.putText(img, self.cue[0], (16, h // 2 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(img, f"{self.cue[1]:.1f} s   (step {self.cue[2]}/{self.cue[3]})", (16, h // 2 + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1, cv2.LINE_AA)
        if self.calib is not None:
            cv2.rectangle(img, (0, h - int(50 * s)), (int(w * self.calib.progress), h - int(40 * s)), (0, 255, 0), -1)
        return img

    def close(self):
        self.running = False
        self.ind.close()
        self.oled.clear()
        if self.cam:
            self.cam.close()
        if self.backend:
            self.backend.close()
        for ep in (self.tx, self.state_ep, self.cmd_ep):
            ep.close()
        if not self.headless:
            cv2.destroyAllWindows()


def main(cfg: AppConfig, source: str = "webcam", headless: bool = False, driver: str = "default") -> None:
    node = DriverMonitorNode(cfg, source, headless, driver)

    def stop(*_):
        node.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node.run()
    finally:
        node.close()
        print("[node1] stopped")
