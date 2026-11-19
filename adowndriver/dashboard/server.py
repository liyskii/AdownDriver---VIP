"""
Layer 4 HMI - FastAPI operator dashboard.

    python -m adowndriver dashboard            # http://<host>:8000

* Receives UDP telemetry + JPEG previews from both nodes (node IPs are learned
  automatically from incoming packets, config values are only a fallback).
* Pushes a merged live state to the browser over a WebSocket at 10 Hz.
* Sends operator commands (START / STOP / E-STOP / RESET, calibration,
  guided protocols, parameter changes, trial recording, simulator tools).
* Serves the audit database: runs, metrics, CSV export, SIL validation.
* Hosts the integrated task launcher ("Tasks" tab): every task / sub-task of the
  project can be chosen and run from here, with live logs and output files.

The dashboard is *not* in the safety loop: if it crashes or the browser
disconnects the robot keeps running its FSM; E-STOP is also on a hardware button.
"""
from __future__ import annotations

import asyncio
import collections
import io
import json
import threading
import time
from pathlib import Path
from typing import Optional

from fastapi import BackgroundTasks, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from ..audit.metrics import compute_metrics
from ..audit.recorder import AuditRecorder
from ..common.protocol import UdpEndpoint, decode, decode_preview
from ..config import RUNTIME_TUNABLE, AppConfig, resolve_path
from ..launcher.catalog import catalog as task_list
from ..launcher.catalog import find as find_task
from ..launcher.jobs import JobManager, repo_file
from ..sim.scenarios import SCENARIOS

STATIC = Path(__file__).parent / "static"


class Hub:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.latest: dict = {"node1": None, "node2": None}
        self.t_rx: dict = {"node1": 0.0, "node2": 0.0}
        self.addr: dict = {"node1": cfg.network.node1_ip, "node2": cfg.network.node2_ip}
        self.frames: dict = {1: None, 2: None}
        self.frame_t: dict = {1: 0.0, 2: 0.0}
        self.history: collections.deque = collections.deque(maxlen=600)  # 60 s at 10 Hz
        self.cmd_log: collections.deque = collections.deque(maxlen=50)
        self.validation: dict = {"running": False, "report": None}
        self.jobs = JobManager()
        n = cfg.network
        self.tel = UdpEndpoint(n.telemetry_port)
        self.tel.start_receiver(self._on_tel)
        self.prev = UdpEndpoint(n.preview_port)
        self.prev.start_receiver(self._on_prev)
        self.tx = UdpEndpoint()

    def _on_tel(self, data: bytes, addr) -> None:
        m = decode(data)
        if not m or m.get("type") not in ("node1", "node2"):
            return
        node = m["type"]
        with self.lock:
            self.latest[node] = m
            self.t_rx[node] = time.monotonic()
            self.addr[node] = addr[0]
            if node == "node2":
                drv = m.get("driver") or {}
                self.history.append({"t": m["t_ms"], "ear": drv.get("ear"), "ear_th": drv.get("ear_th"),
                                     "mar": drv.get("mar"), "v": m.get("v_cmd"), "c_lane": (m.get("lane") or {}).get("c"),
                                     "c_drv": drv.get("c"), "d": (m.get("tof") or {}).get("d_cm"),
                                     "e": (m.get("lane") or {}).get("e_axle_cm"), "state": m.get("state")})

    def _on_prev(self, data: bytes, addr) -> None:
        p = decode_preview(data)
        if p:
            node, jpg = p
            self.frames[node] = jpg
            self.frame_t[node] = time.monotonic()

    def snapshot(self) -> dict:
        now = time.monotonic()
        off = self.cfg.dashboard.node_offline_s
        with self.lock:
            return {
                "node1": self.latest["node1"], "node2": self.latest["node2"],
                "online": {k: (now - t) < off for k, t in self.t_rx.items()},
                "age_ms": {k: round((now - t) * 1000) if t else None for k, t in self.t_rx.items()},
                "addr": dict(self.addr), "server_t": int(time.time() * 1000),
                "validation_running": self.validation["running"],
                "jobs": self.jobs.active(),
            }

    def send(self, target: str, payload: dict) -> bool:
        n = self.cfg.network
        port = n.node2_cmd_port if target == "node2" else n.node1_cmd_port
        payload = {"type": "command", "source": "dashboard", "id": int(time.time() * 1000), **payload}
        ok = self.tx.send(json.dumps(payload).encode(), self.addr[target], port)
        self.cmd_log.append({"t": int(time.time() * 1000), "target": target, **payload, "sent": ok})
        return ok


def create_app(cfg: Optional[AppConfig] = None) -> FastAPI:
    cfg = cfg or AppConfig()
    hub = Hub(cfg)
    db_path = resolve_path(cfg.audit.log_dir) / cfg.audit.db_name
    app = FastAPI(title="AdownDriver Operator Dashboard", version="2.0.0")
    app.state.hub = hub
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.middleware("http")
    async def no_cache(request, call_next):
        # the code on the robot changes during the demo day - never let the browser keep an old UI
        resp = await call_next(request)
        if request.url.path == "/" or request.url.path.startswith("/static"):
            resp.headers["Cache-Control"] = "no-store"
        return resp

    @app.get("/", response_class=HTMLResponse)
    def index():
        return FileResponse(STATIC / "index.html")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        try:
            while True:
                await sock.send_text(json.dumps(hub.snapshot(), default=str))
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            return

    @app.get("/api/state")
    def state():
        return JSONResponse(json.loads(json.dumps(hub.snapshot(), default=str)))

    @app.get("/api/history")
    def history():
        with hub.lock:
            return list(hub.history)

    @app.post("/api/command")
    def command(body: dict):
        target = body.pop("target", "node2")
        if target not in ("node1", "node2"):
            raise HTTPException(400, "target must be node1 or node2")
        if "cmd" not in body:
            raise HTTPException(400, "cmd missing")
        if target == "node2" and body["cmd"] == "set_param":
            sec, _, name = str(body.get("path", "")).partition(".")
            if name not in RUNTIME_TUNABLE.get(sec, []):
                raise HTTPException(400, f"{body.get('path')} is not runtime-tunable")
            cfg.set(body["path"], body["value"])
        return {"sent": hub.send(target, body), "to": hub.addr[target]}

    async def mjpeg(node: int):
        boundary = b"--frame"
        last = None
        while True:
            jpg = hub.frames.get(node)
            if jpg is not None and jpg is not last:
                last = jpg
                yield boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + \
                    b"\r\n\r\n" + jpg + b"\r\n"
            await asyncio.sleep(0.05)

    @app.get("/video/{node}.mjpg")
    def video(node: int):
        if node not in (1, 2):
            raise HTTPException(404)
        return StreamingResponse(mjpeg(node), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/video/{node}.jpg")
    def still(node: int):
        jpg = hub.frames.get(node)
        if jpg is None:
            raise HTTPException(404, "no frame yet")
        return Response(jpg, media_type="image/jpeg")

    @app.get("/api/config")
    def config():
        return {"config": cfg.to_dict(), "tunable": RUNTIME_TUNABLE}

    @app.get("/api/scenarios")
    def scenarios():
        return [{"id": s.id, "name": s.name, "category": s.category, "duration": s.duration,
                 "procedure": s.arena_procedure, "expect_reach": s.expect_reach, "expect_final": s.expect_final}
                for s in SCENARIOS]

    @app.get("/api/runs")
    def runs(limit: int = 100):
        return AuditRecorder.list_runs(db_path, limit)

    @app.get("/api/runs/{run_id}")
    def run_detail(run_id: int, max_points: int = 1500):
        if not db_path.is_file():
            raise HTTPException(404)
        run, ticks, events = AuditRecorder.load_run(db_path, run_id)
        if run is None:
            raise HTTPException(404, "run not found")
        m = compute_metrics(ticks, events)
        step = max(1, len(ticks) // max_points)
        run.pop("config_json", None)
        return {"run": run, "metrics": m, "ticks": ticks[::step], "events": events[-300:]}

    @app.get("/api/runs/{run_id}/csv")
    def run_csv(run_id: int):
        out = resolve_path(cfg.audit.log_dir) / "runs" / f"run_{run_id:04d}.csv"
        AuditRecorder.export_csv(db_path, run_id, out)
        return FileResponse(out, filename=out.name, media_type="text/csv")

    @app.get("/api/runs/{run_id}/events.csv")
    def run_events_csv(run_id: int):
        out = resolve_path(cfg.audit.log_dir) / "runs" / f"run_{run_id:04d}.csv"
        AuditRecorder.export_csv(db_path, run_id, out)
        ev = out.with_name(out.stem + "_events.csv")
        return FileResponse(ev, filename=ev.name, media_type="text/csv")

    @app.post("/api/validate")
    def validate(background: BackgroundTasks, body: Optional[dict] = None):
        if hub.validation["running"]:
            return {"started": False, "reason": "already running"}
        ids = (body or {}).get("ids") or None

        def job():
            from ..sim.runner import run_validation
            hub.validation["running"] = True
            try:
                hub.validation["report"] = run_validation(ids, cfg.copy(), plot=True, record=True)
            finally:
                hub.validation["running"] = False

        threading.Thread(target=job, daemon=True).start()
        return {"started": True}

    @app.get("/api/validation")
    def validation():
        rep = hub.validation["report"]
        if rep is None:
            p = resolve_path(cfg.audit.log_dir) / "validation" / "report.json"
            if p.is_file():
                rep = json.loads(p.read_text())
        return {"running": hub.validation["running"], "report": rep}

    @app.get("/api/validation/plot/{sid}.png")
    def validation_plot(sid: str):
        p = resolve_path(cfg.audit.log_dir) / "validation" / f"{sid}.png"
        if not p.is_file() or "/" in sid:
            raise HTTPException(404)
        return FileResponse(p, media_type="image/png")

    @app.get("/api/commands")
    def commands():
        return list(hub.cmd_log)

    # ------------------------------------------------------------ task launcher
    @app.get("/api/tasks")
    def tasks():
        return {"tasks": task_list(), "jobs": hub.jobs.list()}

    @app.post("/api/tasks/run")
    def task_run(body: dict):
        sub_id = str(body.get("id", ""))
        values = body.get("params") or {}
        try:
            sub = find_task(sub_id)
            if sub.kind == "command":
                _, clean = sub.resolve(values)
                payload = {k: v for k, v in sub.command.items() if k != "target"}
                ok = hub.send(sub.command["target"], {**payload, **clean})
                job = hub.jobs.record(sub, clean, ok, f"{payload['cmd']} {clean} -> {sub.command['target']} "
                                                       f"({hub.addr[sub.command['target']]}): {'sent' if ok else 'send failed'}")
            else:
                job = hub.jobs.start(sub_id, values)
        except KeyError as exc:
            raise HTTPException(404, str(exc))
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(400, str(exc))
        return job.summary()

    @app.get("/api/jobs")
    def jobs():
        return hub.jobs.list()

    @app.get("/api/jobs/{job_id}")
    def job_detail(job_id: int, since: int = 0):
        job = hub.jobs.get(job_id)
        if job is None:
            raise HTTPException(404, "job not found")
        return {**job.summary(), "log": job.tail(since)}

    @app.post("/api/jobs/{job_id}/stop")
    def job_stop(job_id: int):
        return {"stopped": hub.jobs.stop(job_id)}

    @app.get("/api/artifact")
    def artifact(path: str):
        try:
            p = repo_file(path)
        except FileNotFoundError:
            raise HTTPException(404, "not found")
        media = {".png": "image/png", ".md": "text/markdown; charset=utf-8", ".csv": "text/csv",
                 ".json": "application/json", ".stl": "model/stl"}.get(p.suffix.lower(), "application/octet-stream")
        return FileResponse(p, media_type=media, filename=p.name if p.suffix.lower() in (".csv", ".stl") else None)

    import atexit
    atexit.register(hub.jobs.stop_all)         # never leave launcher child processes behind

    return app


def main(cfg: AppConfig, open_browser: bool = False) -> None:
    import uvicorn
    app = create_app(cfg)
    print(f"[dashboard] http://{cfg.dashboard.host}:{cfg.dashboard.port}  (telemetry udp/{cfg.network.telemetry_port}, "
          f"preview udp/{cfg.network.preview_port})")
    if open_browser:
        import webbrowser
        url = f"http://127.0.0.1:{cfg.dashboard.port}/#tasks"
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
        print(f"[dashboard] task launcher: {url}")
    uvicorn.run(app, host=cfg.dashboard.host, port=cfg.dashboard.port, log_level="warning")
