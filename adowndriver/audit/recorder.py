"""
Structured audit logging (SQLite + CSV).

Revision 1 opened and closed the SQLite database *and* the CSV file on every
tick, which on a microSD card costs several milliseconds per write and stalls
the 30 Hz control loop. Here a background thread batches inserts (WAL mode,
commit every 0.5 s); the control loop only does a non-blocking queue.put().

Schema
------
runs    (id, started_ms, ended_ms, scenario, operator, driver, notes, source, config_json, summary_json)
ticks   (run_id, t_ms, state, reason, v_cmd, omega, ear, mar, ear_th, c_driver, c_lane,
         e_y_cm, e_axle_cm, heading_deg, d_obs_cm, t_closed, perclos, link_age_ms,
         node1_proc_ms, node2_proc_ms, rtt_ms, gt, buzzer, led)
events  (run_id, t_ms, kind, detail, state, reason)
"""
from __future__ import annotations

import csv
import json
import queue
import sqlite3
import threading
import time
from pathlib import Path
from typing import Optional

TICK_COLUMNS = ["t_ms", "state", "reason", "v_cmd", "omega", "ear", "mar", "ear_th", "c_driver", "c_lane",
                "e_y_cm", "e_axle_cm", "heading_deg", "d_obs_cm", "t_closed", "perclos", "link_age_ms",
                "node1_proc_ms", "node2_proc_ms", "rtt_ms", "gt", "buzzer", "led"]

SCHEMA = f"""
CREATE TABLE IF NOT EXISTS runs (
    id INTEGER PRIMARY KEY AUTOINCREMENT, started_ms INTEGER, ended_ms INTEGER,
    scenario TEXT, operator TEXT, driver TEXT, notes TEXT, source TEXT,
    config_json TEXT, summary_json TEXT);
CREATE TABLE IF NOT EXISTS ticks (
    run_id INTEGER, {", ".join(c + (" TEXT" if c in ("state", "reason", "gt", "buzzer", "led") else " REAL") for c in TICK_COLUMNS)});
CREATE TABLE IF NOT EXISTS events (
    run_id INTEGER, t_ms INTEGER, kind TEXT, detail TEXT, state TEXT, reason TEXT);
CREATE INDEX IF NOT EXISTS ix_ticks_run ON ticks(run_id);
CREATE INDEX IF NOT EXISTS ix_events_run ON events(run_id);
"""


class AuditRecorder:
    def __init__(self, log_dir: str | Path, db_name: str = "adown_audit.db", tick_hz: float = 20.0):
        self.dir = Path(log_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "runs").mkdir(exist_ok=True)
        self.db_path = self.dir / db_name
        self.tick_period_ms = 1000.0 / tick_hz if tick_hz > 0 else 0
        self._last_tick_ms = 0.0
        self.q: queue.Queue = queue.Queue(maxsize=20000)
        self.run_id: Optional[int] = None
        self.run_meta: dict = {}
        self.dropped = 0
        with self._connect() as con:
            con.executescript(SCHEMA)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._writer, daemon=True)
        self._thread.start()

    def _connect(self) -> sqlite3.Connection:
        con = sqlite3.connect(self.db_path, timeout=5.0)
        con.execute("PRAGMA journal_mode=WAL")
        con.execute("PRAGMA synchronous=NORMAL")
        return con

    # ------------------------------------------------------------------ #
    def start_run(self, scenario: str = "LIVE", operator: str = "", driver: str = "", notes: str = "",
                  source: str = "live", config: Optional[dict] = None) -> int:
        if self.run_id is not None:
            self.stop_run()
        self.flush()
        with self._connect() as con:
            cur = con.execute(
                "INSERT INTO runs(started_ms, scenario, operator, driver, notes, source, config_json) VALUES (?,?,?,?,?,?,?)",
                (int(time.time() * 1000), scenario, operator, driver, notes, source, json.dumps(config or {})))
            self.run_id = int(cur.lastrowid)
        self.run_meta = {"scenario": scenario, "driver": driver}
        return self.run_id

    def stop_run(self, summary: Optional[dict] = None) -> Optional[int]:
        rid = self.run_id
        if rid is None:
            return None
        self.run_id = None
        self.flush()
        with self._connect() as con:
            con.execute("UPDATE runs SET ended_ms=?, summary_json=? WHERE id=?",
                        (int(time.time() * 1000), json.dumps(summary or {}), rid))
        return rid

    def tick(self, row: dict, force: bool = False) -> None:
        if self.run_id is None:
            return
        t = row.get("t_ms", time.time() * 1000)
        if not force and t - self._last_tick_ms < self.tick_period_ms:
            return
        self._last_tick_ms = t
        self._put(("tick", self.run_id, [row.get(c) for c in TICK_COLUMNS]))

    def event(self, kind: str, detail: str = "", state: str = "", reason: str = "", t_ms: Optional[int] = None) -> None:
        if self.run_id is None:
            return
        self._put(("event", self.run_id, [int(t_ms or time.time() * 1000), kind, detail, state, reason]))

    def _put(self, item) -> None:
        try:
            self.q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def flush(self, timeout: float = 3.0) -> None:
        end = time.monotonic() + timeout
        while not self.q.empty() and time.monotonic() < end:
            time.sleep(0.01)
        time.sleep(0.02)

    def _writer(self) -> None:
        con = self._connect()
        ticks, events = [], []
        last_commit = time.monotonic()
        while not self._stop.is_set() or not self.q.empty():
            try:
                kind, rid, vals = self.q.get(timeout=0.1)
                (ticks if kind == "tick" else events).append([rid] + vals)
            except queue.Empty:
                pass
            if (ticks or events) and (time.monotonic() - last_commit > 0.5 or self.q.empty()):
                try:
                    if ticks:
                        con.executemany(f"INSERT INTO ticks(run_id,{','.join(TICK_COLUMNS)}) VALUES "
                                        f"({','.join('?' * (len(TICK_COLUMNS) + 1))})", ticks)
                    if events:
                        con.executemany("INSERT INTO events VALUES (?,?,?,?,?,?)", events)
                    con.commit()
                except sqlite3.Error as exc:
                    print(f"[audit] write error: {exc}")
                ticks, events = [], []
                last_commit = time.monotonic()
        con.close()

    def close(self) -> None:
        if self.run_id is not None:
            self.stop_run()
        self._stop.set()
        self._thread.join(timeout=3)

    # ------------------------------------------------------------------ #
    # Read side (dashboard / metrics)
    # ------------------------------------------------------------------ #
    @staticmethod
    def list_runs(db_path: Path, limit: int = 200) -> list[dict]:
        if not Path(db_path).is_file():
            return []
        con = sqlite3.connect(db_path, timeout=5.0)
        con.row_factory = sqlite3.Row
        rows = con.execute("SELECT r.*, (SELECT COUNT(*) FROM ticks t WHERE t.run_id=r.id) AS n_ticks, "
                           "(SELECT COUNT(*) FROM events e WHERE e.run_id=r.id) AS n_events "
                           "FROM runs r ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
        con.close()
        out = []
        for r in rows:
            d = dict(r)
            d.pop("config_json", None)
            d["summary"] = json.loads(d.pop("summary_json") or "{}")
            out.append(d)
        return out

    @staticmethod
    def load_run(db_path: Path, run_id: int):
        con = sqlite3.connect(db_path, timeout=5.0)
        con.row_factory = sqlite3.Row
        run = con.execute("SELECT * FROM runs WHERE id=?", (run_id,)).fetchone()
        ticks = [dict(r) for r in con.execute("SELECT * FROM ticks WHERE run_id=? ORDER BY t_ms", (run_id,))]
        events = [dict(r) for r in con.execute("SELECT * FROM events WHERE run_id=? ORDER BY t_ms", (run_id,))]
        con.close()
        return (dict(run) if run else None), ticks, events

    @staticmethod
    def export_csv(db_path: Path, run_id: int, out_path: Path) -> Path:
        run, ticks, events = AuditRecorder.load_run(db_path, run_id)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["run_id"] + TICK_COLUMNS)
            for t in ticks:
                w.writerow([run_id] + [t.get(c) for c in TICK_COLUMNS])
        ev_path = out_path.with_name(out_path.stem + "_events.csv")
        with open(ev_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["run_id", "t_ms", "kind", "detail", "state", "reason"])
            for e in events:
                w.writerow([run_id, e["t_ms"], e["kind"], e["detail"], e["state"], e["reason"]])
        return out_path
