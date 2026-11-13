"""
Job manager for the task launcher: runs catalogue sub-tasks as child processes,
captures their output line by line and keeps the last jobs in memory.

Only argv built from the catalogue is ever executed:
    ["node2", "--sim"]          -> python -u -m adowndriver node2 --sim
    ["@pytest", "-q", "tests"]  -> python -u -m pytest -q tests
    ["#tools/x.py", ...]        -> python -u tools/x.py ...
"""
from __future__ import annotations

import collections
import itertools
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from ..config import REPO_ROOT
from .catalog import TASK_OF, SubTask, find

MAX_LINES = 5000
MAX_JOBS = 40


def build_argv(step: list) -> list:
    py = [sys.executable, "-u"]
    head, rest = step[0], list(step[1:])
    if head == "@pytest":
        return py + ["-m", "pytest", *rest]
    if head.startswith("#"):
        script = (REPO_ROOT / head[1:]).resolve()
        if REPO_ROOT not in script.parents:
            raise ValueError("script outside the repository")
        return py + [str(script), *rest]
    return py + ["-m", "adowndriver", *step]


class Job:
    _ids = itertools.count(1)

    def __init__(self, sub: SubTask, steps: list, values: dict):
        self.id = next(Job._ids)
        self.sub = sub
        self.steps = steps
        self.values = values
        self.status = "starting"          # starting | running | ok | failed | stopped
        self.rc: Optional[int] = None
        self.t_start = time.time()
        self.t_end: Optional[float] = None
        self.lines: collections.deque = collections.deque(maxlen=MAX_LINES)
        self.seq = 0
        self.procs: list = []
        self.stop_requested = False
        self.lock = threading.Lock()

    # ------------------------------------------------------------- output
    def log(self, text: str, src: str = "") -> None:
        with self.lock:
            self.seq += 1
            self.lines.append((self.seq, src, text.rstrip("\n")))

    def tail(self, since: int = 0) -> list:
        with self.lock:
            return [{"n": n, "src": s, "text": t} for n, s, t in self.lines if n > since]

    # ------------------------------------------------------------- info
    def artifacts(self) -> list:
        out = []
        for pattern in self.sub.artifacts:
            try:
                rel = pattern.format(**self.values)
            except (KeyError, ValueError, IndexError):
                continue
            p = REPO_ROOT / rel
            if p.is_file() and p.stat().st_mtime >= self.t_start - 1:
                out.append(rel)
        return out

    def summary(self) -> dict:
        end = self.t_end or time.time()
        task = TASK_OF[self.sub.id]
        return {"id": self.id, "sub_id": self.sub.id, "title": self.sub.title, "task": task.title,
                "owners": task.owners, "status": self.status, "rc": self.rc, "service": self.sub.service,
                "values": self.values, "started": int(self.t_start * 1000), "elapsed_s": round(end - self.t_start, 1),
                "lines": self.seq, "artifacts": self.artifacts() if self.status in ("ok", "failed", "stopped") else [],
                "commands": [" ".join(build_argv(s)[2:]) for s in self.steps]}


class JobManager:
    def __init__(self, on_change: Optional[Callable[[], None]] = None):
        self.jobs: "collections.OrderedDict[int, Job]" = collections.OrderedDict()
        self.lock = threading.Lock()
        self.on_change = on_change

    # ------------------------------------------------------------- public
    def start(self, sub_id: str, values: Optional[dict] = None) -> Job:
        sub = find(sub_id)
        if sub.kind == "command":
            raise ValueError("command sub-tasks are sent by the dashboard, not run as processes")
        missing = sub.missing()
        if missing:
            raise RuntimeError(f"missing Python packages on this machine: {', '.join(missing)}")
        steps, clean = sub.resolve(values)
        with self.lock:
            for j in self.jobs.values():
                if j.sub.id == sub_id and j.status in ("starting", "running"):
                    raise RuntimeError(f"{sub_id} is already running (job #{j.id})")
            job = Job(sub, steps, clean)
            self.jobs[job.id] = job
            while len(self.jobs) > MAX_JOBS:
                old = next(iter(self.jobs.values()))
                if old.status in ("starting", "running"):
                    break
                self.jobs.popitem(last=False)
        threading.Thread(target=self._run, args=(job,), daemon=True, name=f"job-{job.id}").start()
        return job

    def record(self, sub: SubTask, values: dict, ok: bool, message: str) -> Job:
        """Log a one-shot command (e.g. calibration request) as a finished job."""
        job = Job(sub, [], values)
        job.log(message, "dashboard")
        job.status, job.rc, job.t_end = ("ok" if ok else "failed"), (0 if ok else 1), time.time()
        with self.lock:
            self.jobs[job.id] = job
        self._changed()
        return job

    def stop(self, job_id: int) -> bool:
        job = self.get(job_id)
        if job is None or job.status not in ("starting", "running"):
            return False
        job.stop_requested = True
        job.log("stop requested by operator", "launcher")
        for p in list(job.procs):
            _terminate(p)
        return True

    def stop_all(self) -> None:
        for j in list(self.jobs.values()):
            self.stop(j.id)

    def get(self, job_id: int) -> Optional[Job]:
        return self.jobs.get(job_id)

    def list(self) -> list:
        return [j.summary() for j in reversed(list(self.jobs.values()))]

    def active(self) -> list:
        return [j.summary() for j in self.jobs.values() if j.status in ("starting", "running")]

    # ------------------------------------------------------------- internals
    def _changed(self) -> None:
        if self.on_change:
            try:
                self.on_change()
            except Exception:  # noqa: BLE001 - UI callback must never break a job
                pass

    def _spawn(self, job: Job, step: list, tag: str) -> subprocess.Popen:
        argv = build_argv(step)
        job.log("$ " + " ".join(["python", *argv[2:]]), tag or "launcher")
        env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8", MPLBACKEND="Agg")
        kw: dict = {}
        if os.name == "nt":
            kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
        else:
            kw["start_new_session"] = True
        p = subprocess.Popen(argv, cwd=str(REPO_ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, env=env, text=True, encoding="utf-8", errors="replace",
                             bufsize=1, **kw)
        job.procs.append(p)
        threading.Thread(target=self._pump, args=(job, p, tag), daemon=True).start()
        return p

    @staticmethod
    def _pump(job: Job, p: subprocess.Popen, tag: str) -> None:
        assert p.stdout is not None
        for line in p.stdout:
            job.log(line, tag)

    def _run(self, job: Job) -> None:
        sub = job.sub
        try:
            job.status = "running"
            self._changed()
            if sub.kind == "parallel":
                procs = [self._spawn(job, s, f"p{i + 1}") for i, s in enumerate(job.steps)]
                while not job.stop_requested and all(p.poll() is None for p in procs):
                    time.sleep(0.2)
                for p in procs:                 # one service died -> stop the others too
                    if p.poll() is None:
                        _terminate(p)
                rcs = [p.wait() for p in procs]
                rc = 0 if job.stop_requested else max((abs(r) for r in rcs), default=0)
            else:
                rc = 0
                for i, step in enumerate(job.steps):
                    if job.stop_requested:
                        break
                    if len(job.steps) > 1:
                        job.log(f"--- step {i + 1}/{len(job.steps)} ---", "launcher")
                    p = self._spawn(job, step, "")
                    rc = p.wait()
                    if rc != 0:
                        break
            job.rc = rc
            if job.stop_requested:
                job.status = "stopped"
            else:
                job.status = "ok" if rc == 0 else "failed"
        except Exception as exc:  # noqa: BLE001
            job.log(f"launcher error: {exc}", "launcher")
            job.status, job.rc = "failed", -1
        finally:
            job.t_end = time.time()
            job.log(f"=== {job.status.upper()} (exit {job.rc}) after {job.t_end - job.t_start:.1f} s ===", "launcher")
            self._changed()


def _terminate(p: subprocess.Popen, grace: float = 4.0) -> None:
    if p.poll() is not None:
        return
    try:
        if os.name == "nt":
            p.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        else:
            os.killpg(p.pid, signal.SIGTERM)
    except (OSError, ValueError):
        p.terminate()

    def killer():
        try:
            p.wait(grace)
        except subprocess.TimeoutExpired:
            p.kill()

    threading.Thread(target=killer, daemon=True).start()


def repo_file(rel: str) -> Path:
    """Resolve a repo-relative artifact path; only logs/, cad_models/ and docs/ are served."""
    p = (REPO_ROOT / rel).resolve()
    allowed = [(REPO_ROOT / d).resolve() for d in ("logs", "cad_models", "docs")]
    if not any(a == p or a in p.parents for a in allowed) or not p.is_file():
        raise FileNotFoundError(rel)
    return p
