"""
Task catalogue of the AdownDriver project - the single source of truth for the
task launcher (web "Tasks" tab and the terminal menu).

Every task of the team plan is listed here with its owner(s) and the runnable
sub-tasks that implement it. A sub-task is one of

* ``process`` - a whitelisted command run as a child process (finite job or a
  long-running service such as a node);
* ``parallel`` - several services started together as one job (e.g. the two
  simulated nodes for a laptop demo);
* ``sequence`` - several commands run one after another, stopping at the first
  failure (e.g. the regression pipeline);
* ``command`` - a UDP command sent to a running node (needs the dashboard).

Nothing outside this catalogue can be started from the UI, and parameters are
validated against their declared type before they are substituted into argv.
"""
from __future__ import annotations

import importlib.util
import re
from dataclasses import dataclass, field
from typing import Optional

from ..sim.scenarios import SCENARIOS

SIM = ["--config", "config/sim.yaml"]
_TEXT_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,32}$")


@dataclass
class Param:
    name: str
    label: str
    kind: str = "text"                 # text | int | select
    default: object = ""
    options: list = field(default_factory=list)   # [(value, label)] for select
    lo: int = 0
    hi: int = 10_000

    def coerce(self, raw) -> str:
        if raw is None or raw == "":
            raw = self.default
        if self.kind == "int":
            try:
                v = int(raw)
            except (TypeError, ValueError):
                raise ValueError(f"{self.label}: integer expected")
            if not self.lo <= v <= self.hi:
                raise ValueError(f"{self.label}: must be in {self.lo}..{self.hi}")
            return str(v)
        if self.kind == "select":
            allowed = [str(o[0]) for o in self.options]
            if str(raw) not in allowed:
                raise ValueError(f"{self.label}: choose one of {', '.join(allowed)}")
            return str(raw)
        if not _TEXT_RE.match(str(raw)):
            raise ValueError(f"{self.label}: letters, digits, _ . - only (max 32)")
        return str(raw)

    def to_dict(self) -> dict:
        return {"name": self.name, "label": self.label, "kind": self.kind, "default": self.default,
                "options": [{"value": o[0], "label": o[1]} for o in self.options], "lo": self.lo, "hi": self.hi}


@dataclass
class SubTask:
    id: str
    title: str
    description: str
    kind: str = "process"              # process | parallel | sequence | command
    steps: list = field(default_factory=list)      # list of argv lists (see Launcher.argv)
    params: list = field(default_factory=list)
    service: bool = False              # long-running until stopped
    where: str = "any"                 # any | laptop | node1 | node2
    needs: list = field(default_factory=list)      # python modules required
    artifacts: list = field(default_factory=list)  # repo-relative files produced
    command: Optional[dict] = None     # for kind == "command"
    est_s: int = 0                     # rough duration for finite jobs

    def missing(self) -> list:
        out = []
        for m in self.needs:
            try:
                if importlib.util.find_spec(m) is None:
                    out.append(m)
            except (ImportError, ValueError):
                out.append(m)
        return out

    def resolve(self, values: Optional[dict]) -> tuple[list, dict]:
        """Validate parameters and return (argv lists with values substituted, clean values)."""
        values = values or {}
        clean = {p.name: p.coerce(values.get(p.name)) for p in self.params}
        steps = [[a.format(**clean) for a in step] for step in self.steps]
        return steps, clean

    def to_dict(self) -> dict:
        return {"id": self.id, "title": self.title, "description": self.description, "kind": self.kind,
                "service": self.service, "where": self.where, "needs": self.needs, "missing": self.missing(),
                "params": [p.to_dict() for p in self.params], "artifacts": self.artifacts, "est_s": self.est_s,
                "preview": [" ".join(s) for s in self.steps]}


@dataclass
class Task:
    id: str
    title: str
    owners: list
    module: str
    goal: str
    subtasks: list

    def to_dict(self) -> dict:
        return {"id": self.id, "title": self.title, "owners": self.owners, "module": self.module,
                "goal": self.goal, "subtasks": [s.to_dict() for s in self.subtasks]}


def _pytest(*files: str) -> list:
    return ["@pytest", "-q", *files]


SCENARIO_OPTS = [(s.id, f"{s.id} · {s.name}") for s in SCENARIOS]

TASKS: list[Task] = [
    Task("T1", "Driver monitoring station (Node 1)", ["Andrey Li"], "adowndriver/driver_monitor",
         "Track the driver's face, compute EAR / MAR / head yaw and C_driver, stream 50 Hz heartbeats.", [
             SubTask("T1.1", "Live driver monitoring - webcam",
                     "MediaPipe FaceLandmarker on the Logitech C270 with HUD window; sends packets to Node 2.",
                     steps=[["node1", "--source", "{camera}", "--driver", "{driver}"]],
                     params=[Param("camera", "Webcam index", "int", 0, lo=0, hi=9),
                             Param("driver", "Driver profile", "text", "default")],
                     service=True, where="node1", needs=["mediapipe", "cv2"]),
             SubTask("T1.2", "Synthetic driver (no camera)",
                     "Scripted driver on localhost - drive the behaviour from the Live tab chips.",
                     steps=[[*SIM, "node1", "--source", "sim", "--headless"]], service=True, where="laptop"),
             SubTask("T1.3", "Calibrate driver EAR threshold (5 s)",
                     "Asks the running Node 1 to record 5 s of open eyes and store TH_EAR = 0.72 x EAR_open.",
                     kind="command", command={"target": "node1", "cmd": "calibrate"},
                     params=[Param("driver", "Driver profile", "text", "default")]),
             SubTask("T1.4", "Unit tests - EAR/MAR analyzer",
                     "Blink filter, calibration, PERCLOS, yawn episodes and C_driver.",
                     steps=[_pytest("tests/test_analyzer.py")], est_s=5),
         ]),
    Task("T2", "Road perception & lane keeping (Node 2)", ["Nikita Pak"], "adowndriver/vehicle",
         "Bird's-eye lane detection (yellow left / white right), curvature feed-forward + PID steering.", [
             SubTask("T2.1", "Vehicle node on the TurtleBot",
                     "Pi Camera V2 + VL53L1X + ROS 2 /cmd_vel with the 250 ms motor watchdog.",
                     steps=[["node2"]], service=True, where="node2", needs=["rclpy"]),
             SubTask("T2.2", "Vehicle node on the simulated track",
                     "Real lane detector, FSM and controller on a rendered track; obstacles / Wi-Fi drop from the Live tab.",
                     steps=[[*SIM, "node2", "--sim"]], service=True, where="laptop"),
             SubTask("T2.3", "Lane homography calibration tool",
                     "Click 4 floor dots to calibrate the bird's-eye view (needs a display / VNC).",
                     steps=[["#tools/calibrate_lane.py"]], service=True, where="node2", needs=["cv2"]),
             SubTask("T2.4", "Unit tests - lane detector & steering",
                     "Bird's-eye fit, one-line fallback, glare suppression, FF+PID and velocity profiler.",
                     steps=[_pytest("tests/test_lane.py", "tests/test_control.py")], est_s=10),
         ]),
    Task("T3", "Supervisory safety FSM", ["Li Vladislav", "Arsen Baktybek uulu"], "adowndriver/vehicle/fsm.py",
         "6-state FSM with decoupled C_driver / C_lane, latched vs auto-clear faults and watchdogs.", [
             SubTask("T3.1", "Unit tests - safety FSM",
                     "Blink immunity, graduated stop, de-escalation, faults, START checklist, RESET rules.",
                     steps=[_pytest("tests/test_fsm.py")], est_s=5),
             SubTask("T3.2", "Run one SIL scenario",
                     "Software-in-the-loop run of a single test scenario with a plot of EAR, speed, lane error and state.",
                     steps=[["validate", "{scenario}", "--plot"]],
                     params=[Param("scenario", "Scenario", "select", "S06", options=SCENARIO_OPTS)],
                     artifacts=["logs/validation/{scenario}.png", "logs/validation/REPORT.md"], est_s=15),
         ]),
    Task("T4", "Telemetry link & audit logging", ["Vadim Lifrov", "Husan Tulaev"], "adowndriver/common, adowndriver/audit",
         "UDP heartbeat protocol, SQLite (WAL) + CSV audit of every tick, automatic scoring of runs.", [
             SubTask("T4.1", "Unit tests - protocol & audit",
                     "Packet round-trip, preview frames, recorder and metric scoring.",
                     steps=[_pytest("tests/test_protocol_audit.py")], est_s=5),
             SubTask("T4.2", "Score a recorded run and export CSV",
                     "Computes F1, blink false alarms, lag p95, lane centring and braking latency of one run.",
                     steps=[["report", "{run_id}"]], params=[Param("run_id", "Run #", "int", 1, lo=1, hi=100000)],
                     artifacts=["logs/runs/run_{run_id:0>4}.csv"], est_s=3),
         ]),
    Task("T5", "Operator HMI & laptop demo", ["Hsu Shwe Sin Htun"], "adowndriver/dashboard",
         "Live console, trials, validation, settings - and this task launcher.", [
             SubTask("T5.1", "One-click demo: simulated robot + synthetic driver",
                     "Starts T1.2 and T2.2 together, attached to this dashboard. Open the Live tab and press START.",
                     kind="parallel", steps=[[*SIM, "node2", "--sim"], [*SIM, "node1", "--source", "sim", "--headless"]],
                     service=True, where="laptop"),
             SubTask("T5.2", "Demo with your own face",
                     "Simulated robot driven by a real webcam driver (MediaPipe on the laptop).",
                     kind="parallel",
                     steps=[[*SIM, "node2", "--sim"], [*SIM, "node1", "--source", "{camera}"]],
                     params=[Param("camera", "Webcam index", "int", 0, lo=0, hi=9)],
                     service=True, where="laptop", needs=["mediapipe", "cv2"]),
         ]),
    Task("T6", "Hardware peripherals & bring-up", ["Saidumar Egamberdiev", "Vadim Lifrov"], "adowndriver/hardware, deploy/",
         "VL53L1X, buzzers, RGB LEDs, OLED and buttons; dependency and device self-test before the arena.", [
             SubTask("T6.1", "Self-test - Node 1 (cockpit)", "Python packages, I2C, OLED, GPIO, camera, face backend.",
                     steps=[["check", "--role", "node1"]], est_s=10),
             SubTask("T6.2", "Self-test - Node 2 (TurtleBot)", "Python packages, ROS 2, Pi Camera, VL53L1X, GPIO.",
                     steps=[["check", "--role", "node2"]], est_s=10),
         ]),
    Task("T7", "CAD & 3D-printed mounts", ["Fazliddin Igamberdiev"], "cad_models/",
         "Parametric Pi Camera 25 deg mount, ToF bracket (10 deg up), C270 stand, Node 2 status pod and Node 1 cockpit panel.", [
             SubTask("T7.1", "Regenerate STL parts + preview",
                     "Rebuilds the seven STL files and parts_preview.png; checks watertight / on the bed / fits the 220 x 220 x 250 mm printer.",
                     steps=[["#cad_models/generate_stl.py"]], needs=["manifold3d", "trimesh"],
                     artifacts=["cad_models/parts_preview.png"], est_s=20),
         ]),
    Task("T8", "Experimental validation & QA", ["Dilshod Obidov"], "adowndriver/sim, tests/",
         "16-scenario SIL suite with fault injection, metrics M1-M5 and regression gate.", [
             SubTask("T8.1", "Full SIL validation - 16 scenarios",
                     "Scores every scenario against M1-M5 and writes logs/validation/REPORT.md + plots.",
                     steps=[["validate", "--plot"]], artifacts=["logs/validation/REPORT.md"], est_s=150),
             SubTask("T8.2", "Full unit-test suite", "All pytest tests of the repository.",
                     steps=[_pytest("tests")], est_s=30),
             SubTask("T8.3", "Regression gate: tests -> SIL",
                     "Runs the unit tests, then the 16 scenarios; stops at the first failure.",
                     kind="sequence", steps=[_pytest("tests"), ["validate", "--plot"]],
                     artifacts=["logs/validation/REPORT.md"], est_s=180),
         ]),
    Task("T9", "Documentation & evidence pack", ["Sevinchxon Khoshimova"], "tools/evidence_pack.py, docs/",
         "Collects test results, SIL metrics, configuration and environment into one report for the submission.", [
             SubTask("T9.1", "Build the evidence pack",
                     "Runs tests + SIL and writes logs/evidence/EVIDENCE.md (results, metrics, versions, config hash).",
                     steps=[["#tools/evidence_pack.py"]], artifacts=["logs/evidence/EVIDENCE.md"], est_s=180),
         ]),
]

INDEX: dict[str, SubTask] = {s.id: s for t in TASKS for s in t.subtasks}
TASK_OF: dict[str, Task] = {s.id: t for t in TASKS for s in t.subtasks}


def catalog() -> list:
    return [t.to_dict() for t in TASKS]


def find(sub_id: str) -> SubTask:
    if sub_id not in INDEX:
        raise KeyError(f"unknown sub-task {sub_id}")
    return INDEX[sub_id]
