"""
Software-in-the-loop validation runner.

Runs the *real* Node 2 control stack (lane detector on rendered images, FSM,
profiler, PID) against the simulated track, ToF sensor, Wi-Fi link and a
scripted driver, for each of the 16 scenarios, then scores the five acceptance
metrics with the same code used for live arena runs.

    python -m adowndriver validate                 # all 16, report in logs/validation/
    python -m adowndriver validate S08 S14 --plot   # selected scenarios with plots
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from ..audit.metrics import aggregate, compute_metrics
from ..audit.recorder import AuditRecorder
from ..common.protocol import DriverPacket
from ..config import AppConfig, resolve_path
from ..vehicle.core import StepInput, VehicleCore
from .scenarios import SCENARIOS, Scenario
from .synthetic_driver import SyntheticDriver
from .world import SimWorld


@dataclass
class ScenarioResult:
    scenario: Scenario
    passed: bool
    checks: list
    metrics: dict
    states_seen: list
    reasons_seen: list
    final_state: str
    lane_dev_true_cm: Optional[float]
    min_clearance_cm: Optional[float]
    max_abs_e_true_cm: float
    ticks: list = field(default_factory=list)
    trace: list = field(default_factory=list)
    events: list = field(default_factory=list)
    wall_s: float = 0.0


def run_scenario(sc: Scenario, cfg: Optional[AppConfig] = None, seed: int = 7,
                 recorder: Optional[AuditRecorder] = None, dt: float = 1 / 30) -> ScenarioResult:
    cfg = (cfg or AppConfig()).copy()
    world = SimWorld(cfg.camera, cfg.lane, gaps=sc.gaps, seed=seed)
    driver = SyntheticDriver(cfg.driver, sc.timeline, seed=seed)
    core = VehicleCore(cfg)
    if recorder:
        recorder.start_run(scenario=sc.id, driver="synthetic", notes=sc.name, source="sim", config=cfg.to_dict())

    t, last_rx, link_down_until = 0.0, -1.0, -1.0
    pending = sorted(sc.actions, key=lambda a: a[0])
    commands_at = [(0.6, "start")]
    ticks, trace, events = [], [], []
    states, reasons = [], []
    dev_stop, max_abs_e = [], 0.0
    min_clear = None
    t_wall = time.perf_counter()
    seq = 0

    while t < sc.duration:
        # ---- scripted actions --------------------------------------------
        while pending and pending[0][0] <= t:
            _, action, arg = pending.pop(0)
            if action == "brightness":
                world.brightness = arg
            elif action == "glare":
                world.glare = arg
            elif action == "obstacle":
                world.place_obstacle_ahead(arg)
            elif action == "clear_obstacles":
                world.obstacles.clear()
            elif action == "link_down":
                link_down_until = t + arg
            elif action == "cmd":
                commands_at.append((t, arg))
        cmds = tuple(c for tt, c in commands_at if tt <= t)
        commands_at = [(tt, c) for tt, c in commands_at if tt > t]

        # ---- Node 1 (synthetic) + network -----------------------------------
        sample = driver.sample(t)
        if t >= link_down_until:
            seq += 1
            last_rx = t
            pkt = DriverPacket(seq=seq, face=sample["face"], ear=sample["ear"], mar=sample["mar"],
                               ear_raw=sample["ear_raw"], ear_th=sample["ear_th"], c_driver=sample["c_driver"],
                               yaw_deg=sample["yaw_deg"], proc_ms=-1.0, fps=30.0, gt=sample["gt"])
            drv_pkt = pkt
        age = (t - last_rx) + 0.008 if last_rx >= 0 else float("inf")

        # ---- Node 2 ------------------------------------------------------
        frame = world.render()
        d = world.tof_cm()
        out = core.step(StepInput(dt=dt, t_ms=int(t * 1000), frame=frame, d_obs_cm=d, tof_age_s=0.0,
                                  driver=drv_pkt if last_rx >= 0 else None, driver_age_s=age,
                                  rtt_ms=12.0, commands=cmds))
        world.step(out.v_cmd, out.omega, dt)

        e_true, _ = world.track.lateral_error(world.robot.x, world.robot.y)
        row = dict(out.tick_row)
        row["gt"] = sample["gt"]
        ticks.append(row)
        trace.append({"t": round(t, 3), "state": out.fsm.state.value, "v": out.v_cmd, "omega": out.omega,
                      "ear": sample["ear"], "ear_th": sample["ear_th"], "mar": sample["mar"],
                      "c_driver": sample["c_driver"], "c_lane": out.lane.c_lane, "e_true_cm": e_true * 100,
                      "e_est_cm": out.tick_row["e_axle_cm"], "d": d, "gt": sample["gt"]})
        for k, detail in out.events:
            events.append({"t_ms": int(t * 1000), "kind": k, "detail": detail, "state": out.fsm.state.value,
                           "reason": out.fsm.reason})
            if recorder:
                recorder.event(k, detail, out.fsm.state.value, out.fsm.reason)
        if recorder:
            recorder.tick(row)
        st = out.fsm.state.value
        if not states or states[-1] != st:
            states.append(st)
        if out.fsm.reason not in reasons:
            reasons.append(out.fsm.reason)
        if t > 3.0 and out.v_cmd > 0:
            max_abs_e = max(max_abs_e, abs(e_true) * 100)
        if st == "CONTROLLED_STOP" and out.v_cmd > 0 or (st == "CONTROLLED_STOP" and world.robot.v > 0.005):
            dev_stop.append(abs(e_true) * 100)
        if world.obstacles:
            c = world.min_clearance_cm()
            min_clear = c if min_clear is None else min(min_clear, c)
        t += dt

    m = compute_metrics(ticks, events)
    lane_true = round(max(dev_stop), 2) if dev_stop else None
    if lane_true is not None:
        m["metrics"]["lane_dev_stop_cm"]["value"] = lane_true
        m["metrics"]["lane_dev_stop_cm"]["pass"] = lane_true <= 5.0

    checks = []
    for s in sc.expect_reach:
        checks.append((f"reaches {s}", s in states))
    for s in sc.expect_never:
        checks.append((f"never {s}", s not in states))
    if sc.expect_final:
        checks.append((f"ends in {sc.expect_final}", states[-1] == sc.expect_final))
    if sc.expect_reason:
        checks.append((f"reason {sc.expect_reason}", sc.expect_reason in reasons))
    if min_clear is not None:
        checks.append(("no contact with obstacle", min_clear > 0.0))
    checks.append(("stays within lane (|e| < 10 cm)", max_abs_e < 10.0))
    for key in ("lane_dev_stop_cm", "brake_latency_ms", "blink_false_alarms"):
        p = m["metrics"][key]["pass"]
        if p is not None and (key != "blink_false_alarms" or sc.category in ("driver", "lighting")):
            checks.append((f"{key} {m['metrics'][key]['target']} (got {m['metrics'][key]['value']})", p))
    passed = all(ok for _, ok in checks)
    if recorder:
        recorder.stop_run({"passed": passed, "metrics": {k: v["value"] for k, v in m["metrics"].items()}})
    return ScenarioResult(sc, passed, checks, m, states, reasons, states[-1], lane_true, min_clear,
                          round(max_abs_e, 2), ticks, trace, events, time.perf_counter() - t_wall)


def plot_result(res: ScenarioResult, out_png: Path) -> Optional[Path]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    order = ["STANDBY", "NORMAL_CRUISE", "WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP", "EMERGENCY_STOP"]
    tr = res.trace
    t = [r["t"] for r in tr]
    fig, ax = plt.subplots(4, 1, figsize=(10, 8.5), sharex=True)
    ax[0].plot(t, [r["ear"] for r in tr], label="EAR (EMA)", color="#2563eb")
    ax[0].plot(t, [r["ear_th"] for r in tr], "--", color="#2563eb", alpha=.5, label="TH_EAR")
    ax[0].plot(t, [r["mar"] for r in tr], label="MAR (EMA)", color="#d97706")
    ax[0].axhline(0.60, ls="--", color="#d97706", alpha=.5)
    ax[0].plot(t, [r["c_driver"] for r in tr], label="C_driver", color="#64748b", lw=1)
    ax[0].legend(loc="upper right", fontsize=8, ncol=4)
    ax[0].set_ylabel("driver")
    ax[1].step(t, [order.index(r["state"]) for r in tr], where="post", color="#111827")
    ax[1].set_yticks(range(len(order)))
    ax[1].set_yticklabels([o.replace("_", " ").title() for o in order], fontsize=7)
    ax[1].set_ylabel("FSM")
    ax[2].plot(t, [r["v"] for r in tr], color="#16a34a", label="v_cmd (m/s)")
    ax2 = ax[2].twinx()
    ax2.plot(t, [r["d"] for r in tr], color="#9333ea", alpha=.5, label="d_obs (cm)")
    ax2.set_ylim(0, 120)
    ax[2].set_ylabel("v (m/s)")
    ax2.set_ylabel("d_obs (cm)")
    ax[3].plot(t, [r["e_true_cm"] for r in tr], color="#dc2626", label="true lateral error")
    ax[3].plot(t, [r["e_est_cm"] for r in tr], color="#dc2626", alpha=.4, lw=1, label="estimated (camera)")
    ax[3].axhspan(-5, 5, color="#16a34a", alpha=.08)
    ax[3].set_ylabel("e_y (cm)")
    ax[3].set_xlabel("time (s)")
    ax[3].legend(loc="upper right", fontsize=8)
    for a in ax:
        a.grid(alpha=.25)
    fig.suptitle(f"{res.scenario.id} - {res.scenario.name}  [{'PASS' if res.passed else 'FAIL'}]")
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    return out_png


def run_validation(ids: Optional[list] = None, cfg: Optional[AppConfig] = None, plot: bool = False,
                   record: bool = True, out_dir: Optional[Path] = None) -> dict:
    cfg = cfg or AppConfig()
    out_dir = out_dir or resolve_path(cfg.audit.log_dir) / "validation"
    out_dir.mkdir(parents=True, exist_ok=True)
    recorder = AuditRecorder(resolve_path(cfg.audit.log_dir), cfg.audit.db_name, tick_hz=cfg.audit.tick_log_hz) \
        if record else None
    chosen = [s for s in SCENARIOS if not ids or s.id in ids]
    results = []
    print("=" * 92)
    print("  AdownDriver software-in-the-loop validation (real lane detector + FSM + controller)")
    print("=" * 92)
    for sc in chosen:
        r = run_scenario(sc, cfg, recorder=recorder)
        results.append(r)
        flag = "PASS" if r.passed else "FAIL"
        print(f"[{flag}] {sc.id} {sc.name:<52} {' -> '.join(s[:4] for s in r.states_seen):<40} ({r.wall_s:.1f}s)")
        for name, ok in r.checks:
            if not ok:
                print(f"        x {name}")
        if plot:
            plot_result(r, out_dir / f"{sc.id}.png")
    agg = aggregate([r.metrics for r in results])
    print("-" * 92)
    for k, v in agg.items():
        if k.startswith("_"):
            continue
        p = {True: "PASS", False: "FAIL", None: "n/a "}[v["pass"]]
        print(f"  [{p}] {k:<22} = {v['value']!s:<8} target {v['target']}")
    n_pass = sum(r.passed for r in results)
    print(f"  Scenarios passed: {n_pass}/{len(results)}")
    print("=" * 92)
    report = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "scenarios": [{"id": r.scenario.id, "name": r.scenario.name, "passed": r.passed,
                       "states": r.states_seen, "final": r.final_state,
                       "checks": [{"check": n, "ok": ok} for n, ok in r.checks],
                       "lane_dev_true_cm": r.lane_dev_true_cm, "min_clearance_cm": r.min_clearance_cm,
                       "max_abs_e_cm": r.max_abs_e_true_cm,
                       "metrics": {k: v["value"] for k, v in r.metrics["metrics"].items()}} for r in results],
        "aggregate": agg, "passed": n_pass, "total": len(results),
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=2, default=str))
    _write_markdown(report, out_dir / "REPORT.md", plot)
    if recorder:
        recorder.close()
    return report


def _write_markdown(rep: dict, path: Path, plots: bool) -> None:
    lines = ["# AdownDriver - SIL validation report", "", f"Generated {rep['generated']} - "
             f"**{rep['passed']}/{rep['total']} scenarios passed**", "", "## Acceptance metrics", "",
             "| Metric | Value | Target | Result |", "|---|---|---|---|"]
    for k, v in rep["aggregate"].items():
        if k.startswith("_"):
            continue
        res = {True: "PASS", False: "FAIL", None: "n/a"}[v["pass"]]
        lines.append(f"| {k} | {v['value']} | {v['target']} | {res} |")
    lines += ["", "## Scenarios", "", "| ID | Scenario | States | Lane dev. in stop (cm) | Result |",
              "|---|---|---|---|---|"]
    for s in rep["scenarios"]:
        lines.append(f"| {s['id']} | {s['name']} | {' → '.join(s['states'])} | {s['lane_dev_true_cm'] or '-'} | "
                     f"{'PASS' if s['passed'] else 'FAIL'} |")
    if plots:
        lines += ["", "## Plots", ""] + [f"![{s['id']}]({s['id']}.png)" for s in rep["scenarios"]]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
