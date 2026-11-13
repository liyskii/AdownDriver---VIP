"""
AdownDriver command-line entry point.

    python -m adowndriver node1 [--source 0|sim] [--headless] [--driver NAME]
    python -m adowndriver node2 [--sim]
    python -m adowndriver dashboard
    python -m adowndriver demo                 # dashboard + simulated Node 1 + Node 2 on one laptop
    python -m adowndriver validate [S01 ...] [--plot]
    python -m adowndriver report RUN_ID        # score a recorded run
    python -m adowndriver check                # hardware / dependency self-test
    python -m adowndriver ui                   # task launcher in the browser (all tasks / sub-tasks)
    python -m adowndriver menu                 # the same launcher as a terminal menu (SSH on a Pi)

Every sub-command accepts ``--config file.yaml`` (merged over config/default.yaml).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import sys
import time

from .config import load_config, resolve_path


def _node1(cfg, a):
    from .driver_monitor.node import main
    main(cfg, a.source, a.headless, a.driver)


def _node2(cfg, a):
    from .vehicle.node import main
    main(cfg, sim=a.sim)


def _dashboard(cfg, a):
    from .dashboard.server import main
    main(cfg)


def _demo(cfg, a):
    """Everything on localhost: simulated robot + synthetic (or webcam) driver + dashboard."""
    for sec in ("node1_ip", "node2_ip", "dashboard_ip"):
        setattr(cfg.network, sec, "127.0.0.1")
    cfg.hardware.gpio_enabled = "no"
    cfg.hardware.oled_enabled = "no"
    from .dashboard.server import main as dash
    from .driver_monitor.node import main as n1
    from .vehicle.node import main as n2
    procs = [mp.Process(target=dash, args=(cfg,), daemon=True),
             mp.Process(target=n2, args=(cfg, True), daemon=True),
             mp.Process(target=n1, args=(cfg, a.source, True, "default"), daemon=True)]
    for p in procs:
        p.start()
        time.sleep(0.8)
    print(f"\n  Open http://127.0.0.1:{cfg.dashboard.port}  (Ctrl+C to quit)\n")
    try:
        while all(p.is_alive() for p in procs):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    for p in procs:
        p.terminate()


def _validate(cfg, a):
    from .sim.runner import run_validation
    rep = run_validation(a.ids or None, cfg, plot=a.plot, record=not a.no_record)
    sys.exit(0 if rep["passed"] == rep["total"] else 1)


def _report(cfg, a):
    from .audit.metrics import compute_metrics
    from .audit.recorder import AuditRecorder
    db = resolve_path(cfg.audit.log_dir) / cfg.audit.db_name
    run, ticks, events = AuditRecorder.load_run(db, a.run_id)
    if run is None:
        sys.exit(f"run {a.run_id} not found in {db}")
    m = compute_metrics(ticks, events)
    print(json.dumps({"run": {k: run[k] for k in ("id", "scenario", "source", "driver", "notes")}, **m}, indent=2))
    out = AuditRecorder.export_csv(db, a.run_id, resolve_path(cfg.audit.log_dir) / "runs" / f"run_{a.run_id:04d}.csv")
    print(f"CSV: {out}")


def _check(cfg, a):
    from .hardware.selftest import run_selftest
    run_selftest(cfg, a.role)


def _ui(cfg, a):
    from .dashboard.server import main
    main(cfg, open_browser=not a.no_browser)


def _menu(cfg, a):
    from .launcher.menu import run_menu
    run_menu(cfg)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="adowndriver", description="AdownDriver - drowsiness-aware TurtleBot 3")
    ap.add_argument("--config", help="YAML file merged over config/default.yaml")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("node1", help="cockpit driver-monitoring station")
    p.add_argument("--source", default="webcam", help="webcam index, 'webcam' (config) or 'sim'")
    p.add_argument("--headless", action="store_true", help="no OpenCV window")
    p.add_argument("--driver", default="default", help="driver profile name (config/drivers/NAME.json)")
    p.set_defaults(fn=_node1)
    p = sub.add_parser("node2", help="TurtleBot vehicle node")
    p.add_argument("--sim", action="store_true", help="simulated track, ToF and motors")
    p.set_defaults(fn=_node2)
    sub.add_parser("dashboard", help="operator web dashboard").set_defaults(fn=_dashboard)
    p = sub.add_parser("demo", help="dashboard + simulated nodes on this computer")
    p.add_argument("--source", default="sim", help="'sim' or a webcam index to use your own face")
    p.set_defaults(fn=_demo)
    p = sub.add_parser("validate", help="16-scenario software-in-the-loop validation")
    p.add_argument("ids", nargs="*")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--no-record", action="store_true")
    p.set_defaults(fn=_validate)
    p = sub.add_parser("report", help="score a recorded run and export CSV")
    p.add_argument("run_id", type=int)
    p.set_defaults(fn=_report)
    p = sub.add_parser("check", help="hardware & dependency self-test")
    p.add_argument("--role", choices=["node1", "node2"], default="node2")
    p.set_defaults(fn=_check)
    p = sub.add_parser("ui", help="task launcher: pick any task / sub-task in the browser")
    p.add_argument("--no-browser", action="store_true")
    p.set_defaults(fn=_ui)
    sub.add_parser("menu", help="task launcher as a terminal menu").set_defaults(fn=_menu)
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    a.fn(cfg, a)


if __name__ == "__main__":
    main()
