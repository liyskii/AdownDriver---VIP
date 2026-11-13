#!/usr/bin/env python3
"""
Evidence pack for the final submission (task T9.1, documentation).

    python tools/evidence_pack.py            # tests + 16-scenario SIL -> logs/evidence/EVIDENCE.md
    python tools/evidence_pack.py --no-sil   # tests only, reuse the last SIL report

Collects, in one Markdown file that can be pasted into the report:
  * environment (OS, Python, key package versions, git commit if available),
  * unit-test results per file (pytest JUnit XML),
  * SIL validation summary (metrics M1-M5 and every scenario),
  * SHA-256 of the configuration and of every source file, so the numbers can be
    traced back to exactly this code.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import platform
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
OUT = ROOT / "logs" / "evidence"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def versions() -> list:
    rows = [("OS", f"{platform.system()} {platform.release()} ({platform.machine()})"),
            ("Python", platform.python_version())]
    for mod in ("numpy", "cv2", "fastapi", "yaml", "mediapipe", "rclpy"):
        try:
            m = importlib.import_module(mod)
            rows.append((mod, getattr(m, "__version__", "installed")))
        except ImportError:
            rows.append((mod, "not installed on this machine"))
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True,
                                text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = ""
    rows.append(("git commit", commit or "n/a (exported copy)"))
    return rows


def run_tests() -> tuple[list, dict]:
    OUT.mkdir(parents=True, exist_ok=True)
    xml = OUT / "pytest.xml"
    print("[evidence] running unit tests ...", flush=True)
    rc = subprocess.run([sys.executable, "-m", "pytest", "-q", "tests", f"--junitxml={xml}"], cwd=ROOT).returncode
    per_file: dict = {}
    total = {"tests": 0, "failed": 0, "time": 0.0, "rc": rc}
    if xml.is_file():
        for case in ET.parse(xml).getroot().iter("testcase"):
            f = (case.get("classname") or "?").split(".")[-1] + ".py"
            d = per_file.setdefault(f, {"n": 0, "fail": 0})
            d["n"] += 1
            if case.find("failure") is not None or case.find("error") is not None:
                d["fail"] += 1
                total["failed"] += 1
            total["tests"] += 1
            total["time"] += float(case.get("time") or 0)
    return sorted(per_file.items()), total


def run_sil(fresh: bool) -> dict:
    rep_path = ROOT / "logs" / "validation" / "report.json"
    if fresh:
        print("[evidence] running the 16 SIL scenarios ...", flush=True)
        from adowndriver.sim.runner import run_validation
        return run_validation(None, None, plot=True, record=True)
    if rep_path.is_file():
        return json.loads(rep_path.read_text())
    raise SystemExit("no SIL report yet - run without --no-sil")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-sil", action="store_true", help="reuse logs/validation/report.json")
    a = ap.parse_args()
    t0 = time.time()
    tests, ttot = run_tests()
    sil = run_sil(not a.no_sil)

    L = ["# AdownDriver - evidence pack", "",
         f"Generated {time.strftime('%Y-%m-%d %H:%M:%S')} by `tools/evidence_pack.py` (Team #Matigoat).", ""]
    L += ["## 1. Environment", "", "| Item | Value |", "|---|---|"]
    L += [f"| {k} | {v} |" for k, v in versions()]
    ok_t = ttot["tests"] - ttot["failed"]
    L += ["", f"## 2. Unit tests - {ok_t}/{ttot['tests']} passed ({ttot['time']:.1f} s)", "",
          "| Test file | Tests | Failed |", "|---|---|---|"]
    L += [f"| {f} | {d['n']} | {d['fail']} |" for f, d in tests]
    L += ["", f"## 3. Software-in-the-loop validation - {sil['passed']}/{sil['total']} scenarios passed", "",
          "| Metric | Value | Target | Result |", "|---|---|---|---|"]
    for k, v in sil["aggregate"].items():
        if k.startswith("_"):
            continue
        res = {True: "PASS", False: "FAIL", None: "n/a"}[v["pass"]]
        L.append(f"| {k} | {v['value']} | {v['target']} | {res} |")
    L += ["", "| ID | Scenario | Final state | Max abs e (cm) | Result |", "|---|---|---|---|---|"]
    for s in sil["scenarios"]:
        L.append(f"| {s['id']} | {s['name']} | {s['final']} | {s.get('max_abs_e_cm', '-')} | "
                 f"{'PASS' if s['passed'] else 'FAIL'} |")
    L += ["", "## 4. Traceability (SHA-256, first 16 hex)", "", "| File | SHA-256 |", "|---|---|"]
    files = [ROOT / "config" / "default.yaml"] + sorted((ROOT / "adowndriver").rglob("*.py"))
    L += [f"| `{p.relative_to(ROOT).as_posix()}` | `{sha(p)}` |" for p in files]
    L += ["", "## 5. Not covered by this pack", "",
          "Arena trials on the real TurtleBot, the lane homography calibration and the Node 1 frame rate on the "
          "Raspberry Pi are measured on hardware with the Trials tab (`logs/runs/`) and are reported separately.",
          "", f"_Built in {time.time() - t0:.0f} s._", ""]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "EVIDENCE.md").write_text("\n".join(L), encoding="utf-8")
    print(f"[evidence] wrote {OUT / 'EVIDENCE.md'}")
    sys.exit(0 if ttot["failed"] == 0 and ttot["rc"] == 0 and sil["passed"] == sil["total"] else 1)


if __name__ == "__main__":
    main()
