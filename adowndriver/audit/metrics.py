"""
The five pass/fail acceptance metrics of the specification, computed from a
logged run (ticks + events), so live arena trials and simulated trials are
scored by exactly the same code.

M1  Drowsiness detection F1 >= 0.90         (episode level, needs ground-truth labels)
M2  Blink false-alarm rate = 0 %            (no driver-caused slow-down/stop outside drowsy episodes)
M3  Processing lag <= 100 ms per frame       (p95 of Node 1 and Node 2 frame processing)
M4  Lane centring during stop within +/-5 cm (max |e_axle| while CONTROLLED_STOP and v > 0)
M5  Obstacle / disconnect braking <= 200 ms  (fault onset -> zero velocity command)
"""
from __future__ import annotations

import json
from typing import Iterable, Optional

import numpy as np

DROWSY_REASONS = {"EYE_CLOSURE", "YAWN", "HIGH_PERCLOS", "SUSTAINED_EYE_CLOSURE", "REPEATED_YAWNING",
                  "PERSISTENT_DROWSINESS"}
POSITIVE_GT = {"drowsy", "yawn"}
SLOW_STATES = {"SPEED_REDUCTION", "CONTROLLED_STOP"}
ALERT_STATES = {"WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP"}

TARGETS = {
    "f1": (">=", 0.90), "blink_false_alarms": ("==", 0), "lag_p95_ms": ("<=", 100.0),
    "lane_dev_stop_cm": ("<=", 5.0), "brake_latency_ms": ("<=", 200.0),
}


def _episodes(flags: list[bool], times: list[float]) -> list[tuple[float, float]]:
    eps, start = [], None
    for f, t in zip(flags, times):
        if f and start is None:
            start = t
        elif not f and start is not None:
            eps.append((start, t))
            start = None
    if start is not None:
        eps.append((start, times[-1]))
    return eps


def detection_scores(ticks: list[dict], grace_ms: float = 2500.0) -> dict:
    labelled = [t for t in ticks if t.get("gt")]
    if not labelled:
        return {"tp": 0, "fp": 0, "fn": 0, "precision": None, "recall": None, "f1": None,
                "blink_false_alarms": None}
    times = [float(t["t_ms"]) for t in labelled]
    gt_eps = _episodes([t["gt"] in POSITIVE_GT for t in labelled], times)
    det_eps = _episodes([t["state"] in ALERT_STATES and t.get("reason") in DROWSY_REASONS for t in labelled], times)
    slow_eps = _episodes([t["state"] in SLOW_STATES and t.get("reason") in DROWSY_REASONS for t in labelled], times)

    def overlaps(a, b, extend):
        return a[0] <= b[1] + extend and b[0] <= a[1] + extend

    tp = sum(1 for g in gt_eps if any(overlaps(g, d, grace_ms) for d in det_eps))
    fn = len(gt_eps) - tp
    fp = sum(1 for d in det_eps if not any(overlaps(g, d, grace_ms) for g in gt_eps))
    false_slow = sum(1 for d in slow_eps if not any(overlaps(g, d, grace_ms) for g in gt_eps))
    precision = tp / (tp + fp) if tp + fp else (1.0 if not gt_eps else 0.0)
    recall = tp / (tp + fn) if tp + fn else 1.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "precision": round(precision, 3), "recall": round(recall, 3),
            "f1": round(f1, 3) if gt_eps else None, "blink_false_alarms": false_slow}


def lag_stats(ticks: list[dict]) -> dict:
    out = {}
    for key in ("node1_proc_ms", "node2_proc_ms", "rtt_ms", "link_age_ms"):
        vals = np.array([t[key] for t in ticks if t.get(key) is not None], dtype=float)
        vals = vals[np.isfinite(vals)]
        if vals.size:
            out[key] = {"mean": round(float(vals.mean()), 2), "p95": round(float(np.percentile(vals, 95)), 2),
                        "max": round(float(vals.max()), 2)}
    p95 = [out[k]["p95"] for k in ("node1_proc_ms", "node2_proc_ms") if k in out]
    out["lag_p95_ms"] = max(p95) if p95 else None
    return out


def lane_dev_during_stop(ticks: list[dict]) -> Optional[float]:
    vals = [abs(t["e_axle_cm"]) for t in ticks
            if t.get("state") == "CONTROLLED_STOP" and (t.get("v_cmd") or 0) > 0 and t.get("e_axle_cm") is not None]
    return round(max(vals), 2) if vals else None


def brake_latency(events: list[dict]) -> Optional[float]:
    lat = []
    for e in events:
        if e.get("kind") == "BRAKE":
            try:
                lat.append(float(json.loads(e["detail"])["latency_ms"]))
            except (ValueError, KeyError, TypeError):
                pass
    return round(max(lat), 1) if lat else None


def evaluate(value, target) -> Optional[bool]:
    if value is None:
        return None
    op, ref = target
    return {">=": value >= ref, "<=": value <= ref, "==": value == ref}[op]


def compute_metrics(ticks: list[dict], events: list[dict]) -> dict:
    det = detection_scores(ticks)
    lag = lag_stats(ticks)
    values = {
        "f1": det["f1"],
        "blink_false_alarms": det["blink_false_alarms"],
        "lag_p95_ms": lag.get("lag_p95_ms"),
        "lane_dev_stop_cm": lane_dev_during_stop(ticks),
        "brake_latency_ms": brake_latency(events),
    }
    result = {k: {"value": v, "target": f"{TARGETS[k][0]} {TARGETS[k][1]}", "pass": evaluate(v, TARGETS[k])}
              for k, v in values.items()}
    return {"metrics": result, "detection": det, "latency": lag, "n_ticks": len(ticks), "n_events": len(events)}


def aggregate(results: Iterable[dict]) -> dict:
    """Combine several runs: pool TP/FP/FN, worst-case for the others."""
    tp = fp = fn = 0
    worst = {"blink_false_alarms": None, "lag_p95_ms": None, "lane_dev_stop_cm": None, "brake_latency_ms": None}
    for r in results:
        d = r["detection"]
        tp, fp, fn = tp + d["tp"], fp + d["fp"], fn + d["fn"]
        for k in worst:
            v = r["metrics"][k]["value"]
            if v is not None:
                worst[k] = v if worst[k] is None else (worst[k] + v if k == "blink_false_alarms" else max(worst[k], v))
    p = tp / (tp + fp) if tp + fp else 0.0
    rc = tp / (tp + fn) if tp + fn else 0.0
    f1 = round(2 * p * rc / (p + rc), 3) if p + rc else (None if tp + fn == 0 else 0.0)
    values = {"f1": f1, **worst}
    return {k: {"value": v, "target": f"{TARGETS[k][0]} {TARGETS[k][1]}", "pass": evaluate(v, TARGETS[k])}
            for k, v in values.items()} | {"_counts": {"tp": tp, "fp": fp, "fn": fn}}
