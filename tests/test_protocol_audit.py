import time

from adowndriver.audit.metrics import compute_metrics
from adowndriver.audit.recorder import AuditRecorder
from adowndriver.common.protocol import (DriverPacket, decode, decode_preview, driver_from_dict, encode,
                                         encode_preview)


def test_packet_roundtrip_and_size():
    p = DriverPacket(seq=7, ear=0.25, gt="alert")
    raw = encode(p)
    assert len(raw) < 400
    assert driver_from_dict(decode(raw)) == p


def test_decode_garbage():
    assert decode(b"\xff\x00") is None and decode(b"[1,2]") is None


def test_preview_roundtrip():
    assert decode_preview(encode_preview(2, b"jpegdata")) == (2, b"jpegdata")


def test_recorder_and_metrics(tmp_path):
    rec = AuditRecorder(tmp_path, tick_hz=0)
    rid = rec.start_run("S06", source="test")
    t0 = int(time.time() * 1000)
    for i in range(100):
        closed = 30 <= i < 60
        state = "WARNING" if 45 <= i < 70 else "NORMAL_CRUISE"
        rec.tick({"t_ms": t0 + i * 50, "state": state, "reason": "EYE_CLOSURE" if state == "WARNING" else "NONE",
                  "v_cmd": 0.16, "gt": "drowsy" if closed else "alert", "node1_proc_ms": 40, "node2_proc_ms": 8})
    rec.event("BRAKE", '{"latency_ms": 120}')
    rec.stop_run()
    rec.close()
    run, ticks, events = AuditRecorder.load_run(tmp_path / "adown_audit.db", rid)
    assert run["scenario"] == "S06" and len(ticks) == 100 and len(events) == 1
    m = compute_metrics(ticks, events)["metrics"]
    assert m["f1"]["value"] == 1.0 and m["f1"]["pass"]
    assert m["brake_latency_ms"]["value"] == 120.0
    assert m["lag_p95_ms"]["value"] == 40
    out = AuditRecorder.export_csv(tmp_path / "adown_audit.db", rid, tmp_path / "r.csv")
    assert out.read_text().count("\n") == 101
