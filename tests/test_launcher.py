import time

import pytest
from fastapi.testclient import TestClient

from adowndriver.config import REPO_ROOT
from adowndriver.launcher.catalog import INDEX, TASKS, Param, find
from adowndriver.launcher.jobs import JobManager, build_argv, repo_file


def test_catalogue_is_consistent():
    ids = [s.id for t in TASKS for s in t.subtasks]
    assert len(ids) == len(set(ids)) and len(TASKS) >= 9
    for t in TASKS:
        assert t.owners and all(s.id.startswith(t.id + ".") for s in t.subtasks)
        for s in t.subtasks:
            steps, _ = s.resolve({})                      # defaults are always valid
            for step in steps:
                argv = build_argv(step)
                if step[0].startswith("#"):
                    assert (REPO_ROOT / step[0][1:]).is_file(), step
                assert argv[1] == "-u"


def test_param_validation_blocks_injection():
    p = Param("driver", "Driver", "text", "default")
    assert p.coerce("anna_2") == "anna_2"
    for bad in ("a b", "x;rm -rf", "$(id)", "../etc", "a" * 40):
        with pytest.raises(ValueError):
            p.coerce(bad)
    sel = find("T3.2").params[0]
    assert sel.coerce("S08") == "S08"
    with pytest.raises(ValueError):
        sel.coerce("S99")
    with pytest.raises(ValueError):
        find("T4.2").params[0].coerce("-5")


def test_job_runs_and_captures_output():
    jm = JobManager()
    job = jm.start("T1.4")                               # analyzer unit tests
    for _ in range(300):
        if job.status in ("ok", "failed"):
            break
        time.sleep(0.1)
    assert job.status == "ok", "\n".join(t for _, _, t in job.lines)
    text = "\n".join(t for _, _, t in job.lines)
    assert "passed" in text and job.summary()["rc"] == 0


def test_service_can_be_stopped_and_not_started_twice():
    jm = JobManager()
    job = jm.start("T2.2")                               # simulated vehicle node (runs until stopped)
    time.sleep(1.5)
    with pytest.raises(RuntimeError):
        jm.start("T2.2")
    assert jm.stop(job.id)
    for _ in range(100):
        if job.status == "stopped":
            break
        time.sleep(0.1)
    assert job.status == "stopped"


def test_artifact_paths_are_sandboxed():
    assert repo_file("cad_models/CAD_SPECIFICATIONS.md").name == "CAD_SPECIFICATIONS.md"
    for bad in ("adowndriver/config.py", "../README.md", "logs/../config/default.yaml"):
        with pytest.raises(FileNotFoundError):
            repo_file(bad)


def test_api_endpoints():
    from adowndriver.config import load_config
    from adowndriver.dashboard.server import create_app
    cfg = load_config()
    cfg.network.telemetry_port, cfg.network.preview_port = 0, 0
    with TestClient(create_app(cfg)) as c:
        d = c.get("/api/tasks").json()
        assert len(d["tasks"]) == len(TASKS)
        assert c.post("/api/tasks/run", json={"id": "T9.9"}).status_code == 404
        assert c.post("/api/tasks/run", json={"id": "T3.2", "params": {"scenario": "S99"}}).status_code == 400
        r = c.post("/api/tasks/run", json={"id": "T1.3", "params": {"driver": "anna"}}).json()
        assert r["status"] in ("ok", "failed") and r["sub_id"] == "T1.3"
        assert c.get(f"/api/jobs/{r['id']}").json()["log"]
        r = c.post("/api/tasks/run", json={"id": "T9.2", "params": {"name": "quick_demo"}}).json()
        assert r["values"] == {"name": "quick_demo"}
        assert c.post("/api/tasks/run", json={"id": "T9.2", "params": {"name": "rm -rf"}}).status_code == 400
        assert c.get("/api/artifact", params={"path": "adowndriver/config.py"}).status_code == 404
        assert "Which task or sub-task" in c.get("/").text
    assert set(INDEX) >= {"T1.1", "T5.1", "T8.3", "T8.4", "T9.1", "T9.2"}
