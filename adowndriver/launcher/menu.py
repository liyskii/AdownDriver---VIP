"""
Terminal task launcher - for an SSH session on a Raspberry Pi where no browser
is at hand. Same catalogue as the web "Tasks" tab.

    python -m adowndriver menu
"""
from __future__ import annotations

import json
import subprocess
import sys
import time

from ..config import REPO_ROOT, AppConfig
from .catalog import TASKS, SubTask
from .jobs import build_argv

BOLD, DIM, GREEN, RED, CYAN, RESET = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[36m", "\033[0m"
if not sys.stdout.isatty():
    BOLD = DIM = GREEN = RED = CYAN = RESET = ""


def _ask(prompt: str, default: str = "") -> str:
    try:
        v = input(f"{prompt}{f' [{default}]' if default != '' else ''}: ").strip()
    except EOFError:
        return "q"
    return v or str(default)


def _header() -> None:
    print(f"\n{BOLD}{CYAN}AdownDriver task launcher{RESET}  {DIM}Team #Matigoat · {REPO_ROOT}{RESET}")
    print(f"{DIM}{'-' * 72}{RESET}")


def _pick_task():
    _header()
    for i, t in enumerate(TASKS, 1):
        print(f"  {BOLD}{i}{RESET}. {t.title:<44} {DIM}{', '.join(t.owners)}{RESET}")
    print(f"  {BOLD}q{RESET}. quit")
    v = _ask("Which task do you want to run?")
    if v.lower() in ("q", "quit", "exit"):
        return None
    if v.isdigit() and 1 <= int(v) <= len(TASKS):
        return TASKS[int(v) - 1]
    for t in TASKS:                          # accept "T3" or "T3.2"
        if v.upper().split(".")[0] == t.id:
            return t
    print(f"{RED}unknown choice{RESET}")
    return False


def _pick_sub(task) -> SubTask | None:
    print(f"\n{BOLD}{task.id} {task.title}{RESET}\n{DIM}{task.goal}{RESET}")
    for i, s in enumerate(task.subtasks, 1):
        miss = s.missing()
        flag = f"{RED}missing {', '.join(miss)}{RESET}" if miss else f"{GREEN}ready{RESET}"
        kind = "service" if s.service else s.kind
        print(f"  {BOLD}{i}{RESET}. {s.id} {s.title:<46} {DIM}{kind:<8}{RESET} {flag}")
    print(f"  {BOLD}b{RESET}. back")
    v = _ask("Which sub-task?")
    if v.isdigit() and 1 <= int(v) <= len(task.subtasks):
        return task.subtasks[int(v) - 1]
    return None


def _run_sub(sub: SubTask, cfg: AppConfig) -> None:
    print(f"\n{DIM}{sub.description}{RESET}")
    values = {p.name: _ask(f"  {p.label}" + (f" ({', '.join(str(o[0]) for o in p.options)})" if p.options else ""),
                           str(p.default)) for p in sub.params}
    try:
        steps, clean = sub.resolve(values)
    except ValueError as exc:
        print(f"{RED}{exc}{RESET}")
        return
    if sub.kind == "command":
        from ..common.protocol import UdpEndpoint
        tgt = sub.command["target"]
        port = cfg.network.node1_cmd_port if tgt == "node1" else cfg.network.node2_cmd_port
        ip = cfg.network.node1_ip if tgt == "node1" else cfg.network.node2_ip
        msg = {"type": "command", "source": "menu", "id": int(time.time() * 1000), "cmd": sub.command["cmd"], **clean}
        ok = UdpEndpoint().send(json.dumps(msg).encode(), ip, port)
        print(f"{GREEN if ok else RED}{'sent' if ok else 'could not send'} {msg['cmd']} to {tgt} ({ip}:{port}){RESET}")
        return
    if sub.missing():
        print(f"{RED}Missing Python packages here: {', '.join(sub.missing())}{RESET}")
        return
    if sub.service:
        print(f"{DIM}Long-running service - press Ctrl+C to stop and return to the menu.{RESET}")
    procs = []
    t0 = time.time()
    rc = 0
    try:
        if sub.kind == "parallel":
            procs = [subprocess.Popen(build_argv(s), cwd=str(REPO_ROOT)) for s in steps]
            while all(p.poll() is None for p in procs):
                time.sleep(0.3)
            rc = max(abs(p.wait()) for p in procs)
        else:
            for s in steps:
                print(f"{CYAN}$ python {' '.join(build_argv(s)[2:])}{RESET}")
                p = subprocess.Popen(build_argv(s), cwd=str(REPO_ROOT))
                procs = [p]
                rc = p.wait()
                if rc:
                    break
    except KeyboardInterrupt:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        print(f"\n{DIM}stopped{RESET}")
        return
    col = GREEN if rc == 0 else RED
    print(f"{col}{'OK' if rc == 0 else 'FAILED'} (exit {rc}) in {time.time() - t0:.1f} s{RESET}")
    for a in sub.artifacts:
        try:
            p = REPO_ROOT / a.format(**clean)
        except (KeyError, ValueError):
            continue
        if p.is_file():
            print(f"  output: {p.relative_to(REPO_ROOT)}")


def run_menu(cfg: AppConfig) -> None:
    while True:
        task = _pick_task()
        if task is None:
            return
        if task is False:
            continue
        sub = _pick_sub(task)
        if sub is not None:
            _run_sub(sub, cfg)
            _ask(f"{DIM}Enter to continue{RESET}")
