"""Background jobs for the app: training runs, evaluations and tools, each a separate Python
process whose output streams into the app. A job can print two kinds of machine-readable lines:
"PROGRESS {json}" (a point on the learning curve) and "RESULT {json}" (its final result)."""
from __future__ import annotations

import itertools
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class Job:
    id: int
    kind: str                       # train | eval | tool
    title: str
    args: list
    stop_file: Path | None = None
    proc: subprocess.Popen | None = None
    started: float = field(default_factory=time.time)
    ended: float | None = None
    status: str = "running"         # running | stopping | done | failed | stopped
    returncode: int | None = None
    lines: deque = field(default_factory=lambda: deque(maxlen=3000))
    n_lines: int = 0                # total lines ever received (for incremental reads)
    progress: list = field(default_factory=list)
    result: dict | None = None
    meta: dict = field(default_factory=dict)

    def public(self, since: int | None = None) -> dict:
        d = {"id": self.id, "kind": self.kind, "title": self.title, "status": self.status,
             "started": self.started, "ended": self.ended, "returncode": self.returncode,
             "result": self.result, "meta": self.meta, "n_lines": self.n_lines,
             "progress": self.progress[-1] if self.progress else None}
        if since is not None:
            first = self.n_lines - len(self.lines)
            d["lines"] = list(self.lines)[max(0, since - first):]
            d["curve"] = self.progress
        return d


def _kill_tree(proc: subprocess.Popen):
    if proc.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    else:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


class JobManager:
    def __init__(self):
        self.jobs: dict[int, Job] = {}
        self._ids = itertools.count(1)
        self.lock = threading.Lock()

    def start(self, kind: str, title: str, args: list[str], stop_file: Path | None = None, meta=None) -> Job:
        job = Job(next(self._ids), kind, title, list(args), stop_file, meta=dict(meta or {}))
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        kw = {}
        if os.name == "nt":
            kw["creationflags"] = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kw["start_new_session"] = True
        exe = Path(sys.executable)
        if exe.name.lower() == "pythonw.exe" and exe.with_name("python.exe").exists():
            exe = exe.with_name("python.exe")              # the app may run windowless; its jobs need stdout
        job.proc = subprocess.Popen([str(exe), "-u", *args], cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True,
                                    encoding="utf-8", errors="replace", bufsize=1, **kw)
        with self.lock:
            self.jobs[job.id] = job
        threading.Thread(target=self._read, args=(job,), daemon=True).start()
        return job

    def _read(self, job: Job):
        for raw in job.proc.stdout:
            line = raw.rstrip("\n")
            if line.startswith("PROGRESS "):
                try:
                    job.progress.append(json.loads(line[9:]))
                    continue
                except ValueError:
                    pass
            elif line.startswith("RESULT "):
                try:
                    job.result = json.loads(line[7:])
                    continue
                except ValueError:
                    pass
            job.lines.append(line)
            job.n_lines += 1
        job.returncode = job.proc.wait()
        job.ended = time.time()
        if job.status == "stopping":
            job.status = "stopped"
        else:
            job.status = "done" if job.returncode == 0 else "failed"

    def stop(self, job_id: int) -> tuple[bool, str]:
        job = self.jobs.get(job_id)
        if job is None:
            return False, "no such job"
        if job.status != "running":
            return False, f"job is {job.status}"
        job.status = "stopping"
        if job.stop_file is not None:
            # ask nicely (training saves the policy on its way out), then insist after 90 s
            job.stop_file.touch()

            def insist():
                try:
                    job.proc.wait(90)
                except subprocess.TimeoutExpired:
                    _kill_tree(job.proc)
            threading.Thread(target=insist, daemon=True).start()
            return True, "stopping after the current round (the policy is saved)"
        _kill_tree(job.proc)
        return True, "stopped"

    def running(self, kind: str | None = None) -> list[Job]:
        return [j for j in self.jobs.values() if j.status in ("running", "stopping") and (kind is None or j.kind == kind)]

    def list(self) -> list[dict]:
        return [j.public() for j in sorted(self.jobs.values(), key=lambda j: -j.id)]

    def get(self, job_id: int, since: int = 0) -> dict | None:
        job = self.jobs.get(job_id)
        return job.public(since) if job else None

    def close_all(self):
        for job in list(self.jobs.values()):
            if job.proc and job.proc.poll() is None:
                if job.stop_file is not None:
                    job.stop_file.touch()
                    try:
                        job.proc.wait(20)
                    except subprocess.TimeoutExpired:
                        pass
                _kill_tree(job.proc)
