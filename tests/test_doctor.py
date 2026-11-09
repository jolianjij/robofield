"""The self-check runs to the end on any machine (missing optional libraries are notes, not crashes)."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_doctor_runs():
    out = subprocess.run([sys.executable, "tools/doctor.py"], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert "RoboField doctor" in out.stdout and "Traceback" not in out.stdout + out.stderr
    assert "config/field.yaml" in out.stdout
    assert out.returncode in (0, 1)
