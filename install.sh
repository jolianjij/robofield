#!/usr/bin/env bash
# RoboField setup for macOS and Linux:   bash install.sh
# Makes a private Python environment in .venv, installs every library RoboField uses and checks
# the result. Safe to run again. (The Raspberry Pi tracker has its own: bash pi/install.sh)
set -e
cd "$(dirname "$0")"
echo
echo " RoboField setup"
echo " ==============="
echo

PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  for c in python3.12 python3.11 python3.10 python3; do
    if command -v "$c" >/dev/null 2>&1; then PY="$c"; break; fi
  done
fi
if [ -z "$PY" ] || ! "$PY" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
  echo "Python 3.10 or newer was not found."
  echo "macOS: install it from https://www.python.org/downloads/   Ubuntu/Debian: sudo apt install python3 python3-venv"
  exit 1
fi

if [ ! -x .venv/bin/python ]; then
  echo "Creating the Python environment in .venv ..."
  "$PY" -m venv .venv
fi
echo "Installing the libraries (the first time downloads about 1 GB; it takes a few minutes) ..."
.venv/bin/python -m pip install --upgrade pip
if [ "$(uname)" = "Linux" ] && [ -z "${ROBOFIELD_GPU:-}" ]; then
  # Linux's default PyTorch download includes GPU code (~2.5 GB) that RoboField's small networks do
  # not need; the CPU build is ~200 MB. Set ROBOFIELD_GPU=1 to get the GPU build instead.
  .venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
fi
.venv/bin/python -m pip install -e ".[all]"

echo
.venv/bin/python tools/doctor.py || true
echo
echo " Done. Start RoboField with:"
echo "     .venv/bin/python tools/robofield_app.py"
echo " (On Linux the app's own window needs GTK or Qt; without them it opens in the browser.)"
echo
