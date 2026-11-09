# Install

RoboField runs on a laptop with Windows, macOS or Linux. The Raspberry Pi that tracks the cars has
its own short setup ([Pi](#the-raspberry-pi)). Nothing here needs the hardware: without it the app
runs in the simulator.

## The quick way

You need **Python 3.10 or newer** ([python.org](https://www.python.org/downloads/); on Windows tick
*Add python.exe to PATH*) and **git** ([git-scm.com](https://git-scm.com/downloads)).

```bash
git clone https://github.com/jolianjij/robofield.git
cd robofield
```

| System | Run | Then |
|---|---|---|
| Windows | double-click **`install.bat`** | a **RoboField** icon appears on the Desktop |
| macOS, Linux | `bash install.sh` | start with `.venv/bin/python tools/robofield_app.py` |

The script makes a private Python environment in `.venv/` (so it does not touch any other Python
on the computer), installs every library, and runs the self-check. The first time it downloads
about 1 GB (mostly PyTorch) and takes a few minutes. Running it again updates everything.

## What gets installed

`pip install -e ".[all]"` installs RoboField and everything the laptop side uses. The options can
also be installed one at a time:

| Option | Libraries | For |
|---|---|---|
| (always) | numpy, opencv-contrib-python, PyYAML, pyserial | the core: markers, vision, configuration |
| `sim` | mujoco | the simulator, and the app in Simulation mode |
| `rl` | gymnasium, pettingzoo | the reinforcement-learning environments |
| `train` | stable-baselines3, torch, tensorboard | training policies and letting them drive |
| `app` | pywebview | the app's own window (otherwise it opens in the browser) |
| `tools` | websockets, matplotlib | screen recording, the older goal GUI |
| `dev` | pytest | the tests |
| `docs` | mkdocs-material, mkdocstrings | building this website |
| `all` | all of the above except `docs` | the laptop |

For example, a computer that only runs the simulator and the app: `pip install -e ".[sim,app]"`.

## Check it

```bash
python tools/doctor.py              # Python, every library, the simulator, the config, the ports
python tools/doctor.py --network    # also looks for the Pi tracker and the cars on the WiFi
pytest                              # the automated tests, no hardware needed
```

The self-check marks anything that needs fixing with **FIX** and says how. Notes (optional
libraries, nothing on the network) are fine.

## The Raspberry Pi

On the Pi (Raspberry Pi OS 64-bit), from a copy of this repository:

```bash
bash pi/install.sh
```

It installs the camera library and OpenCV, and keeps the Pi's own NumPy that the camera library
needs. Then calibrate the lens and start the tracker: [Pi guide](../pi/README.md).

## The cars

The car firmware is flashed from the Arduino IDE: [firmware guide](../firmware/car_esp8266/README.md).

## When something goes wrong

| Problem | Fix |
|---|---|
| `Python 3.10 or newer was not found` | Install Python from python.org; on Windows tick *Add python.exe to PATH*, then run the script again |
| Windows opens the Microsoft Store when you type `python` | Use the `py` launcher (the script does), or turn off the *App execution alias* for python in Windows settings |
| `No matching distribution found for torch` | Your Python is newer than PyTorch supports yet: install the previous Python version and run the script again |
| `OpenCV has no ArUco module` | `pip uninstall opencv-python`, then `pip install opencv-contrib-python` (only one OpenCV may be installed) |
| The app opens in the browser instead of its own window | Fine: everything works the same. On Linux the window needs extra system packages ([pywebview's Linux notes](https://pywebview.flowrl.com/guide/installation.html#linux)) |
| The 3D view or the Lab editor does not load | They load their libraries from the internet the first time; connect once |
| `simulator cannot draw pictures` | Update the graphics driver; only the simulated camera needs it |
| `port 8000 is in use` | The app is already running; close it or start another with `--port 8001` |

Still stuck? Open an issue with the output of `python tools/doctor.py`:
[github.com/jolianjij/robofield/issues](https://github.com/jolianjij/robofield/issues).
