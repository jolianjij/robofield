# The RoboField app

One window for everything: drive the cars, watch the field in 3D, train RL policies and let them
drive, run the calibration and measurement tools, and tune the controllers. It works the same on
the simulator and on the real field, and you can switch between the two at any time.

## Start it

```bash
pip install -e ".[dev,rl,sim,train,app]"      # once
python tools/robofield_app.py                  # opens the RoboField window, in the simulator
```

| Command | Opens |
|---|---|
| `python tools/robofield_app.py` | Its own window, in the simulator |
| `python tools/robofield_app.py --real` | Straight on the real field (it finds the Pi by itself) |
| `python tools/robofield_app.py --browser` | In your web browser instead of its own window |
| `python tools/robofield_app.py --headless` | Nothing: open `http://<this-computer>:8000/app` yourself, e.g. on a tablet |
| `python tools/robofield_app.py --shortcut` | Puts a **RoboField** icon on the Desktop (Windows). Double-click it from then on |

The window needs `pywebview` (the `app` extra). Without it the app opens in the browser.
Closing the window stops every car and every running job.

## The top bar

| Control | What it does |
|---|---|
| **Field · 3D view · Training · Tools · Tuning** | The five tabs |
| **Simulation / Real field** | Switch the whole app. Switching stops the cars first. On the real field the app listens for the Pi tracker's broadcast to find its address |
| **cars** (simulator) | How many simulated cars |
| **Pi** (real field) | The Pi's address. Leave it empty to find it automatically, or type one and press Enter |
| Status | `simulator`, `real field`, or `real field · no Pi` when no tracker is heard |

## Field

The Control Center page: car cards, the live map, go-to-pose, drawing paths, the camera, saved
moves and safety ([Control Center](control_center.md)).

## 3D view

The field in 3D: floor and tile seams, walls, the four towers with their markers, the overhead
camera and what it sees, and every car with its LEDs, name, trail, planned path and goal. When a
policy drives, its game is drawn too (the race track's centre line, the goto target). It shows
the simulator or the real cars, whichever the app is on.

| Control | |
|---|---|
| **3D / Top / Camera / Follow** | Camera positions: free orbit, straight down, from the overhead camera, behind a car |
| paths, trails, names, camera | Show or hide those layers |
| Mouse | Drag to orbit, right-drag to pan, scroll to zoom |

The 3D view loads three.js from `cdn.jsdelivr.net`, so it needs an internet connection.

## Training

Train a policy, watch it learn, test it, and let it drive. The concepts are explained in the
[reinforcement learning guide](reinforcement_learning.md).

1. **Train a new policy.** Choose a game, a name and a length, then **Start training**. *Quick test*
   (100k steps) takes under a minute and is enough to see the curve rise. *Standard* (2M steps) is
   what a real policy needs. Training runs in the background; you can use the other tabs meanwhile.
2. **Watch the curve.** Reward per episode (blue) and the share of episodes that succeed (green),
   updated after every PPO round. **Stop and save** ends it early and still saves the policy.
3. **Evaluate** a policy: 10 episodes in randomized simulators, scored.
4. **Drive** a car with it. Choose the car and a speed. In the simulator, the car is first put on
   one of the game's start positions and the other cars are parked out of its way. On the real
   field it starts from where the car is, if the game allows (the race needs the car on the outer
   ring), after you confirm. Commands go through the safety layer either way. Watch it on the
   Field or 3D tab; the Stop button, Stop all, or Esc stops it.

**Continue from** trains an existing policy further instead of starting from scratch.

Everything is saved in `models/`: `<name>.zip` (the policy), `<name>.json` (game, steps, score),
`<name>.progress.jsonl` (the curve) and `checkpoints/<name>/`. The same training runs from the
command line: `python tools/train.py --game race --timesteps 2000000 --name race_v2`.

Games that need two cars (sumo, and football to come) are listed but cannot be trained yet: they
need self-play, which is the next step.

## Tools

The command-line tools, with a Run button and their output in the panel on the right.

| Tool | Needs |
|---|---|
| Find cars, Car self-test, Measure loop delay, Calibrate motors | The app on **Simulation**: these talk to the cars and the Pi themselves |
| Score motion | Works through the app, on whichever mode it is on |
| Record the camera | The Pi's address (switch to Real field once) |
| Run the test suite, Generate markers | Nothing |

Tools that drive a car ask first.

## Tuning

The live tuning page ([tuning guide](tuning.md)).

## From a phone or tablet

The app is a web server: any device on the same WiFi can open `http://<this-computer>:8000/app`.
The window on the laptop and the browser on the tablet show the same app.

## When something goes wrong

| Problem | Fix |
|---|---|
| "Port 8000 is already in use" | The app or `tools/control_center.py` is already running. Close it, or start with `--port 8001` |
| The 3D view says it cannot load three.js | No internet connection; the other tabs work without it |
| Real field: `no Pi` | The tracker is not running or not on this network. Check the Pi preview at `http://<pi>:8080` |
| A tool's Run button is greyed out | Its card says why: usually switch to Simulation first |
| The app started from the icon does nothing | Look in `logs/app.log` |
