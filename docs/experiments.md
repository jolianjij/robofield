# Experiments

How RoboField's experiments are run, so that every number in a paper can be reproduced. One spec,
one protocol and one log format serve the simulator and the real field alike.

## The pieces

| | |
|---|---|
| `experiments/*.yaml` | An experiment: the task, the controllers to compare, the protocol's numbers |
| `robofield/experiments.py` | The protocol, the metrics, the logs, and the two runners |
| `tools/experiment.py run` | Runs a spec in the simulator or on the real field |
| `tools/experiment.py report` | Turns run folders into tables (Markdown, LaTeX) and figures (PNG, PDF) |

Both runners use the same Control Center code (controllers, RL policy driver, safety layer):

- **`--backend sim`**: an in-process Control Center on the MuJoCo simulator, run as fast as the
  computer allows (60 episodes take about 15 seconds). `--randomized` draws a randomized world per
  target, seeded so that every controller meets the same worlds.
- **`--backend app`**: the running RoboField app over its API. On the real field the app's STOP,
  map, 3D view and camera all stay live while it runs, and the script asks before the car moves.

## Experiment 1: go to a target, sim-to-real

`experiments/goto_sim2real.yaml` compares three controllers on reaching a point:

| Name | Controller |
|---|---|
| `baseline` | The classical controller: turn to face the target, pure pursuit on the delay-predicted pose ([motion control](control.md)) |
| `ppo` | A PPO policy trained 1M steps in the plain simulator (`models/goto_nodr`) |
| `ppo_dr` | The same, trained with domain randomization (`models/goto_dr`) |

**Protocol.** The car starts at (500, 500), facing +x. It then drives to 20 seeded targets in order,
each inside the square 150–850 mm and at least 200 mm from the one before; each target is one
episode, starting where the last one ended (the real field cannot place a car). Every controller
gets the same targets. In the simulator the car is placed at the start; on the real field it drives
there first, outside any episode.

**Metrics, per episode.**

| Metric | Definition |
|---|---|
| success | at rest within 30 mm of the target, within 10 s |
| time to target | from the command to the first moment within 30 mm |
| final error | distance to the target once the car is at rest (0.8 s after it stopped) |
| path efficiency | straight-line distance ÷ distance driven (1 = a perfectly straight path) |
| max deviation | farthest point from the straight line between start and target |
| safety steps | control steps in which the safety layer changed the command |

The positions are what the Control Center sees (on the real field: the camera's, filtered and
predicted 0.2 s ahead), so the simulator and the real field are measured the same way; at rest the
prediction equals the measurement, so the final error is unaffected.

**Run it.**

```bash
python tools/train.py --game goto --name goto_dr --timesteps 1000000                  # ~6 min each
python tools/train.py --game goto --name goto_nodr --no-randomize --timesteps 1000000
python tools/experiment.py run experiments/goto_sim2real.yaml --backend sim
python tools/experiment.py run experiments/goto_sim2real.yaml --backend sim --randomized
python tools/experiment.py run experiments/goto_sim2real.yaml --backend app            # app on Real field
python tools/experiment.py report experiments/runs/goto_sim2real_*
```

On the real field: put the app on **Real field**, check the car is seen, clear the field, then
run. About 20 targets × 3 controllers take 10–15 minutes including the drives to the start.

**Simulator results so far** (2026-11-10, 20 targets per controller, seed 7):

| Controller | Where | Success | Time to target (s) | Final error (mm) | Path efficiency |
|---|---|---|---|---|---|
| baseline | sim | 100% | 3.81 ± 1.16 | 5.6 ± 1.1 | 0.76 ± 0.10 |
| baseline | sim, randomized | 100% | 3.54 ± 1.07 | 8.5 ± 3.3 | 0.74 ± 0.13 |
| ppo | sim | 100% | 1.95 ± 0.63 | 10.4 ± 3.0 | 0.94 ± 0.04 |
| ppo | sim, randomized | 100% | 2.07 ± 0.74 | 18.6 ± 6.9 | 0.89 ± 0.09 |
| ppo_dr | sim | 100% | 1.97 ± 0.69 | 7.7 ± 3.0 | 0.94 ± 0.06 |
| ppo_dr | sim, randomized | 100% | 2.06 ± 0.76 | 14.4 ± 5.2 | 0.89 ± 0.06 |

The learned policies reach the target about twice as fast as the classical controller and drive
straighter (they curve toward the target instead of turning on the spot first), but stop less
precisely. In randomized worlds every controller loses precision; the policy trained with
randomization loses less than the one trained without. The real-field run is what tests this.

## What a run folder holds

`experiments/runs/<experiment>_<backend>_<time>/`:

| File | |
|---|---|
| `episodes.csv`, `episodes.jsonl` | one row per episode: the metrics above, the target, the start pose |
| `trajectories/<controller>_<backend>_<n>.csv` | every control step: time, x, y, heading, speed, turn rate, the wheel commands, safety events |
| `meta.json` | the spec, the git commit, `config/field.yaml` and `config/tuning.yaml` as they were, the policies' training records, the Python version |

Keep the run folders of every result that goes into the paper: with `meta.json` they say exactly
what produced each number.

## Writing a new experiment

Copy `goto_sim2real.yaml`, change the protocol numbers or the conditions, and run it. A condition
is `controller: baseline` or `controller: policy` with a `model` from `models/`. Other tasks need
their own protocol in `robofield/experiments.py`; the logging, the summary and the report work the
same for all of them.
