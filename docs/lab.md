# The Lab: write your own tasks, policies and systems

The **Lab** tab of the [RoboField app](app.md) is where you program the robots yourself, in
Python, without touching the rest of the code. There are three kinds of things you can write:

| Kind | What it is | Lives in | Used on |
|---|---|---|---|
| **Task** | Something a car should **learn** with reinforcement learning: where an episode starts, what the car sees, what earns reward, when it ends | `custom/tasks/` | Training tab: train a policy for it, then let it drive |
| **Policy** | A hand-written program that drives **one** car, 30 times a second | `custom/policies/` | Lab tab: **Run** on a car |
| **System** | A program that drives **several** cars together: formations, follow the leader, patrols | `custom/systems/` | Lab tab: **Run** on several cars |

Each comes with a working example: `custom/tasks/park_center.py`, `custom/policies/patrol_square.py`
and `custom/systems/follow_the_leader.py`.

## Working in the Lab tab

1. **+ New** next to Tasks, Policies or Systems makes a file from a template that already works.
2. Edit it. **Reference** opens a short description of everything you can use.
3. **Check** loads it and runs it for a few seconds in a simulator, separately from the app. It
   reports the first problem with its line number (click it to jump there), or what it saw: how
   far the cars moved, how big the observation and rewards are.
4. **Save** (Ctrl+S) writes the file.
5. **▶ Run** (policies and systems) saves it and starts it on the car(s) you chose, through the
   safety layer. **Train it →** (tasks) opens the Training tab with the task chosen.
6. **■ Stop**, **Stop all** or Esc stops it. If it crashes, it stops by itself and the error and its
   line appear in the output.

On the real field the app asks before a program drives a real car.

## Policies: drive one car

```python
from robofield.lab import Policy


class GoAndCome(Policy):
    """Drive to the far end and back, then stop."""

    def start(self, car, world):
        self.targets = [(800, 500), (200, 500)]

    def step(self, car, world):
        if not self.targets:
            return self.finish()
        x, y = self.targets[0]
        if self.arrived(car, x, y):
            self.targets.pop(0)
        return self.go_to(car, x, y, speed=220)
```

`start()` runs once. `step()` runs 30 times a second and returns a command. Remember things
between steps in `self`.

**What the program sees.** Units are millimetres, degrees and seconds.

| | |
|---|---|
| `car.x`, `car.y` | position, mm; the field is 0..1000 in both directions |
| `car.heading` | degrees, 0 = toward +x, counter-clockwise positive (`car.theta` in radians) |
| `car.speed`, `car.turn_rate` | mm/s forward, degrees per second |
| `world.t` | seconds since the program started |
| `world.cars`, `world.car(id)`, `world.others(car)` | every car / one car / the other cars |
| `world.ball`, `world.things` | the ball (None if there is none), every object |
| `world.size` | the field's size, mm |

**What it can return.**

| | |
|---|---|
| `self.drive(speed, turn)` | forward speed in mm/s (negative = backwards) and turning in degrees per second (positive = left) |
| `self.go_to(car, x, y, speed=200)` | head for a point: turns first if it is far to the side, slows down near it, stops within 20 mm |
| `self.face(car, x, y)` | turn on the spot toward a point |
| `self.wheels(left, right)` | raw wheel commands from -1 to 1 |
| `self.stop()` | stand still |
| `self.finish()` | end the program |

Helpers: `self.distance(car, x, y)` in mm, `self.angle_to(car, x, y)` in degrees (positive =
to the left), `self.arrived(car, x, y, within=25)`.

The car cannot drive slower than about 100 mm/s ([kinematics](kinematics.md)); a slower request is
sped up along the same curve. For precise stops at an exact heading, the Control Center's go-to-pose
is better than `go_to`, which is kept simple on purpose.

## Systems: several cars together

```python
from robofield.lab import System


class SideBySide(System):
    """Two cars drive across the field next to each other."""
    cars = 2

    def start(self, world):
        self.a, self.b = world.cars[0].id, world.cars[1].id

    def step(self, world):
        a, b = world.car(self.a), world.car(self.b)
        if a.x > 750 and b.x > 750:
            return self.finish()
        return {a.id: self.go_to(a, 800, 400), b.id: self.go_to(b, 800, 600)}
```

`cars` says how many cars it needs. `step(world)` returns `{car id: command}`; a car left out
stands still. `world.cars` lists the chosen cars first, in the order you ticked them. The commands
and helpers are the same as for a policy.

## Tasks: something a car learns

A task is a game for reinforcement learning, the same kind as the built-in race and goto
([reinforcement learning guide](reinforcement_learning.md)). Units are millimetres and **radians**.

| Method | Returns |
|---|---|
| `layout()` | where an episode starts: `{"robots": {0: (x, y, angle)}, "objects": {}}`. Choose targets here; use `self.rng` for randomness |
| `observe(ws, i)` | what the car sees: a short list of numbers, relative to the car (`self.to_robot_frame(car, x, y)` gives a point as ahead / to the left), scaled to about -1..1 |
| `reward(prev, ws, cmds)` | `{0: number}`: how good the last step was |
| `done(ws)` | True when the episode is over |
| `info(ws)` | `{"success": True/False, ...}`: shown while training and evaluating |
| `begin(ws)` | start an episode from wherever a real car is (the real field cannot place it) |
| `ready(ws)` | `(True, "")`, or `(False, "why")` if a real car must be moved first |
| `overlays()` | what the 3D view draws while a policy drives it: circles, paths |

`ws.robots[0]` has `x`, `y`, `theta`, `v` (mm/s) and `omega` (rad/s). Set `max_steps` for the
episode length (30 steps per second). `obs_dim` is worked out automatically.

Tips that decide whether a task trains well:

- **Reward progress, not only success.** "Closer than last step" every step teaches far faster than
  a single prize at the end.
- **Charge a little for time** (`- 0.01` per step) so it does not dawdle.
- **Observe relative to the car**, and keep numbers small (divide mm by 500 or 1000).
- **Check, then train a Quick test** (100k steps) first: if the success rate on the curve does
  not start rising, change the reward before training longer.

## Good to know

- Programs run inside the Control Center's loop. Keep `step()` quick: no waiting loops, no
  `time.sleep`. **Check** catches an endless loop; a slow step shows as "slow steps" in the output.
- Every command passes through the safety layer: the geofence, keep-cars-apart, the speed limit and
  the e-stop all still apply.
- Deleting a file moves it to `custom/.deleted/`.
- Files in `custom/` are ordinary Python: you can also edit them in any editor, and commit them.
