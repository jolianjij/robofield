"""Train an RL policy to race 3 laps around the outer ring of tiles, as fast as possible, without
leaving the ring or hitting the outer wall (the `race` game, robofield/games/race.py).

Trains in the MuJoCo simulator only (sim-to-real transfer, not real-field training): domain
randomization is on by default so the policy does not overfit to one exact motor/friction model.

    python tools/train_race.py                                  # train from scratch
    python tools/train_race.py --timesteps 2000000 --envs 8      # longer run, more parallel envs
    python tools/train_race.py --eval models/race_ppo.zip         # watch a trained policy run
    python tools/train_race.py --eval models/race_ppo.zip --real --robot 0 --speed-limit 0.4
                                                                    # on the real car, capped speed

Needs the `train` extra: pip install -e ".[train]" (stable-baselines3 + torch).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.control import compensate_deadzone, deadzones_from_config  # noqa: E402
from robofield.core.state import RobotCommand  # noqa: E402
from robofield.envs.gym_env import RoboFieldEnv  # noqa: E402
from robofield.games import make_game  # noqa: E402


def make_env(randomize: bool, seed: int):
    def _make():
        # predict=True: observe the delay-predicted state, exactly as the real field reports it
        return RoboFieldEnv(game="race", randomize=randomize, seed=seed, predict=True)
    return _make


def train(args):
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # one process per env (each with its own seed, so each draws different friction, delay, motor
    # mismatch...), instead of all of them stepping in this one process
    fns = [make_env(not args.no_randomize, args.seed + i) for i in range(args.envs)]
    env = VecMonitor(SubprocVecEnv(fns) if args.envs > 1 else DummyVecEnv(fns))
    model = PPO("MlpPolicy", env, verbose=1, n_steps=args.n_steps, batch_size=args.batch_size,
                tensorboard_log=str(out.parent / "tb") if args.tensorboard else None)
    ckpt = CheckpointCallback(save_freq=max(1, args.checkpoint_every // args.envs),
                              save_path=str(out.parent), name_prefix=out.stem)
    t0 = time.monotonic()
    model.learn(total_timesteps=args.timesteps, callback=ckpt, progress_bar=False)
    model.save(str(out))
    print(f"saved {out} ({args.timesteps} steps in {time.monotonic() - t0:.0f}s)")
    env.close()


def evaluate_sim(args):
    from stable_baselines3 import PPO

    model = PPO.load(args.eval)
    env = RoboFieldEnv(game="race", randomize=args.randomize_eval, seed=args.seed + 1000, predict=True,
                       render_mode="rgb_array" if args.render else None)
    wins, laps_total = 0, 0
    for ep in range(args.episodes):
        obs, info = env.reset()
        done = truncated = False
        while not (done or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
        wins += int(info.get("success", False))
        laps_total += info.get("laps", 0)
        print(f"episode {ep}: laps={info.get('laps')} off_track={info.get('off_track')} "
              f"progress_m={info.get('progress_m', 0):.2f}")
    print(f"{wins}/{args.episodes} completed 3 laps, avg laps {laps_total / args.episodes:.2f}")
    env.close()


def evaluate_real(args):
    """Run the trained policy on the real car, through the Safety layer (geofence, speed cap,
    tracking-loss handling) - unlike the simulator, a wrong or divergent action here can run the
    car into a wall or off the field for real, and this is the first time this policy has ever
    seen the real plant. `--speed-limit` scales the policy's own output on top of Safety's cap,
    so a first run can be deliberately slow before trusting it at race pace."""
    from stable_baselines3 import PPO
    from robofield.app.safety import Safety, SafetySettings
    from robofield.control import DiffDrive, PoseController, deadzones_from_config, safety_margin_mm
    from robofield.core import load_config
    from robofield.real import RealBackend

    cfg = load_config(args.config)
    model = PPO.load(args.eval)
    game = make_game("race", field_mm=cfg["field"]["width_mm"], margin_mm=safety_margin_mm(cfg))
    game.seed(0)
    backend = RealBackend(cfg, robot_ids=[args.robot])
    dd = DiffDrive.from_config(cfg)
    safety = Safety(dd, cfg["field"]["width_mm"], deadzones_from_config(cfg),
                    SafetySettings(speed_limit=args.speed_limit))
    print(f"speed_limit={args.speed_limit} (fraction of top speed) - raise once this looks right")
    try:
        ws = backend.reset()
        r = ws.robots.get(args.robot)
        if r is None or not r.visible:
            print("robot not visible, aborting without sending any command"); return

        # Policies trained with random starts (the race's default now) orient themselves from any
        # pose in the lane. --align first drives to the nearest centerline point facing along the
        # track with the PoseController, for a policy trained only on clean starts.
        if args.align:
            s_now, _, track_heading = game._project(r.x, r.y)
            gx, gy, _ = game._pose_at(s_now)
            print(f"driving to the centerline ({gx:.0f},{gy:.0f}) facing {math.degrees(track_heading):.0f} deg first...")
            pc = PoseController(dd, margin=safety_margin_mm(cfg))
            pc.set_goal(gx, gy, track_heading, (r.x, r.y))
            t_align = time.monotonic()
            while time.monotonic() - t_align < 20 and not pc.done:
                ws = backend.step()
                r = ws.robots.get(args.robot)
                if r is None or not r.visible:
                    backend.apply({args.robot: RobotCommand()}); continue
                backend.apply({args.robot: pc.command(r.x, r.y, r.theta, r.omega, 1 / backend.control_hz)})
            backend.apply({args.robot: RobotCommand()})
            ws = backend.step()
            r = ws.robots.get(args.robot)
            if r is None or not r.visible:
                print("robot not visible, aborting"); return
        s_now, lateral, track_heading = game._project(r.x, r.y)
        print(f"start ({r.x:.0f},{r.y:.0f}) facing {math.degrees(r.theta):.0f} deg (track goes "
              f"{math.degrees(track_heading):.0f} deg), {abs(lateral):.0f} mm off the centerline "
              f"(lane is +-{game.half_width:.0f} mm)")
        if not args.ignore_off_track and abs(lateral) > game.half_width:
            print("the robot is outside the lane - put it on the outer ring of tiles and retry")
            return
        print("starting the policy")

        game._raw_s_prev, _, _ = game._project(r.x, r.y)
        game._progress, game._laps, game._off_track = 0.0, 0, False
        last_action = np.zeros(2, dtype=np.float32)
        t0 = time.monotonic()
        last_print = 0.0
        while time.monotonic() - t0 < args.max_seconds:
            ws = backend.step()
            r = ws.robots.get(args.robot)
            if r is None or not r.visible:
                backend.apply({args.robot: RobotCommand()}); continue
            obs = np.concatenate([game.observe(ws, args.robot), last_action]).astype(np.float32)
            action, _ = model.predict(obs, deterministic=True)
            action = np.clip(action, -1, 1)
            last_action = action
            left, right = float(action[0]), float(action[1])
            if args.no_reverse:
                # cheap guard for a forward-only race: the policy was observed to reverse on the
                # real car in a way it was never actually trained to do (sim's reward only makes
                # backward progress mildly negative, not disallowed) - zero out any net-backward
                # body speed while leaving its steering (the v/w split) otherwise untouched.
                v, w = dd.forward(left, right)
                left, right = dd.inverse(max(0.0, v), w)
            cmd = {args.robot: RobotCommand(left, right)}
            safe_cmd = safety.filter(ws, cmd, tracker_ok=True, now=time.monotonic())
            if args.car_speed_scale != 1.0:
                # scale the body motion (same as the Control Center's per-car speed slider): on this
                # car's non-linear motors a per-wheel multiply bends the path, while scaling v and
                # w together keeps the turn ratio - the same path, just slower
                c = safe_cmd[args.robot]
                v, w = dd.forward(c.left, c.right)
                safe_cmd = {args.robot: RobotCommand(*dd.inverse(v * args.car_speed_scale, w * args.car_speed_scale),
                                                      c.gripper, c.leds)}
            backend.apply(safe_cmd)
            game.reward(ws, ws, safe_cmd)  # Race.reward only reads `ws`; called for its lap/progress side effects
            if time.monotonic() - last_print > 1.0:
                print(f"  t={time.monotonic()-t0:4.1f}s laps={game._laps} off_track={game._off_track} "
                      f"progress_m={game._progress/1000:.2f} safety={safety.events.get(args.robot, '')}")
                last_print = time.monotonic()
            # --ignore-off-track: keep running regardless of the race's own track-boundary
            # definition (lap-count success still ends the run) - Safety (geofence, speed cap,
            # tracking-loss handling) above is unaffected and still protects the real hardware.
            if game._laps >= game.laps_to_win or (not args.ignore_off_track and game.done(ws)):
                break
        print(f"finished: laps={game._laps} off_track={game._off_track} "
              f"progress_m={game._progress/1000:.2f} elapsed={time.monotonic()-t0:.1f}s")
    finally:
        backend.apply({args.robot: RobotCommand()})
        backend.stop()
        backend.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timesteps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=12, help="parallel sim environments (one process each)")
    ap.add_argument("--n-steps", type=int, default=512, help="PPO rollout length per env per update")
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--checkpoint-every", type=int, default=50_000)
    ap.add_argument("--no-randomize", action="store_true", help="disable domain randomization (debugging only)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="models/race_ppo.zip")
    ap.add_argument("--tensorboard", action="store_true")
    ap.add_argument("--eval", default=None, help="path to a saved model: watch/evaluate instead of training")
    ap.add_argument("--episodes", type=int, default=5, help="evaluation episodes (sim only)")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--real", action="store_true", help="evaluate on the real field instead of the simulator")
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--config", default="config/field.yaml")
    ap.add_argument("--speed-limit", type=float, default=0.4,
                    help="real only: fraction of top speed, on top of the policy's own output (default 0.4, cautious)")
    ap.add_argument("--max-seconds", type=float, default=120.0, help="real only: hard time limit")
    ap.add_argument("--align", action="store_true",
                    help="real only: drive to the centerline facing along the track before starting")
    ap.add_argument("--randomize-eval", action="store_true",
                    help="sim evaluation with domain randomization (friction, delay, motors...)")
    ap.add_argument("--no-reverse", action="store_true",
                    help="real only: clamp out any net-backward body speed (see evaluate_real docstring)")
    ap.add_argument("--car-speed-scale", type=float, default=1.0,
                    help="real only: uniform scale of both wheels on top of Safety (matches the web "
                         "GUI's per-car speed slider, cars.json CarProfile.max_speed)")
    ap.add_argument("--ignore-off-track", action="store_true",
                    help="real only: do not end the run just for leaving the race ring (Safety stays on)")
    args = ap.parse_args()
    if args.eval:
        evaluate_real(args) if args.real else evaluate_sim(args)
    else:
        train(args)


if __name__ == "__main__":
    main()
