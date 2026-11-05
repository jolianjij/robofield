"""Train an RL policy to race 3 laps around the outer ring of tiles, as fast as possible, without
leaving the ring or hitting the outer wall (the `race` game, robofield/games/race.py).

Trains in the MuJoCo simulator only (sim-to-real transfer, not real-field training): domain
randomization is on by default so the policy does not overfit to one exact motor/friction model.

    python tools/train_race.py                                  # train from scratch
    python tools/train_race.py --timesteps 2000000 --envs 8      # longer run, more parallel envs
    python tools/train_race.py --eval models/race_ppo.zip         # watch a trained policy run
    python tools/train_race.py --eval models/race_ppo.zip --real --robot 0   # on the real car

Needs the `train` extra: pip install -e ".[train]" (stable-baselines3 + torch).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robofield.core.state import RobotCommand  # noqa: E402
from robofield.envs.gym_env import RoboFieldEnv  # noqa: E402


def make_env(randomize: bool, seed: int):
    def _make():
        return RoboFieldEnv(game="race", randomize=randomize, seed=seed)
    return _make


def train(args):
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.env_util import make_vec_env

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    env = make_vec_env(make_env(not args.no_randomize, args.seed), n_envs=args.envs, seed=args.seed)
    model = PPO("MlpPolicy", env, verbose=1, n_steps=args.n_steps, batch_size=args.batch_size,
                tensorboard_log=str(out.parent / "tb") if args.tensorboard else None)
    ckpt = CheckpointCallback(save_freq=max(1, args.checkpoint_every // args.envs),
                              save_path=str(out.parent), name_prefix=out.stem)
    t0 = time.monotonic()
    model.learn(total_timesteps=args.timesteps, callback=ckpt, progress_bar=False)
    model.save(str(out))
    print(f"saved {out} ({args.timesteps} steps in {time.monotonic() - t0:.0f}s)")
    env.close()


def evaluate(args):
    from stable_baselines3 import PPO

    model = PPO.load(args.eval)
    if args.real:
        from robofield.real import RealBackend
        from robofield.core import load_config
        cfg = load_config(args.config)
        backend = RealBackend(cfg, robot_ids=[args.robot])
    else:
        backend = None
    env = RoboFieldEnv(game="race", randomize=False, backend=backend,
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
    if backend is not None:
        backend.apply({args.robot: RobotCommand()})
        backend.stop(); backend.close()
    env.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timesteps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=4, help="parallel sim environments")
    ap.add_argument("--n-steps", type=int, default=512, help="PPO rollout length per env per update")
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--checkpoint-every", type=int, default=50_000)
    ap.add_argument("--no-randomize", action="store_true", help="disable domain randomization (debugging only)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="models/race_ppo.zip")
    ap.add_argument("--tensorboard", action="store_true")
    ap.add_argument("--eval", default=None, help="path to a saved model: watch/evaluate instead of training")
    ap.add_argument("--episodes", type=int, default=5, help="evaluation episodes")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--real", action="store_true", help="evaluate on the real field instead of the simulator")
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--config", default="config/field.yaml")
    args = ap.parse_args()
    if args.eval:
        evaluate(args)
    else:
        train(args)


if __name__ == "__main__":
    main()
