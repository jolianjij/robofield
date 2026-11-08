"""Train or evaluate a PPO policy for any one-car game (goto, race) in the MuJoCo simulator.

    python tools/train.py --game race --timesteps 1000000 --name race_v2
    python tools/train.py --game goto --timesteps 300000 --name goto_first
    python tools/train.py --eval models/race_v2.zip --episodes 10
    python tools/train.py --eval models/race_v2.zip --episodes 10 --randomize-eval

Training writes, in models/:
    <name>.zip               the policy
    <name>.json              what it is: game, steps, settings, final score
    <name>.progress.jsonl    the learning curve, one line per PPO round
    checkpoints/<name>/      snapshots along the way
Create models/<name>.stop to end training early: it stops after the current round and still saves.
The RoboField app's Training tab runs this script and reads those files. Lines printed as
"PROGRESS {json}" and "RESULT {json}" are for the app; everything else is for people.

A policy trained here drives a car through the Control Center's safety layer: the app's
Training tab, or POST /api/policy. Needs the train extra: pip install -e ".[train]".
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from robofield.envs.gym_env import RoboFieldEnv  # noqa: E402
from robofield.games import REGISTRY  # noqa: E402

MODELS = ROOT / "models"


def single_car_games() -> list[str]:
    return sorted(name for name, cls in REGISTRY.items() if cls.n_robots == 1)


def game_of(model_path: Path, fallback: str | None = None) -> str:
    """The game a saved policy was trained for: from its .json, else guessed from its file name."""
    meta = model_path.with_suffix(".json")
    if meta.exists():
        return json.loads(meta.read_text(encoding="utf-8"))["game"]
    if fallback:
        return fallback
    for g in single_car_games():
        if model_path.stem.startswith(g):
            return g
    raise SystemExit(f"which game is {model_path.name} for? pass --game")


def make_env(game: str, randomize: bool, seed: int):
    def _make():
        import gymnasium as gym

        class SuccessInfo(gym.Wrapper):
            """Every episode reports success (False if the game does not say), so the success rate
            can be logged for any game."""
            def step(self, action):
                obs, r, term, trunc, info = self.env.step(action)
                info.setdefault("success", False)
                return obs, r, term, trunc, info

        # predict=True: observe the delay-predicted state, exactly as the real field reports it
        return SuccessInfo(RoboFieldEnv(game=game, randomize=randomize, seed=seed, predict=True))
    return _make


def write_json(path: Path, data: dict):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=1), encoding="utf-8")
    os.replace(tmp, path)


def train(a):
    global MODELS
    MODELS = Path(a.dir)
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

    name = a.name or f"{a.game}_{time.strftime('%Y%m%d_%H%M%S')}"
    MODELS.mkdir(exist_ok=True)
    out, meta_path = MODELS / f"{name}.zip", MODELS / f"{name}.json"
    progress_path, stop_file = MODELS / f"{name}.progress.jsonl", MODELS / f"{name}.stop"
    stop_file.unlink(missing_ok=True)
    progress_path.write_text("", encoding="utf-8")
    meta = {"name": name, "game": a.game, "algo": "PPO", "timesteps_target": a.timesteps, "timesteps_done": 0,
            "envs": a.envs, "randomize": not a.no_randomize, "seed": a.seed, "status": "training",
            "started": time.strftime("%Y-%m-%d %H:%M:%S"), "finished": None, "seconds": None,
            "ep_rew_mean": None, "success": None, "from": a.resume}
    write_json(meta_path, meta)

    class Progress(BaseCallback):
        """After every PPO round: append the learning curve and print it for the app.
        Every 256 steps: end training early if the stop file exists (the model is still saved)."""
        def __init__(self):
            super().__init__()
            self.t0 = time.monotonic()
            self.stopped = False
            self.last = {}

        def _on_step(self) -> bool:
            if self.n_calls % 256 == 0 and stop_file.exists():
                self.stopped = True
                return False
            return True

        def _on_rollout_end(self) -> None:
            buf = list(self.model.ep_info_buffer)
            el = time.monotonic() - self.t0
            rec = {"t": round(el, 1), "steps": int(self.num_timesteps), "total": a.timesteps,
                   "episodes": len(buf), "fps": int(self.num_timesteps / max(el, 1e-6))}
            if buf:
                rec["ep_rew_mean"] = round(float(np.mean([e["r"] for e in buf])), 3)
                rec["ep_len_mean"] = round(float(np.mean([e["l"] for e in buf])), 1)
                rec["success"] = round(float(np.mean([float(e.get("success", 0)) for e in buf])), 3)
            self.last = rec
            with progress_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
            print("PROGRESS " + json.dumps(rec), flush=True)

    fns = [make_env(a.game, not a.no_randomize, a.seed + i) for i in range(a.envs)]
    env = VecMonitor(SubprocVecEnv(fns) if a.envs > 1 else DummyVecEnv(fns), info_keywords=("success",))
    if a.resume:
        model = PPO.load(a.resume, env=env, device="cpu")
        print(f"continuing from {a.resume}")
    else:
        model = PPO("MlpPolicy", env, verbose=0, n_steps=a.n_steps, batch_size=a.batch_size,
                    learning_rate=a.learning_rate, seed=a.seed, device="cpu",
                    tensorboard_log=str(MODELS / "tb") if a.tensorboard else None)
    prog = Progress()
    ckpt = CheckpointCallback(save_freq=max(1, a.checkpoint_every // a.envs),
                              save_path=str(MODELS / "checkpoints" / name), name_prefix=name)
    print(f"training '{name}': game {a.game}, {a.timesteps:,} steps, {a.envs} parallel simulators, "
          f"randomization {'on' if not a.no_randomize else 'off'}", flush=True)
    status = "done"
    try:
        model.learn(total_timesteps=a.timesteps, callback=CallbackList([prog, ckpt]), progress_bar=False,
                    tb_log_name=name, reset_num_timesteps=not a.resume)
        if prog.stopped:
            status = "stopped"
    except KeyboardInterrupt:
        status = "stopped"
    model.save(str(out))
    env.close()
    stop_file.unlink(missing_ok=True)
    meta.update(status=status, timesteps_done=int(model.num_timesteps), finished=time.strftime("%Y-%m-%d %H:%M:%S"),
                seconds=round(time.monotonic() - prog.t0, 1), ep_rew_mean=prog.last.get("ep_rew_mean"),
                success=prog.last.get("success"))
    write_json(meta_path, meta)
    print(f"saved {out} ({status}, {model.num_timesteps:,} steps in {meta['seconds']:.0f} s)", flush=True)
    print("RESULT " + json.dumps({"model": out.name, **{k: meta[k] for k in
                                  ("status", "timesteps_done", "seconds", "ep_rew_mean", "success")}}), flush=True)


def evaluate(a):
    from stable_baselines3 import PPO

    path = Path(a.eval)
    game = game_of(path, a.game)
    model = PPO.load(str(path), device="cpu")
    env = RoboFieldEnv(game=game, randomize=a.randomize_eval, seed=a.seed + 1000, predict=True)
    rewards, lengths, wins = [], [], 0
    print(f"evaluating {path.name} on '{game}' for {a.episodes} episodes "
          f"({'randomized' if a.randomize_eval else 'nominal'} simulator)", flush=True)
    for ep in range(a.episodes):
        obs, info = env.reset()
        done = trunc = False
        total, n = 0.0, 0
        while not (done or trunc):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, done, trunc, info = env.step(action)
            total += r
            n += 1
        ok = bool(info.get("success", False))
        wins += ok
        rewards.append(total)
        lengths.append(n)
        shown = {k: (round(v, 2) if isinstance(v, float) else v) for k, v in info.items()}
        print(f"episode {ep}: reward {total:.1f}, {n / 30:.1f} s, {shown}", flush=True)
    env.close()
    res = {"model": path.name, "game": game, "episodes": a.episodes, "success": round(wins / a.episodes, 3),
           "reward_mean": round(float(np.mean(rewards)), 2), "seconds_mean": round(float(np.mean(lengths)) / 30, 1),
           "randomized": a.randomize_eval}
    print(f"{wins}/{a.episodes} successful, mean reward {res['reward_mean']}, mean {res['seconds_mean']} s", flush=True)
    print("RESULT " + json.dumps(res), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--game", choices=single_car_games(), default=None)
    ap.add_argument("--name", default=None, help="file name in models/ (default: <game>_<date>_<time>)")
    ap.add_argument("--timesteps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=max(1, min(12, (os.cpu_count() or 4) - 2)),
                    help="parallel simulators (one process each)")
    ap.add_argument("--n-steps", type=int, default=512, help="PPO steps per simulator per round")
    ap.add_argument("--batch-size", type=int, default=256)
    ap.add_argument("--learning-rate", type=float, default=3e-4)
    ap.add_argument("--checkpoint-every", type=int, default=250_000)
    ap.add_argument("--no-randomize", action="store_true", help="train in one fixed simulator (not recommended)")
    ap.add_argument("--resume", default=None, help="continue training this saved policy")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tensorboard", action="store_true", help="also log for TensorBoard in models/tb")
    ap.add_argument("--dir", default=str(MODELS), help="folder for the policy and its files (default: models/)")
    ap.add_argument("--eval", default=None, help="a saved policy: evaluate it instead of training")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--randomize-eval", action="store_true", help="evaluate in randomized simulators")
    a = ap.parse_args()
    if a.eval:
        evaluate(a)
    else:
        if not a.game:
            ap.error("--game is required for training")
        train(a)


if __name__ == "__main__":
    main()
