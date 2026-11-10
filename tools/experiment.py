"""Run an experiment in the simulator or on the real field, and turn the results into the paper's
figures and tables. The same spec, protocol, controllers and logs on both (robofield/experiments.py).

    python tools/experiment.py run experiments/goto_sim2real.yaml --backend sim
    python tools/experiment.py run experiments/goto_sim2real.yaml --backend sim --randomized
    python tools/experiment.py run experiments/goto_sim2real.yaml --backend app        # real field, via the app
    python tools/experiment.py report experiments/runs/goto_sim2real_sim_* experiments/runs/goto_sim2real_real_*

run     writes experiments/runs/<name>_<backend>_<time>/: episodes.csv, episodes.jsonl,
        trajectories/*.csv and meta.json (the spec, the config, the policies, the git commit)
report  writes results/<name>/: summary.csv, table.md, table.tex and the figures (PNG and PDF)

On the real field, start the app in Real field mode first; the car drives to the start pose, then
to every target. The app's STOP (or Esc on its Field tab) stops it at any time.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from robofield import experiments as ex  # noqa: E402

COLORS = {"baseline": "#6b7280", "ppo": "#2563eb", "ppo_dr": "#059669"}
FALLBACK = ["#d97706", "#7c3aed", "#db2777", "#0891b2"]


def color(cond: str, i: int) -> str:
    return COLORS.get(cond, FALLBACK[i % len(FALLBACK)])


# ------------------------------------------------------------------ run
def cmd_run(a):
    spec = ex.load_spec(a.spec)
    if a.episodes:
        spec["episodes"] = a.episodes
    conds = [c for c in spec["conditions"] if not a.only or c["name"] in a.only.split(",")]
    from robofield.core import load_config
    cfg = load_config(ROOT / "config" / "field.yaml")
    if a.backend == "sim":
        runner = ex.SimRunner(cfg, spec["robot"], seed=spec["seed"], randomize=a.randomized)
    else:
        runner = ex.AppRunner(a.url, spec["robot"])
        if runner.name == "real" and not a.yes:
            print(f"This drives the REAL car {spec['robot']}: to {spec['start'][:2]}, then {spec['episodes']} targets "
                  f"for each of {len(conds)} conditions. Keep the field clear; the app's STOP stops it.")
            if input("Type yes to start: ").strip().lower() != "yes":
                raise SystemExit("not started")
    out = Path(a.out) / f"{spec['name']}_{runner.name}_{time.strftime('%Y%m%d_%H%M%S')}"
    meta = ex.provenance(spec, runner.name)
    episodes = []
    try:
        for c in conds:
            print(f"{c['name']} on {runner.name}:")
            episodes += ex.run_condition(spec, c, runner)
    except KeyboardInterrupt:
        runner.stop()
        print("interrupted: saving what was done")
    finally:
        runner.close()
    if episodes:
        ex.save(out, episodes, meta)
        for s in ex.summarize([e.row() for e in episodes]):
            print(f"  {s['condition']:>10s} {s['backend']:>5s}: success {100 * s['success_rate']:.0f}%  "
                  f"time {s['time_mean_s']} s  error {s['error_mean_mm']} mm")
        print(f"saved {out}")
        print("RESULT " + json.dumps({"dir": str(out), "episodes": len(episodes)}))


# ------------------------------------------------------------------ report
def fmt(m, s, digits=1):
    return "–" if m is None else f"{m:.{digits}f} ± {s:.{digits}f}"


def cmd_report(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = ex.load_runs(a.runs)
    if not rows:
        raise SystemExit("no episodes in those folders")
    name = rows[0]["experiment"]
    out = Path(a.out or ROOT / "results" / name)
    out.mkdir(parents=True, exist_ok=True)
    summ = ex.summarize(rows)
    conds = list(dict.fromkeys(r["condition"] for r in rows))
    backends = list(dict.fromkeys(r["backend"] for r in rows))

    with (out / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    md = ["| Controller | Where | Episodes | Success | Time to target (s) | Final error (mm) | Path efficiency |",
          "|---|---|---|---|---|---|---|"]
    tex = [r"\begin{tabular}{llrrrrr}", r"\toprule",
           r"Controller & Where & $N$ & Success & Time (s) & Final error (mm) & Path efficiency \\", r"\midrule"]
    for c in conds:
        for b in backends:
            s = next((x for x in summ if x["condition"] == c and x["backend"] == b), None)
            if s is None:
                continue
            cells = [c, b, str(s["episodes"]), f"{100 * s['success_rate']:.0f}%", fmt(s["time_mean_s"], s["time_std_s"], 2),
                     fmt(s["error_mean_mm"], s["error_std_mm"]), fmt(s["efficiency_mean"], s["efficiency_std"], 2)]
            md.append("| " + " | ".join(cells) + " |")
            tex.append(" & ".join(x.replace("_", r"\_").replace("%", r"\%").replace("±", r"$\pm$") for x in cells) + r" \\")
    tex += [r"\bottomrule", r"\end{tabular}"]
    (out / "table.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    (out / "table.tex").write_text("\n".join(tex) + "\n", encoding="utf-8")

    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.bbox": "tight", "figure.dpi": 150})
    hatch = {b: ("" if i == 0 else "///") for i, b in enumerate(backends)}
    width = 0.8 / max(1, len(backends))

    def save(fig, stem):
        fig.savefig(out / f"{stem}.png")
        fig.savefig(out / f"{stem}.pdf")
        plt.close(fig)

    # success rate
    fig, ax = plt.subplots(figsize=(3.4, 2.4))
    for j, b in enumerate(backends):
        for i, c in enumerate(conds):
            s = next((x for x in summ if x["condition"] == c and x["backend"] == b), None)
            if s:
                ax.bar(i + (j - (len(backends) - 1) / 2) * width, 100 * s["success_rate"], width * 0.92,
                       color=color(c, i), hatch=hatch[b], edgecolor="white", label=b if i == 0 else None)
    ax.set_xticks(range(len(conds)), conds)
    ax.set_ylabel("success (%)")
    ax.set_ylim(0, 105)
    if len(backends) > 1:
        ax.legend(frameon=False, fontsize=8)
    save(fig, "success_rate")

    # time to target and final error, as box plots
    for key, label, stem, ok_only in (("time_to_target_s", "time to target (s)", "time_to_target", True),
                                      ("final_error_mm", "final error (mm)", "final_error", False)):
        fig, ax = plt.subplots(figsize=(3.4, 2.4))
        pos, data, cols, hs = [], [], [], []
        for i, c in enumerate(conds):
            for j, b in enumerate(backends):
                vals = [r[key] for r in rows if r["condition"] == c and r["backend"] == b
                        and r[key] is not None and (r["success"] or not ok_only)]
                if vals:
                    pos.append(i + (j - (len(backends) - 1) / 2) * width)
                    data.append(vals)
                    cols.append(color(c, i))
                    hs.append(hatch[b])
        if data:
            bp = ax.boxplot(data, positions=pos, widths=width * 0.8, patch_artist=True, showfliers=True,
                            medianprops={"color": "black"}, flierprops={"markersize": 3})
            for patch, col, h in zip(bp["boxes"], cols, hs):
                patch.set_facecolor(col)
                patch.set_alpha(0.85)
                patch.set_hatch(h)
        ax.set_xticks(range(len(conds)), conds)
        ax.set_ylabel(label)
        if len(backends) > 1:                             # which boxes are which: plain = first, hatched = second
            from matplotlib.patches import Patch
            ax.legend(handles=[Patch(facecolor="#d1d5db", hatch=hatch[b], edgecolor="#6b7280", label=b) for b in backends],
                      frameon=False, fontsize=8)
        save(fig, stem)

    # trajectories: one panel per controller and backend
    runs_by_dir = {}
    for d in a.runs:
        for f in (Path(d) / "trajectories").glob("*.csv"):
            runs_by_dir[f.stem] = f
    fig, axes = plt.subplots(len(backends), len(conds), figsize=(2.3 * len(conds), 2.3 * len(backends)), squeeze=False)
    for j, b in enumerate(backends):
        for i, c in enumerate(conds):
            ax = axes[j][i]
            ax.add_patch(plt.Rectangle((0, 0), 1000, 1000, fill=False, lw=0.8, color="#9ca3af"))
            eps = [r for r in rows if r["condition"] == c and r["backend"] == b]
            for r in eps:
                f = runs_by_dir.get(f"{c}_{b}_{r['index']:03d}")
                if f is None:
                    continue
                xs, ys = [], []
                with f.open(encoding="utf-8") as fh:
                    for k, line in enumerate(csv.DictReader(fh)):
                        xs.append(float(line["x"]))
                        ys.append(float(line["y"]))
                ax.plot(xs, ys, color=color(c, i), lw=0.8, alpha=0.8)
                ax.plot(r["target_x"], r["target_y"], marker="o", ms=3,
                        color="black" if r["success"] else "#dc2626", mfc="none" if r["success"] else "#dc2626")
            ax.set_xlim(-20, 1020)
            ax.set_ylim(-20, 1020)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_visible(False)
            ax.set_title(f"{c} · {b}", fontsize=8)
    save(fig, "trajectories")
    print("\n".join(md))
    print(f"\nwrote {out}: summary.csv, table.md, table.tex, success_rate, time_to_target, final_error, trajectories (.png, .pdf)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run an experiment spec")
    r.add_argument("spec")
    r.add_argument("--backend", choices=["sim", "app"], default="sim", help="sim: in-process simulator; app: the running app (real field)")
    r.add_argument("--randomized", action="store_true", help="sim only: evaluate in randomized simulators")
    r.add_argument("--url", default="http://127.0.0.1:8000")
    r.add_argument("--episodes", type=int, default=0, help="override the spec's number of targets")
    r.add_argument("--only", default="", help="comma-separated condition names")
    r.add_argument("--out", default=str(ROOT / "experiments" / "runs"))
    r.add_argument("--yes", action="store_true", help="real field: start without asking")
    rp = sub.add_parser("report", help="figures and tables from run folders")
    rp.add_argument("runs", nargs="+")
    rp.add_argument("--out", default=None, help="default: results/<experiment>/")
    a = ap.parse_args()
    cmd_run(a) if a.cmd == "run" else cmd_report(a)


if __name__ == "__main__":
    main()
