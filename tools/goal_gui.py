"""Click-and-drag goal control, live top view of the field.

    python tools/goal_gui.py                         # simulator (no hardware needed)
    python tools/goal_gui.py --real --robot 0        # real field: Pi tracker + car over WiFi
    python tools/goal_gui.py --real --pi robofield-pi.local    # also draw goal/path on the Pi preview

Press the mouse where the robot should go, drag toward the direction it should face, release.
The robot plans a straight path (kept 60 mm from the walls), turns, follows it, stops, and turns
to the requested heading. Keys: space = stop, r = new random start (simulator only), q = quit.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import matplotlib  # noqa: E402

from robofield.control import DiffDrive, PoseController  # noqa: E402
from robofield.core import RobotCommand, load_config  # noqa: E402


class GoalGUI:
    def __init__(self, backend, cfg, robot=0, overlay=None, show=True):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        self.plt, self.backend, self.cfg, self.robot, self.overlay = plt, backend, cfg, robot, overlay
        self.ctl = PoseController(DiffDrive.from_config(cfg), deadzone=cfg["robot_model"]["deadzone"])
        self.trail, self.press, self.status = [], None, "click and drag to set a goal"
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        ax = self.ax
        ax.set_xlim(-150, 1150); ax.set_ylim(-150, 1150); ax.set_aspect("equal")
        ax.add_patch(Rectangle((0, 0), 1000, 1000, fc="#f2f2ee", ec="#e08a3c", lw=4))
        for v in range(0, 1001, 100):
            lw = 1.2 if v % 250 == 0 else 0.4
            ax.plot([v, v], [0, 1000], color="#cccccc", lw=lw, zorder=0)
            ax.plot([0, 1000], [v, v], color="#cccccc", lw=lw, zorder=0)
        ax.add_patch(Rectangle((60, 60), 880, 880, fill=False, ls="--", ec="#bbbbbb"))
        for tid, (x, y, z) in cfg["markers"]["corners"].items():
            ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc="#222", ec="#1f3a93"))
            ax.text(x, y, f"ID {tid}", color="w", ha="center", va="center", fontsize=8)
        ax.annotate("", (200, 0), (0, 0), arrowprops=dict(arrowstyle="->", color="red", lw=2))
        ax.annotate("", (0, 200), (0, 0), arrowprops=dict(arrowstyle="->", color="green", lw=2))
        ax.text(-60, -110, "FRONT", fontsize=10)
        self.robot_patch = Polygon([[0, 0]], closed=True, fc="#2b6cb0", ec="k", zorder=5)
        ax.add_patch(self.robot_patch)
        (self.trail_line,) = ax.plot([], [], color="#2b6cb0", lw=1, alpha=0.6)
        (self.path_line,) = ax.plot([], [], color="orange", lw=2, ls="-", marker=".", ms=4)
        self.goal_arrow = None
        self.title = ax.set_title(self.status)
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.state = backend.reset()
        if show:
            self.timer = self.fig.canvas.new_timer(interval=int(1000 / backend.control_hz))
            self.timer.add_callback(self.update)
            self.timer.start()

    # ---------------------------------------------------------------- input
    def _on_press(self, e):
        if e.inaxes == self.ax and e.xdata is not None:
            self.press = (e.xdata, e.ydata)

    def _on_release(self, e):
        if self.press is None or e.xdata is None:
            return
        x, y = self.press
        th = math.atan2(e.ydata - y, e.xdata - x) if math.dist(self.press, (e.xdata, e.ydata)) > 15 else 0.0
        self.set_goal(x, y, th)
        self.press = None

    def _on_key(self, e):
        if e.key == " ":
            self.ctl.phase = "IDLE"; self.status = "stopped"
        elif e.key == "r" and hasattr(self.backend, "true_pose"):
            self.state = self.backend.reset(); self.trail.clear()
        elif e.key == "q":
            self.plt.close(self.fig)

    def set_goal(self, x, y, th):
        r = self.state.robots.get(self.robot)
        if r is None:
            self.status = "robot not visible"; return
        self.ctl.set_goal(x, y, th, (r.x, r.y))
        gx, gy, gth = self.ctl.goal
        self.status = f"goal ({gx:.0f}, {gy:.0f}) heading {math.degrees(gth):.0f} deg"
        if self.goal_arrow:
            self.goal_arrow.remove()
        self.goal_arrow = self.ax.annotate("", (gx + 70 * math.cos(gth), gy + 70 * math.sin(gth)), (gx, gy),
                                           arrowprops=dict(arrowstyle="-|>", color="orange", lw=3))
        pts = [(r.x, r.y)] + self.ctl.path
        self.path_line.set_data([p[0] for p in pts], [p[1] for p in pts])
        if self.overlay:
            self.overlay.send(goal=self.ctl.goal, path=pts)

    # ---------------------------------------------------------------- loop
    def update(self):
        r = self.state.robots.get(self.robot)
        cmd = RobotCommand()
        if r is not None and r.visible and self.ctl.phase not in ("IDLE", "DONE"):
            cmd = self.ctl.command(r.x, r.y, r.theta)
            if self.ctl.done:
                gx, gy, gth = self.ctl.goal
                self.status = (f"reached: {math.hypot(r.x - gx, r.y - gy):.0f} mm, "
                               f"{abs(math.degrees(math.remainder(r.theta - gth, 2 * math.pi))):.1f} deg off")
        self.backend.apply({self.robot: cmd})
        self.state = self.backend.step()
        r = self.state.robots.get(self.robot)
        if r is not None:
            c, s = math.cos(r.theta), math.sin(r.theta)
            body = [(45, 0), (-25, 25), (-25, -25)]
            self.robot_patch.set_xy([(r.x + c * bx - s * by, r.y + s * bx + c * by) for bx, by in body])
            self.trail.append((r.x, r.y)); self.trail = self.trail[-400:]
            self.trail_line.set_data([p[0] for p in self.trail], [p[1] for p in self.trail])
            phase = "" if self.ctl.phase in ("IDLE", "DONE") else f"  [{self.ctl.phase}]"
            self.title.set_text(f"robot {self.robot}: ({r.x:.0f}, {r.y:.0f}) {math.degrees(r.theta):.0f} deg   {self.status}{phase}")
        self.fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true", help="use the real field instead of the simulator")
    ap.add_argument("--robot", type=int, default=0)
    ap.add_argument("--pi", default=None, help="Pi hostname/IP: also draw goal + path on its preview")
    ap.add_argument("--config", default="config/field.yaml")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.real:
        from robofield.real import RealBackend
        backend = RealBackend(cfg, robot_ids=[a.robot])
    else:
        from robofield.sim.backend import SimBackend
        backend = SimBackend(cfg, n_robots=1, seed=0, randomize=True, markers=False)
    overlay = None
    if a.pi:
        from robofield.comm.pose_link import OverlaySender
        overlay = OverlaySender(a.pi)
    gui = GoalGUI(backend, cfg, a.robot, overlay)
    try:
        gui.plt.show()
    finally:
        backend.stop(); backend.close()


if __name__ == "__main__":
    main()
