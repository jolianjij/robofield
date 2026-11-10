"""Small HTTP server for the Control Center and the RoboField app: web pages and a JSON API.

Serves either a bare ControlCenter (tools/control_center.py) or an AppRuntime (tools/robofield_app.py:
simulator / real-field switch, training, tools). Routes marked [app] need the AppRuntime.

Pages
GET  /                      the Control Center page (map, cars, go-to-pose, paths, camera, moves)
GET  /tuning                the live tuning page
GET  /view3d                the 3D view of the field (simulator or real)
GET  /app                   [app] the app shell with all the tabs
GET  /training, /tools      [app] the Training and Tools tabs
GET  /camera, /lab          [app] the Camera and Lab tabs
GET  /camera.mjpg           [app] the live camera: the Pi's view, or the simulator's overhead camera

Control Center
GET  /api/state             everything the pages show (robots, objects, tracker, safety, log)
GET  /api/scene             fixed geometry of the field and cars, for the 3D view
POST /api/goal              {"id", "x", "y", "theta"}
POST /api/path              {"id", "points": [[x, y], ...], "theta": null | rad}
POST /api/drive             {"id", "left", "right"}       (repeat at ~10 Hz while driving)
POST /api/stop              {"id"} or {} for all
POST /api/estop             {"on": true | false}
POST /api/car               {"id", "name"?, "color"?, "leds"?, "max_speed"?, "gripper"?, "enabled"?}
POST /api/identify          {"id"}
POST /api/settings          {"speed_limit"?, "geofence_margin_mm"?, "avoid_collisions"?, ...}
POST /api/flash             {"id", "method": "ota" | "usb"}
GET  /api/tuning            current parameter values + the schema that describes them
POST /api/tuning            {param: value, ...}  - applies immediately, next goal/path onward
POST /api/tuning/save       persist the current values into config/tuning.yaml
POST /api/tuning/reset      back to the built-in defaults (not saved until /api/tuning/save)
GET  /api/moves             the saved moves (drawn paths kept to replay, config/moves/*.json)
POST /api/moves/save        {"id", "name"} saves that car's last drawn path; or {"name", "points", "theta"}
POST /api/moves/start       {"id", "name"}  park the car at the move's start, facing along it
POST /api/moves/play        {"id", "name"}  drive the move (going to its start first if needed)
POST /api/policy            {"id", "model", "speed"?, "no_reverse"?, "place"?, "goal"?: [x, y]}  an RL policy drives the car
POST /api/sim/place         {"id", "x", "y", "theta"}   simulator only: put a car there

App [app]
GET  /api/app               mode, Pi address, status
POST /api/app/mode          {"mode": "sim" | "real", "pi"?, "sim_robots"?}
GET  /api/games             the games, and which can be trained
GET  /api/models            the trained policies in models/
GET  /api/models/<name>/curve   its learning curve
GET  /api/tools             the tools the app can run
GET  /api/jobs              background jobs (training, evaluation, tools)
GET  /api/jobs/<id>?since=n one job, with its output from line n on
POST /api/jobs/train        {"game", "timesteps", "envs", "name"?, "randomize"?, "resume"?}
POST /api/jobs/eval         {"model", "episodes"?, "randomize"?}
POST /api/jobs/tool         {"tool", "robot"?, "seconds"?, "write"?}
POST /api/jobs/stop         {"id"}
POST /api/jobs/autotune     {"where": "sim" | "real", "trials"?, "precision"?, "group"?, "robot"?}
POST /api/camera/snapshot   save the current camera frame to recordings/
POST /api/camera/record     {"on": true | false}   record the camera to recordings/*.mp4
GET  /api/lab               the custom tasks, policies and systems (custom/)
GET  /api/lab/<kind>/<name> one file's source
POST /api/lab/new           {"kind", "name"}               a new file from the template
POST /api/lab/save          {"kind", "name", "source"}
POST /api/lab/check         {"kind", "name", "source"}     load it and run it briefly in a simulator
POST /api/lab/delete        {"kind", "name"}               (moved to custom/.deleted/)
POST /api/lab/run           {"kind", "name", "ids": [..], "speed"?}   a policy or system drives cars
"""
from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

STATIC = Path(__file__).resolve().parent / "static"
PAGES = {"/": "index.html", "/index.html": "index.html", "/tuning": "tuning.html", "/tuning.html": "tuning.html",
         "/view3d": "view3d.html", "/app": "app.html", "/training": "training.html", "/tools": "tools.html",
         "/camera": "camera.html", "/lab": "lab.html"}
APP_PAGES = {"/app", "/training", "/tools", "/camera", "/lab"}


def make_handler(target):
    """target: an AppRuntime (has .cc, switchable) or a plain ControlCenter."""
    rt = target if hasattr(target, "switch") else None

    def cc():
        return rt.cc if rt else target

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _page(self, name):
            body = (STATIC / name).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            path, q = url.path, parse_qs(url.query)        # ?caption=... etc. is read by the page itself
            if path == "/camera.mjpg" and rt is not None:
                return self._mjpeg()
            if path == "/robofield.ico":
                body = (STATIC / "robofield.ico").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/x-icon")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                return self.wfile.write(body)
            if path in PAGES:
                if path in APP_PAGES and rt is None:
                    return self._json({"error": "this page is part of the RoboField app (tools/robofield_app.py)"}, 404)
                return self._page(PAGES[path])
            if path == "/api/state":
                return self._json(cc().snapshot())
            if path == "/api/scene":
                return self._json(cc().scene())
            if path == "/api/tuning":
                return self._json(cc().get_tuning())
            if path == "/api/moves":
                return self._json({"moves": cc().list_moves()})
            if rt is not None:
                if path == "/api/app":
                    return self._json(rt.info())
                if path == "/api/games":
                    return self._json({"games": rt.games()})
                if path == "/api/models":
                    return self._json({"models": rt.models()})
                if path.startswith("/api/models/") and path.endswith("/curve"):
                    return self._json({"curve": rt.curve(path[len("/api/models/"):-len("/curve")])})
                if path == "/api/tools":
                    from .runtime import TOOLS
                    return self._json({"tools": TOOLS})
                if path == "/api/jobs":
                    return self._json({"jobs": rt.jobs.list()})
                if path == "/api/lab":
                    from robofield import lab
                    return self._json({"items": lab.list_all(), "errors": rt.lab_errors})
                if path.startswith("/api/lab/"):
                    from robofield import lab
                    try:
                        _, _, _, kind, name = path.split("/")
                        return self._json({"kind": kind, "name": name,
                                           "source": lab.path_of(kind, name).read_text(encoding="utf-8")})
                    except (ValueError, OSError) as e:
                        return self._json({"error": str(e)}, 404)
                if path.startswith("/api/jobs/"):
                    try:
                        job = rt.jobs.get(int(path.rsplit("/", 1)[1]), int(q.get("since", ["0"])[0]))
                    except ValueError:
                        job = None
                    return self._json(job) if job else self._json({"error": "no such job"}, 404)
            self._json({"error": "not found"}, 404)

        def _mjpeg(self):
            """The live camera, as a stream of JPEGs (an <img> can show it)."""
            hub = rt.camera
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while True:
                    seq, jpeg = hub.wait_frame(seq, 2.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            p = urlparse(self.path).path
            c = cc()
            ok, msg = True, "ok"
            try:
                if p == "/api/goal":
                    ok, msg = c.set_goal(int(d["id"]), float(d["x"]), float(d["y"]), float(d.get("theta", 0.0)))
                elif p == "/api/path":
                    th = d.get("theta")
                    ok, msg = c.set_path(int(d["id"]), [(float(x), float(y)) for x, y in d["points"]],
                                         None if th is None else float(th))
                elif p == "/api/drive":
                    c.drive(int(d["id"]), float(d["left"]), float(d["right"]))
                elif p == "/api/stop":
                    c.stop(int(d["id"]) if "id" in d else None)
                elif p == "/api/estop":
                    c.set_estop(bool(d.get("on", True)))
                elif p == "/api/car":
                    rid = int(d.pop("id"))
                    c.update_car(rid, **d)
                elif p == "/api/identify":
                    c.identify(int(d["id"]))
                elif p == "/api/settings":
                    c.update_settings(**d)
                elif p == "/api/flash":
                    ok, msg = c.flash(int(d["id"]), d.get("method", "ota"))
                elif p == "/api/tuning":
                    ok, msg = c.set_tuning(**d)
                elif p == "/api/tuning/save":
                    ok, msg = c.save_tuning()
                elif p == "/api/tuning/reset":
                    ok, msg = c.reset_tuning()
                elif p == "/api/moves/save":
                    pts = d.get("points")
                    th = d.get("theta")
                    ok, msg = c.save_move(int(d.get("id", 0)), d["name"],
                                          None if pts is None else [(float(x), float(y)) for x, y in pts],
                                          None if th is None else float(th))
                elif p == "/api/moves/start":
                    ok, msg = c.move_to_start(int(d["id"]), d["name"])
                elif p == "/api/moves/play":
                    ok, msg = c.play_move(int(d["id"]), d["name"])
                elif p == "/api/policy":
                    goal = d.get("goal")
                    ok, msg = c.run_policy(int(d["id"]), str(d["model"]), float(d.get("speed", 0.6)),
                                           bool(d.get("no_reverse", False)), bool(d.get("place", True)),
                                           None if goal is None else (float(goal[0]), float(goal[1])))
                elif p == "/api/sim/place":
                    ok, msg = c.place_car(int(d["id"]), float(d["x"]), float(d["y"]), float(d.get("theta", 0.0)))
                elif rt is not None and p == "/api/app/mode":
                    ok, msg = rt.switch(d.get("mode", "sim"), d.get("pi") or None, d.get("sim_robots"))
                elif rt is not None and p == "/api/jobs/train":
                    ok, msg = rt.start_training(d["game"], int(d.get("timesteps", 1_000_000)), int(d.get("envs", 8)),
                                                d.get("name") or None, bool(d.get("randomize", True)), d.get("resume") or None)
                elif rt is not None and p == "/api/jobs/eval":
                    ok, msg = rt.start_eval(str(d["model"]), int(d.get("episodes", 10)), bool(d.get("randomize", False)))
                elif rt is not None and p == "/api/jobs/tool":
                    ok, msg = rt.start_tool(str(d["tool"]), int(d.get("robot", 0)), int(d.get("seconds", 60)),
                                            bool(d.get("write", False)))
                elif rt is not None and p == "/api/jobs/stop":
                    ok, msg = rt.jobs.stop(int(d["id"]))
                elif rt is not None and p == "/api/jobs/autotune":
                    ok, msg = rt.start_autotune(d.get("where", "sim"), int(d.get("trials", 0)), float(d.get("precision", 0.5)),
                                                d.get("group", "all"), int(d.get("robot", 0)))
                elif rt is not None and p == "/api/camera/snapshot":
                    ok, msg = rt.camera.snapshot()
                elif rt is not None and p == "/api/camera/record":
                    ok, msg = rt.camera.start_recording() if d.get("on", True) else rt.camera.stop_recording()
                elif rt is not None and p.startswith("/api/lab/"):
                    ok, msg, extra = self._lab(p[len("/api/lab/"):], d)
                    if extra is not None:
                        return self._json({"ok": ok, "msg": msg, **extra})
                else:
                    return self._json({"error": "not found"}, 404)
            except (KeyError, TypeError, ValueError) as e:
                return self._json({"ok": False, "msg": f"bad request: {e}"}, 400)
            self._json({"ok": ok, "msg": msg})

        def _lab(self, action, d):
            from robofield import lab
            kind, name = d.get("kind", ""), d.get("name", "")
            if action == "new":
                p = lab.path_of(kind, name)
                if p.exists():
                    return False, f"{p.name} already exists", None
                src = lab.template(kind, name)
                ok, msg = lab.save(kind, name, src)
                return ok, msg, {"name": p.stem, "source": src}
            if action == "save":
                ok, msg = lab.save(kind, name, d["source"])
                if kind == "task":
                    rt.lab_errors = lab.register_tasks()
                return ok, msg, {"name": lab.path_of(kind, name).stem}
            if action == "check":
                res = lab.check_isolated(kind, name, d["source"])
                return res["ok"], "looks good" if res["ok"] else "problem found", res
            if action == "delete":
                ok, msg = lab.delete(kind, name)
                return ok, msg, None
            if action == "run":
                ok, msg = cc().run_program(kind, name, d.get("ids", [0]), float(d.get("speed", 1.0)))
                return ok, msg, None
            return False, "unknown Lab action", None

    return Handler


def serve(target, host: str = "0.0.0.0", port: int = 8000):
    """target: a ControlCenter, or an AppRuntime for the full app."""
    return ThreadingHTTPServer((host, port), make_handler(target))
