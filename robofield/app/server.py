"""Small HTTP server for the Control Center: serves the web page and a JSON API.

GET  /                      the web GUI
GET  /api/state             everything the GUI shows (robots, tracker, safety, log)
POST /api/goal              {"id", "x", "y", "theta"}
POST /api/path              {"id", "points": [[x, y], ...], "theta": null | rad}
POST /api/drive             {"id", "left", "right"}       (repeat at ~10 Hz while driving)
POST /api/stop              {"id"} or {} for all
POST /api/estop             {"on": true | false}
POST /api/car               {"id", "name"?, "color"?, "leds"?, "max_speed"?, "gripper"?, "enabled"?}
POST /api/identify          {"id"}
POST /api/settings          {"speed_limit"?, "geofence_margin_mm"?, "avoid_collisions"?, ...}
POST /api/flash             {"id", "method": "ota" | "usb"}
GET  /tuning                the live tuning page
GET  /api/tuning            current parameter values + the schema that describes them
POST /api/tuning            {param: value, ...}  - applies immediately, next goal/path onward
POST /api/tuning/save       persist the current values into config/field.yaml
POST /api/tuning/reset      back to the built-in defaults (not saved until /api/tuning/save)

GET  /api/moves             the saved moves (drawn paths kept to replay, config/moves/*.json)
POST /api/moves/save        {"id", "name"} saves that car's last drawn path; or {"name", "points", "theta"}
POST /api/moves/start       {"id", "name"}  park the car at the move's start, facing along it
POST /api/moves/play        {"id", "name"}  drive the move (going to its start first if needed)
"""
from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

STATIC = Path(__file__).resolve().parent / "static"


def make_handler(cc):
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

        def do_GET(self):
            path = self.path.split("?", 1)[0]               # ignore ?caption=... etc. (read by the page itself)
            if path in ("/", "/index.html"):
                body = (STATIC / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path in ("/tuning", "/tuning.html"):
                body = (STATIC / "tuning.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/state":
                self._json(cc.snapshot())
            elif path == "/api/tuning":
                self._json(cc.get_tuning())
            elif path == "/api/moves":
                self._json({"moves": cc.list_moves()})
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            p = self.path
            ok, msg = True, "ok"
            try:
                if p == "/api/goal":
                    ok, msg = cc.set_goal(int(d["id"]), float(d["x"]), float(d["y"]), float(d.get("theta", 0.0)))
                elif p == "/api/path":
                    th = d.get("theta")
                    ok, msg = cc.set_path(int(d["id"]), [(float(x), float(y)) for x, y in d["points"]],
                                          None if th is None else float(th))
                elif p == "/api/drive":
                    cc.drive(int(d["id"]), float(d["left"]), float(d["right"]))
                elif p == "/api/stop":
                    cc.stop(int(d["id"]) if "id" in d else None)
                elif p == "/api/estop":
                    cc.set_estop(bool(d.get("on", True)))
                elif p == "/api/car":
                    rid = int(d.pop("id"))
                    cc.update_car(rid, **d)
                elif p == "/api/identify":
                    cc.identify(int(d["id"]))
                elif p == "/api/settings":
                    cc.update_settings(**d)
                elif p == "/api/flash":
                    ok, msg = cc.flash(int(d["id"]), d.get("method", "ota"))
                elif p == "/api/tuning":
                    ok, msg = cc.set_tuning(**d)
                elif p == "/api/tuning/save":
                    ok, msg = cc.save_tuning()
                elif p == "/api/tuning/reset":
                    ok, msg = cc.reset_tuning()
                elif p == "/api/moves/save":
                    pts = d.get("points")
                    th = d.get("theta")
                    ok, msg = cc.save_move(int(d.get("id", 0)), d["name"],
                                           None if pts is None else [(float(x), float(y)) for x, y in pts],
                                           None if th is None else float(th))
                elif p == "/api/moves/start":
                    ok, msg = cc.move_to_start(int(d["id"]), d["name"])
                elif p == "/api/moves/play":
                    ok, msg = cc.play_move(int(d["id"]), d["name"])
                else:
                    return self._json({"error": "not found"}, 404)
            except (KeyError, TypeError, ValueError) as e:
                return self._json({"ok": False, "msg": f"bad request: {e}"}, 400)
            self._json({"ok": ok, "msg": msg})

    return Handler


def serve(cc, host: str = "0.0.0.0", port: int = 8000):
    httpd = ThreadingHTTPServer((host, port), make_handler(cc))
    return httpd
