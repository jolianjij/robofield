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
            if self.path in ("/", "/index.html"):
                body = (STATIC / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path.startswith("/api/state"):
                self._json(cc.snapshot())
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
                else:
                    return self._json({"error": "not found"}, 404)
            except (KeyError, TypeError, ValueError) as e:
                return self._json({"ok": False, "msg": f"bad request: {e}"}, 400)
            self._json({"ok": ok, "msg": msg})

    return Handler


def serve(cc, host: str = "0.0.0.0", port: int = 8000):
    httpd = ThreadingHTTPServer((host, port), make_handler(cc))
    return httpd
