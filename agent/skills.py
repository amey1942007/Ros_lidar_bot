"""Composite tools executed in the agent process. They only use robot.snapshot() / robot.call()
(Pi tools) and the VLM, so nothing here drives hardware directly. Every loop honours the step cancel
event (set on cancel, stop word or human command) and the E-STOP latch."""
import json
import os
import threading
import time
from pathlib import Path

from robot.base import new_id

FOCUS = ("Look at this image. {q} Reply with ONLY JSON: "
         '{{"found": true|false|null, "description": "<one short sentence>", "bbox": [x1,y1,x2,y2] or null}}. '
         "bbox is in pixels of this {w}x{h} image, only when found is true. Never guess what you cannot see.")


class LocationStore:
    def __init__(self, path):
        self.path = Path(os.path.expandvars(os.path.expanduser(path)))
        self._lock = threading.Lock()

    def load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _save(self, d):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=2)
        os.replace(tmp, self.path)

    def put(self, name, x, y, yaw, note=None):
        with self._lock:
            d = self.load()
            old = d.get(name, {})
            d[name] = {"x": x, "y": y, "yaw": yaw, "note": note if note is not None else old.get("note", "")}
            self._save(d)
            return {"name": name, **d[name]}

    def delete(self, name):
        with self._lock:
            d = self.load()
            if name not in d:
                return False
            del d[name]
            self._save(d)
            return True

    def brief(self, limit=1200):
        d = self.load()
        if not d:
            return ""
        parts = [f"{n}({v['x']:.2f},{v['y']:.2f},{v['yaw']:.2f})" + (f" - {v['note']}" if v.get("note") else "")
                 for n, v in d.items()]
        return ("Saved locations: " + "; ".join(parts))[:limit]


def valid_bbox(b, w, h):
    if not (isinstance(b, list) and len(b) == 4 and all(isinstance(v, (int, float)) for v in b)):
        return None
    x1, y1, x2, y2 = [float(v) for v in b]
    x1, x2 = max(0.0, min(x1, x2)), min(float(w), max(x1, x2))
    y1, y2 = max(0.0, min(y1, y2)), min(float(h), max(y1, y2))
    if x2 - x1 < 1 or y2 - y1 < 1:
        return None
    return [round(x1), round(y1), round(x2), round(y2)]


class Err(Exception):
    def __init__(self, code, msg):
        super().__init__(msg)
        self.code = code


class Skills:
    MOTION = {"start_exploring", "recall_location", "return_home", "go_to_object", "follow", "scan_area"}

    def __init__(self, cfg, model, robot):
        a = cfg.get("agent", {})
        self.model = model
        self.robot = robot
        self.store = LocationStore(a.get("locations_file", "~/.vlm_bridge/locations.json"))
        self.hfov = float(cfg.get("camera", {}).get("hfov_deg", 62))
        self.rec_dir = a.get("record_dir", "recordings")
        self.interval_scale = 1.0       # tests shrink waits
        self._rec = None

    # ---------- helpers ----------
    def _check(self, cancel, motion=False):
        if cancel is not None and cancel.is_set():
            raise Err("CANCELLED", "cancelled")
        if motion and self.robot.estop_latched:
            raise Err("ESTOP_LATCHED", "refused: E-STOP latched")
        if self.robot.human_override:
            raise Err("HUMAN_OVERRIDE", "human is driving")

    def _sleep(self, cancel, secs, motion=False):
        end = time.time() + secs * self.interval_scale
        while time.time() < end:
            self._check(cancel, motion)
            time.sleep(0.02)

    def _call(self, tool, args, cancel, timeout_s=30):
        self._check(cancel, motion=tool in ("goto_pose", "move", "look", "map"))
        res = self.robot.call(tool, args, new_id("k"), cancel, timeout_s=timeout_s)
        if not res.get("ok"):
            e = res.get("error") or {}
            raise Err(e.get("code", "ROBOT_ERROR"), f"{tool}: {e.get('message', res)}")
        return res.get("data") or {}

    def _pose(self, cancel):
        d = self._call("get_status", {}, cancel, 3)
        p = d.get("pose") or {}
        if "x" not in p or "y" not in p:
            raise Err("NO_POSE", "robot status has no pose")
        return float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0))

    def look_once(self, cancel, target=None, goal=None):
        self._check(cancel)
        jpeg, meta = self.robot.snapshot()
        w, h = meta.get("width", 640), meta.get("height", 480)
        if goal:
            q = f"Goal: {goal}. Is the goal met / the thing visible now? found = goal met."
        elif target:
            q = f"Is a '{target}' visible? found = visible."
        else:
            q = "Describe the scene briefly. found = null."
        _, content, _ = self.model.chat([{"role": "user", "content": FOCUS.format(q=q, w=w, h=h), "images": [jpeg]}],
                                        cancel_event=cancel)
        self._check(cancel)
        from .loop import extract_json
        try:
            obj = extract_json(content)
        except Exception:
            obj = {"found": None, "description": content.strip()[:200], "bbox": None}
        found = obj.get("found")
        found = found if isinstance(found, bool) else None
        bbox = valid_bbox(obj.get("bbox"), w, h) if found else None
        return {"found": found, "description": str(obj.get("description", ""))[:300], "bbox": bbox,
                "image": {"width": w, "height": h}}

    def _watch(self, cancel, goal, interval, max_s, keep_history=False, motion=False):
        t0, frames, hist, last = time.time(), 0, [], None
        while True:
            last = self.look_once(cancel, goal=goal) if goal else self.look_once(cancel)
            frames += 1
            if keep_history:
                hist.append(last["description"])
            if goal and last["found"]:
                return {"met": True, "frames": frames, "last": last, "history": hist[-20:], "reason": "goal"}
            if time.time() - t0 + interval * self.interval_scale > max_s * self.interval_scale:
                return {"met": False, "frames": frames, "last": last, "history": hist[-20:], "reason": "timeout"}
            self._sleep(cancel, interval, motion)

    # ---------- tools ----------
    def run(self, tool, args, cancel):
        try:
            if tool in self.MOTION:
                self._check(cancel, motion=True)
            return {"ok": True, "data": getattr(self, "t_" + tool)(args, cancel)}
        except Err as e:
            return {"ok": False, "error": {"code": e.code, "message": str(e)}}

    def t_describe_scene(self, a, c):
        return self.look_once(c, target=a.get("target"))

    def t_feed(self, a, c):
        r = self._watch(c, a.get("goal"), a.get("interval_s", 2.0), a.get("max_s", 30), keep_history=True)
        return {"purpose": a["purpose"], **r}

    def t_watch_for(self, a, c):
        r = self._watch(c, a["goal"], 1.0, a.get("max_s", 60))
        r.pop("history", None)
        return r

    def t_start_exploring(self, a, c):
        args = {"action": "explore", "enable": True}
        if "max_s" in a:
            args["max_s"] = a["max_s"]
        self._call("map", args, c, 10)
        if not a.get("until_seen"):
            return {"exploring": True, "stopped_because": None, "last": None}
        try:
            r = self._watch(c, f"a {a['until_seen']} is visible", a.get("check_interval_s", 2.0),
                            a.get("max_s", 600), motion=True)
        finally:
            try:
                self.robot.call("map", {"action": "explore", "enable": False}, new_id("k"), None, timeout_s=10)
            except Exception:
                pass
        return {"exploring": False, "stopped_because": "seen" if r["met"] else "timeout", "last": r["last"]}

    def t_stop_exploring(self, a, c):
        res = self.robot.call("map", {"action": "explore", "enable": False}, new_id("k"), None, timeout_s=10)
        if not res.get("ok"):
            raise Err((res.get("error") or {}).get("code", "ROBOT_ERROR"), str(res.get("error")))
        return {"exploring": False}

    def t_save_map(self, a, c):
        name = a.get("name") or time.strftime("map-%Y%m%d-%H%M%S")
        return {"saved": name, "map": self._call("map", {"action": "save", "name": name}, c, 30)}

    def t_save_location(self, a, c):
        x, y, yaw = self._pose(c)
        return {"saved": self.store.put(a["name"], x, y, yaw, a.get("note"))}

    def t_note_location(self, a, c):
        name = a.get("name")
        if name and name in self.store.load():
            v = self.store.load()[name]
            return {"saved": self.store.put(name, v["x"], v["y"], v["yaw"], a["note"])}
        x, y, yaw = self._pose(c)
        if not name:
            n = 1
            while f"spot-{n}" in self.store.load():
                n += 1
            name = f"spot-{n}"
        return {"saved": self.store.put(name, x, y, yaw, a["note"])}

    def t_recall_location(self, a, c):
        loc = self.store.load().get(a["name"])
        if not loc:
            raise Err("UNKNOWN_LOCATION", f"no saved location '{a['name']}'")
        nav = self._call("goto_pose", {"x": loc["x"], "y": loc["y"], "yaw": loc["yaw"], "frame": "map"}, c, 120)
        return {"location": {"name": a["name"], **loc}, "nav": nav}

    def t_return_home(self, a, c):
        return self.t_recall_location({"name": "home"}, c)

    def t_list_locations(self, a, c):
        return {"locations": [{"name": n, **v} for n, v in self.store.load().items()]}

    def t_delete_location(self, a, c):
        if not self.store.delete(a["name"]):
            raise Err("UNKNOWN_LOCATION", f"no saved location '{a['name']}'")
        return {"deleted": a["name"]}

    def t_wait(self, a, c):
        t0 = time.time()
        self._sleep(c, a["seconds"])
        return {"waited_s": round(time.time() - t0, 2)}

    def t_scan_area(self, a, c):
        use = a.get("use", "head")
        angles = a.get("angles_deg") or ([-60, -30, 0, 30, 60] if use == "head" else [0, 60, 60, 60, 60, 60])
        views, cur = [], 0.0
        for deg in angles:
            if use == "head":
                self._call("look", {"pan": {"mode": "relative", "deg": deg - cur}, "settle_ms": 300}, c, 10)
                cur = deg
            elif deg:
                self._call("move", {"turn_deg": deg}, c, 30)
                cur += deg
            v = self.look_once(c, target=a["target"])
            views.append({"deg": cur, **{k: v[k] for k in ("found", "description", "bbox")}})
            if v["found"]:
                return {"found": True, "at_deg": cur, "views": views}
        return {"found": False, "at_deg": None, "views": views}

    def _bearing(self, v):
        b, w = v["bbox"], v["image"]["width"]
        return -((b[0] + b[2]) / 2 - w / 2) / w * self.hfov   # positive = left

    def t_go_to_object(self, a, c):
        budget, driven, last = a.get("max_m", 2.0), 0.0, None
        while True:
            last = self.look_once(c, target=a["target"])
            if not last["found"] or not last["bbox"]:
                return {"reached": False, "driven_m": round(driven, 2), "reason": "lost", "last": last}
            b = last["bbox"]
            if (b[3] - b[1]) / last["image"]["height"] >= 0.6:
                return {"reached": True, "driven_m": round(driven, 2), "reason": "close", "last": last}
            if driven >= budget - 1e-6:
                return {"reached": False, "driven_m": round(driven, 2), "reason": "budget", "last": last}
            hop = min(0.5, budget - driven)
            turn = self._bearing(last)
            self._call("move", {"turn_deg": max(-180, min(180, turn)), "distance_m": hop}, c, 30)
            driven += hop

    def t_follow(self, a, c):
        t0, max_s, last = time.time(), a.get("max_s", 60), None
        while True:
            last = self.look_once(c, target=a["target"])
            if not last["found"] or not last["bbox"]:
                return {"followed_s": round(time.time() - t0, 1), "reason": "lost", "last": last}
            if (time.time() - t0) >= max_s * self.interval_scale:
                return {"followed_s": round(time.time() - t0, 1), "reason": "timeout", "last": last}
            b = last["bbox"]
            near = (b[3] - b[1]) / last["image"]["height"] >= 0.6
            self._call("move", {"turn_deg": self._bearing(last), "distance_m": 0.0 if near else 0.3}, c, 30)

    def t_record_video(self, a, c):
        if a["action"] == "stop":
            r = self._rec
            if not r:
                return {"recording": False, "dir": None, "frames": 0}
            r["stop"].set()
            r["thread"].join(3)
            self._rec = None
            return {"recording": False, "dir": str(r["dir"]), "frames": r["n"]}
        if self._rec:
            return {"recording": True, "dir": str(self._rec["dir"]), "frames": self._rec["n"]}
        d = Path(self.rec_dir) / time.strftime("rec-%Y%m%d-%H%M%S")
        d.mkdir(parents=True, exist_ok=True)
        r = {"dir": d, "n": 0, "stop": threading.Event()}
        period = 1.0 / a.get("fps", 2.0)

        def loop():
            while not r["stop"].is_set():
                try:
                    jpeg, _ = self.robot.snapshot()
                    (d / f"{r['n']:06d}.jpg").write_bytes(jpeg)
                    r["n"] += 1
                except Exception:
                    pass
                r["stop"].wait(period)
        r["thread"] = threading.Thread(target=loop, daemon=True, name="record")
        r["thread"].start()
        self._rec = r
        return {"recording": True, "dir": str(d), "frames": 0}
