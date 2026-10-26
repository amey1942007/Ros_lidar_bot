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
        self._jobs = {}                 # job_id -> job dict (background tools)
        self._jobs_lock = threading.Lock()
        self._job_n = 0

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

    def _watch(self, cancel, goal, interval, max_s, keep_history=False, motion=False, progress=None):
        t0, frames, hist, last = time.time(), 0, [], None
        progress = progress if progress is not None else {}
        while True:
            self._check(cancel, motion)
            last = self.look_once(cancel, goal=goal) if goal else self.look_once(cancel)
            frames += 1
            if keep_history:
                hist.append(last["description"])
            progress.update(frames=frames, last=last, **({"history": hist[-20:]} if keep_history else {}))
            if goal and last["found"]:
                return {"met": True, "frames": frames, "last": last, "history": hist[-20:], "reason": "goal"}
            if time.time() - t0 + interval * self.interval_scale > max_s * self.interval_scale:
                return {"met": False, "frames": frames, "last": last, "history": hist[-20:], "reason": "timeout"}
            self._sleep(cancel, interval, motion)

    # ---------- background jobs ----------
    def _start_job(self, tool, fn, info=None):
        """Run fn(job) in a thread. The job has its own cancel event; cancel_jobs() stops it."""
        info = info or {}
        with self._jobs_lock:
            self._job_n += 1
            jid = f"{tool}-{self._job_n}"
            job = {"id": jid, "tool": tool, "cancel": threading.Event(), "done": threading.Event(),
                   "status": "running", "result": None, "progress": {}, "info": info, "t0": time.time()}
            self._jobs[jid] = job

        def body():
            try:
                job["result"] = {**info, **fn(job)}
                job["status"] = "done"
            except Err as e:
                job["result"] = {**info, **job["progress"], "reason": e.code.lower(),
                                 "error": {"code": e.code, "message": str(e)}}
                job["status"] = "cancelled" if e.code in ("CANCELLED", "HUMAN_OVERRIDE", "ESTOP_LATCHED") else "failed"
            except Exception as e:
                job["result"] = {**info, "error": {"code": "INTERNAL", "message": repr(e)}}
                job["status"] = "failed"
            finally:
                job["done"].set()
        job["thread"] = threading.Thread(target=body, daemon=True, name="job-" + jid)
        job["thread"].start()
        return {"job_id": jid, "status": "running", **info}

    def _summary(self, j):
        out = {"job_id": j["id"], "tool": j["tool"], "status": j["status"],
               "elapsed_s": round(time.time() - j["t0"], 1)}
        if j["done"].is_set():
            out["result"] = j["result"]
        else:
            out.update(j["info"])
            out["progress"] = dict(j["progress"])
        return out

    def _resolve(self, refs, running_only=False, skip=()):
        """refs: job ids or tool names; empty = all (running) jobs except tools in skip."""
        with self._jobs_lock:
            jobs = list(self._jobs.values())
        if not refs:
            return [j for j in jobs if j["tool"] not in skip and (not running_only or not j["done"].is_set())]
        out = []
        for r in refs:
            hit = [j for j in jobs if j["id"] == r] or [j for j in jobs if j["tool"] == r and not j["done"].is_set()] \
                or [j for j in jobs if j["tool"] == r][-1:]
            out += [j for j in hit if j not in out]
        return out

    def running_jobs(self):
        return [j["id"] for j in self._resolve(None, running_only=True)]

    def cancel_jobs(self, ids=None, join_s=3.0):
        """Stop background jobs (all when ids is None) and wait for their threads to exit."""
        with self._jobs_lock:
            jobs = [j for j in self._jobs.values() if (ids is None or j["id"] in ids) and not j["done"].is_set()]
        for j in jobs:
            j["cancel"].set()
        for j in jobs:
            j["thread"].join(join_s)
        return [j["id"] for j in jobs]

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
        self._check(c)
        goal, interval, max_s = a.get("goal"), a.get("interval_s", 2.0), a.get("max_s", 30)
        return self._start_job("feed", lambda j: self._watch(j["cancel"], goal, interval, max_s, keep_history=True,
                                                             progress=j["progress"]),
                               {"purpose": a["purpose"], "goal": goal})

    def t_watch_for(self, a, c):
        self._check(c)
        goal, max_s = a["goal"], a.get("max_s", 60)

        def body(j):
            r = self._watch(j["cancel"], goal, 1.0, max_s, progress=j["progress"])
            r.pop("history", None)
            return r
        return self._start_job("watch_for", body, {"goal": goal})

    def _explore_off(self):
        try:
            self.robot.call("map", {"action": "explore", "enable": False}, new_id("k"), None, timeout_s=10)
        except Exception:
            pass

    def t_start_exploring(self, a, c):
        args = {"action": "explore", "enable": True}
        if "max_s" in a:
            args["max_s"] = a["max_s"]
        self._call("map", args, c, 10)
        until, max_s = a.get("until_seen"), a.get("max_s")

        def body(j):
            try:
                if until:
                    r = self._watch(j["cancel"], f"a {until} is visible", a.get("check_interval_s", 2.0),
                                    max_s or 600, motion=True, progress=j["progress"])
                    return {"exploring": False, "stopped_because": "seen" if r["met"] else "timeout", "last": r["last"]}
                self._sleep(j["cancel"], max_s if max_s else 1e9, motion=True)
                return {"exploring": False, "stopped_because": "timeout", "last": None}
            finally:
                self._explore_off()          # always switch exploration off when the job ends
        r = self._start_job("start_exploring", body, {"until_seen": until, "max_s": max_s})
        return {**r, "exploring": True}

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
        running = self._resolve(["record_video"], running_only=True)
        running = [j for j in running if not j["done"].is_set()]
        if a["action"] == "stop":
            if not running:
                return {"recording": False, "dir": None, "frames": 0}
            self.cancel_jobs([j["id"] for j in running])
            j = running[0]
            return {"job_id": j["id"], **(j["result"] or {})}
        if running:
            j = running[0]
            return {"job_id": j["id"], "status": "running", "recording": True, **j["info"],
                    "frames": j["progress"].get("frames", 0)}
        d = Path(self.rec_dir) / time.strftime("rec-%Y%m%d-%H%M%S")
        d.mkdir(parents=True, exist_ok=True)
        period = 1.0 / a.get("fps", 2.0)

        def body(j):
            n = 0
            while not j["cancel"].is_set():
                try:
                    jpeg, _ = self.robot.snapshot()
                    (d / f"{n:06d}.jpg").write_bytes(jpeg)
                    n += 1
                    j["progress"]["frames"] = n
                except Exception:
                    pass
                j["cancel"].wait(period)
            return {"recording": False, "frames": n}
        r = self._start_job("record_video", body, {"dir": str(d)})
        return {**r, "recording": True, "frames": 0}

    def t_await(self, a, c):
        mode = a.get("mode", "any")
        jobs = self._resolve(a.get("job_ids"), running_only=not a.get("job_ids"), skip=("record_video",))
        if not jobs:
            raise Err("NO_JOBS", "no matching background jobs to await")
        stop_others = a.get("stop_others", mode == "any")
        end = time.time() + a.get("timeout_s", 600) * self.interval_scale
        timed_out = False
        while True:
            done = [j for j in jobs if j["done"].is_set()]
            if (mode == "any" and done) or len(done) == len(jobs):
                break
            if time.time() >= end:
                timed_out = True
                break
            self._check(c)
            time.sleep(0.02)
        finished = [j["id"] for j in jobs if j["done"].is_set()]
        stopped = self.cancel_jobs([j["id"] for j in jobs if not j["done"].is_set()]) if stop_others else []
        return {"timed_out": timed_out, "finished": finished, "stopped": stopped,
                "jobs": [self._summary(j) for j in jobs]}

    def t_job_status(self, a, c):
        return {"jobs": [self._summary(j) for j in self._resolve(a.get("job_ids"))]}

    def t_cancel_job(self, a, c):
        jobs = self._resolve(a.get("job_ids"), running_only=not a.get("job_ids"))
        stopped = self.cancel_jobs([j["id"] for j in jobs])
        return {"cancelled": stopped, "jobs": [self._summary(j) for j in jobs]}
