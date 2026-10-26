"""Agent loop (runs on the LAPTOP). The VLM reasons here; tools execute on the Pi (or stubs).

Per turn:  observe (snapshot) -> VLM streams reasoning + a JSON plan -> parse/validate (2 retries)
           -> show the planned tool sequence -> run each tool automatically (cancellable per tool and
           per run) -> feed results back -> repeat until `finish` or a limit is hit.
Pauses:    E-STOP latched (stays paused until the Pi reports it reset on the dashboard AND Amey presses
           Resume), human override / human command (pause, run the human command, re-observe, resume),
           link lost (bot mode).
"""
import json
import re
import threading
import time
from pathlib import Path

from .tools import TOOLS, MOTION_TOOLS, SKILL_TOOLS, validate_step, catalogue_text
from .skills import Skills
from robot.base import new_id

SYSTEM = """You are the reasoning brain of an autonomous mobile robot. You never move hardware directly; dedicated tools do the work, and you use them to carry out the task the user gives you.
Your job is to reason about the task and output the chain of tools, in order, that performs it.
Safety: only the user can reset the E-STOP. Call estop if anything looks unsafe, for example the robot could collide with a surface. Never claim to see what is not in an image; decide only from the data coming from the robot or the image.

Tools (name, what it does, inputs, outputs):
{catalogue}

If the request needs no tools (a question, or describing the image you already have), answer directly in plain text, briefly. No plan is needed then.
If it needs tools, reply with ONE JSON object only: {{"observation": "...", "plan": [{{"tool": "...", "args": {{...}}, "why": "..."}}]}}
End with finish {{"answer": "..."}}. Put snapshot last if you need a new image.
BACKGROUND tools (feed, watch_for, start_exploring, record_video) return a job_id at once and run alongside the next steps. Use await to get their results (job_status to peek, cancel_job to stop). If you await, end the plan there; you get the results and finish next turn.
Example, "is there a bottle anywhere?": snapshot, feed {{"purpose": "find a bottle", "goal": "bottle", "interval_s": 2, "max_s": 300}}, start_exploring {{"max_s": 300}}, await {{"mode": "any", "stop_others": true}}."""


def extract_json(text):
    t = text.strip()
    t = re.sub(r"^```(?:json)?\s*|\s*```$", "", t)
    try:
        return json.loads(t)
    except ValueError:
        pass
    start = t.find("{")
    end = t.rfind("}")
    if start >= 0 and end > start:
        return json.loads(t[start:end + 1])
    raise ValueError("no JSON object found in the reply")


class JsonlLog:
    def __init__(self, log_dir):
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        self.path = Path(log_dir) / time.strftime("agent-%Y%m%d-%H%M%S.jsonl")
        self._lock = threading.Lock()

    def write(self, kind, **data):
        rec = {"t": time.time(), "kind": kind, **data}
        with self._lock, open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, default=str) + "\n")


class AgentRunner:
    def __init__(self, cfg, model, robot, estop_sender, on_event=None, log=None):
        a = cfg["agent"]
        self.max_steps = a.get("max_steps")  # None = no cap on total tool steps
        self.max_task_s = a.get("max_task_s", 900)
        self.max_retries = a.get("max_parse_retries", 2)
        self.model = model
        self.robot = robot
        self.skills = Skills(cfg, model, robot)
        self.estop = estop_sender
        self.on_event = on_event or (lambda ev: None)
        self.log = log or JsonlLog(a.get("log_dir", "logs"))
        self.state = "idle"
        self.run_cancel = threading.Event()
        self.user_paused = threading.Event()
        self.resume_evt = threading.Event()
        self.step_cancels = {}          # step_id -> Event
        self._human_q = []
        self._lock = threading.Lock()
        self._thread = None

    # ---------- public API (called from the GUI thread) ----------
    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    def submit(self, text):
        """New command from Amey. If a task is running, it is a HUMAN COMMAND: pause the task, run this,
        then resume the task."""
        if self.busy():
            with self._lock:
                self._human_q.append(text)
            for ev in list(self.step_cancels.values()):
                ev.set()                 # interrupt the in-flight tool; the task resumes later
            self.skills.cancel_jobs()    # and stop background jobs
            self._emit("human_override", source="gui", text=text)
            return "queued_as_human_command"
        self.run_cancel.clear()
        self._thread = threading.Thread(target=self._run_top, args=(text,), name="agent", daemon=True)
        self._thread.start()
        return "started"

    def cancel_run(self):
        self.run_cancel.set()
        for ev in list(self.step_cancels.values()):
            ev.set()
        self.skills.cancel_jobs()        # background jobs (feed, exploring, ...) stop too

    def hard_stop(self, reason="stop word"):
        """Stop word: cancel the plan and all tools now, then send robot stop() (stub in webcam mode).
        Never touches the E-STOP latch."""
        with self._lock:
            self._human_q.clear()
        self.cancel_run()
        try:
            res = self.robot.call("stop", {}, new_id("stop"), None, timeout_s=3)
        except Exception as e:
            res = {"ok": False, "error": {"code": "INTERNAL", "message": repr(e)}}
        self._emit("hard_stop", reason=reason, result=res)
        return res

    def cancel_step(self, step_id):
        ev = self.step_cancels.get(step_id)
        if ev:
            ev.set()

    def resume(self):
        """Amey pressed Resume. Does NOT touch the E-STOP: if the robot is still latched we stay paused."""
        self.resume_evt.set()

    # ---------- internals ----------
    def _emit(self, typ, **data):
        ev = {"type": typ, "t": time.time(), **data}
        if typ != "model_chunk":
            self.log.write(typ, **{k: v for k, v in data.items() if k != "jpeg"})
        try:
            self.on_event(ev)
        except Exception:
            pass

    def _set_state(self, s, **extra):
        self.state = s
        if hasattr(self.robot, "agent_state"):
            self.robot.agent_state = s
        self._emit("state", state=s, **extra)

    def _run_top(self, text):
        try:
            self._run_task(text, depth=0)
        except Exception as e:
            self._emit("error", message=repr(e))
        finally:
            stopped = self.skills.cancel_jobs()   # no background job outlives the task
            if stopped:
                self._emit("info", message=f"background jobs stopped at task end: {stopped}")
            self._set_state("idle")

    def _wait_if_paused(self, depth):
        """Returns False if the run was cancelled while paused."""
        while True:
            if self.run_cancel.is_set():
                return False
            if self.robot.estop_latched:
                self.skills.cancel_jobs()
                self._set_state("paused_estop", note="E-STOP latched. Reset only on the dashboard (:8080), then press Resume.")
                self.resume_evt.clear()
                while not self.run_cancel.is_set():
                    if self.resume_evt.wait(0.2) and not self.robot.estop_latched:
                        break
                    if self.resume_evt.is_set():      # resume pressed but still latched
                        self._emit("info", message="Still latched on the robot - reset it on the dashboard first.")
                        self.resume_evt.clear()
                continue
            if self.robot.human_override:
                self._set_state("paused_human", note="Human is driving; agent resumes when they finish.")
                while self.robot.human_override and not self.run_cancel.is_set():
                    time.sleep(0.1)
                continue
            if not self.robot.link_ok:
                self._set_state("paused_link")
                while not self.robot.link_ok and not self.run_cancel.is_set():
                    time.sleep(0.2)
                continue
            if depth == 0 and self._human_q:
                return "human"
            return True

    def _observe(self):
        jpeg, meta = self.robot.snapshot()
        self._emit("image", jpeg=jpeg, meta=meta)
        return jpeg, meta

    def _run_task(self, text, depth):
        t_start = time.time()
        sys_msg = SYSTEM.format(catalogue=catalogue_text(False))  # always reason as on the hardware bot
        prefix = "[HUMAN COMMAND from Amey, priority] " if depth else ""
        self._set_state("observing", task=text, depth=depth)
        jpeg, meta = self._observe()
        messages = [{"role": "system", "content": sys_msg},
                    {"role": "user", "content": prefix + text + self._locations_line() + f"\n(image attached: {json.dumps({k: v for k, v in meta.items() if k != 'jpeg'}, default=str)})",
                     "images": [jpeg]}]
        steps = 0
        while True:
            if time.time() - t_start > self.max_task_s:
                self._emit("limit", message=f"time limit {self.max_task_s}s reached")
                return "limit"
            w = self._wait_if_paused(depth)
            if w is False:
                self._emit("cancelled", scope="run")
                return "cancelled"
            if w == "human":
                self._handle_human(depth)
                messages.append(self._reobserve_msg("A human command interrupted you and has finished. Re-check and continue the ORIGINAL task: " + text))
                continue
            # ---- think + plan ----
            plan = None
            for attempt in range(self.max_retries + 1):
                self._set_state("thinking", attempt=attempt)
                self._emit("model_start", attempt=attempt)
                thinking, content, stats = self.model.chat(
                    messages, cancel_event=self.run_cancel,
                    on_chunk=lambda ev: self._emit("model_chunk", kind=ev["type"], text=ev.get("text", "")) if ev["type"] in ("thinking", "content") else None)
                self._emit("model_done", stats=stats, content=content, thinking_len=len(thinking))
                if self.run_cancel.is_set():
                    self._emit("cancelled", scope="run")
                    return "cancelled"
                messages.append({"role": "assistant", "content": content})
                if "{" not in content and content.strip():
                    # plain-text reply = direct answer, no tools needed
                    self._emit("finished", answer=content.strip())
                    return "finished"
                try:
                    obj = extract_json(content)
                except Exception:
                    obj = None
                if isinstance(obj, dict) and "plan" not in obj and isinstance(obj.get("answer"), str):
                    # {"answer": ...} with no plan = direct answer, no tools needed
                    self._emit("finished", answer=obj["answer"])
                    return "finished"
                try:
                    plan = validate_step(extract_json(content))
                    break
                except Exception as e:
                    err = f"Your reply was invalid: {e}. If no tools are needed, just answer in plain text. Otherwise reply with ONLY the JSON plan object."
                    self._emit("parse_error", attempt=attempt, error=str(e))
                    messages.append({"role": "user", "content": err})
            if plan is None:
                self._emit("failed", message=f"model output invalid after {self.max_retries} retries; task stopped")
                return "parse_failed"
            # ---- show the planned sequence ----
            seq = [{"step_id": new_id("s"), "tool": s["tool"], "args": s["args"], "why": s.get("why", ""),
                    "runs_on": TOOLS[s["tool"]]["runs_on"], "status": "planned"} for s in plan["plan"]]
            self._emit("plan", observation=plan.get("observation", ""), steps=seq)
            results = []
            need_image = False
            for st in seq:
                steps += 1
                if self.max_steps is not None and steps > self.max_steps:
                    self._emit("limit", message=f"step limit {self.max_steps} reached")
                    return "limit"
                w = self._wait_if_paused(depth)
                if w is False:
                    self._emit("cancelled", scope="run")
                    return "cancelled"
                if w == "human":
                    self._step_status(st, "interrupted")
                    self._handle_human(depth)
                    break
                res = self._exec_step(st)
                results.append({"tool": st["tool"], "args": st["args"], "result": res})
                if st["tool"] == "finish":
                    self._emit("finished", answer=st["args"].get("answer", ""))
                    return "finished"
                if st["tool"] == "ask_user":
                    self._emit("ask_user", question=st["args"]["question"])
                    return "asked_user"
                if st["tool"] == "snapshot" or st["tool"] in MOTION_TOOLS:
                    need_image = need_image or st["tool"] == "snapshot"
                if res.get("error", {}) and res["error"].get("code") in ("ESTOP_LATCHED", "CANCELLED", "HUMAN_OVERRIDE"):
                    break
            # ---- feed results back ----
            fb = "Tool results:\n" + json.dumps(results, default=str)[:6000]
            if need_image:
                messages.append(self._reobserve_msg(fb + "\nNew image attached. Continue."))
            else:
                messages.append({"role": "user", "content": fb + "\nContinue."})

    def _locations_line(self):
        b = self.skills.store.brief()
        return f"\n{b}" if b else ""

    def _reobserve_msg(self, text):
        jpeg, meta = self._observe()
        return {"role": "user", "content": text, "images": [jpeg]}

    def _handle_human(self, depth):
        with self._lock:
            cmds, self._human_q = self._human_q, []
        self._set_state("paused_human", note="running human command")
        for c in cmds:
            self._run_task(c, depth=depth + 1)
        self._emit("human_done")

    def _step_status(self, st, status, **extra):
        st["status"] = status
        self._emit("step", step_id=st["step_id"], tool=st["tool"], status=status, **extra)

    def _exec_step(self, st):
        tool, args = st["tool"], st["args"]
        spec = TOOLS[tool]
        cancel = threading.Event()
        self.step_cancels[st["step_id"]] = cancel
        self._step_status(st, "running")
        t0 = time.time()
        try:
            if spec["runs_on"] == "estop":
                self.estop.trigger(source="agent", text=args.get("reason"))
                res = {"ok": True, "data": {"estop": "sent", "note": "latched; only Amey can reset on the dashboard"}}
            elif tool == "snapshot":
                res = {"ok": True, "data": {"note": "new image will be attached"}}
            elif tool in ("finish", "ask_user"):
                res = {"ok": True}
            elif tool in SKILL_TOOLS:
                if self.robot.estop_latched and tool in MOTION_TOOLS:
                    res = {"ok": False, "error": {"code": "ESTOP_LATCHED", "message": "refused locally: E-STOP latched"}}
                else:
                    res = self.skills.run(tool, args, cancel)
            else:
                if self.robot.estop_latched and tool in MOTION_TOOLS:
                    res = {"ok": False, "error": {"code": "ESTOP_LATCHED", "message": "refused locally: E-STOP latched"}}
                else:
                    res = self.robot.call(tool, args, st["step_id"], cancel, timeout_s=spec.get("timeout_s", 30))
            if cancel.is_set() and not self.run_cancel.is_set() and not self._human_q:
                res = {"ok": False, "error": {"code": "CANCELLED", "message": "cancelled by operator"}}
        except Exception as e:
            res = {"ok": False, "error": {"code": "INTERNAL", "message": repr(e)}}
        finally:
            self.step_cancels.pop(st["step_id"], None)
        status = "done" if res.get("ok") else ("cancelled" if (res.get("error") or {}).get("code") == "CANCELLED" else "failed")
        self._step_status(st, status, result=res, duration_ms=round((time.time() - t0) * 1000))
        return res
