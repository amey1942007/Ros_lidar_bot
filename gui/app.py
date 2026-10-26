"""Native Windows GUI (Tkinter + Pillow). No ROS, no browser.

Threads:  Tk main thread (UI only) | agent thread (VLM + tool sequence) | E-STOP sender thread
          | bridge WS thread (bot mode heartbeat/events). The E-STOP button never waits on the model.
"""
import io
import queue
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import load_config, read_token  # noqa: E402
from vlm.ollama_client import OllamaClient  # noqa: E402
from agent.loop import AgentRunner  # noqa: E402
from agent import stopwords  # noqa: E402
from robot.stub import StubRobot  # noqa: E402
from robot.estop import EStopSender  # noqa: E402


class App:
    def __init__(self, root, cfg):
        self.root, self.cfg = root, cfg
        self.q = queue.Queue()
        self.model = OllamaClient.from_config(cfg)
        self.robot = None
        self.estop = None
        self.runner = None
        self.step_rows = {}
        self._photo = None
        root.title("Ros_lidar_bot - laptop VLM (reasoning on laptop, tools on Pi)")
        root.geometry("1280x800")
        self._build()
        self._switch_mode(cfg.get("mode", "webcam"))
        root.after(50, self._pump)
        root.after(1000, self._status_tick)

    # ---------- layout ----------
    def _build(self):
        r = self.root
        top = ttk.Frame(r, padding=4)
        top.pack(fill="x")
        ttk.Label(top, text="Mode:").pack(side="left")
        self.mode_var = tk.StringVar(value="webcam")
        for m, label in (("webcam", "Webcam (robot tools STUBBED)"), ("bot", "Bot (tools on Pi over Tailscale)")):
            ttk.Radiobutton(top, text=label, value=m, variable=self.mode_var,
                            command=lambda: self._switch_mode(self.mode_var.get())).pack(side="left", padx=4)
        ttk.Label(top, text=f"   Model: {self.cfg['model']['name']} (laptop GPU via Ollama)").pack(side="left")
        self.estop_btn = tk.Button(top, text="E-STOP", bg="#c00000", fg="white", activebackground="#ff0000",
                                   font=("Segoe UI", 20, "bold"), width=10, command=self._press_estop)
        self.estop_btn.pack(side="right", padx=6)
        r.bind("<Escape>", lambda e: self._press_estop())
        r.bind("<F12>", lambda e: self._press_estop())

        self.banner = tk.Label(r, text="", bg="#fff3cd", anchor="w", font=("Segoe UI", 10, "bold"))
        self.banner.pack(fill="x")

        body = ttk.PanedWindow(r, orient="horizontal")
        body.pack(fill="both", expand=True)
        left = ttk.Frame(body)
        body.add(left, weight=1)
        ttk.Label(left, text="Image used for reasoning").pack(anchor="w")
        self.img_label = ttk.Label(left)
        self.img_label.pack(fill="both", expand=True)
        self.img_meta = ttk.Label(left, text="", wraplength=600)
        self.img_meta.pack(anchor="w")

        right = ttk.PanedWindow(body, orient="vertical")
        body.add(right, weight=1)
        rf = ttk.Frame(right)
        right.add(rf, weight=3)
        ttk.Label(rf, text="Live reasoning (LAPTOP)").pack(anchor="w")
        self.reason = tk.Text(rf, wrap="word", height=18, font=("Consolas", 9))
        self.reason.tag_config("thinking", foreground="#666666")
        self.reason.tag_config("content", foreground="#003399")
        self.reason.tag_config("sys", foreground="#990000")
        self.reason.pack(fill="both", expand=True)
        tf = ttk.Frame(right)
        right.add(tf, weight=2)
        ttk.Label(tf, text="Planned tool sequence (runs automatically; tools execute on the PI / stubs)").pack(anchor="w")
        self.tree = ttk.Treeview(tf, columns=("tool", "where", "args", "status"), show="headings", height=8)
        for c, w in (("tool", 90), ("where", 80), ("args", 260), ("status", 90)):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, stretch=(c == "args"))
        self.tree.pack(fill="both", expand=True)
        bf = ttk.Frame(tf)
        bf.pack(fill="x")
        ttk.Button(bf, text="Cancel selected tool", command=self._cancel_selected).pack(side="left")
        ttk.Button(bf, text="Cancel whole run", command=lambda: self.runner and self.runner.cancel_run()).pack(side="left", padx=4)
        ttk.Button(bf, text="Resume", command=lambda: self.runner and self.runner.resume()).pack(side="left")

        cmd = ttk.Frame(r, padding=4)
        cmd.pack(fill="x")
        ttk.Label(cmd, text="Command:").pack(side="left")
        self.cmd_var = tk.StringVar()
        ent = ttk.Entry(cmd, textvariable=self.cmd_var)
        ent.pack(side="left", fill="x", expand=True, padx=4)
        ent.bind("<Return>", lambda e: self._send())
        ttk.Button(cmd, text="Send", command=self._send).pack(side="left")

        self.status = tk.Label(r, text="", anchor="w", bg="#e0e0e0")
        self.status.pack(fill="x")

    # ---------- mode ----------
    def _switch_mode(self, mode):
        if self.runner and self.runner.busy():
            messagebox.showwarning("Busy", "Cancel the run before switching mode.")
            self.mode_var.set("bot" if self.robot and not self.robot.is_stub else "webcam")
            return
        if self.robot:
            self.robot.stop_all()
        try:
            if mode == "bot":
                from robot.bridge_client import BridgeRobot
                robot = BridgeRobot(self.cfg, read_token(self.cfg))
                estop_url = robot.estop_url
            else:
                robot = StubRobot(self.cfg)
                estop_url = None
        except Exception as e:
            messagebox.showerror("Bot mode unavailable", f"{e}\n\nStaying in webcam mode.")
            self.mode_var.set("webcam")
            robot, estop_url, mode = StubRobot(self.cfg), None, "webcam"
        self.robot = robot
        self.mode_var.set(mode)
        local_latch = robot.latch_estop if robot.is_stub else (lambda src: setattr(robot, "estop_latched", True))
        self.estop = EStopSender(estop_url, on_result=lambda res: self.q.put({"type": "estop_result", **res}),
                                 local_latch=local_latch)
        robot.add_listener(lambda ev: self.q.put({"type": "robot_event", "ev": ev}))
        self.runner = AgentRunner(self.cfg, self.model, robot, self.estop, on_event=self.q.put)
        if mode == "bot":
            def _start():
                try:
                    robot.start()
                except Exception as e:
                    self.q.put({"type": "error", "message": f"bridge start failed: {e}"})
            import threading
            threading.Thread(target=_start, daemon=True).start()
        else:
            robot.start()
        self._log(f"\n=== mode: {mode} ({'robot tools are STUBS, nothing moves' if robot.is_stub else 'Pi bridge ' + robot.api}) ===\n", "sys")

    # ---------- actions ----------
    def _press_estop(self):
        self.estop.trigger(source="gui")          # returns immediately; sender thread does the work
        if self.runner:
            self.runner.cancel_run()
        self.banner.config(text="E-STOP pressed - latched. Reset ONLY on the robot dashboard (:8080).", bg="#f8d7da")

    def _send(self):
        text = self.cmd_var.get().strip()
        if not text:
            return
        self.cmd_var.set("")
        hit = stopwords.check(text)          # checked BEFORE anything reaches the VLM
        if hit:
            if hit["estop"]:
                self._press_estop()
            import threading
            threading.Thread(target=self.runner.hard_stop, args=(",".join(hit["words"]),), daemon=True).start()
            self._log(f"\n> {text}   [STOP WORD {hit['words']} -> plan cancelled, stop() sent"
                      f"{', E-STOP latched' if hit['estop'] else ''}; not sent to the VLM]\n", "sys")
            return
        r = self.runner.submit(text)
        if r == "started":
            self.tree.delete(*self.tree.get_children())
            self.step_rows.clear()
        self._log(f"\n> {text}   [{r}]\n", "sys")

    def _cancel_selected(self):
        for iid in self.tree.selection():
            self.runner.cancel_step(iid)

    # ---------- event pump (Tk thread) ----------
    def _log(self, text, tag=None):
        self.reason.insert("end", text, tag)
        self.reason.see("end")

    def _pump(self):
        try:
            for _ in range(500):
                ev = self.q.get_nowait()
                self._handle(ev)
        except queue.Empty:
            pass
        self.root.after(30, self._pump)

    def _handle(self, ev):
        t = ev["type"]
        if t == "model_chunk":
            self._log(ev["text"], ev["kind"])
        elif t == "model_start":
            self._log(f"\n--- model (attempt {ev['attempt']}) ---\n", "sys")
        elif t == "model_done":
            s = ev.get("stats") or {}
            self._log(f"\n[ttft {s.get('ttft_s')} s, {s.get('tokens_per_s')} tok/s, total {s.get('total_s')} s]\n", "sys")
        elif t == "image":
            self._show_image(ev["jpeg"], ev["meta"])
        elif t == "plan":
            if ev.get("observation"):
                self._log(f"\nobservation: {ev['observation']}\n", "sys")
            for st in ev["steps"]:
                self.tree.insert("", "end", iid=st["step_id"],
                                 values=(st["tool"], "STUB" if (st["runs_on"] == "pi" and self.robot.is_stub) else st["runs_on"],
                                         str(st["args"]), st["status"]))
        elif t == "step":
            if self.tree.exists(ev["step_id"]):
                vals = list(self.tree.item(ev["step_id"], "values"))
                vals[3] = ev["status"]
                self.tree.item(ev["step_id"], values=vals)
            if ev.get("result") is not None:
                self._log(f"\n[{ev['tool']} -> {ev['status']}] {str(ev['result'])[:300]}\n", "sys")
        elif t == "state":
            note = ev.get("note")
            if note:
                self.banner.config(text=note, bg="#fff3cd")
            elif ev["state"] in ("thinking", "observing", "idle"):
                if not self.robot.estop_latched:
                    self.banner.config(text="", bg="#fff3cd")
        elif t in ("finished",):
            self._log(f"\nFINISHED: {ev['answer']}\n", "sys")
        elif t in ("failed", "limit", "error", "cancelled", "info", "ask_user", "parse_error", "human_override", "human_done", "hard_stop"):
            self._log(f"\n[{t}] {ev.get('message') or ev.get('question') or ev.get('error') or ev.get('text') or ev.get('scope') or ''}\n", "sys")
        elif t == "estop_result":
            ok = ev.get("ok")
            self.banner.config(text=("E-STOP LATCHED (%s, %s ms). Reset ONLY on the robot dashboard." % (
                "stub" if ev.get("stub") else "robot", ev.get("latency_ms"))) if ok else
                f"E-STOP SEND FAILED: {ev.get('error')} - USE THE DASHBOARD / GAMEPAD", bg="#f8d7da")
        elif t == "robot_event":
            e = ev["ev"]
            if e.get("type") == "link" and not e["data"].get("ok"):
                self.banner.config(text=f"Link lost: {e['data'].get('error')}", bg="#f8d7da")

    def _show_image(self, jpeg, meta):
        img = Image.open(io.BytesIO(jpeg))
        w = max(200, self.img_label.winfo_width() or 640)
        img.thumbnail((w, int(w * 0.75)))
        self._photo = ImageTk.PhotoImage(img)
        self.img_label.config(image=self._photo)
        self.img_meta.config(text=f"source={meta.get('source')}  stub={meta.get('stub')}  "
                                  f"captured={time.strftime('%H:%M:%S', time.localtime(meta['capture_time'])) if isinstance(meta.get('capture_time'), float) else meta.get('capture_time')}  "
                                  f"pose={meta.get('pose')}  head={meta.get('head')}")

    def _status_tick(self):
        r = self.robot
        parts = [f"mode={'webcam/stub' if r.is_stub else 'bot'}", f"agent={self.runner.state}",
                 f"ollama={'up' if self.model.health() else 'DOWN'}",
                 f"link={'ok' if r.link_ok else 'DOWN'}", f"estop={'LATCHED' if r.estop_latched else 'clear'}"]
        if not r.is_stub:
            parts.append(f"rtt={getattr(r, 'rtt_ms', None)} ms")
        self.status.config(text="   ".join(parts), bg="#f8d7da" if r.estop_latched else "#e0e0e0")
        self.root.after(1000, self._status_tick)


def main():
    cfg = load_config()
    root = tk.Tk()
    App(root, cfg)
    if "--smoke" in sys.argv:          # headless-ish launch test: build the window, then exit
        root.after(2500, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
