"""Native Windows GUI (CustomTkinter on Tkinter + Pillow). No ROS, no browser.

Threads:  Tk main thread (UI only) | agent thread (VLM + tool sequence) | E-STOP sender thread
          | bridge WS thread (bot mode heartbeat/events). The E-STOP button never waits on the model:
          it only calls EStopSender.trigger(), which hands off to the sender thread and returns.

Keys: F11 toggles true fullscreen, Esc leaves fullscreen. Neither key is connected to the E-STOP in
any way (tests/test_gui_keys.py). F12 is the keyboard E-STOP. Nothing in the GUI clears the latch.
"""
import io
import queue
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import messagebox

import customtkinter as ctk
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import load_config, read_token  # noqa: E402
from vlm.ollama_client import OllamaClient  # noqa: E402
from agent.loop import AgentRunner  # noqa: E402
from agent import stopwords  # noqa: E402
from robot.stub import StubRobot  # noqa: E402
from robot.estop import EStopSender  # noqa: E402

# ---------- theme ----------
BG = "#0B0F14"
PANEL = "#121821"
PANEL_2 = "#18202B"
BORDER = "#1F2937"
TEXT = "#E5E7EB"
MUTED = "#8B98A9"
ACCENT = "#22D3EE"
ACCENT_DIM = "#0E7490"
RED = "#EF4444"          # E-STOP only
RED_HOVER = "#DC2626"
RADIUS = 14
PAD = 16
STATUS_COLORS = {          # chip dot colours; red is reserved for E-STOP, so "failed" is amber
    "pending": MUTED, "running": ACCENT, "done": "#34D399", "ok": "#34D399",
    "failed": "#F59E0B", "error": "#F59E0B", "cancelled": "#6B7280", "skipped": "#6B7280",
}


def pick_font_family(root):
    fams = set(tkfont.families(root))
    for f in ("Inter", "Inter Variable", "Segoe UI Variable Text", "Segoe UI Variable", "Segoe UI"):
        if f in fams:
            return f
    return "TkDefaultFont"


def pick_mono_family(root):
    fams = set(tkfont.families(root))
    for f in ("Cascadia Mono", "Cascadia Code", "Consolas"):
        if f in fams:
            return f
    return "TkFixedFont"


class WindowControl:
    """Maximise / fullscreen handling. Deliberately has NO reference to the E-STOP or the robot."""

    def __init__(self, root):
        self.root = root
        self.fullscreen = False

    def maximise(self):
        try:
            self.root.state("zoomed")                 # Windows
        except tk.TclError:
            self.root.attributes("-zoomed", True)     # X11 fallback

    def toggle_fullscreen(self, _e=None):
        self.fullscreen = not self.fullscreen
        self.root.attributes("-fullscreen", self.fullscreen)
        if not self.fullscreen:
            self.root.after(10, self.maximise)
        return "break"

    def exit_fullscreen(self, _e=None):
        if self.fullscreen:
            self.fullscreen = False
            self.root.attributes("-fullscreen", False)
            self.root.after(10, self.maximise)
        return "break"


class StepChip(ctk.CTkFrame):
    def __init__(self, master, step, where, on_cancel, fonts):
        super().__init__(master, fg_color=PANEL_2, corner_radius=10, border_width=1, border_color=BORDER)
        self.step_id = step["step_id"]
        self.status = None
        self.dot = ctk.CTkLabel(self, text="\u25CF", width=18, font=fonts["body"], text_color=MUTED)
        self.dot.pack(side="left", padx=(12, 6), pady=8)
        txt = ctk.CTkFrame(self, fg_color="transparent")
        txt.pack(side="left", fill="x", expand=True, pady=6)
        ctk.CTkLabel(txt, text=f"{step['tool']}  \u00B7  {where}", anchor="w", font=fonts["bold"],
                     text_color=TEXT).pack(fill="x")
        args = str(step.get("args"))
        ctk.CTkLabel(txt, text=args if len(args) < 90 else args[:87] + "...", anchor="w",
                     font=fonts["small"], text_color=MUTED).pack(fill="x")
        self.status_lbl = ctk.CTkLabel(self, text="", width=70, font=fonts["small"], text_color=MUTED)
        self.status_lbl.pack(side="left", padx=4)
        self.x = ctk.CTkButton(self, text="\u00D7", width=30, height=30, corner_radius=15, fg_color="transparent",
                               hover_color=BORDER, text_color=MUTED, font=fonts["bold"],
                               command=lambda: on_cancel(self.step_id))
        self.x.pack(side="right", padx=8)
        self.set_status(step.get("status", "pending"))

    def set_status(self, status):
        self.status = status
        col = STATUS_COLORS.get(status, MUTED)
        self.dot.configure(text_color=col)
        self.status_lbl.configure(text=status, text_color=col)
        if status not in ("pending", "running"):
            self.x.configure(state="disabled", text_color=BORDER)
            self.configure(border_color=BORDER)

    def glow(self, phase):
        if self.status == "running":
            self.configure(border_color=ACCENT if phase else ACCENT_DIM, border_width=2)


class App:
    def __init__(self, root, cfg):
        self.root, self.cfg = root, cfg
        self.q = queue.Queue()
        self.model = OllamaClient.from_config(cfg)
        self.robot = None
        self.estop = None
        self.runner = None
        self.chips = {}
        self._ctk_img = None
        self._last_jpeg = None
        self._phase = False
        self.win = WindowControl(root)
        root.title("Ros_lidar_bot \u2014 laptop VLM")
        root.configure(fg_color=BG)
        root.geometry("1440x900")
        root.minsize(1100, 700)
        fam, mono = pick_font_family(root), pick_mono_family(root)
        self.fonts = {
            "title": ctk.CTkFont(fam, 18, "bold"), "body": ctk.CTkFont(fam, 14),
            "bold": ctk.CTkFont(fam, 14, "bold"), "small": ctk.CTkFont(fam, 12),
            "section": ctk.CTkFont(fam, 13, "bold"), "estop": ctk.CTkFont(fam, 22, "bold"),
            "mono": (mono, 11),
        }
        self._build()
        self._switch_mode(cfg.get("mode", "webcam"))
        # maximised by default; CTk re-applies geometry after init, so zoom on the next idle tick too
        self.win.maximise()
        root.after(200, self.win.maximise)
        root.after(50, self._pump)
        root.after(1000, self._status_tick)
        root.after(500, self._animate)

    # ---------- layout ----------
    def _card(self, master, **kw):
        return ctk.CTkFrame(master, fg_color=PANEL, corner_radius=RADIUS, border_width=1, border_color=BORDER, **kw)

    def _build(self):
        r, f = self.root, self.fonts
        r.grid_columnconfigure(0, weight=1)
        r.grid_rowconfigure(2, weight=1)

        # top bar
        top = self._card(r)
        top.grid(row=0, column=0, sticky="ew", padx=PAD, pady=(PAD, 8))
        ctk.CTkLabel(top, text="VLM \u00B7 Ros_lidar_bot", font=f["title"], text_color=TEXT).pack(side="left", padx=(18, 20), pady=14)
        self.mode_var = tk.StringVar(value="Webcam")
        self.mode_seg = ctk.CTkSegmentedButton(
            top, values=["Webcam", "Bot"], variable=self.mode_var, font=f["bold"], height=36,
            fg_color=PANEL_2, selected_color=ACCENT_DIM, selected_hover_color=ACCENT_DIM,
            unselected_color=PANEL_2, unselected_hover_color=BORDER, text_color=TEXT,
            command=lambda v: self._switch_mode("bot" if v == "Bot" else "webcam"))
        self.mode_seg.pack(side="left", padx=6)
        self.pill = ctk.CTkLabel(top, text="\u25CF  connecting", corner_radius=14, fg_color=PANEL_2, text_color=MUTED,
                                 font=f["small"], height=30, padx=14)
        self.pill.pack(side="left", padx=14)
        self.model_lbl = ctk.CTkLabel(top, text=f"model  {self.cfg['model']['name']}", font=f["small"], text_color=MUTED)
        self.model_lbl.pack(side="left", padx=6)
        self.think_dot = ctk.CTkLabel(top, text="", font=f["bold"], text_color=ACCENT, width=110)
        self.think_dot.pack(side="left", padx=6)
        self.estop_btn = ctk.CTkButton(top, text="E-STOP", fg_color=RED, hover_color=RED_HOVER, text_color="white",
                                       font=f["estop"], width=190, height=56, corner_radius=12,
                                       command=self._press_estop)
        self.estop_btn.pack(side="right", padx=14, pady=10)
        ctk.CTkLabel(top, text="F12 = E-STOP   F11 = fullscreen", font=f["small"], text_color=MUTED).pack(side="right", padx=6)
        r.bind("<F12>", lambda e: self._press_estop())
        r.bind("<F11>", self.win.toggle_fullscreen)
        r.bind("<Escape>", self.win.exit_fullscreen)

        # banner (hidden when empty)
        self.banner = ctk.CTkLabel(r, text="", anchor="w", font=f["bold"], corner_radius=10,
                                   fg_color=PANEL_2, text_color=TEXT, padx=16, height=36)
        self._banner_shown = False

        # body
        body = ctk.CTkFrame(r, fg_color="transparent")
        body.grid(row=2, column=0, sticky="nsew", padx=PAD)
        body.grid_columnconfigure(0, weight=3, uniform="b")
        body.grid_columnconfigure(1, weight=2, uniform="b")
        body.grid_rowconfigure(0, weight=1)

        cam = self._card(body)
        cam.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        cam.grid_rowconfigure(1, weight=1)
        cam.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(cam, text="CAMERA  \u00B7  image used for reasoning", font=f["section"], text_color=MUTED,
                     anchor="w").grid(row=0, column=0, sticky="ew", padx=18, pady=(14, 6))
        self.img_label = ctk.CTkLabel(cam, text="no frame yet \u2014 send a command", fg_color=BG, corner_radius=10,
                                      text_color=MUTED, font=f["body"])
        self.img_label.grid(row=1, column=0, sticky="nsew", padx=14)
        self.img_label.bind("<Configure>", lambda e: self._redraw_image())
        self.img_meta = ctk.CTkLabel(cam, text="", font=f["small"], text_color=MUTED, anchor="w", justify="left")
        self.img_meta.grid(row=2, column=0, sticky="ew", padx=18, pady=(6, 14))

        right = ctk.CTkFrame(body, fg_color="transparent")
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(0, weight=3)
        right.grid_rowconfigure(1, weight=2)

        rc = self._card(right)
        rc.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        hdr = ctk.CTkFrame(rc, fg_color="transparent")
        hdr.pack(fill="x", padx=18, pady=(14, 6))
        ctk.CTkLabel(hdr, text="REASONING  \u00B7  live, on laptop", font=f["section"], text_color=MUTED).pack(side="left")
        self.reason = ctk.CTkTextbox(rc, wrap="word", font=f["mono"], fg_color=BG, text_color=TEXT,
                                     corner_radius=10, border_width=0)
        self.reason.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        self.reason.tag_config("thinking", foreground=MUTED)
        self.reason.tag_config("content", foreground=ACCENT)
        self.reason.tag_config("sys", foreground="#FBBF24")

        tc = self._card(right)
        tc.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        th = ctk.CTkFrame(tc, fg_color="transparent")
        th.pack(fill="x", padx=18, pady=(14, 6))
        ctk.CTkLabel(th, text="TOOL SEQUENCE", font=f["section"], text_color=MUTED).pack(side="left")
        for txt, cmd in (("Resume", lambda: self.runner and self.runner.resume()),
                         ("Cancel run", lambda: self.runner and self.runner.cancel_run())):
            ctk.CTkButton(th, text=txt, command=cmd, width=96, height=28, corner_radius=14, font=f["small"],
                          fg_color=PANEL_2, hover_color=BORDER, border_width=1, border_color=BORDER,
                          text_color=TEXT).pack(side="right", padx=(6, 0))
        self.timeline = ctk.CTkScrollableFrame(tc, fg_color="transparent", corner_radius=0)
        self.timeline.pack(fill="both", expand=True, padx=8, pady=(0, 12))
        self.empty_lbl = ctk.CTkLabel(self.timeline, text="No plan yet.", text_color=MUTED, font=f["small"])
        self.empty_lbl.pack(pady=20)

        # prompt
        pc = self._card(r)
        pc.grid(row=3, column=0, sticky="ew", padx=PAD, pady=(16, 8))
        self.entry = ctk.CTkEntry(pc, height=46, corner_radius=12, font=f["body"],
                                  fg_color=BG, border_color=BORDER, text_color=TEXT,
                                  placeholder_text="Tell the robot what to do\u2026  (Enter to send; halt / freeze / ruko / tham stop it, emergency = E-STOP)")
        self.entry.pack(side="left", fill="x", expand=True, padx=(14, 8), pady=12)
        self.entry.bind("<Return>", lambda e: self._send())
        ctk.CTkButton(pc, text="Send", command=self._send, width=110, height=46, corner_radius=12, font=f["bold"],
                      fg_color=ACCENT, hover_color=ACCENT_DIM, text_color=BG).pack(side="left", padx=(0, 14))

        self.status = ctk.CTkLabel(r, text="", anchor="w", font=f["small"], text_color=MUTED)
        self.status.grid(row=4, column=0, sticky="ew", padx=PAD + 6, pady=(0, 10))

    def _set_banner(self, text, alarm=False):
        if text:
            self.banner.configure(text=text, fg_color=RED if alarm else PANEL_2,
                                  text_color="white" if alarm else "#FBBF24")
            if not self._banner_shown:
                self.banner.grid(row=1, column=0, sticky="ew", padx=PAD, pady=(0, 8))
                self._banner_shown = True
        elif self._banner_shown:
            self.banner.grid_forget()
            self._banner_shown = False

    # ---------- mode ----------
    def _switch_mode(self, mode):
        if self.runner and self.runner.busy():
            messagebox.showwarning("Busy", "Cancel the run before switching mode.")
            self.mode_var.set("Bot" if self.robot and not self.robot.is_stub else "Webcam")
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
            robot, estop_url, mode = StubRobot(self.cfg), None, "webcam"
        self.robot = robot
        self.mode_var.set("Bot" if mode == "bot" else "Webcam")
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
            threading.Thread(target=_start, daemon=True).start()
        else:
            robot.start()
        self._log(f"\n=== mode: {mode} ({'robot tools are STUBS, nothing moves' if robot.is_stub else 'Pi bridge ' + robot.api}) ===\n", "sys")

    # ---------- actions ----------
    def _press_estop(self):
        self.estop.trigger(source="gui")          # returns immediately; sender thread does the work
        if self.runner:
            self.runner.cancel_run()
        self._set_banner("E-STOP pressed \u2014 latched. Reset ONLY on the robot dashboard (:8080).", alarm=True)

    def _send(self):
        text = self.entry.get().strip()
        if not text:
            return
        self.entry.delete(0, "end")
        hit = stopwords.check(text)          # checked BEFORE anything reaches the VLM
        if hit:
            if hit["estop"]:
                self._press_estop()
            threading.Thread(target=self.runner.hard_stop, args=(",".join(hit["words"]),), daemon=True).start()
            self._log(f"\n> {text}   [STOP WORD {hit['words']} -> plan cancelled, stop() sent"
                      f"{', E-STOP latched' if hit['estop'] else ''}; not sent to the VLM]\n", "sys")
            return
        r = self.runner.submit(text)
        if r == "started":
            self._clear_timeline()
        self._log(f"\n> {text}   [{r}]\n", "sys")

    def _cancel_step(self, step_id):
        if self.runner:
            self.runner.cancel_step(step_id)

    def _clear_timeline(self):
        for c in self.chips.values():
            c.destroy()
        self.chips.clear()
        self.empty_lbl.pack(pady=20)

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
            self.empty_lbl.pack_forget()
            for st in ev["steps"]:
                if st["step_id"] in self.chips:
                    continue
                where = "STUB" if (st["runs_on"] == "pi" and self.robot.is_stub) else st["runs_on"]
                chip = StepChip(self.timeline, st, where, self._cancel_step, self.fonts)
                chip.pack(fill="x", padx=6, pady=4)
                self.chips[st["step_id"]] = chip
        elif t == "step":
            chip = self.chips.get(ev["step_id"])
            if chip:
                chip.set_status(ev["status"])
            if ev.get("result") is not None:
                self._log(f"\n[{ev['tool']} -> {ev['status']}] {str(ev['result'])[:300]}\n", "sys")
        elif t == "state":
            note = ev.get("note")
            if note:
                self._set_banner(note)
            elif ev["state"] in ("thinking", "observing", "idle"):
                if not self.robot.estop_latched:
                    self._set_banner("")
        elif t in ("finished",):
            self._log(f"\nFINISHED: {ev['answer']}\n", "sys")
        elif t in ("failed", "limit", "error", "cancelled", "info", "ask_user", "parse_error", "human_override", "human_done", "hard_stop"):
            self._log(f"\n[{t}] {ev.get('message') or ev.get('question') or ev.get('error') or ev.get('text') or ev.get('scope') or ''}\n", "sys")
        elif t == "estop_result":
            ok = ev.get("ok")
            self._set_banner(("E-STOP LATCHED (%s, %s ms). Reset ONLY on the robot dashboard." % (
                "stub" if ev.get("stub") else "robot", ev.get("latency_ms"))) if ok else
                f"E-STOP SEND FAILED: {ev.get('error')} \u2014 USE THE DASHBOARD / GAMEPAD", alarm=True)
        elif t == "robot_event":
            e = ev["ev"]
            if e.get("type") == "link" and not e["data"].get("ok"):
                self._set_banner(f"Link lost: {e['data'].get('error')}", alarm=True)

    def _show_image(self, jpeg, meta):
        self._last_jpeg = jpeg
        self._redraw_image()
        ct = meta.get("capture_time")
        ct = time.strftime('%H:%M:%S', time.localtime(ct)) if isinstance(ct, float) else ct
        self.img_meta.configure(text=f"source {meta.get('source')}   \u00B7   stub {meta.get('stub')}   \u00B7   "
                                     f"captured {ct}   \u00B7   pose {meta.get('pose')}   \u00B7   head {meta.get('head')}")

    def _redraw_image(self):
        if not self._last_jpeg:
            return
        img = Image.open(io.BytesIO(self._last_jpeg))
        w = max(200, self.img_label.winfo_width() - 8)
        h = max(150, self.img_label.winfo_height() - 8)
        s = min(w / img.width, h / img.height)
        size = (max(1, int(img.width * s)), max(1, int(img.height * s)))
        self._ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=size)
        self.img_label.configure(image=self._ctk_img, text="")

    # ---------- status + animation ----------
    def _status_tick(self):
        r = self.robot
        up = self.model.health()
        if r.estop_latched:
            self.pill.configure(text="\u25CF  E-STOP LATCHED", fg_color=RED, text_color="white")
        elif r.is_stub:
            self.pill.configure(text="\u25CF  webcam \u00B7 tools stubbed" + ("" if up else " \u00B7 ollama down"),
                                fg_color=PANEL_2, text_color=ACCENT if up else "#F59E0B")
        else:
            ok = r.link_ok and up
            self.pill.configure(text=f"\u25CF  bot \u00B7 {'linked' if r.link_ok else 'link down'} \u00B7 rtt {getattr(r, 'rtt_ms', None)} ms",
                                fg_color=PANEL_2, text_color=ACCENT if ok else "#F59E0B")
        self.status.configure(text="   \u00B7   ".join([
            f"agent {self.runner.state}", f"ollama {'up' if up else 'DOWN'}",
            f"link {'ok' if r.link_ok else 'DOWN'}", f"estop {'LATCHED' if r.estop_latched else 'clear'}"]))
        self.root.after(1000, self._status_tick)

    def _animate(self):
        self._phase = not self._phase
        st = self.runner.state if self.runner else "idle"
        if st in ("thinking", "observing"):
            self.think_dot.configure(text=("\u25CF  " if self._phase else "\u25CB  ") + st,
                                     text_color=ACCENT if self._phase else ACCENT_DIM)
        else:
            self.think_dot.configure(text="")
        for c in self.chips.values():
            c.glow(self._phase)
        self.root.after(500, self._animate)


def make_root():
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")
    return ctk.CTk()


def main():
    cfg = load_config()
    root = make_root()
    App(root, cfg)
    if "--smoke" in sys.argv:          # launch test: build the window, then exit
        root.after(2500, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
