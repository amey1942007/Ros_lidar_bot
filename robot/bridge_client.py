"""Bot mode: client for the Pi-side vlm_bridge_node (branch main, NOT built yet).

Contract (design rev 4, sections 4-8):
  REST   http://<tailscale_ip>:<api>/v1/{hello,session,call,call/{id},snapshot,status,map,objects}
  WS     ws://<tailscale_ip>:<api>/v1/stream?token=...   events Pi->laptop, heartbeat laptop->Pi every 500 ms
  ESTOP  http://<tailscale_ip>:<estop>/estop             separate listener, no token, see robot/estop.py
Every /v1 request carries Authorization: Bearer <token>.
There is no reset endpoint; the laptop can never clear an E-STOP.
"""
import json
import threading
import time
from datetime import datetime, timezone

import requests
from websockets.sync.client import connect as ws_connect

from .base import RobotBase, new_id

CONTRACT_VERSION = "1.0"


def iso_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class BridgeError(RuntimeError):
    pass


class BridgeRobot(RobotBase):
    name = "pi-bridge"
    is_stub = False

    def __init__(self, cfg, token):
        super().__init__()
        b = cfg["bot"]
        if not b.get("tailscale_ip"):
            raise BridgeError("bot.tailscale_ip is not set in config/laptop.yaml")
        if not token:
            raise BridgeError(f"no token found in {b.get('token_file')}")
        self.ip = b["tailscale_ip"]
        self.api = f"http://{self.ip}:{b['ports']['api']}/v1"
        self.ws_url = f"ws://{self.ip}:{b['ports']['api']}/v1/stream"
        self.estop_url = f"http://{self.ip}:{b['ports']['estop']}/estop"
        self.hb_s = b.get("heartbeat_ms", 500) / 1000.0
        self.link_timeout_s = b.get("link_timeout_ms", 3000) / 1000.0
        self.max_age_ms = b.get("snapshot_max_age_ms", 500)
        self.token = token
        self.http = requests.Session()
        self.http.headers["Authorization"] = f"Bearer {token}"
        self.session_id = None
        self.agent_state = "idle"
        self.last_event_t = 0.0
        self.last_seq = -1
        self.rtt_ms = None
        self._stop = threading.Event()
        self._ws_thread = None
        self.last_status = {}

    # ---------- envelope helpers ----------
    def _envelope(self, tool, args, call_id, deadline_ms=None):
        return {"v": CONTRACT_VERSION, "id": call_id, "tool": tool, "args": args, "task_id": None,
                "plan_id": None, "deadline_ms": deadline_ms, "sent_at": iso_now()}

    def _get(self, path, timeout, **kw):
        r = self.http.get(self.api + path, timeout=timeout, **kw)
        if r.status_code == 401:
            raise BridgeError("UNAUTHORIZED: token rejected by the Pi bridge")
        return r

    # ---------- lifecycle ----------
    def start(self):
        hello = self._get("/hello", 2).json()
        major = str(hello.get("contract_version", "0")).split(".")[0]
        if major != CONTRACT_VERSION.split(".")[0]:
            raise BridgeError(f"VERSION_MISMATCH: Pi {hello.get('contract_version')} vs laptop {CONTRACT_VERSION}")
        r = self.http.post(self.api + "/session", json={"v": CONTRACT_VERSION, "client": "laptop-gui"}, timeout=3)
        r.raise_for_status()
        s = r.json()
        self.session_id = (s.get("data") or s).get("session_id")
        self._stop.clear()
        self._ws_thread = threading.Thread(target=self._ws_loop, name="bridge-ws", daemon=True)
        self._ws_thread.start()
        return hello

    def stop_all(self):
        self._stop.set()

    # ---------- WebSocket: events + heartbeat (own thread, independent of inference) ----------
    def _ws_loop(self):
        backoff = 0.5
        while not self._stop.is_set():
            try:
                with ws_connect(f"{self.ws_url}?token={self.token}", open_timeout=3,
                                additional_headers={"Authorization": f"Bearer {self.token}"}) as ws:
                    self.link_ok = True
                    backoff = 0.5
                    self.emit({"type": "link", "data": {"ok": True}})
                    hb_seq = 0
                    next_hb = 0.0
                    self.last_event_t = time.time()
                    while not self._stop.is_set():
                        now = time.time()
                        if now >= next_hb:
                            hb_seq += 1
                            ws.send(json.dumps({"v": CONTRACT_VERSION, "type": "hb", "session_id": self.session_id,
                                                "seq": hb_seq, "t": iso_now(), "t_local": now,
                                                "agent_state": self.agent_state}))
                            next_hb = now + self.hb_s
                        try:
                            raw = ws.recv(timeout=0.1)
                        except TimeoutError:
                            raw = None
                        if raw:
                            self.last_event_t = time.time()
                            self._on_event(json.loads(raw))
                        if time.time() - self.last_event_t > self.link_timeout_s:
                            raise BridgeError("LINK_LOST: no events for %.1fs" % self.link_timeout_s)
            except Exception as e:
                if self.link_ok:
                    self.emit({"type": "link", "data": {"ok": False, "error": str(e)}})
                self.link_ok = False
                self._stop.wait(backoff)
                backoff = min(backoff * 2, 5.0)

    def _on_event(self, ev):
        seq = ev.get("seq")
        if isinstance(seq, int):
            if seq <= self.last_seq:
                return
            self.last_seq = seq
        typ = ev.get("type")
        data = ev.get("data") or {}
        if typ == "hb_ack":
            t_local = data.get("t_local")
            if t_local:
                self.rtt_ms = round((time.time() - float(t_local)) * 1000)
            return
        if typ == "status":
            self.last_status = data
            est = data.get("estop", {})
            if "latched" in est:
                # Mirror the Pi's latch. It becomes False only after Amey resets on the dashboard.
                self.estop_latched = bool(est["latched"])
        elif typ == "human_override":
            self.human_override = True
        elif typ == "human_done":
            self.human_override = False
        self.emit(ev)

    # ---------- REST ----------
    def snapshot(self, max_age_ms=None):
        r = self._get("/snapshot", 3, params={"max_age_ms": max_age_ms or self.max_age_ms})
        if r.status_code != 200:
            raise BridgeError(f"snapshot HTTP {r.status_code}: {r.text[:200]}")
        h = r.headers
        meta = {"source": "pi", "frame_seq": h.get("X-Frame-Seq"), "capture_time": h.get("X-Capture-Time"),
                "pose": _maybe_json(h.get("X-Pose")), "head": _maybe_json(h.get("X-Head")), "stub": False}
        return r.content, meta

    def status(self):
        return self._get("/status", 2).json()

    def call(self, tool, args, call_id, cancel_event, timeout_s=30):
        env = self._envelope(tool, args, call_id, int(timeout_s * 1000))
        try:
            r = self.http.post(self.api + "/call", json=env, timeout=5)
        except requests.RequestException as e:
            # idempotent retry with the SAME id
            try:
                r = self.http.post(self.api + "/call", json=env, timeout=5)
            except requests.RequestException:
                return {"ok": False, "error": {"code": "LINK_LOST", "message": str(e), "retryable": True}}
        rep = r.json()
        if not rep.get("ok") or rep.get("status") != "accepted":
            return rep
        # long-running: poll until done / cancelled / timeout
        end = time.time() + timeout_s
        while time.time() < end:
            if cancel_event is not None and cancel_event.is_set():
                self.cancel_call(call_id)
                return {"ok": False, "error": {"code": "CANCELLED", "message": "cancelled by operator", "retryable": False}}
            time.sleep(0.25)
            try:
                rep = self._get(f"/call/{call_id}", 2).json()
            except requests.RequestException:
                continue
            if rep.get("status") not in ("accepted", "running"):
                return rep
        self.cancel_call(call_id)
        return {"ok": False, "error": {"code": "TIMEOUT", "message": f"{tool} exceeded {timeout_s}s", "retryable": True}}

    def cancel_call(self, call_id):
        """Cancel one in-flight call. PROPOSED contract addition: DELETE /v1/call/{id}; falls back to the
        `stop` tool (soft stop, never touches the E-STOP latch)."""
        try:
            r = self.http.delete(f"{self.api}/call/{call_id}", timeout=2)
            if r.ok:
                return True
        except requests.RequestException:
            pass
        try:
            self.http.post(self.api + "/call", json=self._envelope("stop", {}, new_id("stop")), timeout=2)
        except requests.RequestException:
            return False
        return True


def _maybe_json(s):
    if not s:
        return None
    try:
        return json.loads(s)
    except ValueError:
        return s
