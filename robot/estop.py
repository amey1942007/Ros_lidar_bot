"""E-STOP sender. Runs on its OWN thread with its OWN HTTP session, so it never waits behind
model inference, the agent loop, or the REST/WS bridge client.

There is deliberately NO reset/clear method here or anywhere in this codebase. The only way to
clear a latched E-STOP is Amey pressing "Reset E-STOP" on the robot dashboard (:8080).
"""
import queue
import threading
import time

import requests


class EStopSender:
    RETRIES = 3
    RETRY_GAP_S = 0.2

    def __init__(self, url=None, on_result=None, local_latch=None):
        """url: e.g. http://100.x.y.z:8091/estop (bot mode). local_latch: callable for webcam/stub mode."""
        self.url = url
        self.on_result = on_result or (lambda r: None)
        self.local_latch = local_latch
        self._q = queue.Queue()
        self._session = requests.Session()
        self._t = threading.Thread(target=self._run, name="estop-sender", daemon=True)
        self._t.start()

    def trigger(self, source="gui", text=None):
        """Non-blocking. Safe to call from the Tk thread or anywhere."""
        self._q.put({"source": source, "text": text, "t_press": time.time()})

    def _run(self):
        while True:
            req = self._q.get()
            result = {"source": req["source"], "ok": False, "attempts": 0, "error": None}
            if self.local_latch is not None:
                try:
                    self.local_latch(req["source"])
                    result.update(ok=True, latched=True, stub=self.url is None)
                except Exception as e:
                    result["error"] = str(e)
            if self.url:
                result["ok"] = False
                for attempt in range(1, self.RETRIES + 1):
                    result["attempts"] = attempt
                    try:
                        r = self._session.post(self.url, json={"source": req["source"], "text": req["text"]},
                                               timeout=1.0)
                        if r.ok:
                            result.update(ok=True, latched=True, http=r.status_code)
                            break
                        result["error"] = f"HTTP {r.status_code}"
                    except requests.RequestException as e:
                        result["error"] = str(e)
                    time.sleep(self.RETRY_GAP_S)
            result["latency_ms"] = round((time.time() - req["t_press"]) * 1000)
            self.on_result(result)
