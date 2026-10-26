"""Robot interface. The laptop never runs robot tools itself: in bot mode it sends them to the Pi
bridge (main branch) over Tailscale; in webcam mode StubRobot answers with clearly-marked stubs."""
import threading
import time
import uuid


def new_id(prefix="c"):
    return f"{prefix}-{uuid.uuid4().hex[:12]}"


class RobotBase:
    name = "base"
    is_stub = True

    def __init__(self):
        self.estop_latched = False      # only the robot side (dashboard reset) can clear this
        self.human_override = False
        self.link_ok = False
        self.listeners = []             # callables(event_dict)
        self._lock = threading.Lock()

    def add_listener(self, fn):
        self.listeners.append(fn)

    def emit(self, ev):
        ev.setdefault("t", time.time())
        for fn in list(self.listeners):
            try:
                fn(ev)
            except Exception:
                pass

    # subclasses implement:
    def start(self): ...
    def stop_all(self): ...
    def snapshot(self, max_age_ms=500): raise NotImplementedError
    def call(self, tool, args, call_id, cancel_event, timeout_s=30): raise NotImplementedError
    def cancel_call(self, call_id): ...
    def status(self): return {}
