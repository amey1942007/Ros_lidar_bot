"""Webcam mode: robot tools are STUBS. Nothing moves; every result says so."""
import threading
import time

from .base import RobotBase
from .webcam import Webcam

STUB_NOTE = "STUB (webcam mode): no robot connected; on the real bot this tool runs on the Pi"


class StubRobot(RobotBase):
    name = "webcam-stub"
    is_stub = True

    def __init__(self, cfg, tool_delay_s=1.0):
        super().__init__()
        w = cfg["webcam"]
        self.cam = Webcam(w.get("index", 0), w.get("width", 640), w.get("height", 480), w.get("test_image"))
        self.pan_home_deg = cfg["robot"].get("pan_home_deg")
        self.tool_delay_s = tool_delay_s
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame": "map"}
        self.head = {"tilt_deg": 0.0, "pan_deg_est": 0.0}
        self.link_ok = True

    def start(self):
        self.emit({"type": "link", "data": {"ok": True, "mode": "webcam"}})

    def latch_estop(self, source="gui"):
        self.estop_latched = True   # one-way: there is no unlatch in this class
        self.emit({"type": "status", "data": {"estop": {"latched": True, "source": source}}})

    def snapshot(self, max_age_ms=500):
        jpeg, meta = self.cam.grab_jpeg()
        meta.update(pose=dict(self.pose), head=dict(self.head), stub=True)
        return jpeg, meta

    def status(self):
        return {"stub": True, "estop": {"latched": self.estop_latched}, "pose": self.pose, "head": self.head,
                "note": STUB_NOTE}

    def _wait(self, cancel_event, secs):
        if secs <= 0:
            return True
        end = time.time() + secs
        while time.time() < end:
            if cancel_event is not None and cancel_event.is_set():
                return False
            time.sleep(0.05)
        return True

    def call(self, tool, args, call_id, cancel_event, timeout_s=30):
        if self.estop_latched and tool in ("goto_pose", "move", "look", "map"):
            return {"ok": False, "error": {"code": "ESTOP_LATCHED", "message": "E-STOP latched; reset only on the dashboard", "retryable": False}}
        if not self._wait(cancel_event, 0 if tool == "stop" else self.tool_delay_s):
            return {"ok": False, "error": {"code": "CANCELLED", "message": "cancelled by operator", "retryable": False}}
        data = {"stub": True, "note": STUB_NOTE}
        if tool == "goto_pose":
            self.pose = {"x": args.get("x", 0), "y": args.get("y", 0), "yaw": args.get("yaw", 0), "frame": "map"}
            data["pose"] = self.pose
        elif tool == "move":
            data["requested"] = args
        elif tool == "look":
            pan = args.get("pan") or {}
            if pan.get("mode") == "home":
                if self.pan_home_deg is None:
                    return {"ok": False, "error": {"code": "PAN_HOME_UNSET", "message": "pan_home_deg not configured yet", "retryable": False}}
                self.head["pan_deg_est"] = self.pan_home_deg
            elif pan.get("mode") == "relative":
                self.head["pan_deg_est"] += float(pan.get("deg", 0))
            if "tilt_deg" in args:
                self.head["tilt_deg"] = float(args["tilt_deg"])
            data["head"] = dict(self.head)
            data["pan_confidence"] = "stub"
        elif tool == "stop":
            data["stopped"] = True
        elif tool == "detect":
            data["detections"] = []
            data["note"] += "; Pi YOLO not available, use the VLM on the snapshot instead"
        elif tool == "map":
            data["map"] = None
        elif tool == "get_status":
            data = self.status()
        else:
            return {"ok": False, "error": {"code": "BAD_REQUEST", "message": f"unknown tool {tool}", "retryable": False}}
        return {"ok": True, "status": "done", "data": data}
