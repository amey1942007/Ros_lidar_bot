"""Tool registry with JSON schemas.

runs_on = "pi"     -> sent to the Pi bridge (bot mode) or the StubRobot (webcam mode). The laptop never
                      drives hardware itself.
runs_on = "laptop" -> handled in this process (snapshot fetch, finish, ask_user, agent/skills.py composites).
runs_on = "estop"  -> handed to the separate E-STOP sender thread (latch only).

Rules enforced here: no tool may clear/reset the E-STOP; no battery tool.
"""
import jsonschema

FORBIDDEN_NAME_PARTS = ("reset", "clear", "unlatch", "release", "battery", "dock", "charge")

NUM = {"type": "number"}

TOOLS = {
    "snapshot": {
        "runs_on": "laptop",
        "description": "Fetch a fresh camera image (from the robot camera in bot mode, the laptop webcam in webcam mode). The image is attached to your next turn.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "timeout_s": 5,
    },
    "goto_pose": {
        "runs_on": "pi",
        "description": "Navigate (Nav2) to a pose in the map frame. x,y metres, yaw radians.",
        "args": {"type": "object", "required": ["x", "y"], "additionalProperties": False,
                 "properties": {"x": NUM, "y": NUM, "yaw": NUM, "frame": {"enum": ["map"]}}},
        "timeout_s": 120,
    },
    "move": {
        "runs_on": "pi",
        "description": "Short relative motion: drive distance_m forward (negative = back) and/or turn turn_deg (positive = left). Max 1.0 m / 180 deg per call.",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"distance_m": {"type": "number", "minimum": -1.0, "maximum": 1.0},
                                "turn_deg": {"type": "number", "minimum": -180, "maximum": 180}}},
        "timeout_s": 30,
    },
    "look": {
        "runs_on": "pi",
        "description": "Point the camera head. tilt_deg absolute (-80..80). pan: {mode:'relative', deg} or {mode:'home'} (home needs pan_home_deg configured). Pan has no encoder, so the angle is an estimate.",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"tilt_deg": {"type": "number", "minimum": -80, "maximum": 80},
                                "pan": {"type": "object", "additionalProperties": False, "required": ["mode"],
                                        "properties": {"mode": {"enum": ["relative", "home"]},
                                                       "deg": {"type": "number", "minimum": -180, "maximum": 180}}},
                                "settle_ms": {"type": "integer", "minimum": 0, "maximum": 3000}}},
        "timeout_s": 10,
    },
    "stop": {
        "runs_on": "pi",
        "description": "Soft stop: cancel navigation and stop moving. Not latched.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "timeout_s": 3,
    },
    "estop": {
        "runs_on": "estop",
        "description": "EMERGENCY STOP. Latches on the robot. You can trigger it but you can NEVER clear it; only Amey can, on the dashboard.",
        "args": {"type": "object", "properties": {"reason": {"type": "string"}}, "additionalProperties": False},
        "timeout_s": 2,
    },
    "detect": {
        "runs_on": "pi",
        "description": "Object detection with the Pi YOLO node. Returns labelled boxes for the given labels.",
        "args": {"type": "object", "required": ["labels"], "additionalProperties": False,
                 "properties": {"labels": {"type": "array", "items": {"type": "string"}, "minItems": 1, "maxItems": 10},
                                "window_ms": {"type": "integer", "minimum": 200, "maximum": 5000},
                                "min_conf": {"type": "number", "minimum": 0, "maximum": 1},
                                "verify": {"type": "boolean"}}},
        "timeout_s": 8,
    },
    "map": {
        "runs_on": "pi",
        "description": "Mapping: action 'status' | 'save' (name) | 'semantic' (labels) | 'coverage' | 'explore' (enable true/false, max_s). Amey allows you to explore and save maps without asking.",
        "args": {"type": "object", "required": ["action"], "additionalProperties": False,
                 "properties": {"action": {"enum": ["status", "save", "semantic", "coverage", "explore"]},
                                "enable": {"type": "boolean"},
                                "max_s": {"type": "integer", "minimum": 10, "maximum": 1800},
                                "name": {"type": "string", "pattern": "^[a-z0-9_-]{1,32}$"},
                                "labels": {"type": "array", "items": {"type": "string"}}}},
        "timeout_s": 10,
    },
    "get_status": {
        "runs_on": "pi",
        "description": "Robot status: pose, head, E-STOP latch, safety_blocked, nav state.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "timeout_s": 3,
    },
    "ask_user": {
        "runs_on": "laptop",
        "description": "Ask Amey a question in the GUI and stop the run until he answers with a new command.",
        "args": {"type": "object", "required": ["question"], "additionalProperties": False,
                 "properties": {"question": {"type": "string"}}},
        "timeout_s": 1,
    },
    "finish": {
        "runs_on": "laptop",
        "description": "End the task with a final answer.",
        "args": {"type": "object", "required": ["answer"], "additionalProperties": False,
                 "properties": {"answer": {"type": "string"}}},
        "timeout_s": 1,
    },
}

NEW_TOOLS = {
    "describe_scene": {
        "runs_on": "laptop",
        "description": "Look at ONE fresh camera image. With 'target', say whether that object is visible and give its bounding box in pixels.",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"target": {"type": "string"}}},
        "outputs": "{found: bool|null, description: str, bbox: [x1,y1,x2,y2] px|null, image: {width,height}}",
        "timeout_s": 120,
    },
    "feed": {
        "runs_on": "laptop",
        "description": "Camera feed at an interval: take a frame every interval_s for up to max_s, describe each; stops early if 'goal' is seen. Use to search a region.",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"goal": {"type": "string"},
                                "interval_s": {"type": "number", "minimum": 0.5, "maximum": 30},
                                "max_s": {"type": "number", "minimum": 1, "maximum": 600}}},
        "outputs": "{met: bool, frames: int, last: {found, description, bbox}, history: [descriptions], reason: goal|timeout}",
        "timeout_s": 660,
    },
    "watch_for": {
        "runs_on": "laptop",
        "description": "Describe the scene continuously, one frame per second, until 'goal' is seen or max_s passes.",
        "args": {"type": "object", "required": ["goal"], "additionalProperties": False,
                 "properties": {"goal": {"type": "string"},
                                "max_s": {"type": "number", "minimum": 1, "maximum": 600}}},
        "outputs": "{met: bool, frames: int, last: {found, description, bbox}, reason: goal|timeout}",
        "timeout_s": 660,
    },
    "start_exploring": {
        "runs_on": "laptop",
        "description": "Turn on frontier exploration (it keeps going by itself). Optional condition: until_seen (stop when that object is seen) and/or max_s.",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"max_s": {"type": "integer", "minimum": 10, "maximum": 1800},
                                "until_seen": {"type": "string"},
                                "check_interval_s": {"type": "number", "minimum": 1, "maximum": 30}}},
        "outputs": "{exploring: bool, stopped_because: seen|timeout|null, last: {...}|null} (without until_seen it returns at once while exploration carries on)",
        "timeout_s": 1900,
    },
    "stop_exploring": {
        "runs_on": "laptop",
        "description": "Stop frontier exploration.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "outputs": "{exploring: false}",
        "timeout_s": 10,
    },
    "save_map": {
        "runs_on": "laptop",
        "description": "Save the current map (to ~/maps/<name>).",
        "args": {"type": "object", "additionalProperties": False,
                 "properties": {"name": {"type": "string", "pattern": "^[a-z0-9_-]{1,32}$"}}},
        "outputs": "{saved: name, map: robot reply}",
        "timeout_s": 30,
    },
    "save_location": {
        "runs_on": "laptop",
        "description": "Save the robot's current x, y, yaw as a named location (overwrites the same name).",
        "args": {"type": "object", "required": ["name"], "additionalProperties": False,
                 "properties": {"name": {"type": "string", "pattern": "^[a-z0-9_-]{1,32}$"},
                                "note": {"type": "string"}}},
        "outputs": "{saved: {name, x, y, yaw, note}}",
        "timeout_s": 5,
    },
    "note_location": {
        "runs_on": "laptop",
        "description": "Remember something special about a location (named, or the current spot if no name; current spot is saved as 'spot-N').",
        "args": {"type": "object", "required": ["note"], "additionalProperties": False,
                 "properties": {"name": {"type": "string", "pattern": "^[a-z0-9_-]{1,32}$"},
                                "note": {"type": "string"}}},
        "outputs": "{saved: {name, x, y, yaw, note}}",
        "timeout_s": 5,
    },
    "recall_location": {
        "runs_on": "laptop",
        "description": "Navigate to a saved named location.",
        "args": {"type": "object", "required": ["name"], "additionalProperties": False,
                 "properties": {"name": {"type": "string"}}},
        "outputs": "{location: {...}, nav: goto_pose reply}",
        "timeout_s": 130,
    },
    "list_locations": {
        "runs_on": "laptop",
        "description": "List saved locations with poses and notes.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "outputs": "{locations: [{name, x, y, yaw, note}]}",
        "timeout_s": 2,
    },
    "delete_location": {
        "runs_on": "laptop",
        "description": "Delete a saved location.",
        "args": {"type": "object", "required": ["name"], "additionalProperties": False,
                 "properties": {"name": {"type": "string"}}},
        "outputs": "{deleted: name}",
        "timeout_s": 2,
    },
    "return_home": {
        "runs_on": "laptop",
        "description": "Navigate to the saved location 'home'.",
        "args": {"type": "object", "properties": {}, "additionalProperties": False},
        "outputs": "{location: {...}, nav: goto_pose reply}",
        "timeout_s": 130,
    },
    "wait": {
        "runs_on": "laptop",
        "description": "Pause for seconds (cancellable).",
        "args": {"type": "object", "required": ["seconds"], "additionalProperties": False,
                 "properties": {"seconds": {"type": "number", "minimum": 0, "maximum": 300}}},
        "outputs": "{waited_s: number}",
        "timeout_s": 310,
    },
    "scan_area": {
        "runs_on": "laptop",
        "description": "Sweep the view: pan the head (use='head') or rotate the body (use='body') through angles_deg, checking each view for 'target'. Stops at the first hit.",
        "args": {"type": "object", "required": ["target"], "additionalProperties": False,
                 "properties": {"target": {"type": "string"},
                                "use": {"enum": ["head", "body"]},
                                "angles_deg": {"type": "array", "minItems": 1, "maxItems": 12,
                                               "items": {"type": "number", "minimum": -180, "maximum": 180}}}},
        "outputs": "{found: bool, at_deg: number|null, views: [{deg, found, description, bbox}]}",
        "timeout_s": 600,
    },
    "go_to_object": {
        "runs_on": "laptop",
        "description": "Approach a visible object: turn toward its bounding box and drive in short hops until it fills enough of the view or stop_m budget ends.",
        "args": {"type": "object", "required": ["target"], "additionalProperties": False,
                 "properties": {"target": {"type": "string"},
                                "max_m": {"type": "number", "minimum": 0.1, "maximum": 5}}},
        "outputs": "{reached: bool, driven_m: number, reason: close|lost|budget, last: {found, description, bbox}}",
        "timeout_s": 300,
    },
    "follow": {
        "runs_on": "laptop",
        "description": "Follow an object: keep turning toward it and closing distance until max_s; stops if it is lost.",
        "args": {"type": "object", "required": ["target"], "additionalProperties": False,
                 "properties": {"target": {"type": "string"},
                                "max_s": {"type": "number", "minimum": 1, "maximum": 600}}},
        "outputs": "{followed_s: number, reason: lost|timeout, last: {...}}",
        "timeout_s": 660,
    },
    "set_speed": {
        "runs_on": "pi",
        "description": "Set the navigation speed limit in m/s (clamped 0.05..0.5).",
        "args": {"type": "object", "required": ["max_mps"], "additionalProperties": False,
                 "properties": {"max_mps": {"type": "number", "minimum": 0.05, "maximum": 0.5}}},
        "outputs": "{max_mps: number}",
        "timeout_s": 3,
    },
    "record_video": {
        "runs_on": "laptop",
        "description": "Start or stop recording camera frames to disk.",
        "args": {"type": "object", "required": ["action"], "additionalProperties": False,
                 "properties": {"action": {"enum": ["start", "stop"]},
                                "fps": {"type": "number", "minimum": 0.5, "maximum": 10}}},
        "outputs": "{recording: bool, dir: path, frames: int}",
        "timeout_s": 5,
    },
}

OLD_OUTPUTS = {
    "snapshot": "{note}; the new image is attached to your next turn",
    "goto_pose": "{pose} when the goal is reached, or error",
    "move": "{requested/done motion}",
    "look": "{head: {tilt_deg, pan_deg_est}, pan_confidence}",
    "stop": "{stopped: true}",
    "estop": "{estop: 'sent'} (latched)",
    "detect": "{detections: [{label, conf, bbox}]}",
    "map": "{map: status / saved path / semantic labels / coverage / explore state}",
    "get_status": "{pose: {x,y,yaw}, head, estop: {latched}, safety_blocked, nav}",
    "ask_user": "run stops until Amey answers",
    "finish": "task ends with your answer",
}

TOOLS.update(NEW_TOOLS)
for _n, _o in OLD_OUTPUTS.items():
    TOOLS[_n].setdefault("outputs", _o)

# robot motion tools and composite tools that move the robot: refused while the E-STOP is latched
MOTION_TOOLS = {"goto_pose", "move", "look", "map", "start_exploring", "recall_location", "return_home",
                "go_to_object", "follow", "scan_area"}
SKILL_TOOLS = set(NEW_TOOLS) - {"set_speed"}


def check_registry(tools=TOOLS):
    for name in tools:
        low = name.lower()
        for bad in FORBIDDEN_NAME_PARTS:
            if bad in low:
                raise ValueError(f"tool '{name}' is not allowed (E-STOP reset / battery tools are forbidden)")
    return True


check_registry()

# Model output: one JSON object per turn.
STEP_SCHEMA = {
    "type": "object",
    "required": ["plan"],
    "properties": {
        "observation": {"type": "string"},
        "plan": {"type": "array", "minItems": 1,
                 "items": {"type": "object", "required": ["tool", "args"],
                           "properties": {"tool": {"enum": sorted(TOOLS)},
                                          "args": {"type": "object"},
                                          "why": {"type": "string"}}}},
    },
}


def validate_step(obj):
    """Raise jsonschema.ValidationError / ValueError with a model-readable message."""
    jsonschema.validate(obj, STEP_SCHEMA)
    for i, st in enumerate(obj["plan"]):
        spec = TOOLS[st["tool"]]
        try:
            jsonschema.validate(st["args"], spec["args"])
        except jsonschema.ValidationError as e:
            raise ValueError(f"plan[{i}] {st['tool']} args invalid: {e.message}") from e
    return obj


def catalogue_text(stub_mode):
    lines = []
    for n, s in TOOLS.items():
        where = {"pi": "runs on the Pi robot" + (" (STUBBED in webcam mode)" if stub_mode else ""),
                 "laptop": "runs on the laptop", "estop": "separate E-STOP channel"}[s["runs_on"]]
        lines.append(f"- {n} [{where}]: {s['description']}\n  args schema: {s['args']}")
    return "\n".join(lines)
