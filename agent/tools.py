"""Tool registry with JSON schemas.

runs_on = "pi"     -> sent to the Pi bridge (bot mode) or the StubRobot (webcam mode). The laptop never
                      drives hardware itself.
runs_on = "laptop" -> handled in this process (snapshot fetch, finish, ask_user).
runs_on = "estop"  -> handed to the separate E-STOP sender thread (latch only).

Rules enforced here: no tool may clear/reset the E-STOP; no battery tool.
"""
import jsonschema

FORBIDDEN_NAME_PARTS = ("reset", "clear", "unlatch", "release", "battery")

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

MOTION_TOOLS = {"goto_pose", "move", "look", "map"}


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
        "plan": {"type": "array", "minItems": 1, "maxItems": 8,
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
