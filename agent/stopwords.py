"""Text stop-word check. Runs on the command text BEFORE it reaches the VLM (V1 has no voice).
Whole-word, case-insensitive. "emergency" additionally fires the latched E-STOP."""
import re

STOP_WORDS = ("halt", "freeze", "emergency", "ruko", "tham")
ESTOP_WORDS = ("emergency",)
_RE = re.compile(r"\b(" + "|".join(STOP_WORDS) + r")\b", re.IGNORECASE)


def check(text):
    """Return None, or {"words": [...], "estop": bool}."""
    found = [m.group(1).lower() for m in _RE.finditer(text or "")]
    if not found:
        return None
    return {"words": found, "estop": any(w in ESTOP_WORDS for w in found)}
