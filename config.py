"""Config loader. Reads config/laptop.yaml (or the path in VLM_CONFIG)."""
import os
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent


def load_config(path=None):
    path = Path(path or os.environ.get("VLM_CONFIG") or ROOT / "config" / "laptop.yaml")
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cfg


def read_token(cfg):
    p = cfg["bot"].get("token_file")
    if not p:
        return None
    p = Path(os.path.expandvars(os.path.expanduser(p)))
    if not p.exists():
        return None
    return p.read_text(encoding="utf-8").strip() or None
