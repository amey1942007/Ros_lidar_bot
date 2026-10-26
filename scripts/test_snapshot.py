"""Grab one webcam frame (or a test image), send it to the VLM, print the reply, timings and VRAM.

  D:\\Vision-Pipeline\\Scripts\\python.exe scripts\\test_snapshot.py [--model qwen3-vl:4b] [--image path] [--no-think]
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import load_config  # noqa: E402
from robot.webcam import Webcam  # noqa: E402
from vlm.ollama_client import OllamaClient  # noqa: E402

PROMPT = ("You are the vision system of a small indoor robot. Describe what you see in 2-3 sentences, then list "
          "up to 5 objects with their rough position (left/centre/right, near/far) as JSON: "
          '{"objects":[{"label":"...","where":"..."}]}')


def vram():
    out = {}
    try:
        out["nvidia_smi"] = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.used,memory.total", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception as e:
        out["nvidia_smi"] = f"n/a ({e})"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model")
    ap.add_argument("--image")
    ap.add_argument("--no-think", action="store_true")
    ap.add_argument("--save", default="logs/test_snapshot.jpg")
    a = ap.parse_args()
    cfg = load_config()
    if a.model:
        cfg["model"]["name"] = a.model
    if a.no_think:
        cfg["model"]["think"] = False
    m = OllamaClient.from_config(cfg)
    print("model:", m.model, "| ollama up:", m.health(), "| model present:", m.has_model())
    print("VRAM before:", vram())

    w = cfg["webcam"]
    if a.image:
        jpeg = Path(a.image).read_bytes()
        meta = {"source": "file:" + a.image}
    else:
        cam = Webcam(w.get("index", 0), w.get("width", 640), w.get("height", 480), w.get("test_image"))
        t = time.perf_counter()
        jpeg, meta = cam.grab_jpeg()
        cam.close()
        meta["grab_s"] = round(time.perf_counter() - t, 3)
    Path(a.save).parent.mkdir(parents=True, exist_ok=True)
    Path(a.save).write_bytes(jpeg)
    print("image:", meta, f"{len(jpeg)//1024} KB -> {a.save}")

    msgs = [{"role": "user", "content": PROMPT, "images": [jpeg]}]
    print("\n--- streaming reply ---")
    last = None

    def show(ev):
        nonlocal last
        if ev["type"] in ("thinking", "content"):
            if ev["type"] != last:
                print(f"\n[{ev['type']}] ", end="")
                last = ev["type"]
            print(ev["text"], end="", flush=True)

    thinking, content, stats = m.chat(msgs, on_chunk=show)
    print("\n\n--- stats ---")
    print(json.dumps(stats, indent=2))
    try:
        print("ollama ps:", json.dumps(m.ps(), indent=2))
    except Exception as e:
        print("ollama ps failed:", e)
    print("VRAM after:", vram())
    try:
        print(subprocess.run(["ollama", "ps"], capture_output=True, text=True, timeout=10).stdout)
    except Exception:
        pass


if __name__ == "__main__":
    main()
