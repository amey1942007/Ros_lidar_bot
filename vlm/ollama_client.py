"""Streaming client for a local Ollama server (native /api/chat). Runs on the LAPTOP GPU."""
import base64
import json
import time
import requests


class OllamaError(RuntimeError):
    pass


class OllamaClient:
    def __init__(self, url="http://127.0.0.1:11434", model="qwen3-vl:8b", think=True,
                 temperature=0.2, num_ctx=8192, keep_alive="10m", timeout_s=180):
        self.url = url.rstrip("/")
        self.model = model
        self.think = think
        self.temperature = temperature
        self.num_ctx = num_ctx
        self.keep_alive = keep_alive
        self.timeout_s = timeout_s

    @classmethod
    def from_config(cls, cfg):
        m = cfg["model"]
        return cls(m["ollama_url"], m["name"], m.get("think", True), m.get("temperature", 0.2),
                   m.get("num_ctx", 8192), m.get("keep_alive", "10m"), m.get("request_timeout_s", 180))

    # ---- health / model management ----
    def health(self):
        try:
            r = requests.get(self.url + "/api/version", timeout=2)
            return r.ok
        except requests.RequestException:
            return False

    def has_model(self):
        r = requests.get(self.url + "/api/tags", timeout=5)
        r.raise_for_status()
        return any(m.get("name") == self.model or m.get("model") == self.model for m in r.json().get("models", []))

    def ps(self):
        r = requests.get(self.url + "/api/ps", timeout=5)
        r.raise_for_status()
        return r.json().get("models", [])

    def load(self):
        """Warm the model into VRAM (empty prompt)."""
        r = requests.post(self.url + "/api/generate", json={"model": self.model, "keep_alive": self.keep_alive},
                          timeout=self.timeout_s)
        r.raise_for_status()

    # ---- chat ----
    @staticmethod
    def encode_image(jpeg_bytes):
        return base64.b64encode(jpeg_bytes).decode("ascii")

    def chat_stream(self, messages, cancel_event=None, fmt=None):
        """Yield dicts: {"type":"thinking"|"content", "text":...} and finally {"type":"done", "stats":{...}}.

        messages: Ollama chat messages; image bytes may be given as message["images"] = [bytes,...].
        """
        msgs = []
        for m in messages:
            m = dict(m)
            if m.get("images"):
                m["images"] = [i if isinstance(i, str) else self.encode_image(i) for i in m["images"]]
            msgs.append(m)
        body = {"model": self.model, "messages": msgs, "stream": True, "keep_alive": self.keep_alive,
                "options": {"temperature": self.temperature, "num_ctx": self.num_ctx}}
        if self.think:
            body["think"] = True
        if fmt:
            body["format"] = fmt
        t0 = time.perf_counter()
        ttft = None
        try:
            resp = requests.post(self.url + "/api/chat", json=body, stream=True, timeout=self.timeout_s)
        except requests.RequestException as e:
            raise OllamaError(f"Ollama unreachable at {self.url}: {e}") from e
        if resp.status_code != 200:
            raise OllamaError(f"Ollama HTTP {resp.status_code}: {resp.text[:300]}")
        try:
            for line in resp.iter_lines():
                if cancel_event is not None and cancel_event.is_set():
                    yield {"type": "cancelled"}
                    return
                if not line:
                    continue
                chunk = json.loads(line)
                if "error" in chunk:
                    raise OllamaError(chunk["error"])
                msg = chunk.get("message", {})
                for key in ("thinking", "content"):
                    txt = msg.get(key)
                    if txt:
                        if ttft is None:
                            ttft = time.perf_counter() - t0
                        yield {"type": key, "text": txt}
                if chunk.get("done"):
                    ev_n = chunk.get("eval_count", 0)
                    ev_d = chunk.get("eval_duration", 0) / 1e9
                    yield {"type": "done", "stats": {
                        "ttft_s": round(ttft, 3) if ttft is not None else None,
                        "total_s": round(time.perf_counter() - t0, 3),
                        "load_s": round(chunk.get("load_duration", 0) / 1e9, 3),
                        "prompt_tokens": chunk.get("prompt_eval_count", 0),
                        "prompt_eval_s": round(chunk.get("prompt_eval_duration", 0) / 1e9, 3),
                        "eval_tokens": ev_n,
                        "tokens_per_s": round(ev_n / ev_d, 2) if ev_d else None,
                    }}
                    return
        finally:
            resp.close()

    def chat(self, messages, cancel_event=None, fmt=None, on_chunk=None):
        """Blocking helper: returns (thinking, content, stats)."""
        thinking, content, stats = [], [], {}
        for ev in self.chat_stream(messages, cancel_event, fmt):
            if on_chunk:
                on_chunk(ev)
            if ev["type"] == "thinking":
                thinking.append(ev["text"])
            elif ev["type"] == "content":
                content.append(ev["text"])
            elif ev["type"] == "done":
                stats = ev["stats"]
            elif ev["type"] == "cancelled":
                stats = {"cancelled": True}
        return "".join(thinking), "".join(content), stats
