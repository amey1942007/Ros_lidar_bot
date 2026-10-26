"""Laptop webcam capture (webcam mode). Falls back to a test image if no camera opens."""
import threading
import time
from pathlib import Path

import cv2
import numpy as np


class Webcam:
    def __init__(self, index=0, width=640, height=480, test_image=None):
        self.index, self.width, self.height = index, width, height
        self.test_image = test_image
        self._cap = None
        self._lock = threading.Lock()
        self.source = None  # "webcam" | "test_image" | "synthetic"

    def _open(self):
        if self._cap is not None:
            return True
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            cap.release()
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        for _ in range(5):  # let auto-exposure settle
            cap.read()
        self._cap = cap
        return True

    def grab_jpeg(self, quality=80):
        """Return (jpeg_bytes, meta)."""
        with self._lock:
            frame = None
            if self._open():
                ok, frame = self._cap.read()
                if not ok:
                    frame = None
                else:
                    self.source = "webcam"
            if frame is None and self.test_image and Path(self.test_image).exists():
                frame = cv2.imread(str(self.test_image))
                self.source = "test_image"
            if frame is None:
                frame = synthetic_frame(self.width, self.height)
                self.source = "synthetic"
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            h, w = frame.shape[:2]
            return buf.tobytes(), {"source": self.source, "capture_time": time.time(), "width": w, "height": h}

    def close(self):
        with self._lock:
            if self._cap is not None:
                self._cap.release()
                self._cap = None


def synthetic_frame(w=640, h=480):
    img = np.full((h, w, 3), 235, np.uint8)
    cv2.rectangle(img, (60, 260), (250, 420), (40, 40, 200), -1)       # red box
    cv2.circle(img, (450, 200), 80, (200, 120, 30), -1)               # blue ball
    cv2.putText(img, "TEST IMAGE", (180, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 3)
    return img
