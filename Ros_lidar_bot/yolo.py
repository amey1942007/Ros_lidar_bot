#!/usr/bin/env python3
"""\nyolo.py — YOLO-World webcam object detector.

================================================================================
UNDERLYING SYSTEM & DATA FLOW
================================================================================
This node handles open-vocabulary object detection using Ultralytics YOLO-World.
- Subscribes to: camera frames via Picamera2 (RPi CSI camera, e.g. Camera Module 3)
  or OpenCV VideoCapture (USB webcam) -- see --backend.
- Publishes to: /yolo (std_msgs/String carrying a JSON payload)
  Format of published string:
  {
    "frame_id": 128,
    "timestamp": 1731000000.123,
    "detections": [
      {"class": "person", "confidence": 0.87, "bbox": [x, y, w, h]}
    ]
  }
  Where x, y = top-left pixel coordinates, w, h = box pixel width & height.

================================================================================
CLASS VOCABULARY & OBJECT CATEGORIES
================================================================================
Configures YOLO-World offline open-vocabulary search limits to target a predefined
list of vocabulary classes including standard industrial and household elements:
'phone', 'person', 'chair', 'box', 'bottle', 'laptop', 'wrench', 'battery', etc.

================================================================================
TIMER-DRIVEN EXECUTION LOOP
================================================================================
To prevent blocking and system latency during heavier inference cycles:
- Runs in a timer-driven callback thread (`on_timer` at 30 Hz default rate).
- Captures frames from OpenCV, runs model prediction, formats JSON metadata,
  publishes String message, and optionally displays debug windows (`--show`).\n"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

# ---------------------------------------------------------------------------
# Your class list
# ---------------------------------------------------------------------------
CLASSES = [
    "phone", "person", "chair", "box", "bottle", "laptop", "cardboard box",
    "keyboard", "wrench", "battery", "desk", "lamp", "bathtub", "microwave",
    "toilet", "bed", "tv", "sink", "sofa", "mouse", "refrigerator", "oven",
    "bicycle", "whiteboard", "air vent", "washing machine", "printer",
    "bookshelf", "nightstand", "filing cabinet",
]
# "keyboard" appeared twice in your original list; duplicates removed here.
CLASSES = list(dict.fromkeys(CLASSES))

# Weights cache lives outside the workspace (~/.cache, not the git repo) so:
#  - a bare filename like "yolov8s-world.pt" always resolves to ONE place,
#    no matter whether this is launched from a terminal or as a dashboard
#    subprocess (different CWDs) -- one download, not one per launch style.
#  - `git pull` / `colcon build` never touches it -- it survives both.
DEFAULT_MODEL = os.path.expanduser("~/.cache/Ros_lidar_bot/yolov8s-world.pt")


def load_model(model_path, classes):
    from ultralytics import YOLO

    os.makedirs(os.path.dirname(model_path) or ".", exist_ok=True)
    model = YOLO(model_path)    # downloads to model_path on first run, cached after
    model.set_classes(classes)  # restrict YOLO-World's open vocabulary to your list
    return model


class YoloWorldPublisher(Node):
    def __init__(self, args):
        super().__init__("yolo_world_publisher")
        self.args = args
        self.pub = self.create_publisher(String, args.topic, 10)
        self.get_logger().info(f"Publishing detections on topic '{args.topic}' (std_msgs/String, JSON payload)")

        self.get_logger().info("Loading YOLO-World model...")
        self.model = load_model(args.model, CLASSES)

        # Camera Module 3 (and other RPi CSI cameras) only speak libcamera on
        # Bookworm -- cv2.VideoCapture's V4L2 backend can't open them. USB
        # webcams still work fine through OpenCV, so pick per --backend.
        backend = args.backend
        if backend == "auto":
            backend = "picamera2" if Picamera2 is not None else "cv2"
        self.backend = backend

        self.cap = None
        self.picam2 = None
        if backend == "picamera2":
            if Picamera2 is None:
                self.get_logger().error(
                    "picamera2 not installed (sudo apt install -y python3-picamera2)")
                sys.exit(1)
            self.picam2 = Picamera2(camera_num=args.camera)
            # semantic_slam_node converts bbox pixels → bearing using IMG_W=640,
            # IMG_H=480 — the capture MUST match or every object angle is wrong.
            # "RGB888" is a Picamera2 naming quirk -- it actually delivers
            # BGR-ordered frames, which is exactly what OpenCV/YOLO expect.
            config = self.picam2.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"})
            self.picam2.configure(config)
            self.picam2.start()
            self.get_logger().info(f"Capturing via Picamera2 (camera_num={args.camera})")
        else:
            self.cap = cv2.VideoCapture(args.camera)
            if not self.cap.isOpened():
                self.get_logger().error(f"Could not open camera index {args.camera}")
                sys.exit(1)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(f"Capturing via OpenCV VideoCapture(index={args.camera})")

        self.log_file = open(args.log, "a") if args.log else None
        self.frame_id = 0
        self._dash_warned = False
        # Throttle the dashboard preview push independently of inference rate.
        # Default 5 Hz means only ~5 JPEG encodes+POSTs per second regardless
        # of how fast YOLO runs. This keeps CPU and loopback traffic tiny.
        self._dash_interval = 1.0 / max(0.1, args.dash_fps)
        self._next_dash_push = 0.0

        # Timer-driven loop so rclpy stays responsive (Ctrl+C, ros2 node info, etc.)
        period = 1.0 / args.rate if args.rate > 0 else 0.0
        self.timer = self.create_timer(period, self.on_timer)

    def read_frame(self):
        if self.backend == "picamera2":
            return True, self.picam2.capture_array()
        return self.cap.read()

    def push_dashboard_frame(self, frame):
        """Best-effort JPEG push to the dashboard's local HTTP server so the
        browser can see the camera feed. Loopback-only, never touches ROS --
        no image topic, no subscribers, no DDS bandwidth.

        Rate is capped at --dash-fps (default 5 Hz) independently of the YOLO
        inference rate so the preview costs almost no extra CPU."""
        if not self.args.dash_url:
            return
        now = time.monotonic()
        if now < self._next_dash_push:
            return   # not yet time for the next preview frame
        self._next_dash_push = now + self._dash_interval
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 55])
        if not ok:
            return
        try:
            req = urllib.request.Request(
                self.args.dash_url, data=buf.tobytes(),
                headers={"Content-Type": "image/jpeg"}, method="POST")
            urllib.request.urlopen(req, timeout=0.5).close()
        except (urllib.error.URLError, OSError):
            if not self._dash_warned:
                self.get_logger().warn(
                    f"dashboard not reachable at {self.args.dash_url} "
                    "(camera preview won't show; detection still runs)")
                self._dash_warned = True

    def on_timer(self):
        ok, frame = self.read_frame()
        if not ok:
            self.get_logger().warn("Failed to grab frame, retrying...")
            return

        self.push_dashboard_frame(frame)

        results = self.model.predict(frame, conf=self.args.conf, verbose=False)
        r = results[0]

        detections = []
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w = x2 - x1
            h = y2 - y1
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])

            detections.append({
                "class": cls_name,
                "confidence": round(conf, 3),
                # [x, y, w, h] — x, y = top-left corner
                "bbox": [round(x1, 1), round(y1, 1), round(w, 1), round(h, 1)],
            })

        payload = {
            "frame_id": self.frame_id,
            "timestamp": time.time(),
            "detections": detections,
        }
        payload_json = json.dumps(payload)

        msg = String()
        msg.data = payload_json
        self.pub.publish(msg)

        if self.log_file:
            self.log_file.write(payload_json + "\n")
            self.log_file.flush()

        if self.args.show:
            annotated = r.plot()
            cv2.imshow("YOLO-World", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        self.frame_id += 1

    def destroy(self):
        if self.backend == "picamera2":
            self.picam2.stop()
        else:
            self.cap.release()
        if self.args.show:
            cv2.destroyAllWindows()
        if self.log_file:
            self.log_file.close()
        self.destroy_node()


def parse_args():
    p = argparse.ArgumentParser(description="YOLO-World webcam detector -> ROS2 /yolo publisher")
    p.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"YOLO-World weights path (auto-downloads on first run, cached after; "
                         f"default {DEFAULT_MODEL})")
    p.add_argument("--camera", type=int, default=0, help="camera index (Picamera2 camera_num, or OpenCV index)")
    p.add_argument("--backend", choices=["auto", "picamera2", "cv2"], default="auto",
                    help="capture backend: auto picks Picamera2 if installed (RPi CSI cameras "
                         "need it), else falls back to OpenCV (USB webcams)")
    p.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    p.add_argument("--topic", default="/yolo", help="ROS2 topic name to publish on")
    p.add_argument("--rate", type=float, default=30.0, help="max publish rate in Hz (0 = as fast as possible)")
    p.add_argument("--show", action="store_true", help="show annotated video window")
    p.add_argument("--log", default=None, help="optional path to append JSON lines to, e.g. detections.jsonl")
    p.add_argument("--dash-url", dest="dash_url", default="http://127.0.0.1:8080/api/camera_frame",
                    help="dashboard camera-frame endpoint (loopback JPEG POST, not a ROS topic); "
                         "empty string disables the preview push")
    p.add_argument("--dash-fps", dest="dash_fps", type=float, default=5.0,
                    help="how many JPEG frames per second to push to the dashboard preview "
                         "(default 5 Hz, independent of --rate; lower = less CPU)")
    # argparse will choke on ROS2's own --ros-args passthrough if used; strip those first.
    return p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])


def main():
    rclpy.init()
    args = parse_args()
    node = YoloWorldPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
