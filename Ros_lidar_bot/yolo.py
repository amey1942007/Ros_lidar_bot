#!/usr/bin/env python3
"""\nyolo.py — YOLO-World webcam object detector.

================================================================================
UNDERLYING SYSTEM & DATA FLOW
================================================================================
This node handles open-vocabulary object detection using Ultralytics YOLO-World.
- Subscribes to: Direct OpenCV USB webcam stream capture (e.g. index 0)
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
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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


def load_model(model_path, classes):
    from ultralytics import YOLO

    model = YOLO(model_path)    # e.g. "yolov8s-world.pt" (auto-downloads)
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

        self.cap = cv2.VideoCapture(args.camera)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera index {args.camera}")
            sys.exit(1)

        self.log_file = open(args.log, "a") if args.log else None
        self.frame_id = 0

        # Timer-driven loop so rclpy stays responsive (Ctrl+C, ros2 node info, etc.)
        period = 1.0 / args.rate if args.rate > 0 else 0.0
        self.timer = self.create_timer(period, self.on_timer)

    def on_timer(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to grab frame, retrying...")
            return

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
        self.cap.release()
        if self.args.show:
            cv2.destroyAllWindows()
        if self.log_file:
            self.log_file.close()
        self.destroy_node()


def parse_args():
    p = argparse.ArgumentParser(description="YOLO-World webcam detector -> ROS2 /yolo publisher")
    p.add_argument("--model", default="yolov8s-world.pt", help="YOLO-World weights (auto-downloads if not present)")
    p.add_argument("--camera", type=int, default=0, help="webcam index")
    p.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    p.add_argument("--topic", default="/yolo", help="ROS2 topic name to publish on")
    p.add_argument("--rate", type=float, default=30.0, help="max publish rate in Hz (0 = as fast as possible)")
    p.add_argument("--show", action="store_true", help="show annotated video window")
    p.add_argument("--log", default=None, help="optional path to append JSON lines to, e.g. detections.jsonl")
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
