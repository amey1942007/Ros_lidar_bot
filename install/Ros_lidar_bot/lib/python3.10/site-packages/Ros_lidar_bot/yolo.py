#!/usr/bin/env python3
"""
YOLO-World webcam object detector — ROS2 Humble (rclpy) version.
Publishes bounding boxes on the topic /yolo.

Each detection is encoded as [x, y, w, h] where:
    x, y  = pixel coordinates of the TOP-LEFT corner of the box
    w     = width of the box  (pixels)
    h     = height of the box (pixels)

All detections found in a single frame are packed into one JSON object
and published as a std_msgs/msg/String on /yolo, e.g.:

{
  "frame_id": 128,
  "timestamp": 1731000000.123,
  "detections": [
    {"class": "person", "confidence": 0.87, "bbox": [120, 45, 80, 210]},
    {"class": "bottle", "confidence": 0.63, "bbox": [300, 210, 40, 90]}
  ]
}

Why JSON in a String msg instead of XML or a custom .msg:
  - keeps [x, y, w, h] as a plain 4-element array per your request,
  - lets multiple detections + class/confidence ride along together,
  - std_msgs/String is available out of the box — no need to build a
    custom message package / colcon build just to get this running.
  (If you want a strongly-typed message instead, e.g. an array of
  float32[4] per detection with no JSON parsing on the subscriber
  side, say so and I'll set up a custom .msg + package for it.)

Requirements:
    pip install ultralytics opencv-python
    sudo apt install ros-humble-rclpy ros-humble-std-msgs   # if not already present
    source /opt/ros/humble/setup.bash

Run as a plain script:
    python3 yolo_world_webcam_ros2.py --show
    python3 yolo_world_webcam_ros2.py --camera 0 --conf 0.25

Or drop into a ROS2 package's console_scripts entry point and:
    ros2 run <your_package> yolo_world_webcam_ros2

Subscribe / inspect from another terminal:
    ros2 topic echo /yolo
"""

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
