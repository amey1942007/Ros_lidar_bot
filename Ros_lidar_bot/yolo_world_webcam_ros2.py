#!/usr/bin/env python3
"""
YOLO-World webcam object detector — ROS2 Humble (rclpy) version.
Publishes bounding boxes on the topic /yolo.

Each detection is encoded as centre-x, centre-y, width, height in pixels:
    x, y  = pixel coordinates of the CENTRE of the box
    w     = width of the box  (pixels)
    h     = height of the box (pixels)

All detections found in a single frame are published as a JSON array
in a std_msgs/msg/String on /yolo, e.g.:

[
  {"class": "person", "conf": 0.87, "x": 160.0, "y": 150.0, "w": 80.0, "h": 210.0},
  {"class": "bottle", "conf": 0.63, "x": 320.0, "y": 255.0, "w": 40.0, "h": 90.0}
]

Format matches semantic_slam_node.py expectations (centre-x/y, not top-left).
"""

import argparse
import json
import sys

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
            cx = x1 + w / 2.0   # centre x
            cy = y1 + h / 2.0   # centre y
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])

            detections.append({
                "class": cls_name,
                "conf": round(conf, 3),
                "x": round(cx, 1),
                "y": round(cy, 1),
                "w": round(w, 1),
                "h": round(h, 1),
            })

        payload_json = json.dumps(detections)

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
