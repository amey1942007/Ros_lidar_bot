#!/usr/bin/env python3
"""
Semantic SLAM node — receives YOLO World detections from /yolo topic,
localises each detected object in the map frame using LiDAR depth + TF,
and publishes named 3-D markers to /semantic_markers.

Pipeline:
  /yolo       (std_msgs/String — JSON array of detections from OpenCV+YOLO World node)
  /scan       (sensor_msgs/LaserScan — LiDAR depth for range estimation)
  TF map→base_link — robot position and heading
  ─────────────────────────────────────────────────────────────
  object (x, y) in map frame = rotate(depth × direction, robot_yaw) + robot_pos
  ─────────────────────────────────────────────────────────────
  /semantic_markers  → RViz2 MarkerArray (coloured boxes + labels)
  /semantic_debug    → annotated image republished for rqt_image_view
  semantic_map.json  → persistent JSON file saved to /tmp

Expected /yolo message format (std_msgs/String, UTF-8 JSON):
  [
    {"class": "chair", "x": 320, "y": 240, "w": 100, "h": 150, "conf": 0.82},
    {"class": "person", "x": 120, "y": 200, "w": 60,  "h": 180, "conf": 0.91},
    ...
  ]
  x, y  — bounding-box centre in pixels (from a 640×480 frame)
  w, h  — bounding-box width / height in pixels
  class — object class name (string)
  conf  — detection confidence 0..1 (optional, defaults to 1.0 if absent)

Camera constants must match the publisher script / physical camera:
  IMG_W, IMG_H, FOV_H  (edit below if your RPi Cam 3 field-of-view differs)
"""

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ── Camera constants ──────────────────────────────────────────────────────────
# RPi Camera Module 3 WIDE: 102° horizontal FOV, 2304x1296 native (streamed at 640x480)
# (non-Wide Module 3 is 66deg -- change this back if the camera is ever swapped)
IMG_W   = 640
IMG_H   = 480
FOV_H   = math.radians(102)                         # horizontal FOV in radians
FOCAL_LEN = IMG_W / (2 * math.tan(FOV_H / 2))       # ≈ 514 px


# ── Colour palette — deterministic colour per class name ─────────────────────
def _class_colour(name: str) -> ColorRGBA:
    h = abs(hash(name)) % 360
    h_norm = h / 60.0
    i = int(h_norm)
    f = h_norm - i
    q = 1 - f
    table = [(1, f, 0), (q, 1, 0), (0, 1, f), (0, q, 1), (f, 0, 1), (1, 0, q)]
    r, g, b = table[i % 6]
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), 0.75
    return c


@dataclass
class SemanticObject:
    label:      str
    x:          float        # map frame metres
    y:          float
    width:      float        # estimated physical width (m)
    depth_size: float        # estimated physical depth (m)
    confidence: float
    sightings:  int          = 1
    colour:     ColorRGBA   = field(default_factory=ColorRGBA)
    last_seen:  float        = field(default_factory=time.monotonic)


class SemanticSLAM(Node):

    # ── Hyperparameters ───────────────────────────────────────────────────────
    DETECT_HZ      = 5.0    # max rate to process /yolo messages (Hz)
    CONF_THRESHOLD = 0.25   # minimum detection confidence to accept
    MERGE_RADIUS   = 0.60   # merge two detections within this distance (m)
    MIN_SIGHTINGS  = 2      # publish after this many successful depth fixes
    MIN_DEPTH      = 0.20   # ignore LiDAR readings closer than this (m)
    MAX_DEPTH      = 8.0    # ignore LiDAR readings farther than this (m)
    SAVE_PATH      = "/tmp/semantic_map.json"
    DEPTH_RAY_SPAN = 3      # sample ±N lidar beams around the bearing (median)
    # Must match description/lidar.xacro laser_yaw. Slamtec 0° is cable-opposite;
    # with yaw=π, LaserScan angle 0 is robot REAR — convert base→scan below.
    LASER_YAW      = math.pi

    def __init__(self) -> None:
        super().__init__("semantic_slam")

        # ── ROS parameters ────────────────────────────────────────────────────
        self.declare_parameter("conf",      self.CONF_THRESHOLD)
        self.declare_parameter("max_depth", self.MAX_DEPTH)
        self.declare_parameter("laser_yaw", self.LASER_YAW)

        self.CONF_THRESHOLD = float(self.get_parameter("conf").value)
        self.MAX_DEPTH      = float(self.get_parameter("max_depth").value)
        self.LASER_YAW      = float(self.get_parameter("laser_yaw").value)

        # ── State ─────────────────────────────────────────────────────────────
        # Latest raw detections from /yolo (list of dicts)
        self._latest_detections: List[Dict] = []
        self.latest_scan:        Optional[LaserScan] = None
        self.objects:            List[SemanticObject] = []
        self._last_process_time: float = 0.0

        # Load previously saved semantic map
        self._load_map()

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        # /yolo — JSON detections from the OpenCV + YOLO World publisher node
        self.create_subscription(String,    "/yolo",  self._yolo_cb,  10)
        # /scan — LiDAR scan for object depth estimation
        self.create_subscription(LaserScan, "/scan",  self._scan_cb,  sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)

        # ── Processing timers ─────────────────────────────────────────────────
        self.create_timer(1.0 / self.DETECT_HZ, self._process)
        # Markers use a short lifetime — republish even when YOLO is quiet so
        # the dashboard/RViz don't go blank, and newly confirmed objects appear.
        self.create_timer(0.5, self._publish_markers)

        self.get_logger().info(
            "Semantic SLAM node ready.\n"
            "  Subscribing: /yolo (std_msgs/String JSON), /scan\n"
            "  Publishing:  /semantic_markers\n"
            f"  Merge radius: {self.MERGE_RADIUS} m | "
            f"Min sightings: {self.MIN_SIGHTINGS} | "
            f"Depth range: {self.MIN_DEPTH}–{self.MAX_DEPTH} m"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _yolo_cb(self, msg: String) -> None:
        """Parse the JSON detection array from the YOLO World publisher."""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and "detections" in data:
                self._latest_detections = data["detections"]
            elif isinstance(data, list):
                self._latest_detections = data
            else:
                self.get_logger().warn(
                    "Received /yolo message that is not in a recognized format — ignoring.",
                    throttle_duration_sec=5.0,
                )
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f"Failed to parse /yolo JSON: {e}",
                throttle_duration_sec=5.0,
            )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ── Main processing tick ──────────────────────────────────────────────────

    def _process(self) -> None:
        """Run on timer: localise latest detections in map frame."""
        if not self._latest_detections:
            return

        if self.latest_scan is None:
            self.get_logger().warn(
                "Waiting for /scan data — check LiDAR topic.",
                throttle_duration_sec=10.0,
            )
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn(
                "Cannot get robot pose from TF (map → base_footprint/base_link) — is SLAM running?",
                throttle_duration_sec=10.0,
            )
            return

        robot_x, robot_y, robot_yaw = robot_pose
        scan = self.latest_scan
        # Consume the latest batch — avoid processing the same frame twice
        detections = self._latest_detections
        self._latest_detections = []

        placed = 0
        skipped_depth = 0
        for det in detections:
            # ── Extract detection fields ──────────────────────────────────────
            label = str(det.get("class", "unknown"))
            conf  = float(det.get("confidence") if det.get("confidence") is not None else det.get("conf", 1.0))
            
            if "bbox" in det:
                x1, y1, w_px, h_px = det["bbox"]
                cx_px = x1 + w_px / 2.0
            else:
                cx_px = float(det.get("x", IMG_W / 2))   # bbox centre x in pixels
                w_px  = float(det.get("w", 10))
                h_px  = float(det.get("h", 10))

            if conf < self.CONF_THRESHOLD:
                continue

            # ── Step 1: horizontal angle from bbox centre ─────────────────────
            # angle from camera forward axis (positive = right in image = robot -Y)
            angle = math.atan2(cx_px - IMG_W / 2, FOCAL_LEN)
            lidar_angle = -angle   # camera X-right → robot -Y

            # ── Step 2: LiDAR depth at that angle ────────────────────────────
            # Scan is in laser_frame (yawed vs base_link) — convert bearing.
            depth = self._lidar_depth(scan, lidar_angle - self.LASER_YAW)
            if depth is None:
                skipped_depth += 1
                continue

            # ── Step 3: object position in base_link frame ────────────────────
            obj_x_base = depth * math.cos(lidar_angle)
            obj_y_base = depth * math.sin(lidar_angle)

            # ── Step 4: rotate to map frame ───────────────────────────────────
            obj_x_map = (robot_x
                         + obj_x_base * math.cos(robot_yaw)
                         - obj_y_base * math.sin(robot_yaw))
            obj_y_map = (robot_y
                         + obj_x_base * math.sin(robot_yaw)
                         + obj_y_base * math.cos(robot_yaw))

            # ── Step 5: estimate physical size from pixel bbox ────────────────
            phys_width = max(0.1, depth * (w_px / FOCAL_LEN))
            phys_depth = max(0.1, depth * (h_px / FOCAL_LEN))

            self._update_objects(label, obj_x_map, obj_y_map,
                                 phys_width, phys_depth, conf)
            placed += 1

        if detections and placed == 0 and skipped_depth:
            self.get_logger().warn(
                f"Got {len(detections)} YOLO dets but no LiDAR depth "
                f"({skipped_depth} rays empty/OOR) — no new map positions.",
                throttle_duration_sec=8.0,
            )

        self._save_map()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _lidar_depth(self, scan: LaserScan, angle: float) -> Optional[float]:
        """Median LiDAR range near the given bearing (radians), or None."""
        while angle > math.pi:  angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi

        if not (scan.angle_min <= angle <= scan.angle_max):
            return None

        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(idx, len(scan.ranges) - 1))
        span = self.DEPTH_RAY_SPAN
        samples = []
        for i in range(idx - span, idx + span + 1):
            if i < 0 or i >= len(scan.ranges):
                continue
            depth = scan.ranges[i]
            if math.isnan(depth) or math.isinf(depth):
                continue
            if self.MIN_DEPTH <= depth <= self.MAX_DEPTH:
                samples.append(depth)
        if not samples:
            return None
        samples.sort()
        return samples[len(samples) // 2]

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of the robot in the map frame."""
        for frame in ("base_footprint", "base_link"):
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time()
                )
            except TransformException:
                continue
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            return x, y, yaw
        return None

    def _update_objects(self, label: str, mx: float, my: float,
                        width: float, depth_size: float, conf: float) -> None:
        """Merge new detection into the known-objects list."""
        for obj in self.objects:
            if obj.label != label:
                continue
            dist = math.hypot(obj.x - mx, obj.y - my)
            if dist < self.MERGE_RADIUS:
                w          = 1.0 / (obj.sightings + 1)
                obj.x      = obj.x * (1 - w) + mx * w
                obj.y      = obj.y * (1 - w) + my * w
                obj.width  = obj.width      * (1 - w) + width      * w
                obj.depth_size = obj.depth_size * (1 - w) + depth_size * w
                obj.confidence = max(obj.confidence, conf)
                obj.sightings += 1
                obj.last_seen  = time.monotonic()
                return

        # New object
        self.objects.append(SemanticObject(
            label=label, x=mx, y=my,
            width=width, depth_size=depth_size,
            confidence=conf, colour=_class_colour(label),
        ))

    def _publish_markers(self) -> None:
        """Publish a MarkerArray: box + text label for every confirmed object."""
        array = MarkerArray()
        mid   = 0

        # Delete all old markers first
        del_m              = Marker()
        del_m.action       = Marker.DELETEALL
        del_m.header.frame_id = "map"
        del_m.header.stamp = self.get_clock().now().to_msg()
        array.markers.append(del_m)

        for obj in self.objects:
            if obj.sightings < self.MIN_SIGHTINGS:
                continue

            stamp = self.get_clock().now().to_msg()

            # Box marker
            box                   = Marker()
            box.header.frame_id   = "map"
            box.header.stamp      = stamp
            box.ns                = "semantic_boxes"
            box.id                = mid; mid += 1
            box.type              = Marker.CUBE
            box.action            = Marker.ADD
            box.pose.position     = Point(x=obj.x, y=obj.y, z=0.5)
            box.scale.x           = max(0.15, obj.width)
            box.scale.y           = max(0.15, obj.depth_size)
            box.scale.z           = 1.0
            box.color             = obj.colour
            box.color.a           = 0.35
            box.lifetime.sec      = 5
            array.markers.append(box)

            # Text label
            txt                   = Marker()
            txt.header.frame_id   = "map"
            txt.header.stamp      = stamp
            txt.ns                = "semantic_labels"
            txt.id                = mid; mid += 1
            txt.type              = Marker.TEXT_VIEW_FACING
            txt.action            = Marker.ADD
            txt.pose.position     = Point(x=obj.x, y=obj.y, z=1.6)
            txt.scale.z           = 0.22
            txt.text              = f"{obj.label} ({obj.confidence:.0%})"
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a           = 1.0
            txt.lifetime.sec      = 5
            array.markers.append(txt)

        self.marker_pub.publish(array)

    # ── Persistence ───────────────────────────────────────────────────────────

    def _save_map(self) -> None:
        data = [
            {
                "label":      o.label,
                "x":          o.x,
                "y":          o.y,
                "width":      o.width,
                "depth_size": o.depth_size,
                "confidence": o.confidence,
                "sightings":  o.sightings,
            }
            for o in self.objects if o.sightings >= self.MIN_SIGHTINGS
        ]
        try:
            with open(self.SAVE_PATH, "w") as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

    def _load_map(self) -> None:
        try:
            with open(self.SAVE_PATH) as f:
                data = json.load(f)
            for d in data:
                self.objects.append(SemanticObject(
                    label=d["label"], x=d["x"], y=d["y"],
                    width=d["width"], depth_size=d["depth_size"],
                    confidence=d["confidence"], sightings=d["sightings"],
                    colour=_class_colour(d["label"]),
                ))
            self.get_logger().info(
                f"Loaded {len(self.objects)} objects from {self.SAVE_PATH}"
            )
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Could not load saved map: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticSLAM()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
