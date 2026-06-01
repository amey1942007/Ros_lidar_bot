#!/usr/bin/env python3
"""
Semantic SLAM node — detects objects from the camera feed using YOLO,
localises them in the map frame using the LiDAR depth, and publishes
named 3D markers that overlay the SLAM map in RViz2.

Full pipeline (see docs/semantic_positioning_math.md for the math):
  /camera/image_raw  →  YOLO (best.pt, runs at ~2-5 Hz)
  /scan              →  LiDAR depth at the object's horizontal angle
  TF map→base_link   →  robot's current position and heading
  ──────────────────────────────────────────────────────────────────
  object position in map = rotate(depth × direction, robot_yaw) + robot_pos
  ──────────────────────────────────────────────────────────────────
  /semantic_markers  →  RViz2 MarkerArray (coloured boxes + labels)
  semantic_map.json  →  persistent JSON file saved to /tmp
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
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ── Camera constants (from camera.xacro) ─────────────────────────────────────
IMG_W = 640
IMG_H = 480
FOV_H = 1.089          # 62° in radians
FOCAL_LEN = IMG_W / (2 * math.tan(FOV_H / 2))   # ≈ 539 pixels

# ── Colour palette — one colour per class, generated from class name hash ────
def _class_colour(name: str) -> ColorRGBA:
    """Deterministic colour for a class name."""
    h = abs(hash(name)) % 360
    # HSV → RGB
    h_norm = h / 60.0
    i = int(h_norm)
    f = h_norm - i
    q = 1 - f
    table = [(1,f,0),(q,1,0),(0,1,f),(0,q,1),(f,0,1),(1,0,q)]
    r, g, b = table[i % 6]
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), 0.75
    return c


@dataclass
class SemanticObject:
    label: str
    x: float          # map frame metres
    y: float
    width: float      # estimated physical width (metres)
    depth_size: float # estimated physical depth
    confidence: float
    sightings: int = 1
    colour: ColorRGBA = field(default_factory=ColorRGBA)
    last_seen: float = field(default_factory=time.monotonic)


class SemanticSLAM(Node):

    # ── Hyperparameters ───────────────────────────────────────────────────────
    DETECT_HZ        = 3.0    # how often to run YOLO (Hz)
    CONF_THRESHOLD   = 0.45   # minimum YOLO confidence to accept
    MERGE_RADIUS     = 0.60   # merge detections within this distance (m)
    MIN_SIGHTINGS    = 3      # don't publish until seen this many times
    MIN_DEPTH        = 0.35   # ignore LiDAR readings closer than this (m)
    MAX_DEPTH        = 8.0    # ignore LiDAR readings farther than this (m)
    SAVE_PATH        = "/tmp/semantic_map.json"

    def __init__(self) -> None:
        super().__init__("semantic_slam")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("detect_hz",   self.DETECT_HZ)
        self.declare_parameter("conf",        self.CONF_THRESHOLD)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            # Default: look in the package share directory
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory("Ros_lidar_bot")
            model_path = os.path.join(pkg, "Vision Model", "best.pt")

        detect_hz       = float(self.get_parameter("detect_hz").value)
        self.CONF_THRESHOLD = float(self.get_parameter("conf").value)

        # ── Load YOLO model ───────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.class_names: Dict[int, str] = self.model.names
            self.get_logger().info(
                f"Model loaded — {len(self.class_names)} classes: "
                f"{list(self.class_names.values())}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None
            self.class_names = {}

        # ── State ─────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_scan:  Optional[LaserScan]  = None
        self.objects:      List[SemanticObject] = []
        self._marker_id = 0

        # Load previously saved semantic map
        self._load_map()

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(Image,     "/camera/image_raw", self._image_cb, sensor_qos)
        self.create_subscription(LaserScan, "/scan",             self._scan_cb,  sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)
        # Debug image: annotated frame with bounding boxes — view in rqt_image_view
        # or RViz2 → Add → By Topic → /semantic_debug → Image
        self.debug_pub  = self.create_publisher(Image, "/semantic_debug", 1)

        # ── Detection timer ───────────────────────────────────────────────────
        self.create_timer(1.0 / detect_hz, self._detect)

        self.get_logger().info("Semantic SLAM node ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    # ── Main detection tick ───────────────────────────────────────────────────

    def _detect(self) -> None:
        if self.latest_image is None:
            return

        # Always publish the raw camera feed so the user can verify the camera
        # works even before the model finishes loading.
        if self.model is None or self.latest_scan is None:
            self._publish_debug(self.latest_image.copy())
            if self.model is None:
                self.get_logger().warn(
                    "YOLO model not loaded — check ultralytics is installed: "
                    "pip3 install ultralytics --user",
                    throttle_duration_sec=10.0,
                )
            return

        # Get robot pose BEFORE running YOLO (saves time, robot barely moves in 0.3 s)
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return
        robot_x, robot_y, robot_yaw = robot_pose

        # Run YOLO
        try:
            results = self.model.predict(
                self.latest_image,
                conf=self.CONF_THRESHOLD,
                verbose=False,
                stream=False,
            )
        except Exception as e:
            self.get_logger().warn(f"YOLO inference failed: {e}", throttle_duration_sec=5.0)
            return

        # ── Draw debug frame (always, even if no detections) ─────────────────
        debug_frame = self.latest_image.copy()

        if not results or results[0].boxes is None:
            self._publish_debug(debug_frame)
            return

        scan = self.latest_scan

        for box in results[0].boxes:
            cls_id    = int(box.cls[0])
            conf      = float(box.conf[0])
            label     = self.class_names.get(cls_id, f"class_{cls_id}")
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # ── Step 1: horizontal angle from bounding box centre ─────────────
            cx_px = (x1 + x2) / 2.0
            # angle from camera forward axis (+ = left, - = right)
            angle = math.atan2(cx_px - IMG_W / 2, FOCAL_LEN)
            # Negate: camera X-right maps to robot -Y (right = negative in ROS)
            lidar_angle = -angle

            # ── Step 2: LiDAR depth at that angle ────────────────────────────
            depth = self._lidar_depth(scan, lidar_angle)
            if depth is None:
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

            # ── Step 5: estimate physical width from bbox pixel span ──────────
            pixel_width  = x2 - x1
            angle_width  = pixel_width / FOCAL_LEN   # radians
            phys_width   = max(0.1, depth * angle_width)
            phys_depth   = phys_width * 0.8          # rough square assumption

            self._update_objects(label, obj_x_map, obj_y_map,
                                 phys_width, phys_depth, conf)

            # Draw bounding box on debug frame
            colour_rgb = self.class_names.get(cls_id, "")
            c = _class_colour(label)
            bgr = (int(c.b * 255), int(c.g * 255), int(c.r * 255))
            cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), bgr, 2)
            cv2.putText(
                debug_frame,
                f"{label} {conf:.0%}  {depth:.1f}m",
                (int(x1), max(int(y1) - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2,
            )

        self._publish_debug(debug_frame)
        self._publish_markers()
        self._save_map()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _lidar_depth(self, scan: LaserScan, angle: float) -> Optional[float]:
        """Return LiDAR range at the given angle (radians), or None if invalid."""
        # Normalise angle to scan range
        while angle > math.pi:  angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi

        if not (scan.angle_min <= angle <= scan.angle_max):
            return None

        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(idx, len(scan.ranges) - 1))

        depth = scan.ranges[idx]
        if math.isnan(depth) or math.isinf(depth):
            return None
        if not (self.MIN_DEPTH <= depth <= self.MAX_DEPTH):
            return None
        return depth

    def _get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of the robot in the map frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except TransformException:
            return None
        x   = tf.transform.translation.x
        y   = tf.transform.translation.y
        qz  = tf.transform.rotation.z
        qw  = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        return x, y, yaw

    def _update_objects(self, label: str, mx: float, my: float,
                        width: float, depth_size: float, conf: float) -> None:
        """Merge new detection into the known-objects list."""
        for obj in self.objects:
            if obj.label != label:
                continue
            dist = math.hypot(obj.x - mx, obj.y - my)
            if dist < self.MERGE_RADIUS:
                # Weighted average — more sightings → slower update
                w = 1.0 / (obj.sightings + 1)
                obj.x         = obj.x * (1 - w) + mx * w
                obj.y         = obj.y * (1 - w) + my * w
                obj.width     = obj.width     * (1 - w) + width     * w
                obj.depth_size= obj.depth_size* (1 - w) + depth_size* w
                obj.confidence= max(obj.confidence, conf)
                obj.sightings += 1
                obj.last_seen = time.monotonic()
                return

        # New object
        colour = _class_colour(label)
        self.objects.append(SemanticObject(
            label=label, x=mx, y=my,
            width=width, depth_size=depth_size,
            confidence=conf, colour=colour,
        ))

    def _publish_markers(self) -> None:
        """Publish a MarkerArray: box + text label for every confirmed object."""
        array = MarkerArray()
        mid = 0

        # First: delete all old markers
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        del_m.header.frame_id = "map"
        del_m.header.stamp = self.get_clock().now().to_msg()
        array.markers.append(del_m)

        for obj in self.objects:
            if obj.sightings < self.MIN_SIGHTINGS:
                continue   # not confirmed yet

            stamp = self.get_clock().now().to_msg()

            # ── Box marker ────────────────────────────────────────────────────
            box = Marker()
            box.header.frame_id = "map"
            box.header.stamp    = stamp
            box.ns              = "semantic_boxes"
            box.id              = mid; mid += 1
            box.type            = Marker.CUBE
            box.action          = Marker.ADD
            box.pose.position   = Point(x=obj.x, y=obj.y, z=0.5)
            box.scale.x         = max(0.15, obj.width)
            box.scale.y         = max(0.15, obj.depth_size)
            box.scale.z         = 1.0
            box.color           = obj.colour
            box.color.a         = 0.35
            box.lifetime.sec    = 3     # disappear if not refreshed
            array.markers.append(box)

            # ── Text label ────────────────────────────────────────────────────
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp    = stamp
            txt.ns              = "semantic_labels"
            txt.id              = mid; mid += 1
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position   = Point(x=obj.x, y=obj.y, z=1.6)
            txt.scale.z         = 0.22
            txt.text            = f"{obj.label} ({obj.confidence:.0%})"
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a         = 1.0
            txt.lifetime.sec    = 3
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
                obj = SemanticObject(
                    label=d["label"], x=d["x"], y=d["y"],
                    width=d["width"], depth_size=d["depth_size"],
                    confidence=d["confidence"], sightings=d["sightings"],
                    colour=_class_colour(d["label"]),
                )
                self.objects.append(obj)
            self.get_logger().info(
                f"Loaded {len(self.objects)} objects from {self.SAVE_PATH}"
            )
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Could not load saved map: {e}")


    def _publish_debug(self, frame: np.ndarray) -> None:
        """Publish annotated camera frame to /semantic_debug.

        View with:  ros2 run rqt_image_view rqt_image_view  → select /semantic_debug
        Or in RViz2: Add → By Topic → /semantic_debug → Image
        """
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link_optical"
            self.debug_pub.publish(msg)
        except Exception:
            pass


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
