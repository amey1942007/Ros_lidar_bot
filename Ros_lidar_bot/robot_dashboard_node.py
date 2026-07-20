#!/usr/bin/env python3
"""
robot_dashboard_node.py — Browser-based robot dashboard served from the RPi.

Replaces the terminal bringup_status board with an interactive web GUI.
The Pi runs headless over SSH, so instead of opening a window there (X11
forwarding is laggy and fragile), this node embeds a small HTTP server and
the laptop/phone simply opens  http://<pi-ip>:8080  in a browser.

Features
  • Live 2D visualization: animated robot model (odom pose), lidar scan
    overlay, live SLAM occupancy map, semantic object labels, motion trail,
    pan-free follow camera, mouse-wheel zoom.
  • Status board: expected-node health, topic rates/ages, TF chain state.
  • Tool launcher — NOTHING runs at bringup; each starts only on click:
      - IMU test        (imu_test_node)
      - IMU calibration (imu_calibration_node — robot rotates in place!)
      - Drive distance  (drive_distance — exact-distance test moves)
    Tool stdout streams into the console panel; one tool at a time; Stop
    kills the tool's whole process group.
  • Semantic vision toggle: starts/stops vision.launch.py (YOLO-World camera
    node + semantic SLAM) in its own slot, independent of the tools above.
    E-STOP leaves it running — perception is passive, it moves nothing.
  • Nav goal mode: click the canvas to send a NavigateToPose goal (canvas
    point is transformed odom→map via TF); result/feedback logged.
  • Nudge pad (short cmd_vel pulses) and a big E-STOP that kills the tool,
    cancels the nav goal, and streams zero cmd_vel.

Transport: Server-Sent Events for Pi→browser data (10 Hz state + log
lines), plain POST for browser→Pi commands. Standard library only — no
websocket dependency, no CDN, works offline.
"""

import base64
import json
import math
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

try:
    from visualization_msgs.msg import MarkerArray
    _MARKERS_OK = True
except ImportError:
    _MARKERS_OK = False

try:
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    _SLAM_SRV_OK = True
except ImportError:
    _SLAM_SRV_OK = False

try:
    from nav2_msgs.action import NavigateToPose
    _NAV2_OK = True
except ImportError:
    _NAV2_OK = False

try:
    import tf2_ros
    _TF2_OK = True
except ImportError:
    _TF2_OK = False


# ── Tool registry ─────────────────────────────────────────────────────────────
# Each entry: builds the ros2 run command from user params (validated/clamped
# here — the browser is not trusted).

def _f(params, key, default, lo, hi):
    try:
        v = float(params.get(key, default))
    except (TypeError, ValueError):
        v = default
    return max(lo, min(hi, v))


def _tool_imu_test(params):
    return ["ros2", "run", "Ros_lidar_bot", "imu_test_node"]


def _tool_imu_calibration(params):
    speed = _f(params, "speed", 0.5, 0.1, 1.0)
    rotations = _f(params, "rotations", 2.0, 0.5, 4.0)
    return [
        "ros2", "run", "Ros_lidar_bot", "imu_calibration_node", "--ros-args",
        # Publish on /cmd_vel so the command passes through safety_stop like
        # every other source (spin-in-place is angular-only, never blocked).
        "-p", "cmd_vel_topic:=/cmd_vel",
        "-p", f"rotation_speed:={speed}",
        "-p", f"rotation_count:={rotations}",
    ]


def _tool_drive_distance(params):
    dist = _f(params, "distance", 1.0, -5.0, 5.0)
    vel = _f(params, "max_vel", 0.3, 0.05, 0.5)
    return [
        "ros2", "run", "Ros_lidar_bot", "drive_distance", "--ros-args",
        "-p", f"distance:={dist}",
        "-p", f"max_vel:={vel}",
    ]


TOOLS = {
    "imu_test": ("IMU test", _tool_imu_test),
    "imu_calibration": ("IMU calibration", _tool_imu_calibration),
    "drive_distance": ("Drive distance", _tool_drive_distance),
}

EXPECTED_NODES = [
    "robot_state_publisher", "imu_node", "driver_node", "odom_node",
    "rplidar_node", "scan_min_range_filter", "ekf_filter_node", "slam_toolbox", "joy_node",
    "joy_teleop", "bt_navigator", "controller_server", "planner_server",
    "behavior_server", "velocity_smoother", "collision_monitor",
]


# ── SSE broadcast hub ─────────────────────────────────────────────────────────

class Hub:
    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()

    def attach(self):
        q = queue.Queue(maxsize=200)
        with self._lock:
            self._clients.append(q)
        return q

    def detach(self, q):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def send(self, event: dict):
        self.send_raw(json.dumps(event, separators=(",", ":")))

    def send_raw(self, data: str):
        with self._lock:
            clients = list(self._clients)
        for q in clients:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass  # slow client — it will drop frames, not block us


# ── ROS node ──────────────────────────────────────────────────────────────────

class Dashboard(Node):
    def __init__(self):
        super().__init__("robot_dashboard")
        self.port = int(self.declare_parameter("port", 8080).value)
        self.expect_frontier = bool(
            self.declare_parameter("expect_frontier", False).value)
        # laser_frame is mounted 180° rotated (laser_yaw=pi in lidar.xacro);
        # applied here so scan points render correctly around the model.
        self.laser_yaw = float(self.declare_parameter("laser_yaw", math.pi).value)

        self.hub = Hub()
        self._lock = threading.Lock()
        self._state = {
            "pose": [0.0, 0.0, 0.0], "vel": [0.0, 0.0],
            "imu": {"gz": 0.0, "ax": 0.0, "ay": 0.0}, "scan": [],
            "rates": {}, "nodes": {}, "tf": {}, "tool": None,
            "cmd": [0.0, 0.0], "map_pose": None, "nav": None,
            "yolo_det": [], "sem": [],
            "frontier": {"zones": [], "queue": []},
            "path": [], "goal": None, "sys": {},
        }
        self._last_msg_time = {}
        self._msg_counts = {}

        self._tool_proc = None
        self._tool_name = None
        self._vision_proc = None
        self._map_saves_pending = 0   # >0 while save_map service calls in flight
        self._latest_map = None
        self._map_dirty = False
        self.last_map_data = None   # cached SSE map event for new clients
        self._nudge_until = 0.0
        self._nudge_cmd = (0.0, 0.0)
        self._estop_until = 0.0
        self._nav_goal_handle = None
        # Bumped on every new goal / cancel so stale accept/result callbacks
        # cannot revive a superseded or cancelled NavigateToPose goal.
        self._nav_gen = 0

        # Camera preview: yolo.py (when vision is running) POSTs JPEG frames
        # to /api/camera_frame over loopback -- never a ROS topic/publisher,
        # so it costs nothing when nobody has the dashboard open. Browsers
        # pull it back out via the /camera.mjpg multipart stream below.
        self._cam_cond = threading.Condition()
        self._cam_frame = None
        self._cam_seq = 0

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Odometry, "/odom_raw", self._mk_rate_cb("/odom_raw"), 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Path, "/plan", self._plan_cb, 5)
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self.create_subscription(String, "/yolo", self._yolo_cb, 10)
        if _MARKERS_OK:
            self.create_subscription(MarkerArray, "/semantic_markers", self._sem_cb, 10)
            self.create_subscription(MarkerArray, "/frontier_debug", self._frontier_cb, 10)

        # manual no-go zones → frontier_explorer (map frame, JSON)
        self._bl_pub = self.create_publisher(String, "/manual_blacklist", 10)

        self._save_map_cli = self._serialize_cli = None
        if _SLAM_SRV_OK:
            self._save_map_cli = self.create_client(SaveMap, "/slam_toolbox/save_map")
            self._serialize_cli = self.create_client(
                SerializePoseGraph, "/slam_toolbox/serialize_map")

        self._prev_cpu = None
        self._vcgencmd_ok = True
        self.create_timer(5.0, self._sys_tick)

        self._tf_buffer = None
        if _TF2_OK:
            self._tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self._tf_buffer, self)

        self._nav_client = None
        if _NAV2_OK:
            self._nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_timer(0.1, self._tick)        # 10 Hz state → browser
        self.create_timer(2.0, self._slow_tick)   # node list, TF, rates
        self.create_timer(0.05, self._motion_tick)  # nudge / e-stop pulses

        self.log(f"Dashboard up — open http://{_local_ip()}:{self.port}")

    # ── Logging to the browser console panel ─────────────────────────────────
    def log(self, text, level="info"):
        self.hub.send({"t": "log", "level": level,
                       "time": time.strftime("%H:%M:%S"), "text": text})

    # ── Subscribers ──────────────────────────────────────────────────────────
    def _mark(self, topic):
        self._last_msg_time[topic] = time.monotonic()
        self._msg_counts[topic] = self._msg_counts.get(topic, 0) + 1

    def _mk_rate_cb(self, topic):
        def cb(_msg):
            self._mark(topic)
        return cb

    def _odom_cb(self, m):
        self._mark("/odom")
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._state["pose"] = [round(m.pose.pose.position.x, 3),
                                   round(m.pose.pose.position.y, 3),
                                   round(yaw, 4)]
            self._state["vel"] = [round(m.twist.twist.linear.x, 3),
                                  round(m.twist.twist.angular.z, 3)]

    def _imu_cb(self, m):
        self._mark("/imu")
        with self._lock:
            self._state["imu"] = {
                "gz": round(m.angular_velocity.z, 4),
                "ax": round(m.linear_acceleration.x, 3),
                "ay": round(m.linear_acceleration.y, 3),
            }

    def _cmd_cb(self, m):
        self._mark("/cmd_vel")
        with self._lock:
            self._state["cmd"] = [round(m.linear.x, 3), round(m.angular.z, 3)]

    def _map_cb(self, m):
        self._mark("/map")
        with self._lock:
            self._latest_map = m
            self._map_dirty = True

    def _yolo_cb(self, m):
        self._mark("/yolo")
        try:
            dets = json.loads(m.data).get("detections", [])
        except (json.JSONDecodeError, AttributeError):
            return
        summary = []
        for d in dets[:6]:
            conf = d.get("confidence", d.get("conf", 0.0)) or 0.0
            summary.append(f'{d.get("class", "?")} {round(100 * float(conf))}%')
        with self._lock:
            self._state["yolo_det"] = summary

    def _sem_cb(self, m):
        objs = [[round(mk.pose.position.x, 2), round(mk.pose.position.y, 2), mk.text]
                for mk in m.markers if mk.ns == "semantic_labels"][:40]
        with self._lock:
            self._state["sem"] = objs

    def _frontier_cb(self, m):
        zones, fqueue = [], []
        kind_by_ns = {"blacklist": "bl", "perm_dead": "perm", "manual_bl": "manual"}
        for mk in m.markers:
            k = kind_by_ns.get(mk.ns)
            if k is not None and mk.type == 3:                 # CYLINDER
                zones.append([round(mk.pose.position.x, 2),
                              round(mk.pose.position.y, 2),
                              round(mk.scale.x / 2.0, 2), k])
            elif mk.ns == "frontier_queue" and mk.type == 2:   # SPHERE
                fqueue.append([round(mk.pose.position.x, 2),
                               round(mk.pose.position.y, 2)])
        with self._lock:
            self._state["frontier"] = {"zones": zones[:40], "queue": fqueue[:8]}

    def _scan_cb(self, m):
        self._mark("/scan")
        pts = []
        n = len(m.ranges)
        step = max(1, n // 160)   # ≤160 points to the browser
        for i in range(0, n, step):
            r = m.ranges[i]
            if not math.isfinite(r) or r < m.range_min or r > m.range_max:
                continue
            a = m.angle_min + i * m.angle_increment + self.laser_yaw
            pts.append([round(a, 3), round(r, 2)])
        with self._lock:
            self._state["scan"] = pts

    # ── Periodic state → browser ─────────────────────────────────────────────
    def _tick(self):
        with self._lock:
            st = dict(self._state)
        st["t"] = "state"
        st["tool"] = self._tool_name
        st["vision"] = bool(self._vision_proc and self._vision_proc.poll() is None)
        st["saving"] = self._map_saves_pending > 0
        self.hub.send(st)

    def _slow_tick(self):
        now = time.monotonic()
        rates = {}
        for topic in ("/scan", "/odom", "/odom_raw", "/imu", "/map", "/cmd_vel", "/yolo"):
            cnt = self._msg_counts.pop(topic, 0)
            last = self._last_msg_time.get(topic)
            rates[topic] = {
                "hz": round(cnt / 2.0, 1),
                "age": round(now - last, 1) if last else None,
            }

        expected = EXPECTED_NODES + (["frontier_explorer"] if self.expect_frontier else [])
        try:
            alive = {name for name, _ns in self.get_node_names_and_namespaces()}
        except Exception:
            alive = set()
        nodes = {n: (n in alive) for n in expected}

        tf_state = {}
        map_pose = None
        if self._tf_buffer is not None:
            for pair in (("map", "odom"), ("odom", "base_footprint")):
                key = f"{pair[0]}→{pair[1]}"
                try:
                    tr = self._tf_buffer.lookup_transform(pair[0], pair[1], rclpy.time.Time())
                    age = (self.get_clock().now() - rclpy.time.Time.from_msg(tr.header.stamp)).nanoseconds * 1e-9
                    tf_state[key] = {"ok": True, "age": round(age, 2)}
                except Exception:
                    tf_state[key] = {"ok": False, "age": None}
            try:
                tr = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                q = tr.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                map_pose = [round(tr.transform.translation.x, 2),
                            round(tr.transform.translation.y, 2), round(yaw, 2)]
            except Exception:
                pass

        with self._lock:
            self._state["rates"] = rates
            self._state["nodes"] = nodes
            self._state["tf"] = tf_state
            self._state["map_pose"] = map_pose

        self._push_map()

    # ── Live map → browser ───────────────────────────────────────────────────
    MAP_MAX_CELLS = 220   # per-side cap after downsampling (browser payload)

    def _push_map(self):
        with self._lock:
            m = self._latest_map
            dirty = self._map_dirty
            self._map_dirty = False
        if m is None or not dirty:
            return

        # map→odom pose so the browser can place the grid in its odom-frame view.
        tf = [0.0, 0.0, 0.0]
        if self._tf_buffer is not None:
            try:
                tr = self._tf_buffer.lookup_transform("odom", "map", rclpy.time.Time())
                q = tr.transform.rotation
                tf = [tr.transform.translation.x, tr.transform.translation.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))]
            except Exception:
                pass  # frames coincide until SLAM publishes map→odom

        w, h = m.info.width, m.info.height
        f = max(1, (max(w, h) + self.MAP_MAX_CELLS - 1) // self.MAP_MAX_CELLS)
        data = m.data
        rows = []
        for r in range(0, h, f):
            base = r * w
            # 0 = unknown, 1 = free, 2 = occupied — 1 byte per downsampled cell
            rows.append(bytes(0 if v < 0 else (2 if v > 50 else 1)
                              for v in data[base:base + w:f]))
        if not rows:
            return
        packed = base64.b64encode(zlib.compress(b"".join(rows), 6)).decode()
        event = json.dumps({
            "t": "map", "w": len(rows[0]), "h": len(rows),
            "res": round(m.info.resolution * f, 4),
            "origin": [round(m.info.origin.position.x, 3),
                       round(m.info.origin.position.y, 3)],
            "tf": [round(tf[0], 3), round(tf[1], 3), round(tf[2], 4)],
            "data": packed,
        }, separators=(",", ":"))
        self.last_map_data = event   # replayed to newly connected clients
        self.hub.send_raw(event)

    # ── System vitals (CPU / RAM / temp / disk) ──────────────────────────────
    def _sys_tick(self):
        info = {}
        try:
            with open("/proc/stat") as f:
                vals = list(map(int, f.readline().split()[1:8]))
            idle, total = vals[3] + vals[4], sum(vals)
            if self._prev_cpu:
                dt = total - self._prev_cpu[1]
                if dt > 0:
                    info["cpu"] = round(100.0 * (1 - (idle - self._prev_cpu[0]) / dt), 1)
            self._prev_cpu = (idle, total)
        except (OSError, ValueError):
            pass
        try:
            info["load"] = round(os.getloadavg()[0], 2)
        except OSError:
            pass
        try:
            mi = {}
            with open("/proc/meminfo") as f:
                for ln in f:
                    k, v = ln.split(":", 1)
                    mi[k] = int(v.split()[0])
            info["mem"] = [round((mi["MemTotal"] - mi["MemAvailable"]) / 1048576, 2),
                           round(mi["MemTotal"] / 1048576, 2)]
        except (OSError, KeyError, ValueError):
            pass
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                info["temp"] = round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            pass
        try:
            du = shutil.disk_usage("/")
            info["disk"] = [round((du.total - du.free) / 2**30, 1),
                            round(du.total / 2**30, 1)]
        except OSError:
            pass
        try:
            with open("/proc/uptime") as f:
                up = int(float(f.read().split()[0]))
            info["up"] = f"{up // 3600}h {(up % 3600) // 60:02d}m"
        except (OSError, ValueError):
            pass
        if self._vcgencmd_ok:   # Pi firmware throttle flags; disabled after first failure
            try:
                out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                     text=True, timeout=1.0).stdout.strip()
                info["throttled"] = int(out.split("=")[1], 16)
            except (OSError, subprocess.TimeoutExpired, ValueError, IndexError):
                self._vcgencmd_ok = False
        with self._lock:
            self._state["sys"] = info

    # ── Save map (slam_toolbox services) ─────────────────────────────────────
    def save_map(self, name):
        if not _SLAM_SRV_OK:
            return {"ok": False, "error": "slam_toolbox srv types not available"}
        name = re.sub(r"[^A-Za-z0-9_-]", "", str(name or ""))
        name = name or time.strftime("map_%Y%m%d_%H%M%S")
        map_dir = os.path.expanduser("~/maps")
        os.makedirs(map_dir, exist_ok=True)
        path = os.path.join(map_dir, name)
        if not self._save_map_cli.service_is_ready():
            return {"ok": False,
                    "error": "slam_toolbox save_map service not up — is SLAM running?"}
        self._map_saves_pending = 2   # save_map + serialize below
        req = SaveMap.Request()
        req.name.data = path
        self._save_map_cli.call_async(req).add_done_callback(
            lambda f: self._map_saved(f, path, "pgm/yaml"))
        # Also serialize the pose-graph so mapping can be resumed later.
        sreq = SerializePoseGraph.Request()
        sreq.filename = path
        self._serialize_cli.call_async(sreq).add_done_callback(
            lambda f: self._map_saved(f, path, "posegraph"))
        self.log(f"💾 saving map → {path}", "run")
        return {"ok": True, "path": path}

    def _map_saved(self, fut, path, kind):
        self._map_saves_pending = max(0, self._map_saves_pending - 1)
        try:
            fut.result()
            self.log(f"map {kind} saved: {path}", "info")
        except Exception as exc:
            self.log(f"map {kind} save FAILED: {exc}", "error")

    # ── Manual blacklist zones → frontier_explorer ───────────────────────────
    def blacklist_zone(self, payload):
        if payload.get("action") == "clear":
            self._bl_pub.publish(String(data=json.dumps({"action": "clear"})))
            self.log("manual blacklist zones cleared", "info")
            return {"ok": True}
        # Canvas points arrive in odom frame; frontier zones live in map frame.
        tx = ty = th = 0.0
        if self._tf_buffer is not None:
            try:
                tr = self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                q = tr.transform.rotation
                th = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
                tx, ty = tr.transform.translation.x, tr.transform.translation.y
            except Exception:
                pass  # frames coincide until SLAM is up
        try:
            xo, yo = float(payload["x"]), float(payload["y"])
            radius = max(0.1, min(5.0, float(payload.get("radius", 0.8))))
            duration = max(0.0, float(payload.get("duration", 0.0)))
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "error": "bad blacklist payload"}
        mx = tx + xo * math.cos(th) - yo * math.sin(th)
        my = ty + xo * math.sin(th) + yo * math.cos(th)
        self._bl_pub.publish(String(data=json.dumps({
            "action": "add", "x": round(mx, 3), "y": round(my, 3),
            "radius": round(radius, 2), "duration": duration})))
        desc = "PERMANENT" if duration <= 0 else f"{duration:.0f}s"
        self.log(f"🚫 no-go zone ({mx:.2f},{my:.2f}) r={radius:.2f} m {desc}", "run")
        return {"ok": True}

    # ── Semantic vision pipeline (independent of the one-shot tool slot) ─────
    def start_vision(self):
        if self._vision_proc and self._vision_proc.poll() is None:
            return {"ok": True, "note": "already running"}
        cmd = ["ros2", "launch", "Ros_lidar_bot", "vision.launch.py"]
        # libcamerify LD_PRELOADs a CameraManager so OpenCV/V4L2 can talk to
        # CSI cameras when picamera2 is missing (e.g. plain Ubuntu). It is
        # NOT harmless alongside picamera2: Picamera2 creates its own
        # CameraManager, and libcamera aborts with
        # "Multiple CameraManager objects are not allowed". Only wrap when
        # picamera2 is unavailable.
        try:
            import picamera2  # noqa: F401
            has_picamera2 = True
        except ImportError:
            has_picamera2 = False
        if shutil.which("libcamerify") and not has_picamera2:
            cmd = ["libcamerify"] + cmd
        self.log("▶ starting semantic vision (YOLO-World + semantic SLAM)", "run")
        try:
            self._vision_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self.log(f"failed to start vision: {exc}", "error")
            return {"ok": False, "error": str(exc)}
        threading.Thread(target=self._pump_vision, args=(self._vision_proc,),
                         daemon=True).start()
        return {"ok": True}

    def _pump_vision(self, proc):
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                self.hub.send({"t": "log", "level": "tool",
                               "time": time.strftime("%H:%M:%S"),
                               "text": "[vision] " + line})
        rc = proc.wait()
        with self._lock:
            self._state["yolo_det"] = []
        with self._cam_cond:
            self._cam_frame = None
            self._cam_seq += 1
            self._cam_cond.notify_all()   # wakes streamers so the <img> clears
        lvl = "info" if rc == 0 else "error"
        self.log(f"■ semantic vision exited (code {rc})", lvl)

    def stop_vision(self):
        proc = self._vision_proc
        if not proc or proc.poll() is not None:
            return {"ok": True, "note": "not running"}
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        return {"ok": True}

    # ── Camera preview frame (from yolo.py, loopback only) ──────────────────
    def set_camera_frame(self, data: bytes):
        with self._cam_cond:
            self._cam_frame = data
            self._cam_seq += 1
            self._cam_cond.notify_all()

    # ── Nudge / E-stop pulse publisher ───────────────────────────────────────
    def _motion_tick(self):
        now = time.monotonic()
        if now < self._estop_until:
            self._cmd_pub.publish(Twist())   # stream zeros
        elif now < self._nudge_until:
            msg = Twist()
            msg.linear.x, msg.angular.z = self._nudge_cmd
            self._cmd_pub.publish(msg)
        elif self._nudge_until and now < self._nudge_until + 0.3:
            self._cmd_pub.publish(Twist())   # one trailing stop window

    def nudge(self, lin, ang):
        lin = max(-0.3, min(0.3, float(lin)))
        ang = max(-1.0, min(1.0, float(ang)))
        self._nudge_cmd = (lin, ang)
        self._nudge_until = time.monotonic() + 0.5
        return {"ok": True}

    def estop(self):
        self._nudge_until = 0.0
        self._estop_until = time.monotonic() + 1.5
        self.stop_tool()
        self.cancel_nav()
        self.log("E-STOP — zero cmd_vel streamed, tool killed, nav goal cancelled", "error")
        return {"ok": True}

    # ── Tool management ──────────────────────────────────────────────────────
    def run_tool(self, tool, params):
        if tool not in TOOLS:
            return {"ok": False, "error": f"unknown tool '{tool}'"}
        if self._tool_proc and self._tool_proc.poll() is None:
            return {"ok": False, "error": f"'{self._tool_name}' is still running — stop it first"}
        label, builder = TOOLS[tool]
        cmd = builder(params or {})
        self.log(f"▶ starting {label}: {' '.join(cmd)}", "run")
        try:
            self._tool_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except OSError as exc:
            self.log(f"failed to start {label}: {exc}", "error")
            return {"ok": False, "error": str(exc)}
        self._tool_name = tool
        threading.Thread(target=self._pump_tool, args=(self._tool_proc, label),
                         daemon=True).start()
        return {"ok": True}

    def _pump_tool(self, proc, label):
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                self.hub.send({"t": "log", "level": "tool",
                               "time": time.strftime("%H:%M:%S"), "text": line})
        rc = proc.wait()
        if proc is self._tool_proc:
            self._tool_name = None
        lvl = "info" if rc == 0 else "error"
        self.log(f"■ {label} exited (code {rc})", lvl)

    def stop_tool(self):
        proc = self._tool_proc
        if not proc or proc.poll() is not None:
            return {"ok": True, "note": "nothing running"}
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._tool_name = None
        return {"ok": True}

    # ── Nav goal from canvas click ───────────────────────────────────────────
    def _map_to_odom(self, mx, my):
        """Transform a map-frame point into odom for canvas drawing."""
        tr = self._tf_buffer.lookup_transform("odom", "map", rclpy.time.Time())
        q = tr.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = tr.transform.translation.x, tr.transform.translation.y
        return (tx + mx * math.cos(th) - my * math.sin(th),
                ty + mx * math.sin(th) + my * math.cos(th))

    def _plan_cb(self, msg: Path):
        """Stream Nav2's global plan to the browser (odom frame)."""
        if not msg.poses:
            return
        with self._lock:
            active = self._state["goal"] is not None
        if not active:
            return
        frame = msg.header.frame_id or "map"
        step = max(1, len(msg.poses) // 60)
        path = []
        for p in msg.poses[::step]:
            x, y = p.pose.position.x, p.pose.position.y
            if frame != "odom":
                try:
                    x, y = self._map_to_odom(x, y)
                except Exception:
                    return
            path.append([round(x, 2), round(y, 2)])
        with self._lock:
            if self._state["goal"] is None:
                return
            self._state["path"] = path

    def _clear_nav_viz(self):
        with self._lock:
            self._state["goal"] = None
            self._state["path"] = []

    def nav_goal(self, x_odom, y_odom):
        if self._nav_client is None:
            return {"ok": False, "error": "nav2_msgs not available"}
        if self._tf_buffer is None:
            return {"ok": False, "error": "tf2 not available"}
        try:
            tr = self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        except Exception as exc:
            return {"ok": False, "error": f"map→odom TF unavailable: {exc}"}
        q = tr.transform.rotation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        gx = tr.transform.translation.x + x_odom * math.cos(th) - y_odom * math.sin(th)
        gy = tr.transform.translation.y + x_odom * math.sin(th) + y_odom * math.cos(th)
        with self._lock:
            rx, ry, _ = self._state["pose"]
        yaw = math.atan2(y_odom - ry, x_odom - rx) + th  # face travel direction

        # Preempt any in-flight goal so a second click always sticks.
        prev = self._nav_goal_handle
        self._nav_goal_handle = None
        self._nav_gen += 1
        gen = self._nav_gen
        if prev is not None:
            try:
                prev.cancel_goal_async()
            except Exception:
                pass

        # Flag appears immediately (odom frame, matches canvas).
        with self._lock:
            self._state["goal"] = [round(float(x_odom), 2), round(float(y_odom), 2)]
            self._state["path"] = []

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        if not self._nav_client.server_is_ready():
            if not self._nav_client.wait_for_server(timeout_sec=0.5):
                self._clear_nav_viz()
                return {"ok": False, "error": "navigate_to_pose server not available"}
        self.log(f"🎯 nav goal → map ({gx:.2f}, {gy:.2f})", "run")
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f, g=gen: self._nav_accepted(f, g))
        return {"ok": True}

    def _nav_accepted(self, fut, gen):
        try:
            gh = fut.result()
        except Exception as exc:
            if gen == self._nav_gen:
                self.log(f"nav goal accept error: {exc}", "error")
                self._clear_nav_viz()
            return
        if gen != self._nav_gen:
            # Superseded / cancelled while waiting for accept — drop it.
            if gh and gh.accepted:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
            return
        if not gh or not gh.accepted:
            self.log("nav goal REJECTED", "error")
            self._clear_nav_viz()
            return
        self._nav_goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, g=gen: self._nav_done(f, g))

    def _nav_done(self, fut, gen):
        if gen != self._nav_gen:
            return
        try:
            status = fut.result().status
        except Exception as exc:
            self.log(f"nav goal error: {exc}", "error")
            self._nav_goal_handle = None
            self._clear_nav_viz()
            return
        names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        lvl = "info" if status in (4, 5) else "error"
        self.log(f"nav goal finished: {names.get(status, status)}", lvl)
        self._nav_goal_handle = None
        self._clear_nav_viz()

    def cancel_nav(self):
        """Cancel immediately — UI clears now; Nav2 cancel is async."""
        self._nav_gen += 1
        gh = self._nav_goal_handle
        self._nav_goal_handle = None
        self._clear_nav_viz()
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
        self.log("✕ nav goal cancelled", "info")
        return {"ok": True}


# ── HTTP layer ────────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):   # silence per-request stderr spam
        pass

    @property
    def dash(self) -> Dashboard:
        return self.server.dash

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = self.dash.hub.attach()
            if self.dash.last_map_data:   # replay latest map to the new client
                try:
                    q.put_nowait(self.dash.last_map_data)
                except queue.Full:
                    pass
            try:
                while True:
                    try:
                        data = q.get(timeout=5.0)
                        self.wfile.write(f"data: {data}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                self.dash.hub.detach(q)
        elif self.path == "/camera.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            last_seq = -1
            try:
                while True:
                    with self.dash._cam_cond:
                        self.dash._cam_cond.wait_for(
                            lambda: self.dash._cam_seq != last_seq, timeout=5.0)
                        frame, last_seq = self.dash._cam_frame, self.dash._cam_seq
                    if frame is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                        str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api/camera_frame":
            length = int(self.headers.get("Content-Length", 0))
            self.dash.set_camera_frame(self.rfile.read(length))
            body = b'{"ok":true}'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        length = int(self.headers.get("Content-Length", 0))
        try:
            payload = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            payload = {}

        if self.path == "/api/run":
            out = self.dash.run_tool(payload.get("tool"), payload.get("params"))
        elif self.path == "/api/stop":
            out = self.dash.stop_tool()
            self.dash.log("tool stopped from GUI", "info")
        elif self.path == "/api/estop":
            out = self.dash.estop()
        elif self.path == "/api/nudge":
            out = self.dash.nudge(payload.get("lin", 0.0), payload.get("ang", 0.0))
        elif self.path == "/api/nav_goal":
            out = self.dash.nav_goal(float(payload.get("x", 0.0)),
                                     float(payload.get("y", 0.0)))
        elif self.path == "/api/nav_cancel":
            self.dash.cancel_nav()
            out = {"ok": True}
        elif self.path == "/api/vision":
            if payload.get("toggle"):
                proc = self.dash._vision_proc
                on = not (proc and proc.poll() is None)
            else:
                on = payload.get("on")
            out = self.dash.start_vision() if on else self.dash.stop_vision()
        elif self.path == "/api/save_map":
            out = self.dash.save_map(payload.get("name"))
        elif self.path == "/api/blacklist":
            out = self.dash.blacklist_zone(payload)
        elif self.path == "/api/log":
            # Lets other nodes (gamepad teleop) drop a line into the console.
            lvl = payload.get("level")
            self.dash.log(str(payload.get("text", ""))[:200],
                          lvl if lvl in ("info", "run", "error") else "info")
            out = {"ok": True}
        else:
            self.send_error(404)
            return

        body = json.dumps(out).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return socket.gethostname()


# ── Web page (embedded — no CDN, works offline) ──────────────────────────────

PAGE = r"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ros_lidar_bot dashboard</title>
<style>
:root{--bg:#0e141b;--card:#161f29;--edge:#243342;--tx:#d7e1ea;--dim:#7d8fa1;
  --acc:#4fc3f7;--ok:#3ddc84;--warn:#ffb74d;--err:#ff5d5d;--run:#c792ea}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:10px 18px;background:var(--card);
  border-bottom:1px solid var(--edge);position:sticky;top:0;z-index:5}
header h1{font-size:17px;letter-spacing:.4px}
.pill{padding:3px 12px;border-radius:99px;font-weight:600;font-size:12px}
.pill.ok{background:#123924;color:var(--ok)} .pill.bad{background:#3c1518;color:var(--err)}
#estop{margin-left:auto;background:var(--err);color:#fff;border:0;border-radius:8px;
  font-size:15px;font-weight:800;padding:10px 26px;cursor:pointer;letter-spacing:1px}
#estop:active{transform:scale(.96)}
main{display:grid;grid-template-columns:minmax(420px,1.4fr) minmax(330px,1fr);gap:14px;padding:14px}
.card{background:var(--card);border:1px solid var(--edge);border-radius:12px;padding:14px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:1.4px;color:var(--dim);margin-bottom:10px}
#viz{position:relative}
#cv{width:100%;border-radius:8px;background:#0a0f14;display:block;cursor:crosshair}
#readouts{display:grid;grid-template-columns:repeat(auto-fit,minmax(96px,1fr));gap:8px;margin-top:10px}
.ro{background:#0a0f14;border:1px solid var(--edge);border-radius:8px;padding:7px 10px}
.ro b{display:block;font-size:11px;color:var(--dim);font-weight:600}
.ro span{font-family:ui-monospace,Consolas,monospace;font-size:15px;color:var(--acc)}
#navmode{margin-top:10px;display:flex;gap:10px;align-items:center;color:var(--dim);font-size:13px}
#pad{display:grid;grid-template-columns:repeat(3,46px);gap:6px;justify-content:center;margin-top:10px}
#pad button{height:40px;background:#0a0f14;border:1px solid var(--edge);color:var(--tx);
  border-radius:8px;font-size:16px;cursor:pointer}
#pad button:active{background:var(--acc);color:#000}
.right{display:flex;flex-direction:column;gap:14px}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:3px 6px;border-bottom:1px solid var(--edge)}
td.num{font-family:ui-monospace,Consolas,monospace;text-align:right}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:7px}
.dot.ok{background:var(--ok)} .dot.bad{background:var(--err)}
#nodes{display:grid;grid-template-columns:1fr 1fr;gap:2px 14px;font-size:13px}
.tool{border:1px solid var(--edge);border-radius:10px;padding:10px;margin-bottom:10px;background:#101820}
.tool h3{font-size:13px;margin-bottom:6px}
.tool p{color:var(--dim);font-size:12px;margin-bottom:8px}
.tool .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.tool label{font-size:12px;color:var(--dim)}
.tool input{width:70px;background:#0a0f14;color:var(--acc);border:1px solid var(--edge);
  border-radius:6px;padding:5px 7px;font-family:ui-monospace,monospace}
.btn{background:var(--acc);color:#00222f;font-weight:700;border:0;border-radius:7px;
  padding:7px 16px;cursor:pointer;font-size:13px}
.btn.stop{background:var(--warn);color:#2b1a00}
.btn:disabled{opacity:.35;cursor:not-allowed}
#running{display:none;align-items:center;gap:10px;background:#1d1430;border:1px solid var(--run);
  border-radius:8px;padding:8px 12px;margin-bottom:10px;font-size:13px;color:var(--run)}
#console{grid-column:1/-1}
#logbox{background:#0a0f14;border:1px solid var(--edge);border-radius:8px;height:230px;
  overflow-y:auto;padding:8px 10px;font:12px/1.6 ui-monospace,Consolas,monospace}
#sysgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.sys{background:#0a0f14;border:1px solid var(--edge);border-radius:8px;padding:7px 10px}
.sys b{display:block;font-size:11px;color:var(--dim);font-weight:600}
.sys span{font-family:ui-monospace,Consolas,monospace;font-size:14px;color:var(--acc)}
.bar{height:5px;border-radius:3px;background:#1c2733;margin-top:5px;overflow:hidden}
.bar i{display:block;height:100%;border-radius:3px}
#navmode{flex-wrap:wrap}
#navmode select{background:#0a0f14;color:var(--acc);border:1px solid var(--edge);
  border-radius:6px;padding:4px 6px}
#mapname{flex:1;background:#0a0f14;color:var(--acc);border:1px solid var(--edge);
  border-radius:6px;padding:6px 8px;font-family:ui-monospace,monospace;min-width:0}
.ln .t{color:var(--dim);margin-right:8px}
.ln.info{color:var(--tx)} .ln.error{color:var(--err)} .ln.run{color:var(--run)} .ln.tool{color:#9fb8cc}
@media (max-width:900px){main{grid-template-columns:1fr}}
#camcard{flex-shrink:0}
#camfeed{width:100%;border-radius:8px;background:#0a0f14;display:block;max-height:220px;object-fit:contain}
#campause{color:var(--dim);font-size:12px;padding:10px 0;text-align:center}
</style></head><body>

<header>
  <h1>🤖 Ros_lidar_bot</h1>
  <span id="overall" class="pill bad">CONNECTING</span>
  <span id="toolpill" class="pill" style="display:none;background:#1d1430;color:var(--run)"></span>
  <span id="visionpill" class="pill" style="display:none;background:#0d2b1a;color:var(--acc)">👁 VISION ON</span>
  <span id="savepill" class="pill" style="display:none;background:#102a3a;color:#4fc3f7">💾 SAVING MAP…</span>
  <button id="estop">E-STOP</button>
</header>

<main>
  <section class="card" id="viz">
    <h2>Live view — odom frame · wheel-zoom · click = nav goal (when armed)</h2>
    <canvas id="cv" width="900" height="620"></canvas>
    <div id="readouts">
      <div class="ro"><b>x (odom)</b><span id="rx">–</span></div>
      <div class="ro"><b>y (odom)</b><span id="ry">–</span></div>
      <div class="ro"><b>yaw°</b><span id="ryaw">–</span></div>
      <div class="ro"><b>v m/s</b><span id="rv">–</span></div>
      <div class="ro"><b>ω rad/s</b><span id="rw">–</span></div>
      <div class="ro"><b>gyro z</b><span id="rgz">–</span></div>
      <div class="ro"><b>cmd v</b><span id="rcv">–</span></div>
      <div class="ro"><b>cmd ω</b><span id="rcw">–</span></div>
      <div class="ro"><b>map pose</b><span id="rmap">–</span></div>
    </div>
    <div id="navmode">
      <label><input type="checkbox" id="navarm"> Nav-goal mode (click canvas to send goal)</label>
      <button class="btn stop" id="navcancel" style="padding:4px 12px">cancel goal</button>
      <label style="margin-left:8px"><input type="checkbox" id="blarm"> 🚫 No-go mode (drag a circle)</label>
      <select id="bldur">
        <option value="60">1 min</option><option value="300" selected>5 min</option>
        <option value="900">15 min</option><option value="0">permanent</option>
      </select>
      <button class="btn stop" id="blclear" style="padding:4px 12px">clear zones</button>
    </div>
    <div id="pad">
      <span></span><button data-c="0.15,0">▲</button><span></span>
      <button data-c="0,0.6">◀</button><button data-c="0,0" id="padstop">■</button><button data-c="0,-0.6">▶</button>
      <span></span><button data-c="-0.15,0">▼</button><span></span>
    </div>
  </section>

  <div class="right">
    <section class="card">
      <h2>System</h2>
      <div id="sysgrid"></div>
      <div style="display:flex;gap:8px;align-items:center;margin-top:10px">
        <input id="mapname" placeholder="map name (blank = timestamp)">
        <button class="btn" id="savemap">💾 Save map</button>
      </div>
    </section>

    <section class="card">
      <h2>System status</h2>
      <div id="nodes"></div>
      <table id="topics" style="margin-top:10px"></table>
      <table id="tftab" style="margin-top:6px"></table>
    </section>

    <section class="card">
      <h2>Tools — run on demand only</h2>
      <div id="running">⏳ <span id="runname"></span> running…
        <button class="btn stop" id="stoptool" style="margin-left:auto">Stop</button></div>

      <div class="tool"><h3>📐 IMU test</h3>
        <p>Static analysis of /imu — bias, noise, rate. Keep the robot still.</p>
        <div class="row"><button class="btn" data-tool="imu_test">Run test</button></div>
      </div>

      <div class="tool"><h3>🧭 IMU calibration</h3>
        <p>⚠ The robot ROTATES IN PLACE. Clear space around it first.</p>
        <div class="row">
          <label>speed rad/s <input id="cal_speed" value="0.5"></label>
          <label>rotations <input id="cal_rot" value="2"></label>
          <button class="btn" data-tool="imu_calibration">Calibrate</button>
        </div>
      </div>

      <div class="tool"><h3>📏 Drive distance</h3>
        <p>⚠ The robot DRIVES the given distance (negative = backward).</p>
        <div class="row">
          <label>distance m <input id="dd_dist" value="1.0"></label>
          <label>max vel m/s <input id="dd_vel" value="0.3"></label>
          <button class="btn" data-tool="drive_distance">Drive</button>
        </div>
      </div>

      <div class="tool"><h3>🎥 Semantic vision</h3>
        <p>YOLO-World camera detections + LiDAR depth → labelled objects on the
        map (needs the camera connected). Runs alongside the tools above.</p>
        <div class="row">
          <button class="btn" id="visionbtn">Start</button>
          <span id="yolodet" style="color:var(--dim);font-size:12px"></span>
        </div>
      </div>

      <div style="color:var(--dim);font-size:12px;margin-top:10px">
        🎮 Gamepad: <b>B</b> save map · <b>X</b> vision on/off ·
        <b>LT+RT+LB+RB</b> IMU calibration</div>
    </section>

    <section class="card" id="camcard">
      <h2>📷 Camera feed</h2>
      <img id="camfeed" style="display:none">
      <p id="campause">Vision off — start vision to see camera feed</p>
    </section>
  </div>

  <section class="card" id="console">
    <h2>Console — data & events</h2>
    <div id="logbox"></div>
  </section>
</main>

<script>
"use strict";
const $=id=>document.getElementById(id);
let S=null, trail=[], scale=70, lastSeen=0;

// ── SSE ──────────────────────────────────────────────────────────────────────
const es=new EventSource("/events");
es.onmessage=e=>{
  const m=JSON.parse(e.data);
  if(m.t==="state"){S=m;lastSeen=Date.now();
    const p=m.pose; const lt=trail[trail.length-1];
    if(!lt||Math.hypot(p[0]-lt[0],p[1]-lt[1])>0.02){trail.push([p[0],p[1]]);if(trail.length>600)trail.shift();}
    updatePanels(m);
  } else if(m.t==="log"){addLog(m);}
  else if(m.t==="map"){loadMap(m);}
};
es.onerror=()=>{$("overall").textContent="DISCONNECTED";$("overall").className="pill bad";};

function post(path,body){return fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},
  body:JSON.stringify(body||{})}).then(r=>r.json()).then(r=>{
    if(r&&r.ok===false)addLog({time:new Date().toTimeString().slice(0,8),level:"error",text:r.error});
    return r;});}

// ── Panels ───────────────────────────────────────────────────────────────────
function updatePanels(m){
  $("rx").textContent=m.pose[0].toFixed(2); $("ry").textContent=m.pose[1].toFixed(2);
  $("ryaw").textContent=(m.pose[2]*57.2958).toFixed(1);
  $("rv").textContent=m.vel[0].toFixed(2); $("rw").textContent=m.vel[1].toFixed(2);
  $("rgz").textContent=m.imu.gz.toFixed(3);
  $("rcv").textContent=m.cmd[0].toFixed(2); $("rcw").textContent=m.cmd[1].toFixed(2);
  $("rmap").textContent=m.map_pose?`${m.map_pose[0]}, ${m.map_pose[1]}`:"–";

  if(m.nodes&&Object.keys(m.nodes).length){
    $("nodes").innerHTML=Object.entries(m.nodes).map(([n,ok])=>
      `<div><span class="dot ${ok?"ok":"bad"}"></span>${n}</div>`).join("");
    const allOk=Object.values(m.nodes).every(v=>v);
    const o=$("overall");o.textContent=allOk?"ALL SYSTEMS GO":"DEGRADED";
    o.className="pill "+(allOk?"ok":"bad");
  }
  if(m.rates&&Object.keys(m.rates).length){
    $("topics").innerHTML=Object.entries(m.rates).map(([t,r])=>
      `<tr><td>${t}</td><td class="num">${r.hz} Hz</td>
       <td class="num">${r.age==null?"never":r.age+"s ago"}</td></tr>`).join("");
  }
  if(m.tf&&Object.keys(m.tf).length){
    $("tftab").innerHTML=Object.entries(m.tf).map(([k,v])=>
      `<tr><td><span class="dot ${v.ok?"ok":"bad"}"></span>TF ${k}</td>
       <td class="num">${v.ok?("age "+v.age+"s"):"MISSING"}</td></tr>`).join("");
  }
  if(m.sys&&Object.keys(m.sys).length){
    const s=m.sys,pct=(v,mx)=>Math.min(100,Math.round(100*v/mx));
    const bar=(p,w,c)=>`<div class="bar"><i style="width:${p}%;background:var(${p>=c?"--err":p>=w?"--warn":"--ok"})"></i></div>`;
    const t=[];
    if(s.cpu!=null)t.push(`<div class="sys"><b>CPU</b><span>${s.cpu}%</span>${bar(s.cpu,70,90)}</div>`);
    if(s.temp!=null)t.push(`<div class="sys"><b>SoC temp</b><span>${s.temp}°C</span>${bar(pct(s.temp,90),67,84)}</div>`);
    if(s.mem)t.push(`<div class="sys"><b>RAM</b><span>${s.mem[0]} / ${s.mem[1]} GB</span>${bar(pct(s.mem[0],s.mem[1]),75,90)}</div>`);
    if(s.disk)t.push(`<div class="sys"><b>Disk /</b><span>${s.disk[0]} / ${s.disk[1]} GB</span>${bar(pct(s.disk[0],s.disk[1]),80,95)}</div>`);
    if(s.load!=null)t.push(`<div class="sys"><b>load 1m</b><span>${s.load}</span></div>`);
    if(s.up)t.push(`<div class="sys"><b>uptime</b><span>${s.up}</span></div>`);
    if(s.throttled)t.push(`<div class="sys" style="border-color:var(--err)"><b>⚠ PI THROTTLED</b><span style="color:var(--err)">0x${s.throttled.toString(16)}</span></div>`);
    $("sysgrid").innerHTML=t.join("");
  }

  const vb=$("visionbtn");
  vb.textContent=m.vision?"Stop":"Start";
  vb.className=m.vision?"btn stop":"btn";
  $("visionpill").style.display=m.vision?"":"none";
  const cam=$("camfeed"),pause=$("campause");
  if(m.vision){
    if(cam.style.display==="none"){cam.src="/camera.mjpg?ts="+Date.now();cam.style.display="block";}
    pause.style.display="none";
  }else if(cam.style.display!=="none"){
    cam.style.display="none";cam.removeAttribute("src");   // closes the mjpeg connection
    pause.style.display="";
  }
  $("savepill").style.display=m.saving?"":"none";
  $("yolodet").textContent=(m.yolo_det&&m.yolo_det.length)?("👁 "+m.yolo_det.join(", ")):"";

  const run=$("running"), pill=$("toolpill");
  if(m.tool){run.style.display="flex";$("runname").textContent=m.tool;
    pill.style.display="";pill.textContent="⏳ "+m.tool;}
  else{run.style.display="none";pill.style.display="none";}
  document.querySelectorAll("[data-tool]").forEach(b=>b.disabled=!!m.tool);
}

// ── Console ──────────────────────────────────────────────────────────────────
function addLog(m){
  const box=$("logbox"),d=document.createElement("div");
  d.className="ln "+(m.level||"info");
  d.innerHTML=`<span class="t">${m.time}</span>${m.text.replace(/</g,"&lt;")}`;
  box.appendChild(d);
  while(box.childNodes.length>500)box.removeChild(box.firstChild);
  box.scrollTop=box.scrollHeight;
}

// ── Live SLAM map layer ──────────────────────────────────────────────────────
let MAP=null;
async function loadMap(m){
  try{
    const raw=Uint8Array.from(atob(m.data),c=>c.charCodeAt(0));
    const ds=new DecompressionStream("deflate");
    const buf=await new Response(new Blob([raw]).stream().pipeThrough(ds)).arrayBuffer();
    const cells=new Uint8Array(buf);
    const off=document.createElement("canvas");off.width=m.w;off.height=m.h;
    const ictx=off.getContext("2d");
    const img=ictx.createImageData(m.w,m.h);
    for(let i=0;i<cells.length;i++){
      const v=cells[i],o=i*4;
      if(v===1){img.data[o]=25;img.data[o+1]=36;img.data[o+2]=48;img.data[o+3]=210;}       // free
      else if(v===2){img.data[o]=176;img.data[o+1]=196;img.data[o+2]=212;img.data[o+3]=255;} // occupied
    }                                                                                        // 0 = unknown → transparent
    ictx.putImageData(img,0,0);
    MAP={img:off,res:m.res,origin:m.origin,tf:m.tf};
  }catch(e){/* old browser without DecompressionStream — no map layer */}
}
// map-frame point → odom-frame point (using the map→odom TF sent with the map)
function mapToOdom(mx,my){
  const[tx,ty,th]=MAP.tf;
  return [tx+mx*Math.cos(th)-my*Math.sin(th), ty+mx*Math.sin(th)+my*Math.cos(th)];
}

// ── Canvas ───────────────────────────────────────────────────────────────────
const cv=$("cv"),ctx=cv.getContext("2d");
if(!ctx.roundRect){ctx.roundRect=function(x,y,w,h){this.rect(x,y,w,h);};}
cv.addEventListener("wheel",e=>{e.preventDefault();
  scale=Math.max(25,Math.min(220,scale*(e.deltaY<0?1.12:0.89)));},{passive:false});
function canvasToWorld(e){
  const r=cv.getBoundingClientRect();
  const px=(e.clientX-r.left)*(cv.width/r.width), py=(e.clientY-r.top)*(cv.height/r.height);
  return [S.pose[0]+(px-cv.width/2)/scale, S.pose[1]-(py-cv.height/2)/scale];
}
cv.addEventListener("click",e=>{
  if(!$("navarm").checked||$("blarm").checked||!S)return;
  const[wx,wy]=canvasToWorld(e);
  // Optimistic flag so the marker appears on the first click, every time.
  S.goal=[+wx.toFixed(2),+wy.toFixed(2)]; S.path=[];
  post("/api/nav_goal",{x:wx,y:wy});
});
// no-go zones: press = centre, drag = radius, release = send
let blDrag=null;   // [cx, cy, r] in odom-frame metres
cv.addEventListener("mousedown",e=>{
  if(!$("blarm").checked||!S)return;
  const[wx,wy]=canvasToWorld(e);blDrag=[wx,wy,0];e.preventDefault();
});
cv.addEventListener("mousemove",e=>{
  if(!blDrag)return;
  const[wx,wy]=canvasToWorld(e);
  blDrag[2]=Math.hypot(wx-blDrag[0],wy-blDrag[1]);
});
window.addEventListener("mouseup",()=>{
  if(!blDrag)return;
  const[zx,zy,zr]=blDrag;blDrag=null;
  if(zr<0.05)return;   // accidental click, not a drag
  post("/api/blacklist",{x:zx,y:zy,radius:zr,duration:parseFloat($("bldur").value)});
});

function draw(){
  requestAnimationFrame(draw);
  ctx.fillStyle="#0a0f14";ctx.fillRect(0,0,cv.width,cv.height);
  if(!S){return;}
  const [rx,ry,yaw]=S.pose, cx=cv.width/2, cy=cv.height/2;
  const W=(x,y)=>[cx+(x-rx)*scale, cy-(y-ry)*scale];

  // 1 m grid
  ctx.strokeStyle="#15202b";ctx.lineWidth=1;ctx.beginPath();
  const x0=Math.floor(rx-cv.width/2/scale),x1=Math.ceil(rx+cv.width/2/scale);
  const y0=Math.floor(ry-cv.height/2/scale),y1=Math.ceil(ry+cv.height/2/scale);
  for(let gx=x0;gx<=x1;gx++){const[a,b]=W(gx,y0),[c,d]=W(gx,y1);ctx.moveTo(a,b);ctx.lineTo(c,d);}
  for(let gy=y0;gy<=y1;gy++){const[a,b]=W(x0,gy),[c,d]=W(x1,gy);ctx.moveTo(a,b);ctx.lineTo(c,d);}
  ctx.stroke();
  // origin marker
  const[ox,oy]=W(0,0);ctx.fillStyle="#31465c";ctx.fillRect(ox-3,oy-3,6,6);

  // Occupancy-grid rendering removed per user — lidar scan only. MAP is still
  // received: its map→odom TF places the frontier/semantic overlays below.

  // frontier explorer overlays (map frame): blacklist / perm / manual zones + queue
  if(MAP&&S.frontier){
    const fill={bl:"rgba(255,60,60,.22)",perm:"rgba(15,15,15,.55)",manual:"rgba(199,146,234,.28)"};
    const edge={bl:"#ff5d5d",perm:"#777",manual:"#c792ea"};
    for(const[zx,zy,zr,k] of (S.frontier.zones||[])){
      const[wx2,wy2]=mapToOdom(zx,zy),[px,py]=W(wx2,wy2);
      ctx.beginPath();ctx.arc(px,py,zr*scale,0,6.2832);
      ctx.fillStyle=fill[k]||fill.bl;ctx.fill();
      ctx.strokeStyle=edge[k]||edge.bl;ctx.lineWidth=1.5;ctx.stroke();
    }
    ctx.font="11px system-ui";ctx.textAlign="center";
    (S.frontier.queue||[]).forEach(([qx,qy],i)=>{
      const[wx2,wy2]=mapToOdom(qx,qy),[px,py]=W(wx2,wy2);
      ctx.fillStyle="rgba(61,220,132,.9)";
      ctx.beginPath();ctx.arc(px,py,7,0,6.2832);ctx.fill();
      ctx.fillStyle="#04270f";ctx.fillText(String(i+1),px,py+4);
    });
  }

  // trail
  if(trail.length>1){ctx.strokeStyle="rgba(79,195,247,.45)";ctx.lineWidth=2;ctx.beginPath();
    trail.forEach((p,i)=>{const[a,b]=W(p[0],p[1]);i?ctx.lineTo(a,b):ctx.moveTo(a,b);});ctx.stroke();}

  // Nav2 global path (odom frame)
  if(S.path&&S.path.length>1){
    ctx.strokeStyle="#ffb74d";ctx.lineWidth=2.5;ctx.setLineDash([8,5]);ctx.beginPath();
    S.path.forEach((p,i)=>{const[a,b]=W(p[0],p[1]);i?ctx.lineTo(a,b):ctx.moveTo(a,b);});
    ctx.stroke();ctx.setLineDash([]);
  }

  // Goal flag
  if(S.goal){
    const[gx,gy]=W(S.goal[0],S.goal[1]);
    ctx.strokeStyle="#ff5d5d";ctx.fillStyle="#ff5d5d";ctx.lineWidth=2;
    ctx.beginPath();ctx.moveTo(gx,gy);ctx.lineTo(gx,gy-28);ctx.stroke();
    ctx.beginPath();ctx.moveTo(gx,gy-28);ctx.lineTo(gx+16,gy-22);ctx.lineTo(gx,gy-16);ctx.closePath();ctx.fill();
    ctx.beginPath();ctx.arc(gx,gy,4,0,Math.PI*2);ctx.fill();
  }

  // lidar
  ctx.fillStyle="rgba(61,220,132,.75)";
  for(const [a,r] of S.scan){
    const th=yaw+a, [px,py]=W(rx+r*Math.cos(th), ry+r*Math.sin(th));
    ctx.fillRect(px-1.5,py-1.5,3,3);
  }

  // no-go drag preview
  if(blDrag&&blDrag[2]>0){
    const[px,py]=W(blDrag[0],blDrag[1]);
    ctx.beginPath();ctx.arc(px,py,blDrag[2]*scale,0,6.2832);
    ctx.fillStyle="rgba(199,146,234,.20)";ctx.fill();
    ctx.strokeStyle="#c792ea";ctx.lineWidth=2;ctx.stroke();
    ctx.font="12px system-ui";ctx.fillStyle="#c792ea";ctx.textAlign="center";
    ctx.fillText(blDrag[2].toFixed(2)+" m",px,py-blDrag[2]*scale-6);
  }

  // semantic objects (map frame → odom via map TF)
  if(MAP&&S.sem&&S.sem.length){
    ctx.font="11px system-ui";ctx.textAlign="center";
    for(const[mx,my,label] of S.sem){
      const[wx2,wy2]=mapToOdom(mx,my);
      const[px,py]=W(wx2,wy2);
      ctx.fillStyle="#ffb74d";
      ctx.beginPath();ctx.moveTo(px,py-5);ctx.lineTo(px+5,py);ctx.lineTo(px,py+5);ctx.lineTo(px-5,py);ctx.closePath();ctx.fill();
      ctx.fillStyle="#ffd9a0";ctx.fillText(label,px,py-8);
    }
  }

  // robot body 0.50×0.36 m
  ctx.save();ctx.translate(cx,cy);ctx.rotate(-yaw);
  const L=0.5*scale,Wd=0.36*scale;
  ctx.fillStyle="#1d2c3a";ctx.strokeStyle="#4fc3f7";ctx.lineWidth=2;
  ctx.beginPath();ctx.roundRect(-L/2,-Wd/2,L,Wd,5);ctx.fill();ctx.stroke();
  // wheels
  ctx.fillStyle="#31465c";
  ctx.fillRect(-L*0.32,-Wd/2-3,L*0.28,6);ctx.fillRect(-L*0.32,Wd/2-3,L*0.28,6);
  ctx.fillRect(L*0.05,-Wd/2-3,L*0.28,6);ctx.fillRect(L*0.05,Wd/2-3,L*0.28,6);
  // heading arrow
  ctx.strokeStyle="#ffb74d";ctx.lineWidth=3;ctx.beginPath();
  ctx.moveTo(0,0);ctx.lineTo(L*0.65,0);ctx.moveTo(L*0.5,-6);ctx.lineTo(L*0.65,0);ctx.lineTo(L*0.5,6);ctx.stroke();
  // gyro ring: sweep ∝ ωz
  const g=Math.max(-2,Math.min(2,S.imu.gz));
  if(Math.abs(g)>0.03){ctx.strokeStyle="#c792ea";ctx.lineWidth=3;ctx.beginPath();
    ctx.arc(0,0,L*0.75,0,-g*1.2,g>0);ctx.stroke();}
  ctx.restore();

  // stale-data veil
  if(Date.now()-lastSeen>2000){ctx.fillStyle="rgba(10,15,20,.65)";
    ctx.fillRect(0,0,cv.width,cv.height);ctx.fillStyle="#ff5d5d";
    ctx.font="20px system-ui";ctx.textAlign="center";
    ctx.fillText("no data from robot…",cv.width/2,cv.height/2);}
}
draw();

// ── Buttons ──────────────────────────────────────────────────────────────────
$("estop").onclick=()=>post("/api/estop");
$("stoptool").onclick=()=>post("/api/stop");
$("navcancel").onclick=()=>{if(S){S.goal=null;S.path=[];} post("/api/nav_cancel");};
$("padstop").onclick=()=>post("/api/estop");
document.querySelectorAll("#pad button[data-c]").forEach(b=>{
  if(b.id==="padstop")return;
  b.onclick=()=>{const[l,a]=b.dataset.c.split(",").map(Number);post("/api/nudge",{lin:l,ang:a});};
});
$("visionbtn").onclick=()=>post("/api/vision",{on:!(S&&S.vision)});
$("savemap").onclick=()=>post("/api/save_map",{name:$("mapname").value});
$("blclear").onclick=()=>post("/api/blacklist",{action:"clear"});
document.querySelectorAll("[data-tool]").forEach(b=>{
  b.onclick=()=>{
    const t=b.dataset.tool,params={};
    if(t==="imu_calibration"){params.speed=$("cal_speed").value;params.rotations=$("cal_rot").value;}
    if(t==="drive_distance"){params.distance=$("dd_dist").value;params.max_vel=$("dd_vel").value;}
    post("/api/run",{tool:t,params});
  };
});
</script>
</body></html>
"""


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()

    server = ThreadingHTTPServer(("0.0.0.0", node.port), Handler)
    server.dash = node
    threading.Thread(target=server.serve_forever, daemon=True).start()

    # print() (not logger): visible even when bringup runs at FATAL log level.
    print(f"\n  🌐 Robot dashboard:  http://{_local_ip()}:{node.port}\n", flush=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_tool()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
