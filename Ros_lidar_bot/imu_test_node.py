#!/usr/bin/env python3
"""
imu_test_node.py — IMU Test Node + Built-in Large GUI Visualiser

Subscribes : /imu  (sensor_msgs/Imu)  from BNO055 via imu_node.py
Opens      : A Tkinter GUI window automatically on launch showing:
               • 1 large 3-D rotating axis widget in the center
               • Live dashboard with numeric readouts: Euler angles, quaternion, gyro, accel
               • Fully aligned coordinate convention (+X, +Y, +Z)

Usage:
    ros2 run Ros_lidar_bot imu_test_node
"""

import math
import threading
import sys
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy,
)
from sensor_msgs.msg import Imu


# ═══════════════════════════════════════════════════════════════════════════════
#  COLOUR PALETTE
# ═══════════════════════════════════════════════════════════════════════════════
BG_DARK   = "#0d1117"
BG_PANEL  = "#161b22"
BG_CARD   = "#1c2128"
BG_HOVER  = "#22272e"
ACCENT_X  = "#58a6ff"   # blue  — X axis
ACCENT_Y  = "#3fb950"   # green — Y axis
ACCENT_Z  = "#f78166"   # red   — Z axis
ACCENT_W  = "#d2a8ff"   # purple
FG_MAIN   = "#e6edf3"
FG_DIM    = "#8b949e"
FG_WARN   = "#f0883e"
BORDER    = "#30363d"


# ═══════════════════════════════════════════════════════════════════════════════
#  MATH HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def quat_to_euler_deg(x, y, z, w):
    """Quaternion → (roll, pitch, yaw) in degrees."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_to_matrix(qx, qy, qz, qw):
    """Quaternion → 3×3 rotation matrix (flat, row-major)."""
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        1-2*(yy+zz),  2*(xy-wz),   2*(xz+wy),
        2*(xy+wz),    1-2*(xx+zz), 2*(yz-wx),
        2*(xz-wy),    2*(yz+wx),   1-2*(xx+yy),
    ]


def mat_vec(m, v):
    return (m[0]*v[0]+m[1]*v[1]+m[2]*v[2],
            m[3]*v[0]+m[4]*v[1]+m[5]*v[2],
            m[6]*v[0]+m[7]*v[1]+m[8]*v[2])


def project(x, y, z, cx, cy, scale=80.0, persp=4.0):
    """Perspective projection."""
    d = persp / (persp + z)
    return cx + x*scale*d, cy - y*scale*d


CUBE_VERTS = [
    (-0.5,-0.5,-0.5),( 0.5,-0.5,-0.5),( 0.5, 0.5,-0.5),(-0.5, 0.5,-0.5),
    (-0.5,-0.5, 0.5),( 0.5,-0.5, 0.5),( 0.5, 0.5, 0.5),(-0.5, 0.5, 0.5),
]
CUBE_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]


# ═══════════════════════════════════════════════════════════════════════════════
#  3-D AXIS CANVAS WIDGET (LARGE CENTER SIZE)
# ═══════════════════════════════════════════════════════════════════════════════
class Axis3DCanvas(tk.Canvas):
    def __init__(self, parent, label="", size=420, **kw):
        super().__init__(parent, width=size, height=size,
                         bg=BG_CARD, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self._size  = size
        self._cx    = size // 2
        self._cy    = size // 2
        self._label = label
        self._quat  = (0.0, 0.0, 0.0, 1.0)
        self._euler = (0.0, 0.0, 0.0)
        self._draw()

    def set_orientation(self, qx, qy, qz, qw, roll=0, pitch=0, yaw=0):
        self._quat  = (qx, qy, qz, qw)
        self._euler = (roll, pitch, yaw)
        self._draw()

    def _draw(self):
        self.delete("all")
        cx, cy = self._cx, self._cy
        m  = quat_to_matrix(*self._quat)
        sc = self._size * 0.32

        # Outer boundary ring
        self.create_oval(8, 8, self._size-8, self._size-8, outline=BORDER, width=1)

        # Wireframe cube
        rotated   = [mat_vec(m, v) for v in CUBE_VERTS]
        projected = [project(v[0], v[1], v[2], cx, cy, scale=sc*0.68) for v in rotated]
        for i, j in CUBE_EDGES:
            z_avg = (rotated[i][2] + rotated[j][2]) / 2
            bright = int(75 + 115 * (z_avg + 0.7) / 1.4)
            bright = max(60, min(190, bright))
            col = f"#{bright:02x}{bright:02x}{bright:02x}"
            x1, y1 = projected[i]; x2, y2 = projected[j]
            self.create_line(x1, y1, x2, y2, fill=col, width=1, dash=(3,2))

        # Main X / Y / Z axis vectors
        ox, oy = project(0, 0, 0, cx, cy, scale=sc)
        for vec, color, name in (
            ([1,0,0], ACCENT_X, "X"),
            ([0,1,0], ACCENT_Y, "Y"),
            ([0,0,1], ACCENT_Z, "Z"),
        ):
            rv = mat_vec(m, vec)
            tx, ty = project(*rv, cx, cy, scale=sc)
            self.create_line(ox, oy, tx, ty, fill=color, width=3,
                             arrow=tk.LAST, arrowshape=(10,13,5))
            lx = tx + (tx-ox)*0.18
            ly = ty + (ty-oy)*0.18
            self.create_text(lx, ly, text=name, fill=color, font=("Consolas", 11, "bold"))

        # Center dot
        r2 = 5
        self.create_oval(ox-r2, oy-r2, ox+r2, oy+r2, fill=FG_DIM, outline=FG_MAIN, width=1.5)

        # Header Label
        self.create_text(cx, 20, text=self._label, fill=FG_DIM, font=("Consolas", 10, "bold"))

        # Inside Euler Readout
        roll, pitch, yaw = self._euler
        for i, (txt, col) in enumerate([
            (f"Roll:  {roll:+7.2f}°",  ACCENT_X),
            (f"Pitch: {pitch:+7.2f}°", ACCENT_Y),
            (f"Yaw:   {yaw:+7.2f}°",   ACCENT_Z),
        ]):
            self.create_text(cx, self._size - 65 + i*16, text=txt, fill=col, font=("Consolas", 10, "bold"))


# ═══════════════════════════════════════════════════════════════════════════════
#  VALUE ROW WIDGET (label + numeric string + mini bar)
# ═══════════════════════════════════════════════════════════════════════════════
class ValueRow(tk.Frame):
    def __init__(self, parent, label, color=FG_MAIN, unit="", **kw):
        super().__init__(parent, bg=BG_CARD, **kw)
        tk.Label(self, text=f"{label:<5}", fg=color, bg=BG_CARD,
                 font=("Consolas", 9, "bold"), width=5, anchor="w"
                 ).pack(side=tk.LEFT, padx=(6,2))
        self._val = tk.Label(self, text="---", fg=FG_MAIN, bg=BG_CARD,
                             font=("Consolas", 10), width=11, anchor="e")
        self._val.pack(side=tk.LEFT)
        if unit:
            tk.Label(self, text=unit, fg=FG_DIM, bg=BG_CARD,
                     font=("Consolas", 8)).pack(side=tk.LEFT, padx=2)
        self._bar = tk.Canvas(self, width=65, height=7, bg=BG_CARD, highlightthickness=0)
        self._bar.pack(side=tk.LEFT, padx=4)
        self._color = color

    def set(self, val, vmin=-180, vmax=180):
        self._val.config(text=f"{val:+11.4f}")
        frac = max(0.0, min(1.0, (val-vmin)/(vmax-vmin) if vmax!=vmin else 0.5))
        self._bar.delete("all")
        self._bar.create_rectangle(0,0,65,7, fill=BG_HOVER, outline="")
        self._bar.create_rectangle(0,0,int(frac*65),7, fill=self._color, outline="")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN GUI DASHBOARD (3-COLUMN CENTRALIZED LAYOUT)
# ═══════════════════════════════════════════════════════════════════════════════
class ImuGuiApp(tk.Tk):
    POLL_MS = 50

    def __init__(self, data_ref: dict, lock: threading.Lock):
        super().__init__()
        self._data = data_ref
        self._lock = lock

        self.title("IMU Visualiser Node")
        self.configure(bg=BG_DARK)
        self.minsize(1050, 560)
        self.resizable(True, True)

        self._build_ui()
        self.after(self.POLL_MS, self._refresh)

    def _build_ui(self):
        # Top Header
        top = tk.Frame(self, bg=BG_PANEL, pady=8)
        top.pack(fill=tk.X)
        tk.Label(top, text="⬡  IMU VISUALISER", fg=ACCENT_X, bg=BG_PANEL,
                 font=("Consolas", 14, "bold")).pack(side=tk.LEFT, padx=16)
        tk.Label(top, text="Subscribing: /imu", fg=FG_DIM, bg=BG_PANEL, font=("Consolas", 9)).pack(side=tk.LEFT)
        
        self._dot = tk.Label(top, text="●", fg="#f85149", bg=BG_PANEL, font=("Consolas", 12))
        self._dot.pack(side=tk.RIGHT, padx=4)
        self._slbl = tk.Label(top, text="Waiting for /imu…", fg=FG_WARN, bg=BG_PANEL, font=("Consolas", 9))
        self._slbl.pack(side=tk.RIGHT, padx=(0,2))

        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X)

        # Dashboard layout body
        body = tk.Frame(self, bg=BG_DARK)
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        # ── Columns ──
        left_col = tk.Frame(body, bg=BG_DARK)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)

        center_col = tk.Frame(body, bg=BG_DARK)
        center_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=12)

        right_col = tk.Frame(body, bg=BG_DARK)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)

        # ── Left Column: Euler & Quaternion telemetry ──
        self._euler_rows = self._make_xyz_section(left_col, "Euler Angles (deg)", unit="°")
        self._quat_rows = self._make_xyzw_section(left_col, "Quaternion")

        # ── Center Column: Large Axis Canvas ──
        self._cv = Axis3DCanvas(center_col, label="IMU / Robot Frame", size=420)
        self._cv.pack(pady=4)

        # Bottom info card
        leg = tk.Frame(center_col, bg=BG_CARD, highlightthickness=1, highlightbackground=BORDER)
        leg.pack(fill=tk.X, pady=(6,0))
        tk.Label(leg, text="Coordinate Alignment (1-to-1)", fg=ACCENT_W, bg=BG_CARD, font=("Consolas",9,"bold")).pack(pady=(4,2))
        tk.Label(leg, text="+X_bot = +X_imu  |  +Y_bot = +Y_imu  |  +Z_bot = +Z_imu",
                 fg=ACCENT_Y, bg=BG_CARD, font=("Consolas", 8)).pack(pady=(1, 4))

        # ── Right Column: Gyro, Accel & Covariance ──
        self._gyro_rows = self._make_xyz_section(right_col, "Angular Velocity (rad/s)", unit="rad/s", vrange=(-10, 10))
        self._accel_rows = self._make_xyz_section(right_col, "Linear Acceleration (m/s²)", unit="m/s²", vrange=(-20, 20))
        
        self._section_header(right_col, "Covariance Diagonals (σ²)")
        cov_frame = tk.Frame(right_col, bg=BG_CARD)
        cov_frame.pack(fill=tk.X, padx=8)
        self._cov_rows = {}
        for name, col in [("Ori (R/P/Y)", ACCENT_X), ("Gyro (X/Y/Z)", ACCENT_Y), ("Accel (X/Y/Z)", ACCENT_Z)]:
            r = ValueRow(cov_frame, name, color=col, unit="")
            r.pack(fill=tk.X, pady=1)
            self._cov_rows[name.split()[0].lower()] = r
        tk.Frame(right_col, bg=BORDER, height=1).pack(fill=tk.X, padx=8, pady=4)

        # Status Footer
        bot = tk.Frame(self, bg=BG_PANEL, pady=4)
        bot.pack(fill=tk.X, side=tk.BOTTOM)
        self._hz_lbl = tk.Label(bot, text="Rate: -- Hz", fg=FG_DIM, bg=BG_PANEL, font=("Consolas", 8))
        self._hz_lbl.pack(side=tk.LEFT, padx=10)
        tk.Label(bot, text="ros2 run Ros_lidar_bot imu_test_node", fg=FG_DIM, bg=BG_PANEL, font=("Consolas", 8)).pack(side=tk.RIGHT, padx=10)

    def _make_xyz_section(self, parent, title, labels=None, unit="°", vrange=(-180,180)):
        if labels is None:
            labels = ["X","Y","Z"]
        self._section_header(parent, title)
        frm = tk.Frame(parent, bg=BG_CARD); frm.pack(fill=tk.X, padx=8)
        rows = {}
        for lbl, col in zip(labels, [ACCENT_X, ACCENT_Y, ACCENT_Z]):
            r = ValueRow(frm, lbl, color=col, unit=unit)
            r.pack(fill=tk.X, pady=1)
            rows[lbl.lower()] = (r, vrange)
        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=8, pady=4)
        return rows

    def _make_xyzw_section(self, parent, title):
        self._section_header(parent, title)
        frm = tk.Frame(parent, bg=BG_CARD); frm.pack(fill=tk.X, padx=8)
        rows = {}
        for lbl, col in zip(["X","Y","Z","W"], [ACCENT_X, ACCENT_Y, ACCENT_Z, ACCENT_W]):
            r = ValueRow(frm, lbl, color=col, unit="")
            r.pack(fill=tk.X, pady=1)
            rows[lbl.lower()] = (r, (-1, 1))
        tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=8, pady=4)
        return rows

    def _section_header(self, parent, title):
        tk.Label(parent, text=title, fg=ACCENT_W, bg=BG_CARD,
                 font=("Consolas",9,"bold"), anchor="w").pack(fill=tk.X, padx=8, pady=(4,2))

    def _set_rows(self, rows_dict, values_dict):
        for key, (row, vrange) in rows_dict.items():
            val = values_dict.get(key, 0.0)
            row.set(val, *vrange)

    def _refresh(self):
        try:
            with self._lock:
                d = dict(self._data)

            has_data = bool(d)
            if has_data:
                self._dot.config(fg="#3fb950")
                self._slbl.config(text="Live  ✓", fg="#3fb950")
            else:
                self._dot.config(fg="#f0883e")
                self._slbl.config(text="Waiting for /imu…", fg=FG_WARN)

            if not has_data:
                return

            imu = d.get("imu", {})
            cov = d.get("cov", {})

            # 3-D rotation widget update
            iq = imu.get("quat", {}); ie = imu.get("euler", {})
            self._cv.set_orientation(
                iq.get("x",0), iq.get("y",0), iq.get("z",0), iq.get("w",1),
                ie.get("roll",0), ie.get("pitch",0), ie.get("yaw",0))

            # Numeric readouts
            self._set_rows(self._euler_rows, ie)
            self._set_rows(self._quat_rows, iq)
            self._set_rows(self._gyro_rows, imu.get("gyro", {}))
            self._set_rows(self._accel_rows, imu.get("accel", {}))

            # Covariances
            oc = cov.get("ori",   [0,0,0])
            gc = cov.get("gyro",  [0,0,0])
            ac = cov.get("accel", [0,0,0])
            self._cov_rows["ori"].set(oc[2], -0.05, 0.05)
            self._cov_rows["gyro"].set(gc[0], -0.01, 0.01)
            self._cov_rows["accel"].set(ac[0], -0.1, 0.1)

            # Frequency rate update
            hz = d.get("_hz", 0.0)
            self._hz_lbl.config(text=f"Rate: {hz:.1f} Hz")

        except Exception:
            pass
        finally:
            self.after(self.POLL_MS, self._refresh)


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 NODE SUBSCRIBER
# ═══════════════════════════════════════════════════════════════════════════════
class ImuTestNode(Node):
    def __init__(self, shared_data: dict, lock: threading.Lock):
        super().__init__("imu_test_node")
        self._data = shared_data
        self._lock = lock
        self._count = 0
        self._t0 = self.get_clock().now().nanoseconds * 1e-9

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        topic = self.declare_parameter("imu_topic", "/imu").value
        self._sub = self.create_subscription(Imu, topic, self._cb, qos)
        self.get_logger().info(f"ImuTestNode ready — subscribing '{topic}' | GUI launched in main thread")

    def _cb(self, msg: Imu):
        self._count += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = now - self._t0
        hz  = self._count / dt if dt > 0 else 0.0

        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        roll, pitch, yaw = quat_to_euler_deg(qx, qy, qz, qw)

        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        payload = {
            "_hz": round(hz, 2),
            "imu": {
                "quat":  {"x": qx,  "y": qy,  "z": qz,  "w": qw},
                "euler": {"x": roll, "y": pitch, "z": yaw, "roll": roll, "pitch": pitch, "yaw": yaw},
                "gyro":  {"x": gx,  "y": gy,  "z": gz},
                "accel": {"x": ax,  "y": ay,  "z": az},
            },
            "cov": {
                "ori":   [msg.orientation_covariance[0], msg.orientation_covariance[4], msg.orientation_covariance[8]],
                "gyro":  [msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[8]],
                "accel": [msg.linear_acceleration_covariance[0], msg.linear_acceleration_covariance[4], msg.linear_acceleration_covariance[8]],
            },
        }

        with self._lock:
            self._data.clear()
            self._data.update(payload)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    shared_data: dict = {}
    lock = threading.Lock()

    # Spin ROS 2 in the background
    rclpy.init(args=args)
    node = ImuTestNode(shared_data, lock)

    def ros_spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Start Tkinter on the main thread
    try:
        app = ImuGuiApp(shared_data, lock)
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
