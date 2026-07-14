#!/usr/bin/env python3
"""
imu_test_node.py — IMU Test Node + Built-in GUI Visualiser

Subscribes : /imu  (sensor_msgs/Imu)  from BNO055 via imu_node.py
Opens      : A Tkinter GUI window automatically on launch showing:
               • 3-D rotating axis widgets (IMU frame vs corrected Robot frame)
               • Live numeric readouts — Euler angles, quaternion, gyro, accel
               • Coordinate frame correction indicator

Coordinate-frame correction (IMU → robot frame):
    robot_x =  +imu_x   (same axis)
    robot_y =  -imu_y   (bot -Y  <->  IMU +Y)
    robot_z =  -imu_z   (bot -Z  <->  IMU +Z)

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


def correct_quaternion(qx, qy, qz, qw):
    """Flip Y and Z axes: robot frame = IMU frame with -Y, -Z."""
    cx, cy, cz, cw = qx, -qy, -qz, qw
    n = math.sqrt(cx*cx + cy*cy + cz*cz + cw*cw)
    if n > 1e-6:
        cx /= n; cy /= n; cz /= n; cw /= n
    return cx, cy, cz, cw


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
    """Simple perspective projection → canvas (px, py)."""
    d = persp / (persp + z)
    return cx + x*scale*d, cy - y*scale*d


# ═══════════════════════════════════════════════════════════════════════════════
#  3-D AXIS CANVAS WIDGET
# ═══════════════════════════════════════════════════════════════════════════════
CUBE_VERTS = [
    (-0.5,-0.5,-0.5),( 0.5,-0.5,-0.5),( 0.5, 0.5,-0.5),(-0.5, 0.5,-0.5),
    (-0.5,-0.5, 0.5),( 0.5,-0.5, 0.5),( 0.5, 0.5, 0.5),(-0.5, 0.5, 0.5),
]
CUBE_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

class Axis3DCanvas(tk.Canvas):
    def __init__(self, parent, label="", size=265, **kw):
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
        sc = 78

        # Outer ring
        r = self._size // 2 - 6
        self.create_oval(6, 6, self._size-6, self._size-6,
                         outline=BORDER, width=1)

        # Cube wireframe
        rotated   = [mat_vec(m, v) for v in CUBE_VERTS]
        projected = [project(v[0], v[1], v[2], cx, cy, scale=sc*0.68) for v in rotated]
        for i, j in CUBE_EDGES:
            z_avg = (rotated[i][2] + rotated[j][2]) / 2
            bright = int(70 + 110 * (z_avg + 0.7) / 1.4)
            bright = max(55, min(185, bright))
            col = f"#{bright:02x}{bright:02x}{bright:02x}"
            x1, y1 = projected[i]; x2, y2 = projected[j]
            self.create_line(x1, y1, x2, y2, fill=col, width=1, dash=(3,2))

        # X / Y / Z axis arrows
        ox, oy = project(0, 0, 0, cx, cy, scale=sc)
        for vec, color, name in (
            ([1,0,0], ACCENT_X, "X"),
            ([0,1,0], ACCENT_Y, "Y"),
            ([0,0,1], ACCENT_Z, "Z"),
        ):
            rv = mat_vec(m, vec)
            tx, ty = project(*rv, cx, cy, scale=sc)
            self.create_line(ox, oy, tx, ty, fill=color, width=2,
                             arrow=tk.LAST, arrowshape=(9,11,4))
            lx = tx + (tx-ox)*0.18
            ly = ty + (ty-oy)*0.18
            self.create_text(lx, ly, text=name, fill=color,
                             font=("Consolas", 9, "bold"))

        # Origin dot
        r2 = 4
        self.create_oval(ox-r2, oy-r2, ox+r2, oy+r2,
                         fill=FG_DIM, outline=FG_MAIN, width=1)

        # Frame label
        self.create_text(cx, 13, text=self._label, fill=FG_DIM,
                         font=("Consolas", 8, "bold"))

        # Euler readout
        roll, pitch, yaw = self._euler
        for i, (txt, col) in enumerate([
            (f"R: {roll:+7.1f}°",  ACCENT_X),
            (f"P: {pitch:+7.1f}°", ACCENT_Y),
            (f"Y: {yaw:+7.1f}°",   ACCENT_Z),
        ]):
            self.create_text(cx, self._size-40+i*13, text=txt,
                             fill=col, font=("Consolas", 8))


# ═══════════════════════════════════════════════════════════════════════════════
#  VALUE ROW WIDGET (label + value + mini bar)
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
        self._bar = tk.Canvas(self, width=65, height=7,
                              bg=BG_CARD, highlightthickness=0)
        self._bar.pack(side=tk.LEFT, padx=4)
        self._color = color

    def set(self, val, vmin=-180, vmax=180):
        self._val.config(text=f"{val:+11.4f}")
        frac = max(0.0, min(1.0, (val-vmin)/(vmax-vmin) if vmax!=vmin else 0.5))
        self._bar.delete("all")
        self._bar.create_rectangle(0,0,65,7, fill=BG_HOVER, outline="")
        self._bar.create_rectangle(0,0,int(frac*65),7, fill=self._color, outline="")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN GUI APPLICATION
# ═══════════════════════════════════════════════════════════════════════════════
class ImuGuiApp(tk.Tk):
    POLL_MS = 50   # 20 Hz GUI refresh

    def __init__(self, data_ref: dict, lock: threading.Lock):
        super().__init__()
        self._data = data_ref
        self._lock = lock

        self.title("IMU Test Node — BNO055 Visualiser")
        self.configure(bg=BG_DARK)
        self.minsize(980, 620)
        self.resizable(True, True)

        self._build_ui()
        self.after(self.POLL_MS, self._refresh)

    # ── Build UI ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Top bar
        top = tk.Frame(self, bg=BG_PANEL, pady=7)
        top.pack(fill=tk.X)
        tk.Label(top, text="⬡  IMU TEST NODE  —  BNO055 Visualiser",
                 fg=ACCENT_X, bg=BG_PANEL, font=("Consolas",13,"bold")
                 ).pack(side=tk.LEFT, padx=14)
        tk.Label(top, text="Subscribing: /imu", fg=FG_DIM, bg=BG_PANEL,
                 font=("Consolas",9)).pack(side=tk.LEFT)
        self._dot = tk.Label(top, text="●", fg="#f85149", bg=BG_PANEL,
                             font=("Consolas",12))
        self._dot.pack(side=tk.RIGHT, padx=4)
        self._slbl = tk.Label(top, text="Waiting for /imu…", fg=FG_WARN,
                              bg=BG_PANEL, font=("Consolas",9))
        self._slbl.pack(side=tk.RIGHT, padx=(0,2))

        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X)

        # Body
        body = tk.Frame(self, bg=BG_DARK)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        # ── Left: 3-D canvases ────────────────────────────────────────────
        left = tk.Frame(body, bg=BG_DARK)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0,10))

        tk.Label(left, text="RAW  IMU  FRAME", fg=ACCENT_X, bg=BG_DARK,
                 font=("Consolas",9,"bold")).pack()
        self._cv_imu = Axis3DCanvas(left, label="IMU Frame", size=268)
        self._cv_imu.pack(pady=(2,8))

        tk.Label(left, text="CORRECTED  ROBOT  FRAME", fg=ACCENT_Y, bg=BG_DARK,
                 font=("Consolas",9,"bold")).pack()
        self._cv_bot = Axis3DCanvas(left, label="Robot Frame", size=268)
        self._cv_bot.pack(pady=(2,0))

        # ── Right: numeric panels ─────────────────────────────────────────
        right = tk.Frame(body, bg=BG_DARK)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Coordinate legend
        leg = tk.Frame(right, bg=BG_CARD)
        leg.pack(fill=tk.X, pady=(0,8))
        tk.Label(leg, text="Frame Correction  ( IMU → Robot )", fg=ACCENT_W,
                 bg=BG_CARD, font=("Consolas",9,"bold"), anchor="w"
                 ).pack(fill=tk.X, padx=10, pady=(6,2))
        for txt, col in [
            ("  Robot X  =  +IMU X   (unchanged)", ACCENT_X),
            ("  Robot Y  =  −IMU Y   (negated)",   ACCENT_Y),
            ("  Robot Z  =  −IMU Z   (negated)",   ACCENT_Z),
        ]:
            tk.Label(leg, text=txt, fg=col, bg=BG_CARD,
                     font=("Consolas",8), anchor="w"
                     ).pack(fill=tk.X, padx=10, pady=1)
        tk.Frame(leg, bg=BG_CARD, height=6).pack()

        # Notebook
        style = ttk.Style(self)
        style.theme_use("default")
        style.configure("TNotebook",      background=BG_DARK,  borderwidth=0)
        style.configure("TNotebook.Tab",  background=BG_PANEL, foreground=FG_DIM,
                        font=("Consolas",8,"bold"), padding=[10,4])
        style.map("TNotebook.Tab",
                  background=[("selected", BG_CARD)],
                  foreground=[("selected", FG_MAIN)])

        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        # Tab: Euler
        t_euler = tk.Frame(nb, bg=BG_CARD); nb.add(t_euler, text=" Euler (°) ")
        self._euler_imu   = self._make_xyz_section(t_euler, "Raw IMU Euler  (deg)")
        self._euler_robot = self._make_xyz_section(t_euler, "Robot Euler    (deg)")

        # Tab: Quaternion
        t_quat = tk.Frame(nb, bg=BG_CARD); nb.add(t_quat, text=" Quaternion ")
        self._quat_imu   = self._make_xyzw_section(t_quat, "Raw IMU Quaternion")
        self._quat_robot = self._make_xyzw_section(t_quat, "Robot Quaternion")

        # Tab: Gyro
        t_gyro = tk.Frame(nb, bg=BG_CARD); nb.add(t_gyro, text=" Gyro (rad/s) ")
        self._gyro_imu   = self._make_xyz_section(t_gyro, "Raw IMU Angular Vel (rad/s)",  unit="rad/s", vrange=(-10, 10))
        self._gyro_robot = self._make_xyz_section(t_gyro, "Robot Angular Vel   (rad/s)",  unit="rad/s", vrange=(-10, 10))

        # Tab: Accel
        t_accel = tk.Frame(nb, bg=BG_CARD); nb.add(t_accel, text=" Accel (m/s²) ")
        self._accel_imu   = self._make_xyz_section(t_accel, "Raw IMU Linear Accel (m/s²)", unit="m/s²", vrange=(-20, 20))
        self._accel_robot = self._make_xyz_section(t_accel, "Robot Linear Accel   (m/s²)", unit="m/s²", vrange=(-20, 20))

        # Tab: Covariance
        t_cov = tk.Frame(nb, bg=BG_CARD); nb.add(t_cov, text=" Covariance ")
        self._cov_ori   = self._make_xyz_section(t_cov, "Orientation Cov σ² (diag)", labels=["XX","YY","ZZ"], vrange=(-0.05, 0.05))
        self._cov_gyro  = self._make_xyz_section(t_cov, "Gyro Cov σ²        (diag)", labels=["XX","YY","ZZ"], vrange=(-0.01, 0.01))
        self._cov_accel = self._make_xyz_section(t_cov, "Accel Cov σ²       (diag)", labels=["XX","YY","ZZ"], vrange=(-0.1,  0.1))

        # Status bar
        bot = tk.Frame(self, bg=BG_PANEL, pady=4)
        bot.pack(fill=tk.X, side=tk.BOTTOM)
        self._hz_lbl = tk.Label(bot, text="Rate: -- Hz", fg=FG_DIM, bg=BG_PANEL,
                                font=("Consolas",8))
        self._hz_lbl.pack(side=tk.LEFT, padx=10)
        tk.Label(bot, text="ros2 run Ros_lidar_bot imu_test_node",
                 fg=FG_DIM, bg=BG_PANEL, font=("Consolas",8)).pack(side=tk.RIGHT, padx=10)

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
                 font=("Consolas",8,"bold"), anchor="w"
                 ).pack(fill=tk.X, padx=8, pady=(8,2))

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _set_rows(self, rows_dict, values_dict):
        for key, (row, vrange) in rows_dict.items():
            val = values_dict.get(key, 0.0)
            row.set(val, *vrange)

    # ── Refresh loop ─────────────────────────────────────────────────────────
    def _refresh(self):
        try:
            with self._lock:
                d = dict(self._data)

            has_data = bool(d)
            if has_data:
                self._dot.config(fg="#3fb950")
                self._slbl.config(text=f"Live  ✓", fg="#3fb950")
            else:
                self._dot.config(fg="#f0883e")
                self._slbl.config(text="Waiting for /imu…", fg=FG_WARN)

            if not has_data:
                return

            imu   = d.get("imu", {})
            robot = d.get("robot", {})
            cov   = d.get("cov", {})

            # 3-D canvases
            iq = imu.get("quat", {}); ie = imu.get("euler", {})
            self._cv_imu.set_orientation(
                iq.get("x",0), iq.get("y",0), iq.get("z",0), iq.get("w",1),
                ie.get("roll",0), ie.get("pitch",0), ie.get("yaw",0))

            rq = robot.get("quat", {}); re = robot.get("euler", {})
            self._cv_bot.set_orientation(
                rq.get("x",0), rq.get("y",0), rq.get("z",0), rq.get("w",1),
                re.get("roll",0), re.get("pitch",0), re.get("yaw",0))

            # Euler
            self._set_rows(self._euler_imu,   ie)
            self._set_rows(self._euler_robot,  re)

            # Quaternion
            self._set_rows(self._quat_imu,   iq)
            self._set_rows(self._quat_robot,  rq)

            # Gyro
            ig = imu.get("gyro",  {}); rg = robot.get("gyro", {})
            self._set_rows(self._gyro_imu,   ig)
            self._set_rows(self._gyro_robot,  rg)

            # Accel
            ia = imu.get("accel", {}); ra = robot.get("accel", {})
            self._set_rows(self._accel_imu,   ia)
            self._set_rows(self._accel_robot,  ra)

            # Covariance
            oc = cov.get("ori",   [0,0,0])
            gc = cov.get("gyro",  [0,0,0])
            ac = cov.get("accel", [0,0,0])
            self._set_rows(self._cov_ori,   {"xx":oc[0],"yy":oc[1],"zz":oc[2]})
            self._set_rows(self._cov_gyro,  {"xx":gc[0],"yy":gc[1],"zz":gc[2]})
            self._set_rows(self._cov_accel, {"xx":ac[0],"yy":ac[1],"zz":ac[2]})

            # Hz
            hz = d.get("_hz", 0.0)
            self._hz_lbl.config(text=f"Rate: {hz:.1f} Hz")

        except Exception:
            pass
        finally:
            self.after(self.POLL_MS, self._refresh)


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 NODE
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
        self.get_logger().info(
            f"ImuTestNode ready — subscribing '{topic}' | GUI launched in main thread")

    def _cb(self, msg: Imu):
        self._count += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = now - self._t0
        hz  = self._count / dt if dt > 0 else 0.0

        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w)
        roll_i, pitch_i, yaw_i = quat_to_euler_deg(qx, qy, qz, qw)

        gx, gy, gz = (msg.angular_velocity.x,
                      msg.angular_velocity.y, msg.angular_velocity.z)
        ax, ay, az = (msg.linear_acceleration.x,
                      msg.linear_acceleration.y, msg.linear_acceleration.z)

        # Corrected robot frame
        qxc, qyc, qzc, qwc = correct_quaternion(qx, qy, qz, qw)
        roll_r, pitch_r, yaw_r = quat_to_euler_deg(qxc, qyc, qzc, qwc)

        payload = {
            "_hz": round(hz, 2),
            "imu": {
                "quat":  {"x": qx,  "y": qy,  "z": qz,  "w": qw},
                "euler": {"x": roll_i, "y": pitch_i, "z": yaw_i,
                          "roll": roll_i, "pitch": pitch_i, "yaw": yaw_i},
                "gyro":  {"x": gx,  "y": gy,  "z": gz},
                "accel": {"x": ax,  "y": ay,  "z": az},
            },
            "robot": {
                "quat":  {"x": qxc, "y": qyc, "z": qzc, "w": qwc},
                "euler": {"x": roll_r, "y": pitch_r, "z": yaw_r,
                          "roll": roll_r, "pitch": pitch_r, "yaw": yaw_r},
                "gyro":  {"x": gx,  "y": -gy, "z": -gz},
                "accel": {"x": ax,  "y": -ay, "z": -az},
            },
            "cov": {
                "ori":   [msg.orientation_covariance[0],
                          msg.orientation_covariance[4],
                          msg.orientation_covariance[8]],
                "gyro":  [msg.angular_velocity_covariance[0],
                          msg.angular_velocity_covariance[4],
                          msg.angular_velocity_covariance[8]],
                "accel": [msg.linear_acceleration_covariance[0],
                          msg.linear_acceleration_covariance[4],
                          msg.linear_acceleration_covariance[8]],
            },
        }

        with self._lock:
            self._data.clear()
            self._data.update(payload)


# ═══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    # Shared state between ROS2 thread and GUI thread
    shared_data: dict = {}
    lock = threading.Lock()

    # ── Start ROS2 in a background thread ────────────────────────────────────
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

    # ── Run GUI on main thread (Tkinter requirement) ──────────────────────────
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
