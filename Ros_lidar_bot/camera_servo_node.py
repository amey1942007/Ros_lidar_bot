#!/usr/bin/env python3
"""
camera_servo_node.py — Pan/tilt camera head driver.

Drives a two-servo camera mount, controlled from the gamepad's RIGHT stick
(joy_teleop publishes normalised rates on /camera_cmd):

  OT5320M (base / PAN)  — 20 kg hobby PWM servo (JR plug, 4-8.4 V, ~180°)
                          on an RPi GPIO pin. This is the default.
  SG90    (head / TILT) — hobby PWM micro servo on an RPi GPIO pin.

The pan axis is pluggable via the `pan_driver` parameter:
  "pwm" (default) — any standard hobby servo (OT5320M, MG996R, …) on a GPIO.
  "bus"           — a Feetech STS/SMS serial BUS servo (ST3215 and friends)
                    on a TTL bus adapter. NOTE: a bus adapter cannot drive a
                    PWM servo like the OT5320M — the two are different
                    protocols, not different connectors.

This node is a PARALLEL control path — it never touches /cmd_vel, Nav2 or
frontier exploration. It only:
  • subscribes  /camera_cmd   (geometry_msgs/Twist: angular.z = pan rate,
                               angular.y = tilt rate, each normalised -1..1)
  • integrates those rates into pan/tilt angle setpoints (rate control, so a
    centred stick HOLDS the current heading), clamped to the mechanical limits
  • writes the setpoints to the two servos every tick
  • publishes /camera_joint_states (sensor_msgs/JointState) so RViz/TF and the
    semantic-marker placement know where the camera actually points. This topic
    is merged by joint_state_publisher (see rsp.launch.py source_list).

HARDWARE ASSUMPTIONS (override via parameters if your setup differs):
  • Both PWM axes use gpiozero.AngularServo (lgpio backend, works on the RPi5).
    Pan defaults to GPIO13 (PWM1, header pin 33), tilt to GPIO12 (PWM0, pin
    32) — the RPi5's two hardware-PWM channels, sharing the ground on pin 34.
    gpiozero drives them with SOFTWARE PWM regardless; the pin choice keeps
    the door open to `rpi-hardware-pwm` if the big servo jitters at rest.
  • The OT5320M's usable travel is ~±85° — hence the tighter default pan limits
    when pan_driver == "pwm" (the bus path keeps the old ±150°).
  • pan_driver == "bus" needs `pip3 install feetech-servo-sdk` (module:
    scservo_sdk) and an STS/SMS-class servo with a 0..4095 tick range over
    360°. For an older SCS/SCSCL-class servo (0..1023) the handler must switch
    from sms_sts to scscl.

Both hardware layers degrade gracefully: if the library or device is missing
the node still runs, publishes joint states, and logs the reason once — so the
rest of the robot is never blocked by a missing servo.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Feetech bus-servo SDK (SC-15 pan). Optional — see module docstring.
try:
    from scservo_sdk import PortHandler, sms_sts
    _SCS_OK = True
except Exception:                       # ImportError or lib load failure
    _SCS_OK = False

# gpiozero PWM servo. Optional. SOFTWARE PWM — the pulse edges are timed by a
# Python thread, so under heavy CPU load (SLAM + Nav2 + YOLO) they land late
# and the servo hunts. Fine on an idle Pi, not on a working robot.
try:
    from gpiozero import AngularServo
    _GPIO_OK = True
except Exception:
    _GPIO_OK = False

# rpi-hardware-pwm. Optional but STRONGLY preferred: the RP1 generates the
# pulses in silicon, so they stay exact no matter what the CPU is doing.
# Needs `pip3 install rpi-hardware-pwm --break-system-packages` AND this line
# in /boot/firmware/config.txt followed by a reboot:
#     dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
try:
    from rpi_hardware_pwm import HardwarePWM
    _HWPWM_OK = True
except Exception:
    _HWPWM_OK = False


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class _PanBus:
    """SC-15 pan via a Feetech STS/SMS bus servo (position control)."""

    def __init__(self, node, port, baud, servo_id, ticks_per_rev,
                 center_tick, speed, acc):
        self._log = node.get_logger()
        self._id = int(servo_id)
        self._tpr = int(ticks_per_rev)
        self._center = int(center_tick)
        self._speed = int(speed)
        self._acc = int(acc)
        self._packet = None
        self._port = None
        if not _SCS_OK:
            self._log.warn("scservo_sdk not installed — pan servo DISABLED "
                           "(pip3 install feetech-servo-sdk). Joint states "
                           "still published.")
            return
        try:
            self._port = PortHandler(port)
            self._packet = sms_sts(self._port)
            if not self._port.openPort():
                raise RuntimeError(f"cannot open {port}")
            if not self._port.setBaudRate(int(baud)):
                raise RuntimeError(f"cannot set baud {baud}")
            self._log.info(f"Pan bus servo ready: id={self._id} on {port}@{baud}")
        except Exception as exc:
            self._log.error(f"pan bus init failed ({exc}) — pan DISABLED")
            self._packet = None

    @property
    def enabled(self):
        return self._packet is not None

    @property
    def backend(self):
        return "bus"

    def write_deg(self, deg):
        """deg is relative to centre (0 = camera forward)."""
        if self._packet is None:
            return
        tick = int(round(self._center + (deg / 360.0) * self._tpr))
        tick = _clamp(tick, 0, self._tpr - 1)
        try:
            self._packet.WritePosEx(self._id, tick, self._speed, self._acc)
        except Exception as exc:
            self._log.warn(f"pan write failed: {exc}",
                           throttle_duration_sec=2.0)

    def close(self):
        if self._port is not None:
            try:
                self._port.closePort()
            except Exception:
                pass


class _PwmServo:
    """A standard hobby PWM servo (OT5320M, SG90, …) via gpiozero."""

    def __init__(self, node, label, pin, min_deg, max_deg,
                 min_pulse_us, max_pulse_us, idle_release_sec=1.5):
        self._log = node.get_logger()
        self._label = label
        self._servo = None
        # Rewriting an unchanged angle every tick keeps the servo actively
        # driving and, with software PWM, re-jitters the pulse — the head
        # hunts and the supply browns out. Write only on a real change, then
        # let the servo go limp once it has had time to arrive.
        self._last_deg = None
        self._idle_release = float(idle_release_sec)
        self._idle_since = None
        if not _GPIO_OK:
            self._log.warn(f"gpiozero not available — {label} servo DISABLED. "
                           "Joint states still published.")
            return
        try:
            self._servo = AngularServo(
                int(pin),
                min_angle=float(min_deg), max_angle=float(max_deg),
                min_pulse_width=float(min_pulse_us) / 1e6,
                max_pulse_width=float(max_pulse_us) / 1e6,
            )
            self._log.warn(
                f"{label}: using SOFTWARE PWM on GPIO{int(pin)}. The pulse is "
                "timed by the CPU, so the servo will hunt once SLAM/Nav2 load "
                "the cores. See the rpi-hardware-pwm notes at the top of "
                "camera_servo_node.py.")
        except Exception as exc:
            self._log.error(f"{label} PWM init failed ({exc}) — {label} DISABLED")
            self._servo = None

    @property
    def enabled(self):
        return self._servo is not None

    @property
    def backend(self):
        return "software"

    def write_deg(self, deg):
        if self._servo is None:
            return
        deg = float(deg)
        try:
            # Unchanged setpoint: hold briefly so the servo reaches it, then
            # detach. A detached servo stops drawing holding current, which is
            # what keeps the supply out of over-current protection.
            if self._last_deg is not None and abs(deg - self._last_deg) < 0.25:
                if (self._idle_release > 0.0 and self._idle_since is not None
                        and time.monotonic() - self._idle_since
                        >= self._idle_release):
                    self._servo.detach()
                    self._idle_since = None
                return
            self._servo.angle = deg
            self._last_deg = deg
            self._idle_since = time.monotonic()
        except Exception as exc:
            self._log.warn(f"{self._label} write failed: {exc}",
                           throttle_duration_sec=2.0)

    def close(self):
        if self._servo is not None:
            try:
                self._servo.detach()
            except Exception:
                pass


class _HwPwmServo:
    """Same servo, driven by the RP1's hardware PWM instead of the CPU.

    Immune to scheduling jitter, so the head holds still while SLAM and Nav2
    are saturating the cores. Requires the pwm-2chan overlay (see the import
    block at the top of this file); without it, construction fails and the
    caller falls back to _PwmServo.
    """

    # Channel order follows `dtoverlay=pwm-2chan,pin=12,...,pin2=13,...`.
    _CHANNEL = {12: 0, 13: 1, 18: 0, 19: 1}
    _PERIOD_US = 20000.0                    # 50 Hz servo frame

    def __init__(self, node, label, pin, min_deg, max_deg,
                 min_pulse_us, max_pulse_us, idle_release_sec=1.5, chip=2):
        self._log = node.get_logger()
        self._label = label
        self._pwm = None
        self._min_deg, self._max_deg = float(min_deg), float(max_deg)
        self._min_us, self._max_us = float(min_pulse_us), float(max_pulse_us)
        self._last_deg = None
        self._idle_release = float(idle_release_sec)
        self._idle_since = None
        if not _HWPWM_OK:
            self._log.warn(
                f"{label}: rpi-hardware-pwm not installed — falling back to "
                "software PWM (pip3 install rpi-hardware-pwm "
                "--break-system-packages)")
            return
        channel = self._CHANNEL.get(int(pin))
        if channel is None:
            self._log.warn(f"GPIO{int(pin)} is not a hardware-PWM pin — "
                           f"{label} falls back to software PWM")
            return
        try:
            self._pwm = HardwarePWM(pwm_channel=channel,
                                    hz=int(1e6 / self._PERIOD_US),
                                    chip=int(chip))
            self._pwm.start(0.0)            # 0% = no pulse = servo released
            self._log.info(f"{label} HARDWARE PWM on GPIO{int(pin)} "
                           f"(chip {chip} channel {channel})")
        except Exception as exc:
            self._log.warn(f"{label} hardware PWM unavailable ({exc}) — "
                           "falling back to software PWM")
            self._pwm = None

    @property
    def enabled(self):
        return self._pwm is not None

    @property
    def backend(self):
        return "hardware"

    def _duty(self, deg):
        span = self._max_deg - self._min_deg
        frac = 0.5 if span == 0 else (deg - self._min_deg) / span
        us = self._min_us + _clamp(frac, 0.0, 1.0) * (self._max_us - self._min_us)
        return us / self._PERIOD_US * 100.0

    def write_deg(self, deg):
        if self._pwm is None:
            return
        deg = _clamp(float(deg), self._min_deg, self._max_deg)
        try:
            if self._last_deg is not None and abs(deg - self._last_deg) < 0.25:
                if (self._idle_release > 0.0 and self._idle_since is not None
                        and time.monotonic() - self._idle_since
                        >= self._idle_release):
                    self._pwm.change_duty_cycle(0.0)
                    self._idle_since = None
                return
            self._pwm.change_duty_cycle(self._duty(deg))
            self._last_deg = deg
            self._idle_since = time.monotonic()
        except Exception as exc:
            self._log.warn(f"{self._label} write failed: {exc}",
                           throttle_duration_sec=2.0)

    def close(self):
        if self._pwm is not None:
            try:
                self._pwm.stop()
            except Exception:
                pass


def _make_pwm_servo(node, backend, label, pin, min_deg, max_deg,
                    min_us, max_us, idle_release, chip):
    """Hardware PWM if we can get it, software PWM if we can't."""
    if backend in ("auto", "hardware"):
        servo = _HwPwmServo(node, label, pin, min_deg, max_deg,
                            min_us, max_us, idle_release, chip)
        if servo.enabled:
            return servo
        if backend == "hardware":
            node.get_logger().error(
                f"{label}: pwm_backend=hardware was requested but is "
                "unavailable — check the pwm-2chan overlay in "
                "/boot/firmware/config.txt, then reboot.")
    return _PwmServo(node, label, pin, min_deg, max_deg,
                     min_us, max_us, idle_release)


class CameraServo(Node):
    def __init__(self):
        super().__init__("camera_servo")

        d = self.declare_parameter
        # ── Topics / joints ─────────────────────────────────────────────────
        self._cmd_topic = d("cmd_topic", "/camera_cmd").value
        self._js_topic = d("joint_state_topic", "/camera_joint_states").value
        self._pan_joint = d("pan_joint_name", "camera_pan_joint").value
        self._tilt_joint = d("tilt_joint_name", "camera_tilt_joint").value
        self._rate_hz = float(d("rate_hz", 30.0).value)
        # "auto" prefers the RP1's hardware PWM and silently falls back to
        # gpiozero; "hardware" or "gpiozero" force one of them.
        pwm_backend = str(d("pwm_backend", "auto").value).lower()
        pwm_chip = int(d("pwm_chip", 2).value)      # RPi5 = 2, RPi4 = 0

        # ── Pan ─────────────────────────────────────────────────────────────
        # "pwm" = hobby servo on a GPIO (OT5320M); "bus" = Feetech STS/SMS.
        pan_driver = str(d("pan_driver", "pwm").value).lower()
        # A PWM servo only has ~±85° of travel; a bus servo has the full turn.
        pan_limit = 150.0 if pan_driver == "bus" else 85.0
        self._pan_min = float(d("pan_min_deg", -pan_limit).value)
        self._pan_max = float(d("pan_max_deg", pan_limit).value)
        self._pan_rate = float(d("pan_max_rate_dps", 90.0).value)
        self._pan_inv = bool(d("invert_pan", False).value)
        pan_pin = int(d("pan_pwm_pin", 13).value)
        # 1000-2000 us is the standard range every hobby servo accepts. Wider
        # values (500/2500) reach further on servos that support them but drive
        # the rest into their mechanical stops, where they stall, buzz and pull
        # locked-rotor current. Widen only after checking the servo still moves
        # freely at both ends.
        pan_min_us = float(d("pan_min_pulse_us", 1000.0).value)
        pan_max_us = float(d("pan_max_pulse_us", 2000.0).value)
        # Pan turns about a vertical axis, so gravity holds it — safe to
        # release. 0.0 disables the release and holds torque forever.
        pan_idle = float(d("pan_idle_release_sec", 1.5).value)
        bus_port = d("bus_port", "/dev/ttyUSB0").value
        bus_baud = int(d("bus_baud", 1000000).value)
        pan_id = int(d("pan_servo_id", 1).value)
        pan_tpr = int(d("pan_ticks_per_rev", 4096).value)
        pan_center = int(d("pan_center_tick", 2048).value)
        pan_speed = int(d("pan_speed", 2400).value)
        pan_acc = int(d("pan_accel", 50).value)

        # ── Tilt (SG90 PWM servo) ───────────────────────────────────────────
        self._tilt_min = float(d("tilt_min_deg", -80.0).value)
        self._tilt_max = float(d("tilt_max_deg", 80.0).value)
        self._tilt_rate = float(d("tilt_max_rate_dps", 90.0).value)
        self._tilt_inv = bool(d("invert_tilt", False).value)
        tilt_pin = int(d("tilt_pwm_pin", 12).value)
        # SG90's usable range really is 1000-2000 us — see the pan note above.
        tilt_min_us = float(d("tilt_min_pulse_us", 1000.0).value)
        tilt_max_us = float(d("tilt_max_pulse_us", 2000.0).value)
        # Tilt carries the camera against gravity — releasing it makes the head
        # droop, so hold by default. Set >0 if your mount is balanced.
        tilt_idle = float(d("tilt_idle_release_sec", 0.0).value)

        # ── State ───────────────────────────────────────────────────────────
        self._pan_deg = float(_clamp(d("pan_center_deg", 0.0).value,
                                     self._pan_min, self._pan_max))
        self._tilt_deg = float(_clamp(d("tilt_center_deg", 0.0).value,
                                      self._tilt_min, self._tilt_max))
        self._pan_cmd = 0.0     # normalised rate -1..1
        self._tilt_cmd = 0.0
        self._lock = threading.Lock()
        self._last = time.monotonic()

        if pan_driver == "bus":
            self._pan = _PanBus(self, bus_port, bus_baud, pan_id, pan_tpr,
                                pan_center, pan_speed, pan_acc)
        else:
            self._pan = _make_pwm_servo(self, pwm_backend, "pan", pan_pin,
                                        self._pan_min, self._pan_max,
                                        pan_min_us, pan_max_us, pan_idle,
                                        pwm_chip)
        self._tilt = _make_pwm_servo(self, pwm_backend, "tilt", tilt_pin,
                                     self._tilt_min, self._tilt_max,
                                     tilt_min_us, tilt_max_us, tilt_idle,
                                     pwm_chip)

        self.create_subscription(Twist, self._cmd_topic, self._cmd_cb, 10)
        self._js_pub = self.create_publisher(JointState, self._js_topic, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)

        # Drive servos to their start pose immediately.
        self._pan.write_deg(-self._pan_deg if self._pan_inv else self._pan_deg)
        self._tilt.write_deg(-self._tilt_deg if self._tilt_inv else self._tilt_deg)
        self.get_logger().info(
            f"Camera pan/tilt ready — "
            f"pan[{self._pan.backend}] {'ON' if self._pan.enabled else 'off'}, "
            f"tilt[{self._tilt.backend}] {'ON' if self._tilt.enabled else 'off'}. "
            f"Cmd on {self._cmd_topic} (right stick).")

    def _cmd_cb(self, msg: Twist):
        with self._lock:
            self._pan_cmd = _clamp(msg.angular.z, -1.0, 1.0)
            self._tilt_cmd = _clamp(msg.angular.y, -1.0, 1.0)

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        # Guard against a stalled timer producing a huge integration step.
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self._rate_hz

        with self._lock:
            pan_cmd, tilt_cmd = self._pan_cmd, self._tilt_cmd

        self._pan_deg = _clamp(self._pan_deg + pan_cmd * self._pan_rate * dt,
                               self._pan_min, self._pan_max)
        self._tilt_deg = _clamp(self._tilt_deg + tilt_cmd * self._tilt_rate * dt,
                                self._tilt_min, self._tilt_max)

        self._pan.write_deg(-self._pan_deg if self._pan_inv else self._pan_deg)
        self._tilt.write_deg(-self._tilt_deg if self._tilt_inv else self._tilt_deg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._pan_joint, self._tilt_joint]
        js.position = [math.radians(self._pan_deg), math.radians(self._tilt_deg)]
        self._js_pub.publish(js)

    def close(self):
        self._pan.close()
        self._tilt.close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraServo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT from a terminal, SIGTERM from `ros2 launch` shutting down.
        # Both are normal; close() below still releases the servos.
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
