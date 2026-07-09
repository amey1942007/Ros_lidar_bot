#!/usr/bin/env python3
"""Small DDSM115 serial CLI used to test motor feedback outside ROS."""

import argparse
import struct
import threading
import time

import serial


def print_info(text):
    print(f"DDSM115_INFO | {text}")


def print_warning(text):
    print(f"DDSM115_WARNING | {text}")


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


def make_packet(payload_9bytes) -> bytes:
    packet = bytearray(payload_9bytes)
    packet.append(crc8_maxim(packet))
    return bytes(packet)


def int16_to_bytes(value: int):
    value = int(max(min(value, 32767), -32768))
    return [(value & 0xFF00) >> 8, value & 0x00FF]


def two_bytes_to_int16(high_byte: int, low_byte: int) -> int:
    value = ((high_byte & 0xFF) << 8) | (low_byte & 0xFF)
    if value > 32767:
        value -= 65536
    return value


def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def current_raw_to_amps(cur_raw: int) -> float:
    return map_value(cur_raw, -32767, 32767, -8.0, 8.0)


def raw_to_degrees(raw_int: int) -> float:
    return (raw_int / 255.0) * 360.0


class MotorControl:
    def __init__(self, device="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(device, baudrate, timeout=0.01)
        self.lock = threading.Lock()
        self.monitor_running = False
        self.monitor_thread = None
        self.prev_fb_rpm = [0.0] * 16
        self.prev_fb_cur = [0.0] * 16
        self.prev_fb_pos = [0.0] * 16

    def close(self):
        self.stop_monitoring()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _cache_index(self, motor_id: int) -> int:
        return max(0, min(len(self.prev_fb_rpm) - 1, motor_id - 1))

    def _write(self, packet: bytes):
        while not self.ser.writable():
            print_warning("serial port not writable")
            time.sleep(0.01)
        self.ser.write(packet)
        self.ser.flush()

    def send_rpm(self, motor_id: int, rpm):
        rpm_hi, rpm_lo = int16_to_bytes(int(rpm))
        packet = make_packet([
            motor_id,
            0x64,
            rpm_hi,
            rpm_lo,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ])
        with self.lock:
            self._write(packet)
            fb_rpm, fb_cur, fb_pos, error = self.read_reply(motor_id)
        print_info(f"ID {motor_id} | RPM: {fb_rpm}, Current: {fb_cur:.3f}A, Position: {fb_pos:.2f}, Error: {error}")

    def set_brake(self, motor_id: int):
        packet = make_packet([
            motor_id,
            0x64,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0xFF,
            0x00,
        ])
        with self.lock:
            self._write(packet)
            self.read_reply(motor_id)

    def set_drive_mode(self, motor_id: int, mode: int):
        if mode not in (1, 2, 3):
            raise ValueError("mode must be 1=current, 2=velocity, or 3=position")
        packet = bytes([
            motor_id,
            0xA0,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            mode,
        ])
        with self.lock:
            self._write(packet)
        print_info(f"Set ID {motor_id} mode to {mode}")

    def reset_motor_state(self, motor_id: int):
        print_info(f"Braking ID {motor_id}")
        self.set_brake(motor_id)
        time.sleep(0.1)
        print_info(f"Setting ID {motor_id} to velocity mode")
        self.set_drive_mode(motor_id, 2)

    def get_motor_feedback(self, motor_id: int):
        packet = make_packet([
            motor_id,
            0x74,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ])
        with self.lock:
            self._write(packet)
            fb_rpm, fb_cur, fb_pos, error = self.read_reply(motor_id)

        if error:
            print_warning(f"ID {motor_id} error byte: {error}")

        return fb_rpm, fb_cur, fb_pos

    def read_reply(self, motor_id: int, timeout=0.02):
        ring_buffer = bytearray()
        start_time = time.monotonic()

        while (time.monotonic() - start_time) <= timeout:
            try:
                data = self.ser.read(1)
            except serial.SerialException as exc:
                print_warning(exc)
                break

            if not data:
                continue

            byte = data[0]
            if len(ring_buffer) == 0:
                if byte == (motor_id & 0xFF):
                    ring_buffer.append(byte)
                continue

            if len(ring_buffer) == 1:
                if byte in (0x01, 0x02, 0x03):
                    ring_buffer.append(byte)
                else:
                    ring_buffer.clear()
                continue

            ring_buffer.append(byte)
            if len(ring_buffer) < 10:
                continue

            if crc8_maxim(ring_buffer[:-1]) != ring_buffer[9]:
                print_warning("crc error")
                ring_buffer.clear()
                continue

            cur_raw = two_bytes_to_int16(ring_buffer[2], ring_buffer[3])
            rpm = two_bytes_to_int16(ring_buffer[4], ring_buffer[5])
            pos = raw_to_degrees(ring_buffer[7])
            error = ring_buffer[8]

            idx = self._cache_index(motor_id)
            self.prev_fb_rpm[idx] = float(rpm)
            self.prev_fb_cur[idx] = current_raw_to_amps(cur_raw)
            self.prev_fb_pos[idx] = pos
            return self.prev_fb_rpm[idx], self.prev_fb_cur[idx], self.prev_fb_pos[idx], error

        idx = self._cache_index(motor_id)
        return self.prev_fb_rpm[idx], self.prev_fb_cur[idx], self.prev_fb_pos[idx], 0

    def start_monitoring(self, motor_ids):
        if self.monitor_running:
            print_warning("monitor already running")
            return
        self.monitor_running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, args=(motor_ids,), daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        if not self.monitor_running:
            return
        self.monitor_running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        self.monitor_thread = None

    def _monitor_loop(self, motor_ids):
        while self.monitor_running:
            for motor_id in motor_ids:
                fb_rpm, fb_cur, fb_pos = self.get_motor_feedback(motor_id)
                print_info(f"ID {motor_id} | RPM: {fb_rpm}, Current: {fb_cur:.3f}A, Position: {fb_pos:.2f}")
            time.sleep(0.033)


def main():
    parser = argparse.ArgumentParser(description="DDSM115 motor feedback/control CLI")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USB serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    args = parser.parse_args()

    motor = MotorControl(device=args.port, baudrate=args.baud)
    print_info("Commands: feedback, monitor, stop_monitor, rpm, brake, mode, reset, q")

    try:
        while True:
            cmd = input("Enter command: ").strip()
            if cmd == "q":
                break
            if cmd == "feedback":
                motor_id = int(input("Motor ID: "))
                fb_rpm, fb_cur, fb_pos = motor.get_motor_feedback(motor_id)
                print_info(f"ID {motor_id} | RPM: {fb_rpm}, Current: {fb_cur:.3f}A, Position: {fb_pos:.2f}")
            elif cmd == "monitor":
                ids = input("Motor IDs separated by space: ")
                motor.start_monitoring([int(value) for value in ids.split()])
            elif cmd == "stop_monitor":
                motor.stop_monitoring()
            elif cmd == "rpm":
                motor_id = int(input("Motor ID: "))
                rpm = float(input("RPM: "))
                motor.send_rpm(motor_id, rpm)
            elif cmd == "brake":
                motor.set_brake(int(input("Motor ID: ")))
            elif cmd == "mode":
                motor_id = int(input("Motor ID: "))
                mode = int(input("Mode 1=current, 2=velocity, 3=position: "))
                motor.set_drive_mode(motor_id, mode)
            elif cmd == "reset":
                motor.reset_motor_state(int(input("Motor ID: ")))
            else:
                print_warning("unknown command")
    finally:
        motor.close()


if __name__ == "__main__":
    main()
