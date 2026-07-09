#!/usr/bin/env python3

import serial
import argparse
import struct
import crcmod.predefined
import numpy as np
import threading
import time
import readline

# ==========================================
# LOGGING HELPERS
# ==========================================
def print_info(text):
    print("DDSM115_INFO | {}".format(text))

def print_warning(text):
    print("DDSM115_WARNING | {}".format(text))

# ==========================================
# MOTOR CONTROL CLASS
# ==========================================

class MotorControl:
    def __init__(self, device="/dev/tty.usbmodem5AAF2797111", baudrate=115200):
        self.ser = serial.Serial(device, baudrate, timeout=0.5)
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
        self.str_10bytes = ">BBBBBBBBBB"
        self.str_9bytes = ">BBBBBBBBB"
        self.lock = threading.Lock()
        self.monitor_running = False
        self.monitor_thread = None

        # Previous feedback values -- used as fallback on read timeouts
        self.prev_fb_rpm = [0, 0, 0, 0, 0, 0, 0, 0]
        self.prev_fb_cur = [0, 0, 0, 0, 0, 0, 0, 0]
        self.prev_fb_pos = [0, 0, 0, 0, 0, 0, 0, 0]

    def close(self):
        self.stop_monitoring()
        self.ser.close()

    # ==========================================
    # BACKGROUND MONITORING
    # ==========================================

    def start_monitoring(self, motor_id):
        """Start a daemon thread that polls feedback for one motor."""
        if self.monitor_running:
            print_warning("Monitor already running")
            return
        self.monitor_running = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, args=(motor_id,)
        )
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print_info(f"Started background monitoring for ID {motor_id}")

    def stop_monitoring(self):
        """Stop the background monitoring thread."""
        if not self.monitor_running:
            print_warning("Monitor not running")
            return
        self.monitor_running = False
        self.monitor_thread.join()
        print_info("Stopped background monitoring")

    def _monitor_loop(self, motor_id):
        """Internal loop that runs in the monitor thread."""
        while self.monitor_running:
            with self.lock:
                fb_rpm, fb_cur, fb_pos = self.get_motor_feedback(motor_id)
                print_info(
                    f"Motor ID {motor_id} | "
                    f"RPM: {fb_rpm}, Current: {fb_cur}A, Position: {fb_pos}"
                )
            time.sleep(0.033)

    # ==========================================
    # MATH HELPERS
    # ==========================================

    def Int16ToBytesArray(self, data: int):
        byte1 = (data & 0xFF00) >> 8
        byte2 = (data & 0x00FF)
        return [byte1, byte2]

    def TwoBytesTo16Int(self, high_byte: int, lo_byte: int):
        int16 = ((high_byte & 0xFF)) << 8 | (lo_byte & 0xFF)
        if int16 > 32767:
            int16 -= 65536
        return int16

    def map(self, val, in_min, in_max, out_min, out_max):
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def crc_attach(self, data_bytes: bytes):
        """Append a CRC-8 Maxim checksum byte to the command."""
        crc_int = self.crc8(data_bytes)
        data_bytesarray = bytearray(data_bytes)
        data_bytesarray.append(crc_int)
        return bytes(data_bytesarray)

    def currentRawToCurrentAmp(self, cur_raw: int):
        return self.map(cur_raw, -32767, 32767, -8.0, 8.0)

    def RawToDegrees(self, raw_int):
        return (raw_int / 255.0) * 360.0

    # ==========================================
    # WRITE COMMANDS
    # ==========================================

    def set_id(self, _id: int):
        """Assign a new ID to a single connected motor (one motor at a time)."""
        SET_ID = struct.pack(
            self.str_10bytes,
            0xAA, 0x55, 0x53, _id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        )
        with self.lock:
            for i in range(5):
                self.ser.write(SET_ID)
                data = self.ser.read_until(size=10)
                print(data)

    def send_rpm(self, _id: int, rpm):
        """Send a velocity command (Mode 2 -- velocity loop)."""
        rpm = int(rpm)
        rpm_ints = self.Int16ToBytesArray(rpm)
        cmd_bytes = struct.pack(
            self.str_9bytes,
            _id, 0x64, rpm_ints[0], rpm_ints[1],
            0x00, 0x00, 0x00, 0x00, 0x00,
        )
        cmd_bytes = self.crc_attach(cmd_bytes)

        with self.lock:
            while not self.ser.writable():
                print_warning("send_rpm not writable")
            self.ser.write(cmd_bytes)
            a, b, c, d = self.read_reply(_id)
            print(f"FB RPM: {a}, FB CUR: {b}, FB POS: {c}, ERR: {d}")

    def send_torque(self, _id: int, torque: float):
        """
        Send a torque (current) command.

        Args:
            torque: in Amps, range -8.0 to 8.0 for DDSM115.

        IMPORTANT: Switch to Mode 1 (current loop) first with set_drive_mode().
        """
        torque = max(min(torque, 8.0), -8.0)
        raw = int(self.map(torque, -8.0, 8.0, -32767, 32767))
        trq_ints = self.Int16ToBytesArray(raw)

        cmd_bytes = struct.pack(
            self.str_9bytes,
            _id, 0x64, trq_ints[0], trq_ints[1],
            0x00, 0x00, 0x00, 0x00, 0x00,
        )
        cmd_bytes = self.crc_attach(cmd_bytes)

        with self.lock:
            while not self.ser.writable():
                print_warning("send_torque not writable")
            self.ser.write(cmd_bytes)
            a, b, c, d = self.read_reply(_id)
            print(f"FB RPM: {a}, FB CUR: {b}, FB POS: {c}, ERR: {d}")
    def reset_motor_state(self, motor_id=0x01):
        with self.lock:
            # 1. Send Brake/Stop Command (Forces PID to drop and halt)
            print_info("Sending Brake command to clear errors...")
            
            # Pack the 9 bytes (struct.pack) instead of using a list
            brake_payload = struct.pack(
                self.str_9bytes,
                motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00
            )
            
            # Let your existing class method calculate and attach the CRC-8 byte
            BRAKE_CMD = self.crc_attach(brake_payload)
            
            self.ser.write(BRAKE_CMD)
            self.ser.read_until(size=10) # Clear the reply buffer
            
            time.sleep(0.1) # Give the motor brain a fraction of a second to process

            # 2. Switch Mode Back to Velocity Loop (Mode 0x02)
            print_info("Re-initializing Velocity Mode...")
            
            # Pack the 9 bytes for the mode command
            mode_payload = struct.pack(
                self.str_9bytes,
                motor_id, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            )
            
            # Let your existing class method calculate and attach the CRC-8 byte
            MODE_CMD = self.crc_attach(mode_payload)
            
            self.ser.write(MODE_CMD)
            self.ser.read_until(size=10)
            
            print_info("Motor reset complete. Try sending RPM now.")

            
    def reset_motor_state(self, motor_id=0x01):
        with self.lock:
            # 1. Send Brake/Stop Command (Forces PID to drop and halt)
            print_info("Sending Brake command to clear errors...")
            
            # Pack the 9 bytes (struct.pack) instead of using a list
            brake_payload = struct.pack(
                self.str_9bytes,
                motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00
            )
            
            # Let your existing class method calculate and attach the CRC-8 byte
            BRAKE_CMD = self.crc_attach(brake_payload)
            
            self.ser.write(BRAKE_CMD)
            self.ser.read_until(size=10) # Clear the reply buffer
            
            time.sleep(0.1) # Give the motor brain a fraction of a second to process

            # 2. Switch Mode Back to Velocity Loop (Mode 0x02)
            print_info("Re-initializing Velocity Mode...")
            
            # Pack the 9 bytes for the mode command
            mode_payload = struct.pack(
                self.str_9bytes,
                motor_id, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
            )
            
            # Let your existing class method calculate and attach the CRC-8 byte
            MODE_CMD = self.crc_attach(mode_payload)
            
            self.ser.write(MODE_CMD)
            self.ser.read_until(size=10)
            
            print_info("Motor reset complete. Try sending RPM now.")

        
    def send_degree(self, _id: int, deg):
        """Send an absolute position command (Mode 3 -- position loop)."""
        raw = int(self.map(deg, 0, 360, 0, 32767))
        deg_ints = self.Int16ToBytesArray(raw)
        cmd_bytes = struct.pack(
            self.str_9bytes,
            _id, 0x64, deg_ints[0], deg_ints[1],
            0x00, 0x00, 0x00, 0x00, 0x00,
        )
        cmd_bytes = self.crc_attach(cmd_bytes)

        with self.lock:
            self.ser.write(cmd_bytes)
            _, _, _, _ = self.read_reply(_id)

    def set_brake(self, _id: int):
        """Engage the motor brake."""
        cmd_bytes = struct.pack(
            self.str_9bytes,
            _id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00,
        )
        cmd_bytes = self.crc_attach(cmd_bytes)
        self.ser.write(cmd_bytes)
        self.ser.read_until(size=10)

    def set_drive_mode(self, _id: int, _mode: int):
        """
        Switch the motor's control mode.

        Modes:
            1 -- current (torque) loop
            2 -- velocity loop
            3 -- position loop
        """
        mode_names = {1: "current (torque)", 2: "velocity", 3: "position"}
        print_info(f"Set {_id} as {mode_names.get(_mode, 'unknown')} mode")

        cmd_bytes = struct.pack(
            self.str_10bytes,
            _id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _mode,
        )
        with self.lock:
            self.ser.write(cmd_bytes)

    # ==========================================
    # READ COMMANDS
    # ==========================================

    def reset_motor_state(self, motor_id=0x01):
        """Force the motor into a stopped state and re-initialize velocity mode."""
    
        with self.lock:
            # 1. Send Brake/Stop Command (Forces PID to drop and halt)
            # Format: ID, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, CRC
            print_info("Sending Brake command to clear errors...")
            brake_payload = [motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]
            # Replace 0x00 at the end with your actual CRC8 calculation
            brake_crc = self.calculate_crc8(brake_payload) 
            
            BRAKE_CMD = struct.pack(
                self.str_10bytes,
                motor_id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, brake_crc
            )
            self.ser.write(BRAKE_CMD)
            self.ser.read_until(size=10) # Clear the reply buffer
            
            time.sleep(0.1) # Give the motor brain a fraction of a second to process

            # 2. Switch Mode Back to Velocity Loop (Mode 0x02)
            # Format: ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CRC
            print_info("Re-initializing Velocity Mode...")
            mode_payload = [motor_id, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            mode_crc = self.calculate_crc8(mode_payload)
            
            MODE_CMD = struct.pack(
                self.str_10bytes,
                motor_id, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode_crc
            )
            self.ser.write(MODE_CMD)
            self.ser.read_until(size=10)
            
            print_info("Motor reset complete. Try sending RPM now.")

    def get_motor_id(self):
        """Query the ID of the single connected motor."""
        ID_QUE = struct.pack(
            self.str_10bytes,
            0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE,
        )
        with self.lock:
            self.ser.write(ID_QUE)
            data = self.ser.read_until(size=10)
        print_info(f"ID: {data[0]}")
        print_info(f"Mode: {data[1]}")
        print_info(f"Error: {data[8]}")

    def get_motor_feedback(self, _id: int):
        """
        Request encoder feedback from one motor.

        Returns:
            (fb_rpm, fb_cur, fb_pos) -- RPM, current in Amps, position in degrees
        """
        fb_req_cmd = struct.pack(
            self.str_9bytes,
            _id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        )
        fb_req_cmd = self.crc_attach(fb_req_cmd)

        while not self.ser.writable():
            print_warning("get_motor_feedback not writable")
        self.ser.write(fb_req_cmd)

        fb_rpm, fb_cur, fb_pos, error = self.read_reply(_id)

        if error != 0:
            sensor_error       = error & 0b00000001
            over_current_error = error & 0b00000010
            phase_over_error   = error & 0b00000100
            stall_error        = error & 0b00001000
            print_warning(f"error {error}")
            print_warning(
                f"sens_err: {sensor_error} "
                f"phase_err: {phase_over_error} "
                f"stall_err: {stall_error}"
            )

        return fb_rpm, fb_cur, fb_pos

    # ==========================================
    # REPLY PARSER (RING BUFFER)
    # ==========================================

    def read_reply(self, _id, timeout=0.01):
        """
        Read a 10-byte reply frame from the motor.

        Uses a ring-buffer approach: bytes are checked one at a time
        until a valid frame (correct ID, mode, and CRC) is assembled.
        If timeout is exceeded, returns the most recent cached values.
        """
        got_reply = False
        ring_buffer = bytearray()
        start_time = time.time()

        while not got_reply:
            try:
                res = self.ser.read()
            except serial.serialutil.SerialException as e:
                print_warning(e)
                res = b''

            if len(res) != 0:
                # -- First byte: must match the expected motor ID --
                if (len(ring_buffer) == 0) and (res == _id.to_bytes(1, 'big')):
                    ring_buffer.append(int.from_bytes(res, 'big'))

                # -- Second byte: must be a valid mode (1, 2, or 3) --
                elif (len(ring_buffer) == 1) and (res in (b'\x01', b'\x02', b'\x03')):
                    ring_buffer.append(int.from_bytes(res, 'big'))

                # -- Bytes 3-10: accumulate until we have 10 --
                elif 2 <= len(ring_buffer) < 10:
                    ring_buffer.append(int.from_bytes(res, 'big'))

                    if len(ring_buffer) == 10:
                        # Verify CRC
                        crc_value = ring_buffer[-1]
                        crc_check = self.crc8(bytes(ring_buffer[:-1]))

                        if crc_value == crc_check:
                            cur_hi = ring_buffer[2]
                            cur_lo = ring_buffer[3]
                            rpm_hi = ring_buffer[4]
                            rpm_lo = ring_buffer[5]
                            position_lo = ring_buffer[7]
                            error = ring_buffer[8]

                            fb_cur = self.currentRawToCurrentAmp(
                                self.TwoBytesTo16Int(cur_hi, cur_lo)
                            )
                            fb_rpm = self.TwoBytesTo16Int(rpm_hi, rpm_lo)
                            fb_pos = self.RawToDegrees(position_lo)

                            self.prev_fb_rpm[_id - 1] = fb_rpm
                            self.prev_fb_cur[_id - 1] = fb_cur
                            self.prev_fb_pos[_id - 1] = fb_pos

                            got_reply = True
                        else:
                            print_warning("crc error")
                            ring_buffer = bytearray()
                else:
                    ring_buffer = bytearray()
            else:
                ring_buffer = bytearray()

            # -- Timeout: fall back to cached values --
            if (time.time() - start_time) > timeout:
                got_reply = True
                fb_rpm = self.prev_fb_rpm[_id - 1]
                fb_cur = self.prev_fb_cur[_id - 1]
                fb_pos = self.prev_fb_pos[_id - 1]
                error = 0
                break

        return fb_rpm, fb_cur, fb_pos, error


# ==========================================
# INTERACTIVE CLI
# ==========================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DDSM115 hub motor CLI")
    parser.add_argument("usb_name", help="USB device path (e.g. /dev/ttyACM1)", type=str)
    args = parser.parse_args()

    a = MotorControl(device=args.usb_name)

    while True:
        s = input("Enter command: ")

        if s == "q":
            break
        elif s == "set_id":
            motor_id = int(input("Enter motor ID: "))
            a.set_id(motor_id)
        elif s == "get_id":
            a.get_motor_id()
        elif s == "set_mode":
            motor_id = int(input("Enter motor ID: "))
            mode = int(input("Enter mode (1: current, 2: velocity, 3: position): "))
            a.set_drive_mode(motor_id, mode)
        elif s == "send_rpm":
            id = int(input("Enter motor ID: "))
            rpm = float(input("Enter RPM: "))
            a.send_rpm(id, rpm)
            # while True:
            #     a.send_rpm(id, rpm)
            #     time.sleep(0.05)
        elif s == "send_torque":
            torque = float(input("Enter torque (Amps): "))
            while True:
                a.send_torque(3, torque)
                a.send_torque(4, torque)
                time.sleep(0.1)
        elif s == "monitor_start":
            motor_id = int(input("Enter motor ID to monitor: "))
            a.start_monitoring(motor_id)
        elif s == "monitor_stop":
            a.stop_monitoring()
        elif s == "reset":
            motor_id = int(input("Enter motor ID to reset: "))
            a.reset_motor_state(motor_id)
        elif s == "climb":
            a.send_rpm(1, -200)
            a.send_rpm(2, 200)
            a.send_rpm(3, -100)
            a.send_rpm(4, 100)
            a.send_rpm(5, 0)
            a.send_rpm(6, 0)
        elif s == "stop":
            a.send_rpm(1, 0)
            a.send_rpm(2, 0)
            a.send_rpm(3, 0)
            a.send_rpm(4, 0)
            a.send_rpm(5, 0)
            a.send_rpm(6, 0)
        else:
            print("Unknown command")

    a.close()