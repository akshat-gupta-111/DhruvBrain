#!/usr/bin/env python3
# =============================================================================
#  DHRUV NAVIGATION — ROS-to-Arduino Motor Command Bridge
#  Translates /cmd_vel and /cmd_vel_nav into Mecanum <MIX, vx, vy, w, 0> commands.
# =============================================================================

import os
import sys
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.serial = None
        self._last_cmd = ""
        self._last_send_time = 0.0
        self._last_print = 0.0
        self._last_no_serial_warn = 0.0
        self._last_reconnect_attempt = 0.0
        self._last_smoothed_time = 0.0
        self._is_moving = False

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.sub_cmd_vel_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_nav_cb, 10)
        
        self._connect_arduino()

        self.last_msg_time = 0.0
        self.timer = self.create_timer(0.1, self.watchdog)

    def _drain_serial(self):
        if self.serial and self.serial.is_open:
            try:
                waiting = self.serial.in_waiting
                if waiting > 0:
                    raw = self.serial.read(waiting).decode('utf-8', errors='ignore')
                    if "ERR" in raw or "Watchdog" in raw:
                        for l in raw.strip().split('\n'):
                            l = l.strip()
                            if l:
                                self.get_logger().warn(f"[Arduino] {l}")
            except Exception:
                pass

    def _connect_arduino(self):
        if self.serial and self.serial.is_open:
            return True

        now = time.time()
        if now - self._last_reconnect_attempt < 3.0:
            return False
        self._last_reconnect_attempt = now

        # ONLY scan ACM ports for Arduino — never touch /dev/ttyUSB* (LiDAR & IMU)
        candidate_ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
        for port in candidate_ports:
            if os.path.exists(port):
                try:
                    s = serial.Serial(port, 115200, timeout=0.8, write_timeout=0.8)
                    time.sleep(1.8)
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                    
                    s.write(b"MODE?\n")
                    s.flush()
                    
                    response = s.read(128).decode('utf-8', errors='ignore').strip()
                    resp_upper = response.upper()
                    if any(k in resp_upper for k in ["MODE", "STATUS", "ACK", "CABLE", "AUTONOMOUS", "MANUAL"]):
                        self.serial = s
                        self.get_logger().info(f"[Bridge] Connected to live Arduino on {port} (rx: {repr(response)})")
                        self.serial.write(b"MODE:AUTO\n")
                        self.serial.flush()
                        time.sleep(0.05)
                        self.serial.write(b"DURATION:RAW\n")
                        self.serial.flush()
                        time.sleep(0.05)
                        self.serial.write(b"<LED,CURIOSITY_GREEN>\n")
                        self.serial.flush()
                        return True
                    else:
                        s.close()
                except Exception as e:
                    pass
                    
        return False

    def cmd_vel_cb(self, msg):
        self._last_smoothed_time = time.time()
        self.process_twist(msg, source="/cmd_vel")

    def cmd_vel_nav_cb(self, msg):
        if time.time() - self._last_smoothed_time < 0.6:
            return
        self.process_twist(msg, source="/cmd_vel_nav")

    def process_twist(self, msg, source="/cmd_vel"):
        now = time.time()
        self.last_msg_time = now
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        MAX_LINEAR = 0.26
        MAX_Y = 0.12
        MAX_Z = 1.0
        MIN_PWM = 70
        MAX_PWM = 175

        self._drain_serial()

        x = 0.0 if abs(x) < 0.01 else max(-MAX_LINEAR, min(MAX_LINEAR, x))
        y = 0.0 if abs(y) < 0.01 else max(-MAX_Y, min(MAX_Y, y))
        z = 0.0 if abs(z) < 0.02 else max(-MAX_Z, min(MAX_Z, z))

        if x == 0.0 and y == 0.0 and z == 0.0:
            cmd = "<STOP>\n"
            is_active_move = False
        else:
            is_active_move = True
            vx = int(round((x / MAX_LINEAR) * MAX_PWM))
            vy = int(round((y / MAX_LINEAR) * MAX_PWM))
            w = int(round((z / MAX_Z) * MAX_PWM))

            largest_component = max(abs(vx), abs(vy), abs(w))
            if 0 < largest_component < MIN_PWM:
                compensation = MIN_PWM / largest_component
                vx = int(round(vx * compensation))
                vy = int(round(vy * compensation))
                w = int(round(w * compensation))

            if vx == 0 and vy == 0 and w == 0:
                cmd = "<STOP>\n"
                is_active_move = False
            else:
                cmd = f"<MIX,{vx},{vy},{w},0>\n"

        if now - self._last_print > 0.5:
            self.get_logger().info(f"[{source}] x:{x:.2f} y:{y:.2f} z:{z:.2f} -> {cmd.strip()}")
            self._last_print = now

        should_send = (cmd != self._last_cmd) or (is_active_move and (now - self._last_send_time > 0.08))

        if should_send:
            if not self.serial or not self.serial.is_open:
                self._connect_arduino()

            if self.serial and self.serial.is_open:
                try:
                    self.serial.write(cmd.encode('utf-8'))
                    self.serial.flush()
                    self._is_moving = is_active_move
                except Exception as e:
                    self.get_logger().error(f"Serial write failed: {e}. Reconnecting...")
                    try:
                        self.serial.close()
                    except Exception:
                        pass
                    self.serial = None
            else:
                if now - self._last_no_serial_warn > 2.0:
                    self.get_logger().error(f"[Bridge] Cannot send {cmd.strip()}: Arduino disconnected!")
                    self._last_no_serial_warn = now

            self._last_cmd = cmd
            self._last_send_time = now

    def watchdog(self):
        self._drain_serial()

        if not self.serial or not self.serial.is_open:
            self._connect_arduino()
            return

        now = time.time()
        if (now - self.last_msg_time > 0.8) and self._is_moving:
            try:
                self.serial.write(b"<STOP>\n")
                self.serial.flush()
                self._is_moving = False
                self._last_cmd = "<STOP>\n"
                self.get_logger().info("[Bridge Watchdog] Nav2 idle/timeout (0.8s): sent STOP.")
            except Exception as e:
                self.get_logger().error(f"Watchdog serial write failed: {e}")
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            try:
                node.serial.write(b"<STOP>\n")
                node.serial.flush()
                node.serial.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
