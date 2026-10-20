#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import os

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.serial = None
        self._last_cmd = ""
        self._last_send_time = 0.0
        self._last_print = 0.0
        self._last_no_serial_warn = 0.0
        self._last_reconnect_attempt = 0.0
        self._is_moving = False

        # Only subscribe to /cmd_vel (the final smoothed velocity stream).
        # Subscribing to both /cmd_vel and /cmd_vel_nav caused conflicting interleaving commands.
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        
        # Connect to Arduino
        self._connect_arduino()

        self.last_msg_time = 0.0
        # Safety watchdog: check every 100ms. If Nav2 stops publishing, send STOP once.
        self.timer = self.create_timer(0.1, self.watchdog)

    def _connect_arduino(self):
        if self.serial and self.serial.is_open:
            return True

        now = time.time()
        if now - self._last_reconnect_attempt < 3.0:
            return False
        self._last_reconnect_attempt = now

        # ONLY scan ACM ports for Arduino — NEVER touch ANY /dev/ttyUSB* device!
        candidate_ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
        for port in candidate_ports:
            if os.path.exists(port):
                try:
                    s = serial.Serial(port, 115200, timeout=0.8, write_timeout=0.8)
                    time.sleep(1.8)  # Wait for Arduino bootloader reset
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                    
                    # Test if the port is ALIVE by asking for status
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
                        self.get_logger().warn(f"[Bridge] Port {port} responded with unexpected data: {repr(response)}. Skipping.")
                        s.close()
                except Exception as e:
                    self.get_logger().warn(f"[Bridge] Failed on {port}: {e}")
                    pass
                    
        self.get_logger().warn("[Bridge] Arduino USB port not connected. Will retry in background...")
        return False

    def vel_callback(self, msg):
        now = time.time()
        self.last_msg_time = now
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # Proportional mapping parameters
        MAX_X = 0.5   # m/s Nav2 max linear speed
        MAX_Z = 1.0   # rad/s Nav2 max angular speed
        MIN_PWM = 70  # minimum PWM to overcome static friction
        MAX_PWM = 175 # keep under hardware SPEED_LIMIT (180)

        # Holonomic Mecanum mixed drive calculation
        # Map linear x, y and angular z directly into motor velocity components
        if abs(x) < 0.01 and abs(y) < 0.01 and abs(z) < 0.02:
            cmd = "<STOP>\n"
            is_active_move = False
        else:
            is_active_move = True
            vx = 0
            vy = 0
            w  = 0
            if abs(x) >= 0.01:
                sign_x = 1 if x > 0 else -1
                vx = int(sign_x * (MIN_PWM + (min(abs(x), MAX_X) / MAX_X) * (MAX_PWM - MIN_PWM)))
            if abs(y) >= 0.01:
                sign_y = 1 if y > 0 else -1
                vy = int(sign_y * (MIN_PWM + (min(abs(y), MAX_X) / MAX_X) * (MAX_PWM - MIN_PWM)))
            if abs(z) >= 0.02:
                sign_z = 1 if z > 0 else -1
                w = int(sign_z * (MIN_PWM + (min(abs(z), MAX_Z) / MAX_Z) * (MAX_PWM - MIN_PWM)))
            
            cmd = f"<MIX,{vx},{vy},{w},0>\n"

        if now - self._last_print > 0.5:
            self.get_logger().info(f"Nav2 cmd_vel x:{x:.2f} y:{y:.2f} z:{z:.2f} -> {cmd.strip()}")
            self._last_print = now

        # Only send if command changed or 80ms elapsed during continuous active movement
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
        # If we have no serial, attempt auto-reconnect with backoff
        if not self.serial or not self.serial.is_open:
            self._connect_arduino()
            return

        # If Nav2 has stopped sending cmd_vel for > 0.4s and we were previously moving,
        # send <STOP> once to bring the robot to a halt without spamming the bus.
        now = time.time()
        if (now - self.last_msg_time > 0.4) and self._is_moving:
            try:
                self.serial.write(b"<STOP>\n")
                self.serial.flush()
                self._is_moving = False
                self._last_cmd = "<STOP>\n"
                self.get_logger().info("[Bridge Watchdog] Nav2 idle/timeout: sent STOP.")
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
