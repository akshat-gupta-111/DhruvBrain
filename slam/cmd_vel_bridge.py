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
        # Listen to both /cmd_vel (from velocity smoother) and /cmd_vel_nav (direct from controller_server)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.vel_callback, 10)
        self.serial = None
        # Scan potential Arduino ports: ACM ports first, then USB1/USB2.
        # NEVER scan /dev/ttyUSB0 — that is reserved for RPLidar A1.
        candidate_ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyUSB1", "/dev/ttyUSB2"]
        for port in candidate_ports:
            if os.path.exists(port):
                try:
                    s = serial.Serial(port, 115200, timeout=1.0, write_timeout=1.0)
                    time.sleep(2.0)  # Wait for Arduino bootloader/reset
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                    
                    # Test if the port is ALIVE by asking for status
                    s.write(b"MODE?\n")
                    s.flush()
                    
                    # Read response (Arduino replies "ACK\n" then "STATUS: Mode=...")
                    response = s.read(128).decode('utf-8', errors='ignore').strip()
                    resp_upper = response.upper()
                    if any(k in resp_upper for k in ["MODE", "STATUS", "ACK", "CABLE", "AUTONOMOUS", "MANUAL"]):
                        self.serial = s
                        self.get_logger().info(f"[Bridge] Connected to live Arduino on {port} (rx: {repr(response)})")
                        self.serial.write(b"MODE:AUTO\n")
                        self.serial.flush()
                        time.sleep(0.1)
                        self.serial.write(b"DURATION:RAW\n")
                        self.serial.flush()
                        time.sleep(0.1)
                        self.serial.write(b"<LED,CURIOSITY_GREEN>\n")
                        self.serial.flush()
                        break
                    else:
                        self.get_logger().warn(f"[Bridge] Port {port} responded with unexpected data: {repr(response)}. Skipping.")
                        s.close()
                except Exception as e:
                    self.get_logger().warn(f"[Bridge] Failed on {port}: {e}")
                    pass
                    
        if not self.serial:
            self.get_logger().error("[Bridge] Could not find any responsive Arduino USB port!")

        self.last_msg_time = time.time()
        # Safety watchdog: if Nav2 crashes or stops sending cmds, stop the robot!
        self.timer = self.create_timer(0.3, self.watchdog)

    def vel_callback(self, msg):
        self.last_msg_time = time.time()
        x = msg.linear.x
        z = msg.angular.z

        cmd = "<STOP>\n"

        # Proportional speed scaling: map Nav2's velocity magnitudes to Arduino PWM range.
        # Nav2 typically sends x in [−0.5, 0.5] m/s and z in [−1.0, 1.0] rad/s.
        # We scale these to PWM [80, 200] so the robot doesn't lurch or stall.
        MAX_X   = 0.5   # m/s  — Nav2 max linear speed (tune via nav2 params)
        MAX_Z   = 1.0   # rad/s — Nav2 max angular speed
        MIN_PWM = 80    # minimum PWM that actually moves the robot
        MAX_PWM = 180   # keep well below SPEED_LIMIT=180 for safety

        fwd_pwm  = int(MIN_PWM + (min(abs(x), MAX_X) / MAX_X) * (MAX_PWM - MIN_PWM))
        turn_pwm = int(MIN_PWM + (min(abs(z), MAX_Z) / MAX_Z) * (MAX_PWM - MIN_PWM))

        # Decision: prioritise turning when angular component dominates
        if abs(z) > abs(x) * 1.5 and abs(z) > 0.05:
            # Pure pivot with proportional speed
            cmd = f"<PIVOT_L,{turn_pwm},200>\n" if z > 0 else f"<PIVOT_R,{turn_pwm},200>\n"
        elif abs(x) > 0.02:
            # Translate forward/backward with proportional speed
            cmd = f"<FWD,{fwd_pwm},200>\n" if x > 0 else f"<REV,{fwd_pwm},200>\n"
        # else: x and z both near-zero → STOP (already set above)

        # Log every half second so we can see what Nav2 is doing without flooding
        if time.time() - getattr(self, '_last_print', 0) > 0.5:
            self.get_logger().info(f"Nav2 cmd_vel x:{x:.2f} z:{z:.2f} → {cmd.strip()}")
            self._last_print = time.time()
            
        now = time.time()
        # Rate-limit to prevent overflowing the Arduino's 64-byte serial buffer.
        # Send immediately if command changes, otherwise refresh every 100ms.
        if cmd != getattr(self, '_last_cmd', '') or (now - getattr(self, '_last_send_time', 0)) > 0.1:
            if self.serial:
                try:
                    self.serial.write(cmd.encode('utf-8'))
                    self.serial.flush()
                except Exception as e:
                    self.get_logger().error(f"Serial write failed! Did the Arduino brown-out and disconnect? {e}")
                    self.serial = None
            else:
                if now - getattr(self, '_last_no_serial_warn', 0) > 1.0:
                    self.get_logger().error(f"[Bridge] CANNOT SEND {cmd.strip()}: Arduino serial port is NOT connected!")
                    self._last_no_serial_warn = now
            self._last_cmd = cmd
            self._last_send_time = now

    def watchdog(self):
        if time.time() - self.last_msg_time > 0.4:
            if self.serial:
                try:
                    self.serial.write(b"<STOP>\n")
                    self.serial.flush()
                except Exception as e:
                    self.get_logger().error(f"Watchdog serial write failed: {e}")
                    self.serial = None

def main():
    rclpy.init()
    node = CmdVelBridge()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
