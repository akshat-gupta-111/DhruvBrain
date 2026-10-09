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
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        
        # Connect to Arduino (trying all standard ports)
        self.serial = None
        for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyUSB2", "/dev/ttyUSB3"]:
            if os.path.exists(port):
                try:
                    s = serial.Serial(port, 115200, timeout=0.5)
                    time.sleep(2.0)  # Wait for Arduino bootloader to finish
                    
                    # Read boot messages to verify this is actually our Arduino!
                    boot_msg = s.read(1000).decode('utf-8', errors='ignore')
                    if "DHRUVBRAIN" in boot_msg or "Arbitrator" in boot_msg or "AUTONOMOUS" in boot_msg:
                        self.serial = s
                        self.serial.reset_input_buffer()
                        self.serial.reset_output_buffer()
                        self.get_logger().info(f"[Bridge] Verified Arduino on {port}")
                        self.serial.write(b"DURATION:RAW\n")
                        self.serial.flush()
                        break
                    else:
                        self.get_logger().info(f"[Bridge] Port {port} is not the Arduino. Skipping.")
                        s.close()
                except Exception:
                    pass
                    
        if not self.serial:
            self.get_logger().error("[Bridge] Could not find Arduino USB port!")

        self.last_msg_time = time.time()
        # Safety watchdog: if Nav2 crashes or stops sending cmds, stop the robot!
        self.timer = self.create_timer(0.3, self.watchdog)

    def vel_callback(self, msg):
        self.last_msg_time = time.time()
        x = msg.linear.x
        z = msg.angular.z
        
        cmd = "<STOP>\n"
        
        # Give priority to turning if Nav2 wants to spin (z is much larger than x)
        if abs(z) > abs(x) * 2.0 and abs(z) > 0.02:
            if z > 0:
                cmd = "<PIVOT_L,255,200>\n"
            else:
                cmd = "<PIVOT_R,255,200>\n"
        else:
            if x > 0.01:
                cmd = "<FWD,255,200>\n"
            elif x < -0.01:
                cmd = "<REV,255,200>\n"
            
        # Log every half second so we can see what Nav2 is doing without flooding
        if time.time() - getattr(self, '_last_print', 0) > 0.5:
            self.get_logger().info(f"Nav2 sending cmd_vel -> x: {x:.2f}, z: {z:.2f} | Sending to Arduino: {cmd.strip()}")
            self._last_print = time.time()
            
        if self.serial:
            self.serial.write(cmd.encode())
            self.serial.flush()

    def watchdog(self):
        if time.time() - self.last_msg_time > 0.4:
            if self.serial:
                self.serial.write(b"<STOP>\n")
                self.serial.flush()

def main():
    rclpy.init()
    node = CmdVelBridge()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
