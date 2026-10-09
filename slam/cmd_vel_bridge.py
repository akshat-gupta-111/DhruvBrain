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
        for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if os.path.exists(port):
                try:
                    self.serial = serial.Serial(port, 115200, timeout=0.1)
                    self.get_logger().info(f"[Bridge] Connected to Arduino on {port}")
                    self.serial.write(b"DURATION:RAW\n")
                    break
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
        
        # Translate ROS Twist into Dhruv's custom serial strings
        cmd = "<STOP>\n"
        
        if x > 0.05:
            cmd = "<FWD,150,0>\n"
        elif x < -0.05:
            cmd = "<REV,150,0>\n"
        elif z > 0.1:
            cmd = "<PIVOT_L,150,0>\n"
        elif z < -0.1:
            cmd = "<PIVOT_R,150,0>\n"
            
        if self.serial:
            self.serial.write(cmd.encode())

    def watchdog(self):
        if time.time() - self.last_msg_time > 0.4:
            if self.serial:
                self.serial.write(b"<STOP>\n")

def main():
    rclpy.init()
    node = CmdVelBridge()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
