#!/usr/bin/env python3
# =============================================================================
#  DHRUV NAVIGATION — Sensor Diagnostic & Calibration Tool
#  Run this inside the container or host to independently verify each sensor:
#    python3 /workspace/test_sensors.py
# =============================================================================

import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


class SensorDiagnostic(Node):
    def __init__(self):
        super().__init__('sensor_diagnostic_node')

        self.scan_count = 0
        self.imu_count = 0
        self.vo_count = 0
        self.rf2o_count = 0
        self.ekf_count = 0

        self.last_scan = None
        self.last_imu = None
        self.last_vo = None
        self.last_rf2o = None
        self.last_ekf = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/vo/odom', self.vo_cb, 10)
        self.create_subscription(Odometry, '/rf2o/odom', self.rf2o_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_cb(self, msg):
        self.scan_count += 1
        self.last_scan = msg

    def imu_cb(self, msg):
        self.imu_count += 1
        self.last_imu = msg

    def vo_cb(self, msg):
        self.vo_count += 1
        self.last_vo = msg

    def rf2o_cb(self, msg):
        self.rf2o_count += 1
        self.last_rf2o = msg

    def ekf_cb(self, msg):
        self.ekf_count += 1
        self.last_ekf = msg


def main():
    rclpy.init()
    node = SensorDiagnostic()

    print("\n" + "=" * 65)
    print("  DHRUV NAVIGATION SENSOR SUITE DIAGNOSTICS")
    print("  Sampling ROS 2 topics for 5 seconds...")
    print("=" * 65 + "\n")

    start_t = time.time()
    while time.time() - start_t < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("-" * 65)
    print("1. RPLiDAR (/scan):")
    if node.scan_count > 0:
        rate = node.scan_count / 5.0
        ranges = [r for r in node.last_scan.ranges if 0.1 < r < 12.0]
        min_r = min(ranges) if ranges else 0.0
        print(f"   [PASS] Received {node.scan_count} scans (~{rate:.1f} Hz). Closest obst: {min_r:.2f} m")
    else:
        print("   [FAIL] No /scan data received! Check USB cable & permissions (/dev/ttyUSB0).")

    print("\n2. 6-Axis IMU (/imu/data):")
    if node.imu_count > 0:
        rate = node.imu_count / 5.0
        wz = node.last_imu.angular_velocity.z
        az = node.last_imu.linear_acceleration.z
        print(f"   [PASS] Received {node.imu_count} samples (~{rate:.1f} Hz). Yaw rate: {wz:+.3f} rad/s, Az: {az:.2f} m/s^2")
    else:
        print("   [WARN] No /imu/data received! Check IMU serial port (/dev/ttyUSB1 or /dev/i2c-1).")

    print("\n3. Monocular Visual Odometry (/vo/odom):")
    if node.vo_count > 0:
        rate = node.vo_count / 5.0
        vx = node.last_vo.twist.twist.linear.x
        print(f"   [PASS] Received {node.vo_count} frames (~{rate:.1f} Hz). Current VO vx: {vx:+.2f} m/s")
    else:
        print("   [WARN] No /vo/odom received! Check camera connection (/dev/video0).")

    print("\n4. RF2O Laser Odometry (/rf2o/odom):")
    if node.rf2o_count > 0:
        rate = node.rf2o_count / 5.0
        vx = node.last_rf2o.twist.twist.linear.x
        print(f"   [PASS] Received {node.rf2o_count} messages (~{rate:.1f} Hz). Current laser vx: {vx:+.2f} m/s")
    else:
        print("   [FAIL] No /rf2o/odom received! RF2O requires valid /scan.")

    print("\n5. EKF Fusion Output (/odometry/filtered):")
    if node.ekf_count > 0:
        rate = node.ekf_count / 5.0
        x = node.last_ekf.pose.pose.position.x
        y = node.last_ekf.pose.pose.position.y
        print(f"   [PASS] EKF is ACTIVE (~{rate:.1f} Hz). Fused Pose: x={x:+.3f}m, y={y:+.3f}m")
    else:
        print("   [FAIL] EKF is NOT publishing! Check /tmp/ekf.log.")

    print("\n6. Transform Tree (TF):")
    frames = ['laser', 'imu_link', 'camera_link', 'base_footprint']
    for f in frames:
        try:
            node.tf_buffer.lookup_transform('base_link', f, rclpy.time.Time())
            print(f"   [PASS] base_link -> {f} transform verified.")
        except Exception:
            print(f"   [WARN] base_link -> {f} transform lookup failed.")

    try:
        node.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        print("   [PASS] odom -> base_link (EKF) transform verified.")
    except Exception:
        print("   [WARN] odom -> base_link transform lookup failed.")

    print("\n" + "=" * 65 + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
