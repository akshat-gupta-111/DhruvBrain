#!/usr/bin/env python3
# =============================================================================
#  DHRUV 6-AXIS IMU DRIVER & ATTITUDE FILTER NODE
#  Supports:
#    1. Direct Jetson I2C (/dev/i2c-1 or /dev/i2c-7) for MPU6050 / GY-521 (Address 0x68)
#    2. USB-Serial IMUs (WT901, BNO055, or Arduino pass-through)
#    3. Graceful passive mode if no IMU is currently wired
# =============================================================================

import os
import sys
import time
import math
import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class IMUAHRSFilter:
    """Lightweight 6-DOF Madgwick AHRS implementation (Gyro + Accel only)."""
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 0.001:
            return self.q
        ax /= norm
        ay /= norm
        az /= norm

        f_1 = 2.0 * (q1 * q3 - q0 * q2) - ax
        f_2 = 2.0 * (q0 * q1 + q2 * q3) - ay
        f_3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az
        j_11or24 = 2.0 * q2
        j_12or23 = 2.0 * q3
        j_13or22 = 2.0 * q0
        j_14or21 = 2.0 * q1
        j_32 = 2.0 * j_14or21
        j_33 = 2.0 * j_11or24

        step_0 = j_14or21 * f_2 - j_11or24 * f_1
        step_1 = j_12or23 * f_1 + j_13or22 * f_2 - j_32 * f_3
        step_2 = j_12or23 * f_2 - j_33 * f_3 - j_13or22 * f_1
        step_3 = j_14or21 * f_1 + j_11or24 * f_2

        norm = math.sqrt(step_0 * step_0 + step_1 * step_1 + step_2 * step_2 + step_3 * step_3)
        if norm > 0.0001:
            step_0 /= norm
            step_1 /= norm
            step_2 /= norm
            step_3 /= norm

        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * step_0
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * step_1
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * step_2
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * step_3

        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm > 0.0001:
            self.q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]
        return self.q


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        self.declare_parameter('imu_port', os.environ.get('IMU_PORT', '/dev/ttyUSB1'))
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('frequency', 50.0)

        self.imu_port = self.get_parameter('imu_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.freq = float(self.get_parameter('frequency').value)

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.ahrs = IMUAHRSFilter(beta=0.1)

        self.last_update_time = time.time()
        self.running = True
        self.device_connected = False

        self.worker_thread = threading.Thread(target=self.detect_and_run, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("6-Axis IMU Driver initialized. Scanning for MPU6050 I2C or Serial...")

    def detect_and_run(self):
        """Attempts connection via direct Jetson I2C bus first, then serial USB."""
        while self.running and not self.device_connected:
            # 1. Check all Jetson Orin Nano I2C buses.
            # MPU6050 confirmed on i2c-7 (Jetson Orin Nano pins 3/5 mapping).
            for bus_path in ['/dev/i2c-7', '/dev/i2c-2', '/dev/i2c-1',
                             '/dev/i2c-0', '/dev/i2c-4', '/dev/i2c-5', '/dev/i2c-9']:
                if os.path.exists(bus_path):
                    if self.try_init_mpu6050_i2c(bus_path):
                        self.get_logger().info(f"[IMU] Successfully locked onto MPU6050 via Jetson I2C ({bus_path})!")
                        self.device_connected = True
                        self.i2c_loop(bus_path)
                        return

            # 2. Check Serial port
            if os.path.exists(self.imu_port):
                self.get_logger().info(f"[IMU] Trying serial connection on {self.imu_port}...")
                self.device_connected = True
                self.serial_loop()
                return

            # If no sensor is connected right now, sleep quietly
            time.sleep(3.0)

    def try_init_mpu6050_i2c(self, bus_path):
        """Attempts to ping and wake up MPU6050 on address 0x68."""
        try:
            import fcntl
            I2C_SLAVE = 0x0703
            MPU6050_ADDR = 0x68

            fd = os.open(bus_path, os.O_RDWR)
            fcntl.ioctl(fd, I2C_SLAVE, MPU6050_ADDR)
            # Wake up MPU6050 by writing 0 to PWR_MGMT_1 (0x6B)
            os.write(fd, bytes([0x6B, 0x00]))
            time.sleep(0.05)
            # Set gyro full scale ±250 deg/s (GYRO_CONFIG 0x1B = 0x00)
            os.write(fd, bytes([0x1B, 0x00]))
            # Set accel full scale ±2g (ACCEL_CONFIG 0x1C = 0x00)
            os.write(fd, bytes([0x1C, 0x00]))
            os.close(fd)
            return True
        except Exception:
            return False

    def i2c_loop(self, bus_path):
        """Reads MPU6050 14-byte telemetry block at 50Hz via I2C."""
        import fcntl
        I2C_SLAVE = 0x0703
        MPU6050_ADDR = 0x68

        try:
            fd = os.open(bus_path, os.O_RDWR)
            fcntl.ioctl(fd, I2C_SLAVE, MPU6050_ADDR)
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {bus_path}: {e}")
            self.device_connected = False
            return

        dt_target = 1.0 / self.freq
        while self.running:
            t0 = time.time()
            try:
                # Write register pointer to 0x3B (ACCEL_XOUT_H)
                os.write(fd, bytes([0x3B]))
                data = os.read(fd, 14)
                if len(data) == 14:
                    raw = struct.unpack('>7h', data)
                    # raw: [ax, ay, az, temp, gx, gy, gz]
                    # Accel sensitivity at ±2g: 16384 LSB/g, g = 9.80665 m/s^2
                    ax = (raw[0] / 16384.0) * 9.80665
                    ay = (raw[1] / 16384.0) * 9.80665
                    az = (raw[2] / 16384.0) * 9.80665
                    # Gyro sensitivity at ±250 deg/s: 131 LSB/(deg/s) -> convert to rad/s
                    gx = math.radians(raw[4] / 131.0)
                    gy = math.radians(raw[5] / 131.0)
                    gz = math.radians(raw[6] / 131.0)

                    # -------------------------------------------------------
                    # UPSIDE-DOWN MOUNTING CORRECTION (180-deg rotation around X-axis)
                    # The MPU6050 is mounted hanging upside down on the robot.
                    # This flips Y and Z axes for both accel and gyro:
                    #   ay -> -ay  (Y axis inverted)
                    #   az -> -az  (gravity direction corrected: -9.8 when level)
                    #   gy -> -gy  (Y rotation inverted)
                    #   gz -> -gz  (CRITICAL: yaw rate sign corrected for EKF)
                    # Without gz correction, EKF integrates yaw backwards.
                    # -------------------------------------------------------
                    ay = -ay
                    az = -az
                    gy = -gy
                    gz = -gz

                    now = time.time()
                    dt = max(now - self.last_update_time, 0.001)
                    self.last_update_time = now

                    q = self.ahrs.update(gx, gy, gz, ax, ay, az, dt)
                    self.publish_imu(q, gx, gy, gz, ax, ay, az)
            except Exception as ex:
                time.sleep(0.05)

            elapsed = time.time() - t0
            sleep_time = dt_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def serial_loop(self):
        """Fallback reading from serial USB port."""
        import serial
        ser = None
        while self.running:
            if ser is None or not ser.is_open:
                try:
                    ser = serial.Serial(self.imu_port, self.baudrate, timeout=0.1)
                except Exception:
                    time.sleep(2.0)
                    continue

            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("<IMU") and line.endswith(">"):
                    line = line[5:-1]
                parts = [p.strip() for p in line.split(',') if p.strip()]
                if len(parts) >= 6:
                    ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
                    gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])
                    if max(abs(gx), abs(gy), abs(gz)) > 15.0:
                        gx, gy, gz = math.radians(gx), math.radians(gy), math.radians(gz)

                    # Upside-down mounting correction (same as i2c_loop)
                    ay = -ay
                    az = -az
                    gy = -gy
                    gz = -gz

                    now = time.time()
                    dt = max(now - self.last_update_time, 0.001)
                    self.last_update_time = now
                    q = self.ahrs.update(gx, gy, gz, ax, ay, az, dt)
                    self.publish_imu(q, gx, gy, gz, ax, ay, az)
            except Exception:
                time.sleep(0.01)

    def publish_imu(self, q, gx, gy, gz, ax, ay, az):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.orientation_covariance = [0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0005]
        msg.linear_acceleration_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.05]

        self.pub_imu.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
