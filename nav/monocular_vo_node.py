#!/usr/bin/env python3
# =============================================================================
#  DHRUV MONOCULAR VISUAL ODOMETRY NODE
#  Designed for standard single cameras (USB webcam / CSI mono camera)
#  Avoids the high-overhead stereo/depth requirements of Isaac ROS cuVSLAM.
#
#  Features:
#    - Multi-scale Lucas-Kanade optical flow on Shi-Tomasi / FAST corners
#    - Ground-plane motion constraint & RANSAC outlier rejection (filters moving people)
#    - Publishes nav_msgs/msg/Odometry on /vo/odom with linear vx and angular wz
#    - Dynamic covariance inflation when feature tracking quality drops
#    - Optional direct V4L2 device capture if no external camera node is present
# =============================================================================

import os
import sys
import time
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance, Twist, Vector3
from cv_bridge import CvBridge


class MonocularVisualOdometry(Node):
    def __init__(self):
        super().__init__('monocular_vo_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('odom_topic', '/vo/odom')
        self.declare_parameter('camera_device', os.environ.get('CAMERA_DEVICE', '/dev/video0'))
        self.declare_parameter('use_device_capture', False) # True if running without v4l2_camera node
        self.declare_parameter('camera_height_m', 0.85)     # Height of camera from floor
        self.declare_parameter('camera_pitch_deg', 0.0)     # Tilt down angle in degrees
        self.declare_parameter('max_features', 300)
        self.declare_parameter('min_features', 60)
        self.declare_parameter('focal_length_px', 500.0)    # Fallback if camera_info is not available

        self.camera_topic = self.get_parameter('camera_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cam_device = self.get_parameter('camera_device').value
        self.use_device_capture = self.get_parameter('use_device_capture').value
        self.cam_h = float(self.get_parameter('camera_height_m').value)
        self.max_features = int(self.get_parameter('max_features').value)
        self.min_features = int(self.get_parameter('min_features').value)
        self.focal_px = float(self.get_parameter('focal_length_px').value)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_pts = None
        self.prev_time = 0.0

        # Motion integration for fallback pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ROS 2 Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.debug_pub = self.create_publisher(Image, '/vo/debug_image', 2)

        # Camera calibration matrix K (defaults to 640x480 standard FOV)
        self.K = np.array([
            [self.focal_px, 0.0, 320.0],
            [0.0, self.focal_px, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        if self.use_device_capture:
            self.get_logger().info(f"Opening V4L2 device directly: {self.cam_device}")
            self.cap = cv2.VideoCapture(self.cam_device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.timer = self.create_timer(0.033, self.device_loop) # ~30 FPS
        else:
            self.get_logger().info(f"Subscribing to camera feed: {self.camera_topic}")
            self.sub_cam = self.create_subscription(Image, self.camera_topic, self.image_cb, 10)
            self.sub_info = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.info_cb, 10)

        self.get_logger().info("Monocular Visual Odometry Node Initialized.")

    def info_cb(self, msg: CameraInfo):
        if len(msg.k) == 9 and msg.k[0] > 0:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.focal_px = (self.K[0, 0] + self.K[1, 1]) / 2.0

    def device_loop(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return
        now = time.time()
        self.process_frame(frame, now)

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now <= 0:
            now = time.time()
        self.process_frame(frame, now)

    def process_frame(self, frame: np.ndarray, current_time: float):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(
                gray, maxCorners=self.max_features, qualityLevel=0.01, minDistance=10, blockSize=3
            )
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt <= 0.001 or dt > 0.5:
            # Drop frame if time jumped backwards or too large gap
            self.prev_gray = gray
            self.prev_time = current_time
            return

        # If keypoint count fell below threshold, detect fresh corners
        if self.prev_pts is None or len(self.prev_pts) < self.min_features:
            self.prev_pts = cv2.goodFeaturesToTrack(
                self.prev_gray, maxCorners=self.max_features, qualityLevel=0.01, minDistance=10, blockSize=3
            )

        if self.prev_pts is None or len(self.prev_pts) < 15:
            # Not enough features in frame (e.g. facing plain blank wall)
            self.prev_gray = gray
            self.prev_time = current_time
            self.publish_zero_velocity(current_time, quality=0.0)
            return

        # 1. Pyramidal Lucas-Kanade Optical Flow
        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        good_prev = self.prev_pts[status == 1]
        good_next = next_pts[status == 1]

        if len(good_next) < 12:
            self.prev_gray = gray
            self.prev_time = current_time
            self.publish_zero_velocity(current_time, quality=0.0)
            return

        # 2. 5-Point Essential Matrix with RANSAC
        E, mask = cv2.findEssentialMat(
            good_next, good_prev,
            focal=self.focal_px,
            pp=(self.K[0, 2], self.K[1, 2]),
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.5
        )

        vx = 0.0
        vy = 0.0
        wz = 0.0
        quality = 0.0

        if E is not None and mask is not None and np.sum(mask) >= 8:
            inliers_next = good_next[mask.ravel() == 1]
            inliers_prev = good_prev[mask.ravel() == 1]

            _, R, t, pass_mask = cv2.recoverPose(
                E, inliers_next, inliers_prev,
                focal=self.focal_px,
                pp=(self.K[0, 2], self.K[1, 2])
            )

            # Recover yaw angular rate from rotation matrix R
            # Yaw = atan2(R[1, 0], R[0, 0])
            delta_yaw = math.atan2(R[1, 0], R[0, 0])
            wz = delta_yaw / dt

            # Ground-plane metric scale recovery:
            # Points located in the lower half of the image belong to the floor plane.
            # Floor points have known height Y = cam_h.
            # Z = (f * cam_h) / (v - cy), where v is vertical image row.
            lower_mask = inliers_prev[:, 1] > (self.K[1, 2] + 20)
            if np.sum(lower_mask) >= 5:
                floor_pts_prev = inliers_prev[lower_mask]
                floor_pts_next = inliers_next[lower_mask]
                # Average vertical displacement on floor points
                dv = floor_pts_next[:, 1] - floor_pts_prev[:, 1]
                # Filter outliers
                median_dv = np.median(dv)
                # Metric velocity formula from floor optical flow:
                # v_forward = (median_dv * cam_h) / (focal_px * dt)
                metric_vx = (-median_dv * self.cam_h) / (self.focal_px * dt)
                # Clamp within robot physical kinematic capability (max ~0.4 m/s)
                vx = float(np.clip(metric_vx, -0.4, 0.4))
            else:
                # Fallback: estimate from normalized translation vector t
                # Note: t is a unit vector [tx, ty, tz]
                unit_tz = float(t[2][0])
                # Scale assumption: normal crawling speed
                vx = float(np.clip(unit_tz * 0.25, -0.4, 0.4))

            quality = float(np.sum(mask)) / float(len(good_next))

        # Publish debug image with tracked vectors (always publish so stream doesn't freeze)
        if self.debug_pub.get_subscription_count() > 0:
            dbg = frame.copy()
            if E is not None and mask is not None and np.sum(mask) >= 8:
                for p0, p1 in zip(inliers_prev[:40], inliers_next[:40]):
                    x0, y0 = int(p0[0]), int(p0[1])
                    x1, y1 = int(p1[0]), int(p1[1])
                    cv2.line(dbg, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    cv2.circle(dbg, (x1, y1), 3, (0, 0, 255), -1)
                status_txt = f"VO vx: {vx:+.2f} m/s wz: {wz:+.2f} rad/s"
            else:
                if self.prev_pts is not None:
                    for p in self.prev_pts[:40]:
                        cv2.circle(dbg, (int(p[0][0]), int(p[0][1])), 3, (0, 255, 255), -1)
                status_txt = "VO: Stationary / Ready (Tracking Keypoints)"

            cv2.putText(dbg, status_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
            except Exception:
                pass

        # Publish Odometry message with appropriate covariance
        self.publish_odometry(current_time, vx, vy, wz, quality)

        # Update state for next frame
        self.prev_gray = gray
        self.prev_pts = good_next.reshape(-1, 1, 2)
        self.prev_time = current_time

    def publish_odometry(self, timestamp: float, vx: float, vy: float, wz: float, quality: float):
        msg = Odometry()
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Twist
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        # Covariance: High confidence when quality is near 1.0; low confidence when near 0
        cov_val = 0.04 if quality > 0.5 else (0.2 if quality > 0.2 else 1.0)
        cov = [0.0] * 36
        cov[0] = cov_val       # vx variance
        cov[7] = cov_val * 2.0 # vy variance
        cov[14] = 9999.0       # vz variance (flat floor)
        cov[21] = 9999.0       # roll variance
        cov[28] = 9999.0       # pitch variance
        cov[35] = cov_val      # yaw rate variance
        msg.twist.covariance = cov

        self.odom_pub.publish(msg)

    def publish_zero_velocity(self, timestamp: float, quality: float):
        self.publish_odometry(timestamp, 0.0, 0.0, 0.0, quality)


def main(args=None):
    rclpy.init(args=args)
    node = MonocularVisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
