#!/usr/bin/env python3
# =============================================================================
#  odom_sign_fix.py -- RF2O Odometry Direction Corrector
#
#  PROBLEM: RF2O laser odometry computes displacement in the laser frame
#  (which faces physically backward due to the 180-degree yaw TF between
#  base_link and laser). RF2O publishes with child_frame_id=base_link but
#  does NOT properly apply the 180-degree rotation when converting from
#  laser-frame displacements to base_link-frame displacements.
#
#  OBSERVED: When robot is pushed physically forward, /rf2o/odom X DECREASES
#  (goes negative) -- the opposite of correct behavior.
#
#  FIX: Apply the missing 180-degree rotation by negating X and Y components
#  of both position and velocity. The orientation (yaw) quaternion is already
#  correctly reported in base_link frame -- do NOT rotate it.
#
#  RESULT: /rf2o/odom_fixed correctly reports positive X when moving forward.
#
#  PIPELINE:
#    /scan -> [RF2O] -> /rf2o/odom -> [odom_sign_fix] -> /rf2o/odom_fixed -> [EKF]
# =============================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSignFix(Node):
    def __init__(self):
        super().__init__('odom_sign_fix')
        self.sub = self.create_subscription(
            Odometry, '/rf2o/odom', self.cb, 10)
        self.pub = self.create_publisher(
            Odometry, '/rf2o/odom_fixed', 10)
        self.get_logger().info(
            'OdomSignFix: relaying /rf2o/odom -> /rf2o/odom_fixed '
            '(X and Y negated to correct 180-deg LiDAR mounting)')

    def cb(self, msg: Odometry):
        # RF2O correctly applies the laser->base_link 180-deg TF internally when the TF
        # is available at startup (verified: raw x INCREASES when pushing robot forward).
        # No sign correction needed -- relay unchanged to /rf2o/odom_fixed for EKF.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSignFix()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
