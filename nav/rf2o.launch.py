import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/rf2o/odom',
                # publish_tf should be False but parameter is not respected by some builds.
                # The /tf remapping below is the reliable workaround.
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 12.0
            }],
            # Redirect RF2O's /tf output to a dead-end topic so it never conflicts
            # with the EKF (robot_localization) which is the sole odom->base_link authority.
            remappings=[
                ('/tf', '/rf2o/tf_unused'),
            ]
        )
    ])
