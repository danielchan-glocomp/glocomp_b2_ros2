from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glocomp_b2_ros2',
            executable='b2_tf_node',
            name='b2_tf_node',
            output='screen'
        ),
        Node(
            package='glocomp_b2_ros2',
            executable='b2_odom_node',
            name='b2_tf_node',
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/rslidar_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'rslidar',
                'transform_tolerance': 0.01,
                'min_height': 0.00,
                'max_height': 0.05,
                'angle_min': -3.14,  # -M_PI/2
                'angle_max': 3.14,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='tf_pub',
            executable='clock_pub',
            name='clock_pub',
            output='screen'
        ),

    ])

