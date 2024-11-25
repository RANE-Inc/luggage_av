#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')
    
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 460800, 
                         'frame_id': 'laser',
                         'inverted': False, 
                         'angle_compensate': True, 
                         'scan_mode': 'Standard'}
            ],
            output='screen',
            # namespace="luggage_av"
        ),
    ])
