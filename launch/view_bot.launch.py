import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
# from lauch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_share = os.path.join(get_package_share_directory('luggage_av'))
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": xacro.process_file(os.path.join(pkg_share, "urdf", "robot.urdf.xacro")).toxml(),
                'use_sim_time': True,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_bot.rviz')],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])
