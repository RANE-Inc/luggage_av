#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, IfElseSubstitution, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    namespace = LaunchConfiguration("namespace")

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            os.path.join(pkg_share, "parameters", "sllidar.yaml"),
            {
                "frame_id": IfElseSubstitution(EqualsSubstitution(namespace, ""), "laser", [namespace, "/laser"]) # TODO: Remove prefix to allow for stricter namespace passing
            }
        ],
        output='screen',
        namespace=["/", namespace],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        sllidar_node,
    ])
