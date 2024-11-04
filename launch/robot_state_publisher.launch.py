import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": xacro.process_file(os.path.join(pkg_share, "urdf", "robot.urdf.xacro")).toxml()},
            os.path.join(pkg_share, "parameters", "robot_state_publisher.yaml"),    
        ],
        namespace="luggage_av",
    )
    

    return LaunchDescription([
        robot_state_publisher_node
    ])
