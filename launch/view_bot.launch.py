import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value="/luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "robot_state_publisher.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "rviz.launch.py")
            ]),
            launch_arguments=[
                ("config_file", os.path.join(pkg_share, "rviz", "view_bot.rviz")),
                ("namespace", namespace),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "joint_state_publisher_gui.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
        ),
    ])
