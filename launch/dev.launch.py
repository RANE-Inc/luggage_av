import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    namespace = LaunchConfiguration("namespace")
    local_dev = LaunchConfiguration("local_dev")


    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),
        DeclareLaunchArgument(
            "local_dev",
            description="Set to true when using local machine as controller",
            default_value="false"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "rviz.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
            condition=IfCondition(local_dev)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "prod.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
        ),
    ])
