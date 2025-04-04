import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    namespace = LaunchConfiguration("namespace")
    slam_mode = LaunchConfiguration("namespace")


    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value="mapping",
            description="Mapping mode for slam_toolbox (either 'mapping' or 'localization')"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "ros2_control.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "robot_state_publisher.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
                ("sim_mode", "false"),
                ("mock_hardware", "false"),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "sllidar.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "slam_toolbox.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
                ("sim_mode", "false"),
                ("slam_mode", slam_mode),
            ],
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(pkg_share, "launch", "nodes", "gscam2.launch.py")
        #     ]),
        #     launch_arguments=[
        #         ("namespace", namespace),
        #     ],
        # ),
    ])
