import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")
    slam_mode = LaunchConfiguration("slam_mode")
    map_filename = LaunchConfiguration("map_filename")
    sim_mode = LaunchConfiguration("sim_mode")


    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, "configs", "dev.rviz"),
            description="RVIZ configuration file"
        ),
        DeclareLaunchArgument(
            "slam_mode",
            default_value="localization",
            description="A mode the slam_toolbox will run in (either 'mapping' or 'localization')"
        ),
        DeclareLaunchArgument(
            "map_filename",
            default_value="",
            description="Path and filenames of the map files (.posegraph and .data). All files should be in the same directory. Do not include file extension in the name"
        ),
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false"
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value="mapping",
            description="Mapping mode for slam_toolbox (either 'mapping' or 'localization')"
        ),


        # Launching Descriptions

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "teleop.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
                ("slam_mode", slam_mode),
                # ("map_filename", map_filename),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "rviz.launch.py")
            ]),
            launch_arguments=[
                ("namespace", namespace),
                ("config_file", config_file),
            ],
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(pkg_share, "launch", "nodes", "slam_toolbox.launch.py")
        #     ]),
        #     launch_arguments=[
        #         ("namespace", namespace),
        #         ("config_file", config_file),
        #         ("slam_mode", slam_mode),
        #         ("map_filename", map_filename),
        #         ("sim_mode", sim_mode),
        #     ],
        # ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(pkg_share, "launch", "nodes", "sllidar.launch.py")
        #     ]),
        #     launch_arguments=[
        #         ("namespace", namespace),
        #         ("slam_mode", slam_mode),
        #     ],
        # ),
    ])
