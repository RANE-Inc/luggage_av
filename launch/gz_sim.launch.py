import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

# IMPORTANT: Ubuntu Wayland fix for gz sim
os.environ["QT_QPA_PLATFORM"]="xcb"

def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    world = LaunchConfiguration("world")
    namespace = LaunchConfiguration("namespace")


    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="obstacles",
            description="Name of the world"
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ]),
            launch_arguments=[
                ('gz_args', ['-r -v 4 ', PathJoinSubstitution([pkg_share, 'worlds', world]), ".world"]),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "ros_gz_spawn_model.launch.py")
            ]),
            launch_arguments=[
                ("bridge_name", "parameter_bridge"),
                ("config_file", os.path.join(pkg_share, "configs", "gz_bridge.yaml")),
                ("namespace", namespace), # TODO: Why didn't I have to add slash in the beginning here??? It allows me to add a slash if I pass it from the CLI
                # ("bridge_params", os.path.join(pkg_share, "parameters", "gz_bridge.yaml")),
                ("world", ""),
                ("topic", [namespace, "/robot_description"]),
                ("name", "luggage_av"),
                ("allow_renaming", "true"),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "ros2_control.launch.py")
            ]),
            launch_arguments=[
                ("start_controller", "false"),
                ("namespace", namespace),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "robot_state_publisher.launch.py")
            ]),
            launch_arguments=[
                ("sim_mode", "true"),
                ("namespace", namespace),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "slam_toolbox.launch.py")
            ]),
            launch_arguments=[
                ("sim_mode", "true"),
                ("map_filename", [PathJoinSubstitution([pkg_share, "worlds", world]), ".yaml"]),
                ("namespace", namespace),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "nodes", "rviz.launch.py")
            ]),
            launch_arguments=[
                ("config_file", os.path.join(pkg_share, "configs", "dev.rviz")), # Gotta set explicitvely as it grabs configs/gz_bridge.yaml for unknown reason
                ("namespace", namespace),
            ]
        ),
    ])
