import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    camera_name = LaunchConfiguration("camera_name")
    namespace = LaunchConfiguration("namespace")

    gscam2_node = Node(
        package='gscam2',
        executable='gscam_main',
        name='gscam_publisher',
        output='screen',
        parameters=[
            os.path.join(pkg_share, "parameters", "gscam2.yaml"),
            {
                "camera_name": camera_name,
                "camera_info_url": ["file://", PathJoinSubstitution([pkg_share, "configs", camera_name]),".ini"],
            }
        ],
        namespace=["/", namespace],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value="logitech_c920",
            description="The name of the camera. Note: Do not include whitespaces in your name"
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        gscam2_node
    ])
