import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    config_file = LaunchConfiguration("config_file")
    namespace = LaunchConfiguration("namespace")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, "rviz", config_file])],
        namespace=["/", namespace],
        remappings=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, "configs", "dev.rviz"),
            description="RVIZ configuration file"
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        rviz_node
    ])
