import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('luggage_av')

    namespace = LaunchConfiguration("namespace")

    bt_execution = Node(
            package='luggage_av',
            executable='behavior_tree_node',
            name='behavior_tree_node',
            output='screen',
            namespace=["/", namespace],
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),
        bt_execution,
    ])