import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('luggage_av')

    config_file = LaunchConfiguration("config_file")
    namespaced_tf = LaunchConfiguration("namespaced_tf")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, "rviz", config_file])],
        namespace="luggage_av",
        condition=UnlessCondition(namespaced_tf)
    )

    rviz_node_ns_tf = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, "rviz", config_file])],
        remappings=[('/tf', 'tf'), ('/tf_static','tf_static')],
        namespace="luggage_av",
        condition=IfCondition(namespaced_tf)
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, "rviz", "dev.rviz"),
            description="RVIZ configuration file"
        ),
        DeclareLaunchArgument(
            'namespaced_tf',
            default_value="false",
            description="Whether to use namespaced tf topic aka /ns/tf and /ns/tf_static"
        ), 

        rviz_node,
        rviz_node_ns_tf
    ])
