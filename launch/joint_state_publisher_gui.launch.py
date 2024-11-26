from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespaced_tf = LaunchConfiguration("namespaced_tf")

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        namespace="luggage_av",
        condition=UnlessCondition(namespaced_tf),
    )

    joint_state_publisher_gui_ns_tf = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static','tf_static')],
        namespace="luggage_av",
        condition=IfCondition(namespaced_tf),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespaced_tf',
            default_value="false",
            description="Whether to use namespaced tf topic aka /ns/tf and /ns/tf_static"
        ), 

        joint_state_publisher_gui,
        joint_state_publisher_gui_ns_tf
    ])
