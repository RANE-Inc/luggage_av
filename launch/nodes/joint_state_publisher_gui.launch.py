from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        namespace=["/", namespace],
        remappings=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value="luggage_av",
            description="Namespace of the bot (usually its unique identifier)"
        ),

        jsp_gui_node
    ])
