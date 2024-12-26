import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    start_controller = LaunchConfiguration("start_controller")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(pkg_share, "parameters", "controller_manager.yaml"),
        ],
        output="screen",
        namespace="luggage_av",
        remappings=[
            # ("/tf", "tf"), # TODO: namespaced_tf
            ("/luggage_av/diff_drive_controller/cmd_vel", "/luggage_av/cmd_vel"),
        ],
        condition=IfCondition(start_controller)
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace="luggage_av",
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file', os.path.join(pkg_share, "parameters", "diff_drive_controller.yaml"),
        ],
        # FIXME: Can't remap this way, need to find a better way
        # remappings=[
        #     ("/tf", "/luggage_av/tf"),
        # ],
        namespace="luggage_av",
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_controller", 
            default_value="true"
        ),

        controller_manager,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        )
    ])
