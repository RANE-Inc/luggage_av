import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

os.environ["QT_QPA_PLATFORM"]="xcb"

def generate_launch_description():

    pkg_share = os.path.join(get_package_share_directory("luggage_av"))
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'luggage_av', 
            '-allow_renaming', 'true'
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            os.path.join(pkg_share, "parameters", "diff_drive_controller.yaml"),
            ],
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ]),
            launch_arguments=[
                ("gz_args", [" -r -v 4 empty.sdf"]),
            ],
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": xacro.process_file(os.path.join(pkg_share, "urdf", "robot.urdf.xacro")).toxml(),
                'use_sim_time': True,
            }]
        ),
        gz_spawn_entity,
        Node(
            package="joy",
            executable="game_controller_node",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            parameters=[{
                "publish_stamped_twist": True,
                "require_enable_button": False,
            }],
            remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        )
    ])
