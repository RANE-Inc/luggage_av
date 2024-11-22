import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


os.environ["QT_QPA_PLATFORM"]="xcb"

def generate_launch_description():

    pkg_share = get_package_share_directory("luggage_av")

    world = LaunchConfiguration('world')

    gz_entity_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/luggage_av/robot_description',
            '-name',  'luggage_av', 
            '-allow_renaming', 'true'
        ]
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"config_file": os.path.join(pkg_share,'parameters','gz_bridge.yaml')},  
        ],
    )

    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "rviz.launch.py")
            ])
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share,'worlds','obstacles.world'),
            description='World to load'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ]),
            launch_arguments=[
                ('gz_args', ['-r -v 4 ', PathJoinSubstitution([pkg_share, 'worlds', world])]),
            ]
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_entity_spawner,
                on_exit=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            os.path.join(pkg_share, "launch", "ros2_control.launch.py")
                        ]),
                        launch_arguments=[
                            ("start_controller", "false"),
                        ],
                    )
                ],
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share, "launch", "robot_state_publisher.launch.py")
            ]),
            launch_arguments=[
                ("sim_mode", "true")
            ]
        ),
        ros_gz_bridge,
        gz_entity_spawner,
        rviz,
    ])
